#include <stddef.h>
#include "stepper.h"
#include "stepper_reg.h"

#define MM_PER_INCH  25.4f

/* ---------------------------------------------------------------
 * Guard helpers
 * --------------------------------------------------------------- */
#define MOTOR_OK(m)      ((m) != NULL && (m)->initialised)
#define MOTOR_OK_SPI(m)  (MOTOR_OK(m) && (m)->hal->spi_write_read != NULL)

/* ---------------------------------------------------------------
 * CHOPCONF.MRES encoding
 * Microsteps 256..1 → MRES 0..8
 * INTPOL=1 always — chip interpolates to 256 for smooth motion.
 * --------------------------------------------------------------- */
static uint32_t mres_encode(uint16_t usteps)
{
    switch (usteps)
    {
        case 256: return 0U;
        case 128: return 1U;
        case  64: return 2U;
        case  32: return 3U;
        case  16: return 4U;
        case   8: return 5U;
        case   4: return 6U;
        case   2: return 7U;
        case   1: return 8U;
        default:  return 4U;   /* fallback: 16 microsteps */
    }
}

static uint32_t build_ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay)
{
    return ((uint32_t)(ihold      & 0x1FU) << TMC5160_IHOLD_SHIFT)
         | ((uint32_t)(irun       & 0x1FU) << TMC5160_IRUN_SHIFT)
         | ((uint32_t)(iholddelay & 0x0FU) << TMC5160_IHOLDDELAY_SHIFT);
}

/*
 * Build CHOPCONF register.
 * VSENSE=0 for BTT PRO (75 mΩ), VSENSE=1 for standard (110 mΩ).
 */
static uint32_t build_chopconf(const TMC5160_Config_t *cfg, bool stealthchop_en)
{
    uint32_t mres   = mres_encode(cfg->microsteps);
    uint32_t vsense = (cfg->rsense_mohm >= 110U) ? TMC5160_CHOPCONF_VSENSE : 0U;
    uint32_t chopconf;

    if (stealthchop_en)
    {
        /* StealthChop: TOFF=3, TBL=1 */
        chopconf = 3U
                 | (1UL << 15U)
                 | vsense
                 | (mres << TMC5160_CHOPCONF_MRES_SHIFT)
                 | TMC5160_CHOPCONF_INTPOL;
    }
    else
    {
        /* SpreadCycle: TOFF=3, TBL=2, HSTRT=4, HEND=1 */
        chopconf = 3U
                 | (4UL << 4U)
                 | (1UL << 7U)
                 | (2UL << 15U)
                 | vsense
                 | (mres << TMC5160_CHOPCONF_MRES_SHIFT)
                 | TMC5160_CHOPCONF_INTPOL;
    }

    return chopconf;
}

/* ---------------------------------------------------------------
 * Engineering unit conversions
 *
 *   VMAX_reg = v [usteps/s] × 2^23 / fCLK
 * --------------------------------------------------------------- */
static float usteps_per_mm(const Stepper_t *motor)
{
    return motor->cfg.steps_per_mm * (float)motor->cfg.microsteps;
}

static float usteps_per_deg(const Stepper_t *motor)
{
    return motor->cfg.steps_per_deg * (float)motor->cfg.microsteps;
}

uint32_t stepper_mmps_to_vmax(Stepper_t *motor, float mm_per_sec)
{
    if (!MOTOR_OK(motor)) { return 0U; }
    float v = mm_per_sec * usteps_per_mm(motor);
    return (uint32_t)(v * 8388608.0f / (float)motor->cfg.fclk_hz);
}

uint32_t stepper_dps_to_vmax(Stepper_t *motor, float deg_per_sec)
{
    if (!MOTOR_OK(motor) || motor->cfg.steps_per_deg == 0.0f) { return 0U; }
    float v = deg_per_sec * usteps_per_deg(motor);
    return (uint32_t)(v * 8388608.0f / (float)motor->cfg.fclk_hz);
}

int32_t stepper_pos_to_usteps(Stepper_t *motor, float mm)
{
    if (!MOTOR_OK(motor)) { return 0; }
    return (int32_t)(mm * usteps_per_mm(motor));
}

float stepper_usteps_to_mm(Stepper_t *motor, int32_t usteps)
{
    if (!MOTOR_OK(motor)) { return 0.0f; }
    return (float)usteps / usteps_per_mm(motor);
}

/* ---------------------------------------------------------------
 * Initialisation
 * --------------------------------------------------------------- */
bool stepper_init(Stepper_t *motor, const TMC5160_Config_t *cfg, const Stepper_HAL_t *hal)
{
    if (motor == NULL || cfg == NULL || hal == NULL) { return false; }

    motor->cfg         = *cfg;
    motor->hal         = hal;
    motor->stall_cb    = NULL;
    motor->home_cb     = NULL;
    motor->initialised = true;
    motor->pos_usteps  = 0;

    stepper_disable(motor);   /* keep driver off during register setup */

    /* --- SPI register init (TMC5160) — skipped for step/dir-only drivers --- */
    if (hal->spi_write_read != NULL)
    {
        /* GCONF */
        uint32_t gconf = TMC5160_GCONF_MULTISTEP_FILT;

        /* DIAG0 routing */
        gconf |= TMC5160_GCONF_DIAG0_ERROR;              /* always route errors to DIAG0 */
        if (cfg->diag0_otpw)   { gconf |= TMC5160_GCONF_DIAG0_OTPW; }

        /* DIAG1 routing */
        if (cfg->diag1_index)   { gconf |= TMC5160_GCONF_DIAG1_INDEX; }
        else if (cfg->diag1_onstate) { gconf |= TMC5160_GCONF_DIAG1_ONSTATE; }
        else                    { gconf |= TMC5160_GCONF_DIAG1_STALL; } /* default */

        if (cfg->invert_dir)
        {
            gconf |= TMC5160_GCONF_SHAFT;
        }
        if (cfg->chopper_mode == STEPPER_CHOP_STEALTHCHOP ||
            cfg->chopper_mode == STEPPER_CHOP_AUTO         ||
            cfg->chopper_mode == STEPPER_CHOP_AUTO_DCSTEP)
        {
            gconf |= TMC5160_GCONF_EN_PWM_MODE;
        }

        STEPPER_WriteReg(hal, TMC5160_REG_GCONF, gconf, NULL);

        /* Clear GSTAT flags from power-on */
        STEPPER_WriteReg(hal, TMC5160_REG_GSTAT, 0x07U, NULL);

        /* Current control */
        STEPPER_WriteReg(hal, TMC5160_REG_IHOLD_IRUN,
                         build_ihold_irun(cfg->ihold, cfg->irun, cfg->iholddelay),
                         NULL);

        /* TPOWERDOWN: coil hold delay after standstill */
        STEPPER_WriteReg(hal, TMC5160_REG_TPOWERDOWN,
                         (cfg->tpowerdown > 0U) ? (uint32_t)cfg->tpowerdown : 10U,
                         NULL);

        /* Chopper */
        bool stealthchop_en = (cfg->chopper_mode == STEPPER_CHOP_STEALTHCHOP ||
                               cfg->chopper_mode == STEPPER_CHOP_AUTO         ||
                               cfg->chopper_mode == STEPPER_CHOP_AUTO_DCSTEP);
        STEPPER_WriteReg(hal, TMC5160_REG_CHOPCONF,
                         build_chopconf(cfg, stealthchop_en),
                         NULL);

        /* StealthChop auto-tuning */
        if (cfg->chopper_mode == STEPPER_CHOP_STEALTHCHOP ||
            cfg->chopper_mode == STEPPER_CHOP_AUTO         ||
            cfg->chopper_mode == STEPPER_CHOP_AUTO_DCSTEP)
        {
            /* PWM_GRAD auto, PWM_OFS auto, pwm_autoscale, pwm_autograd, freq=2 */
            uint32_t pwmconf = (1UL << 18U) | (1UL << 19U) | (12UL << 28U);

            /* Apply freewheel bits (20:21) */
            pwmconf |= (((uint32_t)cfg->freewheel) & 3UL) << TMC5160_PWMCONF_FREEWHEEL_SHIFT;

            STEPPER_WriteReg(hal, TMC5160_REG_PWMCONF, pwmconf, NULL);
        }
        else
        {
            /* SpreadCycle only: still set freewheel if non-zero */
            if (cfg->freewheel != STEPPER_FREEWHEEL_NORMAL)
            {
                uint32_t pwmconf = (((uint32_t)cfg->freewheel) & 3UL)
                                    << TMC5160_PWMCONF_FREEWHEEL_SHIFT;
                STEPPER_WriteReg(hal, TMC5160_REG_PWMCONF, pwmconf, NULL);
            }
        }

        /* Auto chopper switching thresholds */
        if (cfg->chopper_mode == STEPPER_CHOP_AUTO ||
            cfg->chopper_mode == STEPPER_CHOP_AUTO_DCSTEP)
        {
            STEPPER_WriteReg(hal, TMC5160_REG_TPWMTHRS,  500U,  NULL);
            STEPPER_WriteReg(hal, TMC5160_REG_TCOOLTHRS, 2000U, NULL);
        }
        else
        {
            STEPPER_WriteReg(hal, TMC5160_REG_TPWMTHRS,  0U, NULL);
            STEPPER_WriteReg(hal, TMC5160_REG_TCOOLTHRS, 0U, NULL);
        }

        /* THIGH: enables DCStep / fullstep above this velocity */
        STEPPER_WriteReg(hal, TMC5160_REG_THIGH, cfg->thigh, NULL);

        /* DCStep minimum velocity (0 = disabled) */
        STEPPER_WriteReg(hal, TMC5160_REG_VDCMIN, cfg->dcstep_vmin, NULL);

        /* TZEROWAIT: pause at V=0 between direction changes */
        STEPPER_WriteReg(hal, TMC5160_REG_TZEROWAIT, (uint32_t)cfg->tzerowait, NULL);

        /* Encoder interface */
        if (cfg->encoder_enable)
        {
            STEPPER_WriteReg(hal, TMC5160_REG_ENCMODE, 0x01U, NULL);
        }

        /* Ramp generator initial state */
        if (cfg->drive_mode == STEPPER_MODE_RAMP)
        {
            STEPPER_WriteReg(hal, TMC5160_REG_RAMPMODE, TMC5160_RAMPMODE_POSITION, NULL);
            STEPPER_WriteReg(hal, TMC5160_REG_XACTUAL,  0U, NULL);
            STEPPER_WriteReg(hal, TMC5160_REG_XTARGET,  0U, NULL);
        }
    }

    return true;
}

/* ---------------------------------------------------------------
 * Enable / Disable  (ENN = active LOW)
 * --------------------------------------------------------------- */
void stepper_enable(Stepper_t *motor)
{
    if (MOTOR_OK(motor)) { motor->hal->en_assert(); }
}

void stepper_disable(Stepper_t *motor)
{
    if (MOTOR_OK(motor)) { motor->hal->en_deassert(); }
}

/* ---------------------------------------------------------------
 * Ramp configuration  (TMC5160 ramp mode)
 * --------------------------------------------------------------- */
void stepper_set_ramp(Stepper_t *motor, const TMC5160_RampConfig_t *ramp)
{
    if (!MOTOR_OK_SPI(motor) || ramp == NULL) { return; }

    motor->ramp = *ramp;

    STEPPER_WriteReg(motor->hal, TMC5160_REG_VSTART, ramp->VSTART, NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_A1,     ramp->A1,     NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_V1,     ramp->V1,     NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_AMAX,   ramp->AMAX,   NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_VMAX,   ramp->VMAX,   NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_DMAX,   ramp->DMAX,   NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_D1,     ramp->D1,     NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_VSTOP,  ramp->VSTOP,  NULL);
}

/* ---------------------------------------------------------------
 * Motion — ramp mode
 * --------------------------------------------------------------- */
void stepper_move_to(Stepper_t *motor, int32_t position)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    STEPPER_WriteReg(motor->hal, TMC5160_REG_RAMPMODE, TMC5160_RAMPMODE_POSITION, NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_XTARGET,  (uint32_t)position,        NULL);
}

void stepper_move_relative(Stepper_t *motor, int32_t delta)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    int32_t current = (int32_t)STEPPER_ReadReg(motor->hal, TMC5160_REG_XACTUAL, NULL);
    stepper_move_to(motor, current + delta);
}

void stepper_run(Stepper_t *motor, uint32_t vmax, bool reverse)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    uint32_t mode = reverse ? TMC5160_RAMPMODE_VEL_NEG : TMC5160_RAMPMODE_VEL_POS;
    STEPPER_WriteReg(motor->hal, TMC5160_REG_VMAX,     vmax, NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_RAMPMODE, mode, NULL);
}

void stepper_stop(Stepper_t *motor)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    STEPPER_WriteReg(motor->hal, TMC5160_REG_VMAX, 0U, NULL);
}

void stepper_set_position_zero(Stepper_t *motor)
{
    if (MOTOR_OK_SPI(motor))
    {
        STEPPER_WriteReg(motor->hal, TMC5160_REG_XACTUAL, 0U, NULL);
        STEPPER_WriteReg(motor->hal, TMC5160_REG_XTARGET, 0U, NULL);
    }
    if (MOTOR_OK(motor)) { motor->pos_usteps = 0; }
}

/* ---------------------------------------------------------------
 * Engineering unit wrappers — linear (mm / inch)
 * --------------------------------------------------------------- */
void stepper_move_mm(Stepper_t *motor, float mm)
{
    stepper_move_relative(motor, stepper_pos_to_usteps(motor, mm));
}

void stepper_move_to_mm(Stepper_t *motor, float mm)
{
    if (!MOTOR_OK(motor)) { return; }
    stepper_move_to(motor, (int32_t)(mm * usteps_per_mm(motor)));
}

void stepper_run_mmps(Stepper_t *motor, float mm_per_sec, bool reverse)
{
    stepper_run(motor, stepper_mmps_to_vmax(motor, mm_per_sec), reverse);
}

void stepper_move_inch(Stepper_t *motor, float inch)
{
    stepper_move_mm(motor, inch * MM_PER_INCH);
}

void stepper_move_to_inch(Stepper_t *motor, float inch)
{
    stepper_move_to_mm(motor, inch * MM_PER_INCH);
}

void stepper_run_ips(Stepper_t *motor, float inch_per_sec, bool reverse)
{
    stepper_run_mmps(motor, inch_per_sec * MM_PER_INCH, reverse);
}

float stepper_get_position_mm(Stepper_t *motor)
{
    if (!MOTOR_OK(motor)) { return 0.0f; }
    int32_t pos = (motor->cfg.drive_mode == STEPPER_MODE_STEPDIR)
                ? motor->pos_usteps
                : motor->status.xactual;
    float mm = (float)pos / usteps_per_mm(motor);
    return (motor->cfg.units == STEPPER_UNITS_INCH) ? mm / MM_PER_INCH : mm;
}

float stepper_get_position_inch(Stepper_t *motor)
{
    if (!MOTOR_OK(motor)) { return 0.0f; }
    int32_t pos = (motor->cfg.drive_mode == STEPPER_MODE_STEPDIR)
                ? motor->pos_usteps
                : motor->status.xactual;
    return (float)pos / usteps_per_mm(motor) / MM_PER_INCH;
}

void stepper_set_position_mm(Stepper_t *motor, float mm)
{
    if (!MOTOR_OK(motor)) { return; }
    int32_t usteps = (int32_t)(mm * usteps_per_mm(motor));
    motor->pos_usteps = usteps;
    if (MOTOR_OK_SPI(motor))
    {
        STEPPER_WriteReg(motor->hal, TMC5160_REG_XACTUAL, (uint32_t)usteps, NULL);
        STEPPER_WriteReg(motor->hal, TMC5160_REG_XTARGET, (uint32_t)usteps, NULL);
    }
}

/* ---------------------------------------------------------------
 * Rotation axis wrappers (steps_per_deg must be > 0)
 * --------------------------------------------------------------- */
void stepper_move_deg(Stepper_t *motor, float deg)
{
    if (!MOTOR_OK(motor) || motor->cfg.steps_per_deg == 0.0f) { return; }
    stepper_move_relative(motor, (int32_t)(deg * usteps_per_deg(motor)));
}

void stepper_move_to_deg(Stepper_t *motor, float deg)
{
    if (!MOTOR_OK(motor) || motor->cfg.steps_per_deg == 0.0f) { return; }
    stepper_move_to(motor, (int32_t)(deg * usteps_per_deg(motor)));
}

void stepper_run_dps(Stepper_t *motor, float deg_per_sec, bool reverse)
{
    stepper_run(motor, stepper_dps_to_vmax(motor, deg_per_sec), reverse);
}

float stepper_get_position_deg(Stepper_t *motor)
{
    if (!MOTOR_OK(motor) || motor->cfg.steps_per_deg == 0.0f) { return 0.0f; }
    int32_t pos = (motor->cfg.drive_mode == STEPPER_MODE_STEPDIR)
                ? motor->pos_usteps
                : motor->status.xactual;
    return (float)pos / usteps_per_deg(motor);
}

void stepper_set_position_deg(Stepper_t *motor, float deg)
{
    if (!MOTOR_OK(motor) || motor->cfg.steps_per_deg == 0.0f) { return; }
    int32_t usteps = (int32_t)(deg * usteps_per_deg(motor));
    motor->pos_usteps = usteps;
    if (MOTOR_OK_SPI(motor))
    {
        STEPPER_WriteReg(motor->hal, TMC5160_REG_XACTUAL, (uint32_t)usteps, NULL);
        STEPPER_WriteReg(motor->hal, TMC5160_REG_XTARGET, (uint32_t)usteps, NULL);
    }
}

/* ---------------------------------------------------------------
 * Status polling — call from main loop
 *
 * Reads XACTUAL, VACTUAL, DRV_STATUS (3 SPI transactions).
 * Fires home callback (one-shot) or stall callback on stall.
 * No-op for step/dir-only drivers.
 * --------------------------------------------------------------- */
void stepper_poll(Stepper_t *motor, TMC5160_MotorStatus_t *out)
{
    if (!MOTOR_OK_SPI(motor)) { return; }

    TMC5160_Status_t spi_st;
    uint32_t drv;

    motor->status.xactual = (int32_t)STEPPER_ReadReg(motor->hal, TMC5160_REG_XACTUAL,   &spi_st);
    motor->status.vactual = (int32_t)STEPPER_ReadReg(motor->hal, TMC5160_REG_VACTUAL,   NULL);
    drv                   =           STEPPER_ReadReg(motor->hal, TMC5160_REG_DRVSTATUS, NULL);

    motor->status.spi_status    = spi_st;
    motor->status.sg_result     = (uint16_t)(drv & TMC5160_DRVSTATUS_SG_RESULT_MASK);
    motor->status.stalled       = (drv & TMC5160_DRVSTATUS_STALLGUARD) != 0U;
    motor->status.standstill    = (drv & TMC5160_DRVSTATUS_STST)       != 0U;
    motor->status.overtemp      = (drv & TMC5160_DRVSTATUS_OT)         != 0U;
    motor->status.overtemp_warn = (drv & TMC5160_DRVSTATUS_OTPW)       != 0U;
    motor->status.open_load_a   = (drv & TMC5160_DRVSTATUS_OLA)        != 0U;
    motor->status.open_load_b   = (drv & TMC5160_DRVSTATUS_OLB)        != 0U;
    motor->status.pos_reached   = spi_st.pos_reached;
    motor->status.vel_reached   = spi_st.vel_reached;

    /* Keep software counter in sync with ramp mode XACTUAL */
    if (motor->cfg.drive_mode == STEPPER_MODE_RAMP)
    {
        motor->pos_usteps = motor->status.xactual;
    }

    if (motor->status.stalled)
    {
        if (motor->home_cb != NULL)
        {
            /* Non-blocking home complete: stop, zero, fire callback (one-shot) */
            TMC5160_HomeCallback_t cb = motor->home_cb;
            motor->home_cb = NULL;                   /* clear before calling     */
            stepper_stop(motor);
            motor->hal->delay_us(50000U);            /* 50 ms debounce via HAL   */
            stepper_set_position_zero(motor);
            cb();
        }
        else if (motor->stall_cb != NULL)
        {
            motor->stall_cb();
        }
    }

    if (out != NULL) { *out = motor->status; }
}

bool stepper_is_stalled(Stepper_t *motor)  { return MOTOR_OK(motor) && motor->status.stalled;     }
bool stepper_pos_reached(Stepper_t *motor) { return MOTOR_OK(motor) && motor->status.pos_reached; }

/* ---------------------------------------------------------------
 * Step/Dir software position counter update
 *
 * Call this from the HAL step ISR (or from move_steps() after
 * each pulse) to keep s_pos_usteps accurate in step/dir mode.
 * --------------------------------------------------------------- */
void stepper_step_tick(Stepper_t *motor, bool forward)
{
    if (!MOTOR_OK(motor)) { return; }
    if (forward) { motor->pos_usteps++; }
    else         { motor->pos_usteps--; }
}

/* ---------------------------------------------------------------
 * Runtime current control
 * --------------------------------------------------------------- */
void stepper_set_irun(Stepper_t *motor, uint8_t irun)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    motor->cfg.irun = irun & 0x1FU;
    STEPPER_WriteReg(motor->hal, TMC5160_REG_IHOLD_IRUN,
                     build_ihold_irun(motor->cfg.ihold,
                                      motor->cfg.irun,
                                      motor->cfg.iholddelay),
                     NULL);
}

void stepper_set_ihold(Stepper_t *motor, uint8_t ihold)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    motor->cfg.ihold = ihold & 0x1FU;
    STEPPER_WriteReg(motor->hal, TMC5160_REG_IHOLD_IRUN,
                     build_ihold_irun(motor->cfg.ihold,
                                      motor->cfg.irun,
                                      motor->cfg.iholddelay),
                     NULL);
}

/* ---------------------------------------------------------------
 * StallGuard configuration
 * threshold: -64 to +63 (lower = more sensitive)
 * filter_en: true = filtered (more stable, slower response)
 * --------------------------------------------------------------- */
void stepper_stallguard_config(Stepper_t *motor, int8_t threshold, bool filter_en)
{
    if (!MOTOR_OK_SPI(motor)) { return; }

    uint32_t coolconf = STEPPER_ReadReg(motor->hal, TMC5160_REG_COOLCONF, NULL);

    coolconf &= ~((0x7FUL << TMC5160_COOLCONF_SGT_SHIFT) | TMC5160_COOLCONF_SFILT);
    coolconf |= ((uint32_t)((int32_t)threshold & 0x7F) << TMC5160_COOLCONF_SGT_SHIFT);

    if (filter_en) { coolconf |= TMC5160_COOLCONF_SFILT; }

    STEPPER_WriteReg(motor->hal, TMC5160_REG_COOLCONF, coolconf, NULL);
}

/* ---------------------------------------------------------------
 * CoolStep configuration
 * semin:  lower SG threshold for current increase (1-15, 0=disable)
 * semax:  upper SG threshold band width (0-15)
 * seup:   current increment step size (0-3 → 1/2/4/8)
 * sedn:   current decrement speed (0-3 → per 32/8/2/1 SG samples)
 * seimin: minimum current floor (false=½ IRUN, true=¼ IRUN)
 * --------------------------------------------------------------- */
void stepper_coolstep_config(Stepper_t *motor,
                             uint8_t semin,
                             uint8_t semax,
                             uint8_t seup,
                             uint8_t sedn,
                             bool    seimin)
{
    if (!MOTOR_OK_SPI(motor)) { return; }

    uint32_t coolconf = STEPPER_ReadReg(motor->hal, TMC5160_REG_COOLCONF, NULL);

    coolconf &= ~((0x0FUL << TMC5160_COOLCONF_SEMIN_SHIFT) |
                  (0x03UL << TMC5160_COOLCONF_SEUP_SHIFT)  |
                  (0x0FUL << TMC5160_COOLCONF_SEMAX_SHIFT) |
                  (0x03UL << TMC5160_COOLCONF_SEDN_SHIFT)  |
                  TMC5160_COOLCONF_SEIMIN);

    coolconf |= ((uint32_t)(semin & 0x0FU) << TMC5160_COOLCONF_SEMIN_SHIFT);
    coolconf |= ((uint32_t)(semax & 0x0FU) << TMC5160_COOLCONF_SEMAX_SHIFT);
    coolconf |= ((uint32_t)(seup  & 0x03U) << TMC5160_COOLCONF_SEUP_SHIFT);
    coolconf |= ((uint32_t)(sedn  & 0x03U) << TMC5160_COOLCONF_SEDN_SHIFT);
    if (seimin) { coolconf |= TMC5160_COOLCONF_SEIMIN; }

    STEPPER_WriteReg(motor->hal, TMC5160_REG_COOLCONF, coolconf, NULL);
}

void stepper_set_stall_callback(Stepper_t *motor, TMC5160_StallCallback_t cb)
{
    if (motor != NULL) { motor->stall_cb = cb; }
}

/* ---------------------------------------------------------------
 * Sensorless homing via StallGuard
 *
 * Run stepper_stallguard_config() first to tune sensitivity.
 *
 * on_complete = NULL  : blocking — polls for stall, stops, zeros, returns.
 * on_complete = fn    : non-blocking — starts motion and returns immediately;
 *                       fn(axis) is called from stepper_poll() on stall.
 *                       Caller must pump stepper_poll() from the main loop.
 *
 * StallGuard requires SPI — no-op for step/dir-only drivers.
 * --------------------------------------------------------------- */
void stepper_home(Stepper_t *motor, bool reverse, uint32_t creep_vmax,
                  TMC5160_HomeCallback_t on_complete)
{
    if (!MOTOR_OK_SPI(motor)) { return; }

    if (on_complete != NULL)
    {
        /* Non-blocking: register one-shot callback and start motion */
        motor->home_cb = on_complete;
        stepper_run(motor, creep_vmax, reverse);
    }
    else
    {
        /* Blocking: poll until stall, then stop and zero */
        TMC5160_MotorStatus_t st;
        stepper_run(motor, creep_vmax, reverse);
        do { stepper_poll(motor, &st); } while (!st.stalled);
        stepper_stop(motor);
        motor->hal->delay_us(50000U);   /* 50 ms debounce via HAL */
        stepper_set_position_zero(motor);
    }
}

/* ---------------------------------------------------------------
 * Raw register access — tuning / debug / encoder setup
 * --------------------------------------------------------------- */
void stepper_write_reg(Stepper_t *motor, uint8_t addr, uint32_t data)
{
    if (MOTOR_OK_SPI(motor)) { STEPPER_WriteReg(motor->hal, addr, data, NULL); }
}

uint32_t stepper_read_reg(Stepper_t *motor, uint8_t addr)
{
    if (!MOTOR_OK_SPI(motor)) { return 0U; }
    return STEPPER_ReadReg(motor->hal, addr, NULL);
}

/* ---------------------------------------------------------------
 * Hardware stop switch configuration  (SW_MODE register)
 *
 * Maps every field of TMC5160_SwMode_t directly to the SW_MODE
 * register bits.  Call after stepper_init_axis().
 * --------------------------------------------------------------- */
void stepper_sw_mode_config(Stepper_t *motor, const TMC5160_SwMode_t *sw)
{
    if (!MOTOR_OK_SPI(motor) || sw == NULL) { return; }

    uint32_t reg = 0U;
    if (sw->stop_l_enable)    { reg |= TMC5160_SWMODE_STOP_L_ENABLE; }
    if (sw->stop_r_enable)    { reg |= TMC5160_SWMODE_STOP_R_ENABLE; }
    if (sw->pol_stop_l)       { reg |= TMC5160_SWMODE_POL_STOP_L; }
    if (sw->pol_stop_r)       { reg |= TMC5160_SWMODE_POL_STOP_R; }
    if (sw->swap_lr)          { reg |= TMC5160_SWMODE_SWAP_LR; }
    if (sw->latch_l_active)   { reg |= TMC5160_SWMODE_LATCH_L_ACTIVE; }
    if (sw->latch_l_inactive) { reg |= TMC5160_SWMODE_LATCH_L_INACTIVE; }
    if (sw->latch_r_active)   { reg |= TMC5160_SWMODE_LATCH_R_ACTIVE; }
    if (sw->latch_r_inactive) { reg |= TMC5160_SWMODE_LATCH_R_INACTIVE; }
    if (sw->en_latch_encoder) { reg |= TMC5160_SWMODE_EN_LATCH_ENCODER; }
    if (sw->sg_stop)          { reg |= TMC5160_SWMODE_SG_STOP; }
    if (sw->en_softstop)      { reg |= TMC5160_SWMODE_EN_SOFTSTOP; }

    STEPPER_WriteReg(motor->hal, TMC5160_REG_SW_MODE, reg, NULL);
}

/* Return latched position captured on last stop event */
int32_t stepper_get_xlatch(Stepper_t *motor)
{
    if (!MOTOR_OK_SPI(motor)) { return 0; }
    return (int32_t)STEPPER_ReadReg(motor->hal, TMC5160_REG_XLATCH, NULL);
}

/* Return raw RAMP_STAT register — test against TMC5160_RAMPSTAT_* masks */
uint32_t stepper_get_ramp_stat(Stepper_t *motor)
{
    if (!MOTOR_OK_SPI(motor)) { return 0U; }
    return STEPPER_ReadReg(motor->hal, TMC5160_REG_RAMP_STAT, NULL);
}

/* ---------------------------------------------------------------
 * Runtime velocity band tuning
 *
 * tpwmthrs  : TSTEP threshold for StealthChop -> SpreadCycle switch
 * tcoolthrs : TSTEP threshold to enable StallGuard / CoolStep
 * thigh     : TSTEP threshold to enable fullstep / DCStep (0=off)
 *
 * All three are TSTEP-based (inverse velocity) — larger value = lower speed.
 * --------------------------------------------------------------- */
void stepper_set_velocity_bands(Stepper_t *motor,
                                uint32_t tpwmthrs,
                                uint32_t tcoolthrs,
                                uint32_t thigh)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    STEPPER_WriteReg(motor->hal, TMC5160_REG_TPWMTHRS,  tpwmthrs,  NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_TCOOLTHRS, tcoolthrs, NULL);
    STEPPER_WriteReg(motor->hal, TMC5160_REG_THIGH,     thigh,     NULL);
}

/* ---------------------------------------------------------------
 * Position compare — fires a pulse on DIAG1 when XACTUAL == pos
 * Requires GCONF.DIAG1_POSCOMP to be set (not set by default).
 * --------------------------------------------------------------- */
void stepper_set_position_compare(Stepper_t *motor, int32_t pos)
{
    if (!MOTOR_OK_SPI(motor)) { return; }
    STEPPER_WriteReg(motor->hal, TMC5160_REG_X_COMPARE, (uint32_t)pos, NULL);
}