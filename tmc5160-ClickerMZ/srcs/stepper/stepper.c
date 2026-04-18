#include <stddef.h>
#include "stepper.h"
#include "tmc5160_reg.h"

/* ---------------------------------------------------------------
 * Module state — one slot per axis
 * --------------------------------------------------------------- */
static TMC5160_Config_t        s_cfg[STEPPER_MAX_AXES];
static TMC5160_RampConfig_t    s_ramp[STEPPER_MAX_AXES];
static TMC5160_StallCallback_t s_stall_cb[STEPPER_MAX_AXES];
static TMC5160_HomeCallback_t  s_home_cb[STEPPER_MAX_AXES];
static TMC5160_MotorStatus_t   s_status[STEPPER_MAX_AXES];
static const Stepper_HAL_t    *s_hal[STEPPER_MAX_AXES];
static bool                    s_initialised[STEPPER_MAX_AXES];

/* ---------------------------------------------------------------
 * Guard helpers
 * --------------------------------------------------------------- */
#define AXIS_OK(a)      ((a) < STEPPER_MAX_AXES && s_initialised[(a)])
#define AXIS_OK_SPI(a)  (AXIS_OK(a) && s_hal[(a)]->spi_write_read != NULL)

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
uint32_t stepper_mmps_to_vmax(uint8_t axis, float mm_per_sec)
{
    if (!AXIS_OK(axis)) { return 0U; }
    float usteps_per_mm = s_cfg[axis].steps_per_mm * (float)s_cfg[axis].microsteps;
    float v_usteps      = mm_per_sec * usteps_per_mm;
    return (uint32_t)(v_usteps * 8388608.0f / (float)s_cfg[axis].fclk_hz);
}

int32_t stepper_pos_to_usteps(uint8_t axis, float mm)
{
    if (!AXIS_OK(axis)) { return 0; }
    float usteps_per_mm = s_cfg[axis].steps_per_mm * (float)s_cfg[axis].microsteps;
    return (int32_t)(mm * usteps_per_mm);
}

float stepper_usteps_to_mm(uint8_t axis, int32_t usteps)
{
    if (!AXIS_OK(axis)) { return 0.0f; }
    float usteps_per_mm = s_cfg[axis].steps_per_mm * (float)s_cfg[axis].microsteps;
    return (float)usteps / usteps_per_mm;
}

/* ---------------------------------------------------------------
 * Initialisation
 * --------------------------------------------------------------- */
bool stepper_init_axis(uint8_t axis, const TMC5160_Config_t *cfg, const Stepper_HAL_t *hal)
{
    if (axis >= STEPPER_MAX_AXES || cfg == NULL || hal == NULL) { return false; }

    s_cfg[axis]         = *cfg;
    s_hal[axis]         = hal;
    s_stall_cb[axis]    = NULL;
    s_home_cb[axis]     = NULL;
    s_initialised[axis] = true;

    stepper_disable(axis);   /* keep driver off during register setup */

    /* --- SPI register init (TMC5160) — skipped for step/dir-only drivers --- */
    if (hal->spi_write_read != NULL)
    {
        /* GCONF */
        uint32_t gconf = TMC5160_GCONF_MULTISTEP_FILT
                       | TMC5160_GCONF_DIAG0_ERROR
                       | TMC5160_GCONF_DIAG1_STALL;

        if (cfg->invert_dir)
        {
            gconf |= TMC5160_GCONF_SHAFT;
        }
        if (cfg->chopper_mode == STEPPER_CHOP_STEALTHCHOP ||
            cfg->chopper_mode == STEPPER_CHOP_AUTO)
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

        /* TPOWERDOWN: ~160 ms */
        STEPPER_WriteReg(hal, TMC5160_REG_TPOWERDOWN, 10U, NULL);

        /* Chopper */
        bool stealthchop_en = (cfg->chopper_mode != STEPPER_CHOP_SPREADCYCLE);
        STEPPER_WriteReg(hal, TMC5160_REG_CHOPCONF,
                         build_chopconf(cfg, stealthchop_en),
                         NULL);

        /* StealthChop auto-tuning */
        if (cfg->chopper_mode != STEPPER_CHOP_SPREADCYCLE)
        {
            uint32_t pwmconf = (1UL << 18U) | (1UL << 19U) | (12UL << 28U);
            STEPPER_WriteReg(hal, TMC5160_REG_PWMCONF, pwmconf, NULL);
        }

        /* Auto chopper switching thresholds */
        if (cfg->chopper_mode == STEPPER_CHOP_AUTO)
        {
            STEPPER_WriteReg(hal, TMC5160_REG_TPWMTHRS,  500U,  NULL);
            STEPPER_WriteReg(hal, TMC5160_REG_TCOOLTHRS, 2000U, NULL);
        }
        else
        {
            STEPPER_WriteReg(hal, TMC5160_REG_TPWMTHRS,  0U, NULL);
            STEPPER_WriteReg(hal, TMC5160_REG_TCOOLTHRS, 0U, NULL);
        }

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
void stepper_enable(uint8_t axis)
{
    if (AXIS_OK(axis)) { s_hal[axis]->en_assert(); }
}

void stepper_disable(uint8_t axis)
{
    if (AXIS_OK(axis)) { s_hal[axis]->en_deassert(); }
}

/* ---------------------------------------------------------------
 * Ramp configuration  (TMC5160 ramp mode)
 * --------------------------------------------------------------- */
void stepper_set_ramp(uint8_t axis, const TMC5160_RampConfig_t *ramp)
{
    if (!AXIS_OK_SPI(axis) || ramp == NULL) { return; }

    s_ramp[axis] = *ramp;

    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_VSTART, ramp->VSTART, NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_A1,     ramp->A1,     NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_V1,     ramp->V1,     NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_AMAX,   ramp->AMAX,   NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_VMAX,   ramp->VMAX,   NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_DMAX,   ramp->DMAX,   NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_D1,     ramp->D1,     NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_VSTOP,  ramp->VSTOP,  NULL);
}

/* ---------------------------------------------------------------
 * Motion — ramp mode
 * --------------------------------------------------------------- */
void stepper_move_to(uint8_t axis, int32_t position)
{
    if (!AXIS_OK_SPI(axis)) { return; }
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_RAMPMODE, TMC5160_RAMPMODE_POSITION, NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_XTARGET,  (uint32_t)position,        NULL);
}

void stepper_move_relative(uint8_t axis, int32_t delta)
{
    if (!AXIS_OK_SPI(axis)) { return; }
    int32_t current = (int32_t)STEPPER_ReadReg(s_hal[axis], TMC5160_REG_XACTUAL, NULL);
    stepper_move_to(axis, current + delta);
}

void stepper_run(uint8_t axis, uint32_t vmax, bool reverse)
{
    if (!AXIS_OK_SPI(axis)) { return; }
    uint32_t mode = reverse ? TMC5160_RAMPMODE_VEL_NEG : TMC5160_RAMPMODE_VEL_POS;
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_VMAX,     vmax, NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_RAMPMODE, mode, NULL);
}

void stepper_stop(uint8_t axis)
{
    if (!AXIS_OK_SPI(axis)) { return; }
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_VMAX, 0U, NULL);
}

void stepper_set_position_zero(uint8_t axis)
{
    if (!AXIS_OK_SPI(axis)) { return; }
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_XACTUAL, 0U, NULL);
    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_XTARGET, 0U, NULL);
}

/* ---------------------------------------------------------------
 * Engineering unit wrappers
 * --------------------------------------------------------------- */
void stepper_move_mm(uint8_t axis, float mm)
{
    stepper_move_relative(axis, stepper_pos_to_usteps(axis, mm));
}

void stepper_run_mmps(uint8_t axis, float mm_per_sec, bool reverse)
{
    stepper_run(axis, stepper_mmps_to_vmax(axis, mm_per_sec), reverse);
}

/* ---------------------------------------------------------------
 * Status polling — call from main loop
 *
 * Reads XACTUAL, VACTUAL, DRV_STATUS (3 SPI transactions).
 * Fires home callback (one-shot) or stall callback on stall.
 * No-op for step/dir-only drivers.
 * --------------------------------------------------------------- */
void stepper_poll(uint8_t axis, TMC5160_MotorStatus_t *out)
{
    if (!AXIS_OK_SPI(axis)) { return; }

    TMC5160_Status_t spi_st;
    uint32_t drv;

    s_status[axis].xactual = (int32_t)STEPPER_ReadReg(s_hal[axis], TMC5160_REG_XACTUAL,   &spi_st);
    s_status[axis].vactual = (int32_t)STEPPER_ReadReg(s_hal[axis], TMC5160_REG_VACTUAL,   NULL);
    drv                    =           STEPPER_ReadReg(s_hal[axis], TMC5160_REG_DRVSTATUS, NULL);

    s_status[axis].spi_status    = spi_st;
    s_status[axis].sg_result     = (uint16_t)(drv & TMC5160_DRVSTATUS_SG_RESULT_MASK);
    s_status[axis].stalled       = (drv & TMC5160_DRVSTATUS_STALLGUARD) != 0U;
    s_status[axis].standstill    = (drv & TMC5160_DRVSTATUS_STST)       != 0U;
    s_status[axis].overtemp      = (drv & TMC5160_DRVSTATUS_OT)         != 0U;
    s_status[axis].overtemp_warn = (drv & TMC5160_DRVSTATUS_OTPW)       != 0U;
    s_status[axis].open_load_a   = (drv & TMC5160_DRVSTATUS_OLA)        != 0U;
    s_status[axis].open_load_b   = (drv & TMC5160_DRVSTATUS_OLB)        != 0U;
    s_status[axis].pos_reached   = spi_st.pos_reached;
    s_status[axis].vel_reached   = spi_st.vel_reached;

    if (s_status[axis].stalled)
    {
        if (s_home_cb[axis] != NULL)
        {
            /* Non-blocking home complete: stop, zero, fire callback (one-shot) */
            TMC5160_HomeCallback_t cb = s_home_cb[axis];
            s_home_cb[axis] = NULL;                  /* clear before calling     */
            stepper_stop(axis);
            s_hal[axis]->delay_us(50000U);           /* 50 ms debounce via HAL   */
            stepper_set_position_zero(axis);
            cb(axis);
        }
        else if (s_stall_cb[axis] != NULL)
        {
            s_stall_cb[axis]();
        }
    }

    if (out != NULL) { *out = s_status[axis]; }
}

bool stepper_is_stalled(uint8_t axis)  { return AXIS_OK(axis) && s_status[axis].stalled;     }
bool stepper_pos_reached(uint8_t axis) { return AXIS_OK(axis) && s_status[axis].pos_reached; }

/* ---------------------------------------------------------------
 * StallGuard configuration
 * threshold: -64 to +63 (lower = more sensitive)
 * filter_en: true = filtered (more stable, slower response)
 * --------------------------------------------------------------- */
void stepper_stallguard_config(uint8_t axis, int8_t threshold, bool filter_en)
{
    if (!AXIS_OK_SPI(axis)) { return; }

    uint32_t coolconf = STEPPER_ReadReg(s_hal[axis], TMC5160_REG_COOLCONF, NULL);

    coolconf &= ~((0x7FUL << TMC5160_COOLCONF_SGT_SHIFT) | TMC5160_COOLCONF_SFILT);
    coolconf |= ((uint32_t)((int32_t)threshold & 0x7F) << TMC5160_COOLCONF_SGT_SHIFT);

    if (filter_en) { coolconf |= TMC5160_COOLCONF_SFILT; }

    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_COOLCONF, coolconf, NULL);
}

/* ---------------------------------------------------------------
 * CoolStep configuration
 * semin: lower SG threshold for current increase (1-15, 0=disable)
 * semax: upper SG threshold band width (0-15)
 * --------------------------------------------------------------- */
void stepper_coolstep_config(uint8_t axis, uint8_t semin, uint8_t semax)
{
    if (!AXIS_OK_SPI(axis)) { return; }

    uint32_t coolconf = STEPPER_ReadReg(s_hal[axis], TMC5160_REG_COOLCONF, NULL);

    coolconf &= ~((0x0FUL << TMC5160_COOLCONF_SEMIN_SHIFT) |
                  (0x0FUL << TMC5160_COOLCONF_SEMAX_SHIFT));
    coolconf |= ((uint32_t)(semin & 0x0FU) << TMC5160_COOLCONF_SEMIN_SHIFT);
    coolconf |= ((uint32_t)(semax & 0x0FU) << TMC5160_COOLCONF_SEMAX_SHIFT);

    STEPPER_WriteReg(s_hal[axis], TMC5160_REG_COOLCONF, coolconf, NULL);
}

void stepper_set_stall_callback(uint8_t axis, TMC5160_StallCallback_t cb)
{
    if (axis < STEPPER_MAX_AXES) { s_stall_cb[axis] = cb; }
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
void stepper_home(uint8_t axis, bool reverse, uint32_t creep_vmax,
                  TMC5160_HomeCallback_t on_complete)
{
    if (!AXIS_OK_SPI(axis)) { return; }

    if (on_complete != NULL)
    {
        /* Non-blocking: register one-shot callback and start motion */
        s_home_cb[axis] = on_complete;
        stepper_run(axis, creep_vmax, reverse);
    }
    else
    {
        /* Blocking: poll until stall, then stop and zero */
        TMC5160_MotorStatus_t st;
        stepper_run(axis, creep_vmax, reverse);
        do { stepper_poll(axis, &st); } while (!st.stalled);
        stepper_stop(axis);
        s_hal[axis]->delay_us(50000U);   /* 50 ms debounce via HAL */
        stepper_set_position_zero(axis);
    }
}

/* ---------------------------------------------------------------
 * Raw register access — tuning / debug / encoder setup
 * --------------------------------------------------------------- */
void stepper_write_reg(uint8_t axis, uint8_t addr, uint32_t data)
{
    if (AXIS_OK_SPI(axis)) { STEPPER_WriteReg(s_hal[axis], addr, data, NULL); }
}

uint32_t stepper_read_reg(uint8_t axis, uint8_t addr)
{
    if (!AXIS_OK_SPI(axis)) { return 0U; }
    return STEPPER_ReadReg(s_hal[axis], addr, NULL);
}