#include "stepper.h"
#include "tmc5160_reg.h"
#include "definitions.h"

/* ---------------------------------------------------------------
 * Module state
 * --------------------------------------------------------------- */
static TMC5160_Config_t        s_cfg;
static TMC5160_RampConfig_t    s_ramp;
static TMC5160_StallCallback_t s_stall_cb    = NULL;
static TMC5160_MotorStatus_t   s_status;
static bool                    s_initialised = false;

/* ---------------------------------------------------------------
 * Private helpers
 * --------------------------------------------------------------- */

/*
 * CHOPCONF.MRES encoding
 * Microsteps 256..1 map to MRES 0..8
 * INTPOL=1 always — interpolates internally to 256 microsteps
 * for smooth motion regardless of commanded resolution.
 */
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
 * Build CHOPCONF register
 *
 * BTT TMC5160 PRO uses external MOSFETs — SpreadCycle values
 * TOFF=3, TBL=2, HSTRT=4, HEND=1 are safe defaults.
 * VSENSE=0 for high-current external FET boards (rsense <= 75mΩ).
 * VSENSE=1 reduces full-scale by ~45% — use only on 110mΩ+ boards.
 */
static uint32_t build_chopconf(const TMC5160_Config_t *cfg, bool stealthchop_en)
{
    uint32_t mres = mres_encode(cfg->microsteps);
    uint32_t chopconf;

    /* VSENSE: 0 for BTT PRO (75mΩ), 1 for standard (110mΩ) */
    uint32_t vsense = (cfg->rsense_mohm >= 110U) ? TMC5160_CHOPCONF_VSENSE : 0U;

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
 * TMC5160 velocity unit:
 *   v [usteps/s] = VACTUAL * fCLK / 2^23
 *   VMAX_reg     = v_usteps_per_s * 2^23 / fCLK
 *
 * Acceleration unit:
 *   a [usteps/s²] = AMAX * fCLK² / (512 * 256)
 *   AMAX_reg      = a_usteps_per_s2 * 131072 / fCLK²
 *
 * usteps_per_mm = steps_per_mm * microsteps
 * --------------------------------------------------------------- */
uint32_t stepper_mmps_to_vmax(float mm_per_sec)
{
    float usteps_per_mm = s_cfg.steps_per_mm * (float)s_cfg.microsteps;
    float v_usteps      = mm_per_sec * usteps_per_mm;
    return (uint32_t)(v_usteps * 8388608.0f / (float)s_cfg.fclk_hz);
}

int32_t stepper_pos_to_usteps(float mm)
{
    float usteps_per_mm = s_cfg.steps_per_mm * (float)s_cfg.microsteps;
    return (int32_t)(mm * usteps_per_mm);
}

float stepper_usteps_to_mm(int32_t usteps)
{
    float usteps_per_mm = s_cfg.steps_per_mm * (float)s_cfg.microsteps;
    return (float)usteps / usteps_per_mm;
}

/* ---------------------------------------------------------------
 * Initialisation
 * --------------------------------------------------------------- */
bool stepper_init(const TMC5160_Config_t *cfg)
{
    if (cfg == NULL) { return false; }

    s_cfg         = *cfg;
    s_initialised = true;

    stepper_disable();   /* keep driver off during register setup */

    /* --- GCONF --- */
    uint32_t gconf = TMC5160_GCONF_MULTISTEP_FILT
                   | TMC5160_GCONF_DIAG0_ERROR    /* DIAG0 pin = error flag   */
                   | TMC5160_GCONF_DIAG1_STALL;   /* DIAG1 pin = stall flag   */

    if (cfg->invert_dir)
    {
        gconf |= TMC5160_GCONF_SHAFT;
    }
    if (cfg->chopper_mode == STEPPER_CHOP_STEALTHCHOP ||
        cfg->chopper_mode == STEPPER_CHOP_AUTO)
    {
        gconf |= TMC5160_GCONF_EN_PWM_MODE;
    }

    TMC5160_WriteReg(TMC5160_REG_GCONF, gconf, NULL);

    /* Clear GSTAT reset/error flags from power-on */
    TMC5160_WriteReg(TMC5160_REG_GSTAT, 0x07U, NULL);

    /* --- Current control --- */
    TMC5160_WriteReg(TMC5160_REG_IHOLD_IRUN,
                     build_ihold_irun(cfg->ihold, cfg->irun, cfg->iholddelay),
                     NULL);

    /* TPOWERDOWN: time before standstill current reduction (~160ms at default 10) */
    TMC5160_WriteReg(TMC5160_REG_TPOWERDOWN, 10U, NULL);

    /* --- Chopper --- */
    bool stealthchop_en = (cfg->chopper_mode != STEPPER_CHOP_SPREADCYCLE);
    TMC5160_WriteReg(TMC5160_REG_CHOPCONF,
                     build_chopconf(cfg, stealthchop_en),
                     NULL);

    /* --- StealthChop auto-tuning --- */
    if (cfg->chopper_mode != STEPPER_CHOP_SPREADCYCLE)
    {
        /* PWM_AUTOSCALE | PWM_AUTOGRAD | PWM_LIM=12 */
        uint32_t pwmconf = (1UL << 18U) | (1UL << 19U) | (12UL << 28U);
        TMC5160_WriteReg(TMC5160_REG_PWMCONF, pwmconf, NULL);
    }

    /* --- Auto chopper switching thresholds --- */
    if (cfg->chopper_mode == STEPPER_CHOP_AUTO)
    {
        /* These are starting points — tune per motor via stepper_write_reg() */
        TMC5160_WriteReg(TMC5160_REG_TPWMTHRS,  500U,  NULL); /* StealthChop -> SpreadCycle */
        TMC5160_WriteReg(TMC5160_REG_TCOOLTHRS, 2000U, NULL); /* StallGuard active below    */
    }
    else
    {
        TMC5160_WriteReg(TMC5160_REG_TPWMTHRS,  0U, NULL);
        TMC5160_WriteReg(TMC5160_REG_TCOOLTHRS, 0U, NULL);
    }

    /* --- Encoder interface --- */
    if (cfg->encoder_enable)
    {
        /* Basic ABN encoder: enable, no index polarity inversion
         * Caller must configure ENC_CONST separately via stepper_write_reg()
         * once encoder CPR is known */
        TMC5160_WriteReg(TMC5160_REG_ENCMODE, 0x01U, NULL);
    }

    /* --- Ramp generator --- */
    if (cfg->drive_mode == STEPPER_MODE_RAMP)
    {
        TMC5160_WriteReg(TMC5160_REG_RAMPMODE, TMC5160_RAMPMODE_POSITION, NULL);
        TMC5160_WriteReg(TMC5160_REG_XACTUAL,  0U, NULL);
        TMC5160_WriteReg(TMC5160_REG_XTARGET,  0U, NULL);
    }

    /*
     * STEPDIR mode:
     * In this mode the ramp generator is bypassed. The MCU generates
     * STEP pulses and controls DIR. The TMC5160 still uses CHOPCONF/
     * IHOLD_IRUN/COOLCONF. A hardware timer (OC module) is needed on
     * the PIC32MZ to generate accurate step pulses — this will be
     * added as a separate module (stepdir_timer.c) once ramp mode
     * is validated on hardware.
     */

    return true;
}

/* ---------------------------------------------------------------
 * Enable / Disable  (ENN = active LOW)
 * --------------------------------------------------------------- */
void stepper_enable(void)  { EN_Clear(); }
void stepper_disable(void) { EN_Set();   }

/* ---------------------------------------------------------------
 * Ramp configuration
 * --------------------------------------------------------------- */
void stepper_set_ramp(const TMC5160_RampConfig_t *ramp)
{
    if (ramp == NULL) { return; }
    s_ramp = *ramp;

    TMC5160_WriteReg(TMC5160_REG_VSTART, ramp->VSTART, NULL);
    TMC5160_WriteReg(TMC5160_REG_A1,     ramp->A1,     NULL);
    TMC5160_WriteReg(TMC5160_REG_V1,     ramp->V1,     NULL);
    TMC5160_WriteReg(TMC5160_REG_AMAX,   ramp->AMAX,   NULL);
    TMC5160_WriteReg(TMC5160_REG_VMAX,   ramp->VMAX,   NULL);
    TMC5160_WriteReg(TMC5160_REG_DMAX,   ramp->DMAX,   NULL);
    TMC5160_WriteReg(TMC5160_REG_D1,     ramp->D1,     NULL);
    TMC5160_WriteReg(TMC5160_REG_VSTOP,  ramp->VSTOP,  NULL);
}

/* ---------------------------------------------------------------
 * Motion — ramp mode
 * --------------------------------------------------------------- */
void stepper_move_to(int32_t position)
{
    TMC5160_WriteReg(TMC5160_REG_RAMPMODE, TMC5160_RAMPMODE_POSITION, NULL);
    TMC5160_WriteReg(TMC5160_REG_XTARGET,  (uint32_t)position,        NULL);
}

void stepper_move_relative(int32_t delta)
{
    int32_t current = (int32_t)TMC5160_ReadReg(TMC5160_REG_XACTUAL, NULL);
    stepper_move_to(current + delta);
}

void stepper_run(uint32_t vmax, bool reverse)
{
    uint32_t mode = reverse ? TMC5160_RAMPMODE_VEL_NEG : TMC5160_RAMPMODE_VEL_POS;
    TMC5160_WriteReg(TMC5160_REG_VMAX,     vmax, NULL);
    TMC5160_WriteReg(TMC5160_REG_RAMPMODE, mode, NULL);
}

void stepper_stop(void)
{
    /* Writing VMAX=0 triggers controlled decel using DMAX/D1 */
    TMC5160_WriteReg(TMC5160_REG_VMAX, 0U, NULL);
}

void stepper_set_position_zero(void)
{
    TMC5160_WriteReg(TMC5160_REG_XACTUAL, 0U, NULL);
    TMC5160_WriteReg(TMC5160_REG_XTARGET, 0U, NULL);
}

/* ---------------------------------------------------------------
 * Engineering unit wrappers
 * --------------------------------------------------------------- */
void stepper_move_mm(float mm)
{
    stepper_move_relative(stepper_pos_to_usteps(mm));
}

void stepper_run_mmps(float mm_per_sec, bool reverse)
{
    stepper_run(stepper_mmps_to_vmax(mm_per_sec), reverse);
}

/* ---------------------------------------------------------------
 * Status polling — call from main loop
 *
 * Reads XACTUAL, VACTUAL, DRV_STATUS in 3 SPI transactions.
 * Fires stall callback if registered.
 * --------------------------------------------------------------- */
void stepper_poll(TMC5160_MotorStatus_t *status)
{
    TMC5160_Status_t spi_st;
    uint32_t drv;

    s_status.xactual = (int32_t)TMC5160_ReadReg(TMC5160_REG_XACTUAL,   &spi_st);
    s_status.vactual = (int32_t)TMC5160_ReadReg(TMC5160_REG_VACTUAL,   NULL);
    drv              =           TMC5160_ReadReg(TMC5160_REG_DRVSTATUS, NULL);

    s_status.spi_status    = spi_st;
    s_status.sg_result     = (uint16_t)(drv & TMC5160_DRVSTATUS_SG_RESULT_MASK);
    s_status.stalled       = (drv & TMC5160_DRVSTATUS_STALLGUARD) != 0U;
    s_status.standstill    = (drv & TMC5160_DRVSTATUS_STST)       != 0U;
    s_status.overtemp      = (drv & TMC5160_DRVSTATUS_OT)         != 0U;
    s_status.overtemp_warn = (drv & TMC5160_DRVSTATUS_OTPW)       != 0U;
    s_status.open_load_a   = (drv & TMC5160_DRVSTATUS_OLA)        != 0U;
    s_status.open_load_b   = (drv & TMC5160_DRVSTATUS_OLB)        != 0U;
    s_status.pos_reached   = spi_st.pos_reached;
    s_status.vel_reached   = spi_st.vel_reached;

    if (s_status.stalled && (s_stall_cb != NULL))
    {
        s_stall_cb();
    }

    if (status != NULL)
    {
        *status = s_status;
    }
}

bool stepper_is_stalled(void)  { return s_status.stalled;     }
bool stepper_pos_reached(void) { return s_status.pos_reached; }

/* ---------------------------------------------------------------
 * StallGuard configuration
 * threshold: -64 to +63 (lower = more sensitive)
 * filter_en: true = StallGuard filtered (more stable, slower response)
 * --------------------------------------------------------------- */
void stepper_stallguard_config(int8_t threshold, bool filter_en)
{
    uint32_t coolconf = TMC5160_ReadReg(TMC5160_REG_COOLCONF, NULL);

    coolconf &= ~((0x7FUL << TMC5160_COOLCONF_SGT_SHIFT) | TMC5160_COOLCONF_SFILT);
    coolconf |=  ((uint32_t)((int32_t)threshold & 0x7F) << TMC5160_COOLCONF_SGT_SHIFT);

    if (filter_en) { coolconf |= TMC5160_COOLCONF_SFILT; }

    TMC5160_WriteReg(TMC5160_REG_COOLCONF, coolconf, NULL);
}

/* ---------------------------------------------------------------
 * CoolStep configuration
 * semin: lower SG threshold for current increase (1-15, 0=disable)
 * semax: upper SG threshold band width (0-15)
 * --------------------------------------------------------------- */
void stepper_coolstep_config(uint8_t semin, uint8_t semax)
{
    uint32_t coolconf = TMC5160_ReadReg(TMC5160_REG_COOLCONF, NULL);

    coolconf &= ~((0x0FUL << TMC5160_COOLCONF_SEMIN_SHIFT) |
                  (0x0FUL << TMC5160_COOLCONF_SEMAX_SHIFT));

    coolconf |= ((uint32_t)(semin & 0x0FU) << TMC5160_COOLCONF_SEMIN_SHIFT);
    coolconf |= ((uint32_t)(semax & 0x0FU) << TMC5160_COOLCONF_SEMAX_SHIFT);

    TMC5160_WriteReg(TMC5160_REG_COOLCONF, coolconf, NULL);
}

void stepper_set_stall_callback(TMC5160_StallCallback_t cb)
{
    s_stall_cb = cb;
}

/* ---------------------------------------------------------------
 * Sensorless homing via StallGuard
 *
 * Run stepper_stallguard_config() first to set sensitivity.
 * Motor runs at creep_vmax until stall, then stops and zeros.
 * --------------------------------------------------------------- */
void stepper_home(bool reverse, uint32_t creep_vmax)
{
    TMC5160_MotorStatus_t st;

    stepper_run(creep_vmax, reverse);

    do {
        stepper_poll(&st);
    } while (!st.stalled);

    stepper_stop();
    CORETIMER_DelayMs(50U);
    stepper_set_position_zero();
}

/* ---------------------------------------------------------------
 * Raw register access — tuning / debug / encoder setup
 * --------------------------------------------------------------- */
void stepper_write_reg(uint8_t addr, uint32_t data)
{
    TMC5160_WriteReg(addr, data, NULL);
}

uint32_t stepper_read_reg(uint8_t addr)
{
    return TMC5160_ReadReg(addr, NULL);
}