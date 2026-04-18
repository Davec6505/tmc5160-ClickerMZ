#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "tmc5160_spi.h"

/* ============================================================
 * Enumerations
 * ============================================================ */

/* How motion commands are executed */
typedef enum {
    STEPPER_MODE_RAMP    = 0,  /* TMC5160 internal ramp generator (SPI only) */
    STEPPER_MODE_STEPDIR = 1,  /* MCU generates STEP pulses, DIR pin for dir  */
} TMC5160_DriveMode_t;

/* Chopper / current control mode */
typedef enum {
    STEPPER_CHOP_STEALTHCHOP  = 0,  /* Silent sine-wave, low speed            */
    STEPPER_CHOP_SPREADCYCLE  = 1,  /* High-accuracy chopper, high speed      */
    STEPPER_CHOP_AUTO         = 2,  /* Chip auto-switches on TPWMTHRS/TCOOLTHRS */
} TMC5160_ChopperMode_t;

/* ============================================================
 * Ramp configuration struct
 *
 * Maps directly to TMC5160 ramp registers.
 * All values are in raw register units (microsteps and internal
 * time base). Use stepper_calc_vmax() / stepper_calc_amax() to
 * convert from mm/s and mm/s² if needed.
 *
 * Two-stage S-curve:  set V1 > 0, A1/D1 != AMAX/DMAX
 * Simple trapezoid:   set V1 = 0  (A1, D1 ignored by chip)
 * ============================================================ */
typedef struct {
    uint32_t VSTART;  /* Start velocity before ramp (min 0)           */
    uint32_t A1;      /* First  acceleration (to V1)                   */
    uint32_t V1;      /* Threshold between A1 and AMAX (0 = disable)   */
    uint32_t AMAX;    /* Maximum acceleration (above V1)               */
    uint32_t VMAX;    /* Target velocity                               */
    uint32_t DMAX;    /* Maximum deceleration (above V1 in decel)      */
    uint32_t D1;      /* Final deceleration (below V1 into stop)       */
    uint32_t VSTOP;   /* Stop velocity — MUST be >= 10                 */
} TMC5160_RampConfig_t;

/* ============================================================
 * Motor configuration struct
 * Passed once to stepper_init()
 * ============================================================ */
typedef struct {
    TMC5160_DriveMode_t   drive_mode;
    TMC5160_ChopperMode_t chopper_mode;
    uint16_t              microsteps;      /* 1, 2, 4, 8, 16, 32, 64, 128, 256 */
    uint8_t               irun;            /* Run current  0-31 (31 = 100%)     */
    uint8_t               ihold;           /* Hold current 0-31                 */
    uint8_t               iholddelay;      /* Delay before hold current 0-15    */
    float                 steps_per_mm;    /* Full steps per mm (mechanics)     */
    uint32_t              fclk_hz;         /* TMC5160 clock — 12000000 internal */
    uint16_t              rsense_mohm;     /* Sense resistor mΩ — 75 for BTT PRO, 110 standard */
    bool                  invert_dir;      /* Board-level direction invert (GCONF.SHAFT)        */
    bool                  encoder_enable;  /* Enable TMC5160 ABN encoder interface              */
} TMC5160_Config_t;

/* ============================================================
 * Live motor status — updated by stepper_poll()
 * ============================================================ */
typedef struct {
    int32_t  xactual;        /* Current position in microsteps        */
    int32_t  vactual;        /* Current velocity (raw)                */
    uint16_t sg_result;      /* StallGuard result 0-1023 (0 = stall)  */
    bool     stalled;        /* StallGuard threshold exceeded         */
    bool     standstill;     /* Motor at standstill                   */
    bool     pos_reached;    /* XTARGET reached                       */
    bool     vel_reached;    /* VMAX reached                          */
    bool     overtemp;       /* Overtemperature shutdown              */
    bool     overtemp_warn;  /* Overtemperature warning               */
    bool     open_load_a;    /* Open load on coil A                   */
    bool     open_load_b;    /* Open load on coil B                   */
    TMC5160_Status_t spi_status; /* Last raw SPI status byte          */
} TMC5160_MotorStatus_t;

/* ============================================================
 * Stall callback — set NULL for polling only
 * Called from stepper_poll() when stall is detected
 * ============================================================ */
typedef void (*TMC5160_StallCallback_t)(void);

/* ============================================================
 * Initialisation & control
 * ============================================================ */
bool    stepper_init(const TMC5160_Config_t *cfg);
void    stepper_enable(void);
void    stepper_disable(void);

/* ============================================================
 * Ramp configuration
 * ============================================================ */
void    stepper_set_ramp(const TMC5160_RampConfig_t *ramp);

/* ============================================================
 * Motion — RAMP mode
 * ============================================================ */
void    stepper_move_to(int32_t position);          /* absolute microsteps   */
void    stepper_move_relative(int32_t delta);       /* relative microsteps   */
void    stepper_run(uint32_t vmax, bool reverse);   /* velocity mode         */
void    stepper_stop(void);                         /* decel to stop         */
void    stepper_set_position_zero(void);            /* zero XACTUAL          */

/* ============================================================
 * Motion — engineering units (requires steps_per_mm set)
 * ============================================================ */
void    stepper_move_mm(float mm);
void    stepper_run_mmps(float mm_per_sec, bool reverse);
int32_t stepper_pos_to_usteps(float mm);
float   stepper_usteps_to_mm(int32_t usteps);
uint32_t stepper_mmps_to_vmax(float mm_per_sec);

/* ============================================================
 * Status & polling
 * ============================================================ */
void    stepper_poll(TMC5160_MotorStatus_t *status);
bool    stepper_is_stalled(void);
bool    stepper_pos_reached(void);

/* ============================================================
 * StallGuard / CoolStep
 * ============================================================ */
void    stepper_stallguard_config(int8_t threshold, bool filter_en);
void    stepper_coolstep_config(uint8_t semin, uint8_t semax);
void    stepper_set_stall_callback(TMC5160_StallCallback_t cb);

/* ============================================================
 * Homing (sensorless via StallGuard)
 * ============================================================ */
void    stepper_home(bool reverse, uint32_t creep_vmax);

/* ============================================================
 * Raw register access (advanced / debug)
 * ============================================================ */
void     stepper_write_reg(uint8_t addr, uint32_t data);
uint32_t stepper_read_reg(uint8_t addr);

#endif /* STEPPER_H */