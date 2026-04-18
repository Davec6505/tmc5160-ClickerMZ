#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "stepper_transport.h"  /* TMC5160_Status_t                    */
#include "stepper_hal.h"        /* Stepper_HAL_t                       */

/* ============================================================
 * Maximum axes supported by this library instance.
 * Override at compile time: -DSTEPPER_MAX_AXES=2
 * ============================================================ */
#ifndef STEPPER_MAX_AXES
#define STEPPER_MAX_AXES  4U
#endif

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
    STEPPER_CHOP_STEALTHCHOP  = 0,  /* Silent sine-wave, low speed               */
    STEPPER_CHOP_SPREADCYCLE  = 1,  /* High-accuracy chopper, high speed         */
    STEPPER_CHOP_AUTO         = 2,  /* Chip auto-switches on TPWMTHRS/TCOOLTHRS  */
} TMC5160_ChopperMode_t;

/* ============================================================
 * Ramp configuration struct
 *
 * Maps directly to TMC5160 ramp registers.
 * All values are in raw register units (microsteps and internal
 * time base). Use stepper_mmps_to_vmax() to convert from mm/s.
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
 * Passed once to stepper_init_axis()
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
    uint16_t              rsense_mohm;     /* Sense resistor mΩ — 75 BTT PRO, 110 standard */
    bool                  invert_dir;      /* Board-level direction invert (GCONF.SHAFT)   */
    bool                  encoder_enable;  /* Enable TMC5160 ABN encoder interface         */
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
 * Callbacks
 * ============================================================ */

/* Stall callback — registered per axis; called from stepper_poll()
   when StallGuard threshold is exceeded.  Pass NULL to disable.    */
typedef void (*TMC5160_StallCallback_t)(void);

/* Home-complete callback — passed to stepper_home().
   axis: the axis index that completed homing.
   NULL → blocking homing mode.  Non-NULL → non-blocking; caller must
   pump stepper_poll() from the main loop.                           */
typedef void (*TMC5160_HomeCallback_t)(uint8_t axis);

/* ============================================================
 * Initialisation & control
 *
 * stepper_init_axis() is the primary entry point.
 * Each axis is independent; up to STEPPER_MAX_AXES may be active.
 * ============================================================ */
bool stepper_init_axis(uint8_t axis, const TMC5160_Config_t *cfg, const Stepper_HAL_t *hal);
void stepper_enable(uint8_t axis);
void stepper_disable(uint8_t axis);

/* ============================================================
 * Ramp configuration  (TMC5160 ramp mode only)
 * ============================================================ */
void stepper_set_ramp(uint8_t axis, const TMC5160_RampConfig_t *ramp);

/* ============================================================
 * Motion — RAMP mode  (TMC5160 internal ramp generator)
 * ============================================================ */
void stepper_move_to(uint8_t axis, int32_t position);        /* absolute microsteps */
void stepper_move_relative(uint8_t axis, int32_t delta);     /* relative microsteps */
void stepper_run(uint8_t axis, uint32_t vmax, bool reverse); /* velocity mode       */
void stepper_stop(uint8_t axis);                             /* decel to stop       */
void stepper_set_position_zero(uint8_t axis);                /* zero XACTUAL        */

/* ============================================================
 * Motion — engineering units (requires steps_per_mm set)
 * ============================================================ */
void     stepper_move_mm(uint8_t axis, float mm);
void     stepper_run_mmps(uint8_t axis, float mm_per_sec, bool reverse);
int32_t  stepper_pos_to_usteps(uint8_t axis, float mm);
float    stepper_usteps_to_mm(uint8_t axis, int32_t usteps);
uint32_t stepper_mmps_to_vmax(uint8_t axis, float mm_per_sec);

/* ============================================================
 * Status & polling  (TMC5160 SPI only — no-op for step/dir drivers)
 * ============================================================ */
void stepper_poll(uint8_t axis, TMC5160_MotorStatus_t *status);
bool stepper_is_stalled(uint8_t axis);
bool stepper_pos_reached(uint8_t axis);

/* ============================================================
 * StallGuard / CoolStep  (TMC5160 SPI only)
 * ============================================================ */
void stepper_stallguard_config(uint8_t axis, int8_t threshold, bool filter_en);
void stepper_coolstep_config(uint8_t axis, uint8_t semin, uint8_t semax);
void stepper_set_stall_callback(uint8_t axis, TMC5160_StallCallback_t cb);

/* ============================================================
 * Homing — sensorless via StallGuard  (TMC5160 SPI only)
 *
 * Run stepper_stallguard_config() first to set sensitivity.
 * on_complete = NULL  → blocking: returns when stall detected + zeroed.
 * on_complete = fn    → non-blocking: starts motion and returns; fn is
 *                       called from stepper_poll() when homing completes.
 * ============================================================ */
void stepper_home(uint8_t axis, bool reverse, uint32_t creep_vmax,
                  TMC5160_HomeCallback_t on_complete);

/* ============================================================
 * Raw register access (advanced / debug)
 * ============================================================ */
void     stepper_write_reg(uint8_t axis, uint8_t addr, uint32_t data);
uint32_t stepper_read_reg(uint8_t axis, uint8_t addr);

#endif /* STEPPER_H */