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

/* ============================================================
 * Engineering unit system
 * ============================================================ */
typedef enum {
    STEPPER_UNITS_MM   = 0,  /* millimetres (default) */
    STEPPER_UNITS_INCH = 1,  /* inches                */
} TMC5160_Units_t;

/* How motion commands are executed */
typedef enum {
    STEPPER_MODE_RAMP    = 0,  /* TMC5160 internal ramp generator (SPI only) */
    STEPPER_MODE_STEPDIR = 1,  /* MCU generates STEP pulses, DIR pin for dir  */
} TMC5160_DriveMode_t;

/* Chopper / current control mode */
typedef enum {
    STEPPER_CHOP_STEALTHCHOP  = 0,  /* StealthChop2 only — silent PWM, low speed                   */
    STEPPER_CHOP_SPREADCYCLE  = 1,  /* SpreadCycle only — high-accuracy constant off-time           */
    STEPPER_CHOP_AUTO         = 2,  /* StealthChop → SpreadCycle auto on TPWMTHRS / TCOOLTHRS       */
    STEPPER_CHOP_DCSTEP       = 3,  /* SpreadCycle + DCStep loss-free above THIGH / VDCMIN          */
    STEPPER_CHOP_AUTO_DCSTEP  = 4,  /* Full range: StealthChop → SpreadCycle → DCStep               */
} TMC5160_ChopperMode_t;

/* Coil state at standstill (PWMCONF.FREEWHEEL) */
typedef enum {
    STEPPER_FREEWHEEL_NORMAL    = 0,  /* Coils energised — full hold torque    */
    STEPPER_FREEWHEEL_FREEWHEEL = 1,  /* Open circuit — motor spins freely     */
    STEPPER_FREEWHEEL_LS_SHORT  = 2,  /* Passive brake via low-side switches   */
    STEPPER_FREEWHEEL_HS_SHORT  = 3,  /* Passive brake via high-side switches  */
} TMC5160_Freewheel_t;

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
    uint32_t              enc_const;       /* ENC_CONST scaling factor (0 = skip write)    */

    /* Rotation axis — set non-zero for angular axes (nozzle, rotary table) */
    float                 steps_per_deg;   /* full steps per degree; 0 = linear axis only  */

    /* Unit system for public API — does not affect internal ustep calculations */
    TMC5160_Units_t       units;           /* STEPPER_UNITS_MM (default) or INCH           */

    /* Standstill power-down delay (TPOWERDOWN).                                */
    /* Each count = 2^18 / fclk_hz seconds. 10 ≈ 218 ms at 12 MHz.             */
    uint8_t               tpowerdown;      /* 0 = use default (10); max 255                */

    /* Standstill freewheel / braking (PWMCONF.FREEWHEEL) */
    TMC5160_Freewheel_t   freewheel;       /* coil state at standstill; default = NORMAL   */

    /* High-velocity / loss-free motion */
    uint32_t              dcstep_vmin;     /* VDCMIN: 0=off; DCStep kicks in above this    */
    uint32_t              thigh;           /* THIGH:  0=off; enables fullstep + DCStep     */

    /* Zero-crossing wait between direction changes (ramp mode only) */
    uint16_t              tzerowait;       /* TZEROWAIT tCLK cycles at V=0; 0 = disabled   */

    /* Additional DIAG pin routing (GCONF) */
    bool                  diag0_otpw;      /* DIAG0 = overtemperature pre-warning          */
    bool                  diag1_index;     /* DIAG1 = microstep index pulse                */
    bool                  diag1_onstate;   /* DIAG1 = on-state (active step pulse)         */
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
 * Hardware stop switch configuration
 *
 * Zero-initialise to disable all stops (all fields default to false).
 * Pass to stepper_sw_mode_config() — maps field-for-field to SW_MODE.
 * ============================================================ */
typedef struct {
    bool stop_l_enable;     /* enable left  stop switch                     */
    bool stop_r_enable;     /* enable right stop switch                     */
    bool pol_stop_l;        /* polarity: true = active-high                 */
    bool pol_stop_r;        /* polarity: true = active-high                 */
    bool swap_lr;           /* swap left/right switch pin assignment        */
    bool latch_l_active;    /* latch XACTUAL on active   edge — left        */
    bool latch_l_inactive;  /* latch XACTUAL on inactive edge — left        */
    bool latch_r_active;    /* latch XACTUAL on active   edge — right       */
    bool latch_r_inactive;  /* latch XACTUAL on inactive edge — right       */
    bool en_latch_encoder;  /* also latch encoder position when stop fires  */
    bool sg_stop;           /* stop on StallGuard event (SWMODE.SG_STOP)   */
    bool en_softstop;       /* soft stop (ramp to 0) vs hard stop           */
} TMC5160_SwMode_t;

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
void stepper_stop_all(void);           /* emergency stop — all initialised axes */

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
 * All functions respect the axis unit system (mm or inch).
 * ============================================================ */
void     stepper_move_mm(uint8_t axis, float mm);             /* relative in mm           */
void     stepper_move_to_mm(uint8_t axis, float mm);          /* absolute in mm           */
void     stepper_run_mmps(uint8_t axis, float mm_per_sec, bool reverse);
void     stepper_move_inch(uint8_t axis, float inch);         /* relative in inches       */
void     stepper_move_to_inch(uint8_t axis, float inch);      /* absolute in inches       */
void     stepper_run_ips(uint8_t axis, float inch_per_sec, bool reverse);
float    stepper_get_position_mm(uint8_t axis);               /* current position in mm   */
float    stepper_get_position_inch(uint8_t axis);             /* current position in inch */
void     stepper_set_position_mm(uint8_t axis, float mm);     /* set known offset, not 0  */

/* ============================================================
 * Rotation axis  (requires steps_per_deg > 0 in config)
 * ============================================================ */
void     stepper_move_deg(uint8_t axis, float deg);           /* relative in degrees      */
void     stepper_move_to_deg(uint8_t axis, float deg);        /* absolute in degrees      */
void     stepper_run_dps(uint8_t axis, float deg_per_sec, bool reverse);
float    stepper_get_position_deg(uint8_t axis);              /* current position in deg  */
void     stepper_set_position_deg(uint8_t axis, float deg);   /* set known angular offset */
uint32_t stepper_dps_to_vmax(uint8_t axis, float deg_per_sec);

/* ============================================================
 * Unit conversions (internal — also useful for callers)
 * ============================================================ */
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
 * Step/Dir software position counter
 *
 * Call stepper_step_tick() from your step ISR or after every
 * pulse in move_steps() so the position counter stays accurate
 * in STEPPER_MODE_STEPDIR.  stepper_get_position_mm/deg() and
 * stepper_poll() all use this counter in step/dir mode.
 * ============================================================ */
void stepper_step_tick(uint8_t axis, bool forward);

/* ============================================================
 * Runtime current control
 * ============================================================ */
void stepper_set_irun(uint8_t axis, uint8_t irun);    /* 0-31, updates IHOLD_IRUN immediately */
void stepper_set_ihold(uint8_t axis, uint8_t ihold);  /* 0-31, updates IHOLD_IRUN immediately */

/* ============================================================
 * StallGuard / CoolStep  (TMC5160 SPI only)
 * ============================================================ */
void stepper_stallguard_config(uint8_t axis, int8_t threshold, bool filter_en);
void stepper_coolstep_config(uint8_t axis,
                             uint8_t semin,   /* 1-15 lower SG bound; 0 = disable  */
                             uint8_t semax,   /* 0-15 upper SG band width          */
                             uint8_t seup,    /* 0-3  current increment step       */
                             uint8_t sedn,    /* 0-3  current decrement speed      */
                             bool    seimin); /* false=½ IRUN floor, true=¼ IRUN   */
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

/* ============================================================
 * Hardware stop switches  (SW_MODE register — ramp mode only)
 *
 * stepper_sw_mode_config() — write all fields of SW_MODE at once.
 * stepper_get_xlatch()     — read XLATCH: position latched on stop event.
 * stepper_get_ramp_stat()  — read RAMP_STAT: stop/latch/event status bits.
 * ============================================================ */
void     stepper_sw_mode_config(uint8_t axis, const TMC5160_SwMode_t *sw);
int32_t  stepper_get_xlatch(uint8_t axis);
uint32_t stepper_get_ramp_stat(uint8_t axis);

/* ============================================================
 * Runtime velocity band tuning
 *
 * tpwmthrs  — StealthChop → SpreadCycle upper threshold
 * tcoolthrs — StallGuard / CoolStep enable threshold
 * thigh     — DCStep / fullstep enable threshold (0 = disabled)
 * ============================================================ */
void stepper_set_velocity_bands(uint8_t axis,
                                uint32_t tpwmthrs,
                                uint32_t tcoolthrs,
                                uint32_t thigh);

/* ============================================================
 * Position compare output (X_COMPARE — fires pulse on DIAG1)
 * ============================================================ */
void stepper_set_position_compare(uint8_t axis, int32_t pos);

#endif /* STEPPER_H */