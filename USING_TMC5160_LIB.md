# Using the TMC5160 Stepper Library

Full reference for every feature exposed by `libstepper.a`.  
For build instructions and hardware setup see [README.md](README.md).

---

## Contents

- [Using the TMC5160 Stepper Library](#using-the-tmc5160-stepper-library)
  - [Contents](#contents)
  - [1. Quick-Start](#1-quick-start)
  - [2. HAL Wiring](#2-hal-wiring)
  - [3. Motor Configuration](#3-motor-configuration)
    - [Drive Mode](#drive-mode)
    - [Chopper Mode](#chopper-mode)
    - [Current Settings](#current-settings)
    - [Standstill Freewheel](#standstill-freewheel)
    - [DCStep / High-Velocity](#dcstep--high-velocity)
    - [Direction-Change Wait (TZEROWAIT)](#direction-change-wait-tzerowait)
    - [Encoder Interface](#encoder-interface)
    - [DIAG Pin Routing](#diag-pin-routing)
    - [Unit System](#unit-system)
    - [Rotation Axis Config](#rotation-axis-config)
    - [Standstill Power-Down](#standstill-power-down)
  - [4. Ramp Configuration](#4-ramp-configuration)
  - [5. Initialisation API](#5-initialisation-api)
  - [6. Motion API](#6-motion-api)
  - [7. Linear Unit Helpers — mm and inch](#7-linear-unit-helpers--mm-and-inch)
  - [8. Rotation Axis API](#8-rotation-axis-api)
    - [Example — 360° rotation loop](#example--360-rotation-loop)
  - [9. Step/Dir Position Counter](#9-stepdir-position-counter)
    - [Integration pattern](#integration-pattern)
    - [Reading position in step/dir mode](#reading-position-in-stepdir-mode)
    - [Zeroing](#zeroing)
  - [10. Status Polling](#10-status-polling)
  - [11. Runtime Current Control](#11-runtime-current-control)
  - [12. StallGuard](#12-stallguard)
  - [13. CoolStep](#13-coolstep)
    - [Field guide](#field-guide)
    - [Prerequisites](#prerequisites)
    - [Typical settings](#typical-settings)
  - [14. Sensorless Homing](#14-sensorless-homing)
  - [15. Hardware Stop Switches](#15-hardware-stop-switches)
  - [16. Velocity Band Tuning](#16-velocity-band-tuning)
  - [17. Position Compare Output](#17-position-compare-output)
  - [18. Raw Register Access](#18-raw-register-access)
  - [19. SPI Verification](#19-spi-verification)
  - [20. Multi-Axis](#20-multi-axis)
  - [21. Common Pitfalls](#21-common-pitfalls)

---

## 1. Quick-Start

Minimum code to get a TMC5160 axis moving in ramp mode:

```c
#include "stepper.h"
#include "stepper_hal.h"

/* 1. Wire HAL to your MCU peripherals */
static void hal_spi_wr(const uint8_t *tx, uint8_t *rx, uint8_t len)
    { SPI2_WriteRead((uint8_t *)tx, len, rx, len); }
static bool hal_spi_busy(void)    { return SPI2_IsBusy(); }
static void hal_cs_lo(void)       { CS_Clear(); }
static void hal_cs_hi(void)       { CS_Set(); }
static void hal_en_lo(void)       { EN_Clear(); }
static void hal_en_hi(void)       { EN_Set(); }
static void hal_delay(uint32_t u) { CORETIMER_DelayUs(u); }

static const Stepper_HAL_t hal = {
    hal_spi_wr, hal_spi_busy,
    hal_cs_lo,  hal_cs_hi,
    hal_en_lo,  hal_en_hi,
    NULL, NULL, NULL, NULL,   /* no step/dir pins needed in ramp mode */
    hal_delay
};

/* 2. Configure */
static const TMC5160_Config_t cfg = {
    .drive_mode   = STEPPER_MODE_RAMP,
    .chopper_mode = STEPPER_CHOP_AUTO,
    .microsteps   = 16U,
    .irun         = 20U,
    .ihold        = 8U,
    .iholddelay   = 6U,
    .steps_per_mm = 80.0f,
    .fclk_hz      = 12000000U,
    .rsense_mohm  = 75U,
};

static const TMC5160_RampConfig_t ramp = {
    .VSTART = 0U,
    .A1     = 1000U,
    .V1     = 50000U,
    .AMAX   = 5000U,
    .VMAX   = 200000U,
    .DMAX   = 5000U,
    .D1     = 1000U,
    .VSTOP  = 10U,
};

/* 3. Init, ramp, enable, move */
stepper_init_axis(0, &cfg, &hal);
stepper_set_ramp(0, &ramp);
stepper_enable(0);
stepper_move_mm(0, 50.0f);   /* move 50 mm */
```

---

## 2. HAL Wiring

Fill in `Stepper_HAL_t` once per axis. Fields not needed for your driver
class may be set to `NULL`.

```c
typedef struct {
    void  (*spi_write_read)(const uint8_t *tx, uint8_t *rx, uint8_t len);
    bool  (*spi_is_busy)(void);
    void  (*cs_assert)(void);    /* CS low  */
    void  (*cs_deassert)(void);  /* CS high */

    void  (*en_assert)(void);    /* ENN low  = driver enabled  */
    void  (*en_deassert)(void);  /* ENN high = driver disabled */

    void  (*step_set)(void);
    void  (*step_clear)(void);
    void  (*dir_set)(void);
    void  (*dir_clear)(void);

    void  (*delay_us)(uint32_t us);
} Stepper_HAL_t;
```

| Driver class | SPI fields | Step/Dir fields |
|---|---|---|
| TMC5160 ramp mode | required | `NULL` |
| TMC5160 step/dir | required | required |
| DRV8825 / A4988 / TMC2208 | `NULL` | required |

> When `spi_write_read == NULL` the entire SPI register-init block inside
> `stepper_init_axis()` is skipped. All SPI-only API functions become no-ops.

---

## 3. Motor Configuration

All fields are passed once to `stepper_init_axis()` inside `TMC5160_Config_t`.

```c
TMC5160_Config_t cfg = {
    /* --- required for all drivers --- */
    .drive_mode   = STEPPER_MODE_RAMP,
    .microsteps   = 16U,
    .irun         = 20U,
    .ihold        = 8U,
    .iholddelay   = 6U,
    .steps_per_mm = 80.0f,

    /* --- TMC5160 SPI fields --- */
    .chopper_mode = STEPPER_CHOP_AUTO,
    .fclk_hz      = 12000000U,
    .rsense_mohm  = 75U,
    .invert_dir   = false,

    /* --- optional / advanced (safe to zero-init) --- */
    .encoder_enable = false,
    .enc_const      = 0U,
    .freewheel      = STEPPER_FREEWHEEL_NORMAL,
    .dcstep_vmin    = 0U,
    .thigh          = 0U,
    .tzerowait      = 0U,
    .diag0_otpw     = false,
    .diag1_index    = false,
    .diag1_onstate  = false,

    /* --- unit system --- */
    .units          = STEPPER_UNITS_MM,   /* or STEPPER_UNITS_INCH */

    /* --- rotation axis (set non-zero for angular axes) --- */
    .steps_per_deg  = 0.0f,              /* e.g. 200*16/360 = 8.889 for 200spr/16ustep */

    /* --- standstill power-down delay (0 = default ~218 ms at 12 MHz) --- */
    .tpowerdown     = 0U,
};
```

---

### Drive Mode

| Value | Description |
|---|---|
| `STEPPER_MODE_RAMP` | TMC5160 internal ramp generator. Motion via `stepper_move_to()` / `stepper_run()`. Requires SPI. |
| `STEPPER_MODE_STEPDIR` | MCU generates STEP pulses, DIR pin sets direction. SPI still used for config + StallGuard. |

---

### Chopper Mode

Controls how the chip drives current through the motor coils. Also
determines which velocity-switching thresholds (`TPWMTHRS`, `TCOOLTHRS`)
are active.

| Value | Register behaviour | Best for |
|---|---|---|
| `STEPPER_CHOP_STEALTHCHOP` | StealthChop2 always on (`EN_PWM_MODE=1`). Silent sinusoidal current. | Very quiet low-speed applications |
| `STEPPER_CHOP_SPREADCYCLE` | SpreadCycle always on (`EN_PWM_MODE=0`). Constant off-time chopper. | High-speed, high-torque, less noise-sensitive |
| `STEPPER_CHOP_AUTO` | StealthChop below `TPWMTHRS`, SpreadCycle above it. Default thresholds written at init (`TPWMTHRS=500`, `TCOOLTHRS=2000`). | General purpose — best starting point |
| `STEPPER_CHOP_DCSTEP` | SpreadCycle + DCStep loss-free stepping above `THIGH` / `VDCMIN`. | Very high velocity, must set `thigh` and `dcstep_vmin`. |
| `STEPPER_CHOP_AUTO_DCSTEP` | StealthChop → SpreadCycle → DCStep across full velocity range. | Full-range applications needing quiet low-speed and fast high-speed. |

> For `DCSTEP` and `AUTO_DCSTEP` you **must** also set `cfg.thigh` and
> `cfg.dcstep_vmin` to non-zero values, or DCStep will never engage.

---

### Current Settings

| Field | Range | Notes |
|---|---|---|
| `irun` | 0–31 | Run current. 31 = 100% of Imax set by Rsense. |
| `ihold` | 0–31 | Standstill hold current. Typically 30–50% of `irun`. |
| `iholddelay` | 0–15 | Delay steps before transitioning from `irun` to `ihold` after standstill detected. Each step is ~2^18 clock cycles. |
| `rsense_mohm` | — | Sense resistor value in mΩ. BTT PRO = 75, standard = 110. Sets `VSENSE` bit in CHOPCONF. |

---

### Standstill Freewheel

Controls what happens to the motor coils when the motor reaches standstill
(`PWMCONF.FREEWHEEL`).

| Value | Coil state |
|---|---|
| `STEPPER_FREEWHEEL_NORMAL` | Coils remain energised — full hold torque (default) |
| `STEPPER_FREEWHEEL_FREEWHEEL` | Open circuit — motor spins freely, no hold |
| `STEPPER_FREEWHEEL_LS_SHORT` | Passive braking via low-side switches |
| `STEPPER_FREEWHEEL_HS_SHORT` | Passive braking via high-side switches |

---

### DCStep / High-Velocity

Two config fields enable the TMC5160 high-velocity features:

```c
.dcstep_vmin = 4000000U,  /* VDCMIN: DCStep kicks in above this velocity   */
.thigh       = 30000U,    /* THIGH:  enables fullstep + DCStep above this  */
```

Both are raw register values (internal velocity units). Leave at `0` to
disable. These fields are written regardless of chopper mode, so they can
also be used to enable fullstep-at-high-speed without DCStep by setting
`thigh` only.

Use `stepper_set_velocity_bands()` to adjust them at runtime without
re-initialising.

---

### Direction-Change Wait (TZEROWAIT)

In ramp mode, `tzerowait` inserts a pause at V=0 between direction changes,
allowing coil currents to fully decay before reversing. Prevents resonance
artefacts on fast direction reversals.

```c
.tzerowait = 200U,   /* pause = 200 × 512 tCLK cycles ≈ 8.5 ms at 12 MHz */
```

Set to `0` to disable (default). Only effective in `STEPPER_MODE_RAMP`.

---

### Encoder Interface

Enables the TMC5160 ABN (quadrature) encoder input.

```c
.encoder_enable = true,
.enc_const      = 65536U,  /* ENC_CONST scaling: usteps per encoder step × 2^16 */
```

`enc_const = 0` skips the `ENC_CONST` register write (useful if you set it
via `stepper_write_reg()` manually).

---

### DIAG Pin Routing

Three `GCONF` routing options. Only one `DIAG1` source is active at a time
(priority: `diag1_index` > `diag1_onstate` > default stall output).

```c
.diag0_otpw    = true,   /* DIAG0 = overtemperature pre-warning pulse  */
.diag1_index   = true,   /* DIAG1 = microstep index pulse              */
.diag1_onstate = false,  /* DIAG1 = step pulse active indicator        */
```

> `DIAG0_ERROR` is always routed to DIAG0 regardless of `diag0_otpw`.

---

### Unit System

Set once in `TMC5160_Config_t`. All `_mm` and `_inch` API calls respect it.
Internal calculations always use microsteps — units only affect the API boundary.

```c
.units = STEPPER_UNITS_MM,    /* default */
.units = STEPPER_UNITS_INCH,  /* for imperial machinery */
```

`stepper_get_position_mm()` returns the value in the configured unit, so if
you set `STEPPER_UNITS_INCH` the result is already in inches even though the
function is named `_mm`. Use `stepper_get_position_inch()` for an
unambiguous imperial call regardless of the `units` setting.

---

### Rotation Axis Config

Set `steps_per_deg` to a non-zero value to enable the rotation API for that
axis. The linear mm/inch API remains available on the same axis.

```c
/*
 * Example: 200 full-step motor, 16 microsteps, 1:1 direct drive
 * 200 steps × 16 ustep = 3200 usteps per revolution
 * 3200 / 360 = 8.889 usteps per degree
 */
.steps_per_deg = 8.889f,

/*
 * Example: stepper with 10:1 gearbox
 * 200 × 16 × 10 / 360 = 88.89 usteps per degree
 */
.steps_per_deg = 88.889f,
```

---

### Standstill Power-Down

Delay before the driver transitions from `irun` to `ihold` after standstill
is detected. Each count = $2^{18}$ / `fclk_hz` seconds.

```c
.tpowerdown = 0U,    /* 0 = default (10 = ~218 ms at 12 MHz) */
.tpowerdown = 40U,   /* ~870 ms — useful for Z axis holding load */
.tpowerdown = 255U,  /* ~5.5 s maximum */
```

## 4. Ramp Configuration

Applies only to `STEPPER_MODE_RAMP`. Maps directly to TMC5160 ramp registers.

```c
TMC5160_RampConfig_t ramp = {
    .VSTART = 0U,       /* velocity before first acceleration phase — 0 is fine  */
    .A1     = 1000U,    /* first  acceleration (VSTART → V1)                     */
    .V1     = 50000U,   /* switch point between A1 and AMAX (0 = single-stage)   */
    .AMAX   = 5000U,    /* main acceleration (V1 → VMAX)                         */
    .VMAX   = 200000U,  /* target / cruise velocity                              */
    .DMAX   = 5000U,    /* main deceleration (VMAX → V1)                         */
    .D1     = 1000U,    /* final deceleration (V1 → VSTOP)                       */
    .VSTOP  = 10U,      /* stop velocity — must be >= 10 or chip will not stop   */
};
stepper_set_ramp(0U, &ramp);
```

**Two-stage S-curve**: set `V1 > 0` with `A1 ≠ AMAX` / `D1 ≠ DMAX`.  
**Simple trapezoid**: set `V1 = 0` — `A1` and `D1` are ignored by the chip.

All values are in raw TMC5160 internal units. Use `stepper_mmps_to_vmax()` to
convert mm/s to a `VMAX` register value.

---

## 5. Initialisation API

```c
bool stepper_init_axis(uint8_t axis,
                       const TMC5160_Config_t *cfg,
                       const Stepper_HAL_t    *hal);

void stepper_enable(uint8_t axis);    /* Assert ENN — motor energised  */
void stepper_disable(uint8_t axis);   /* Release ENN — motor free      */
void stepper_stop_all(void);          /* Emergency stop all axes        */
```

`stepper_stop_all()` calls `stepper_stop()` on every initialised axis in one
call — use on faults, limit events, or e-stop signals.

`stepper_disable()` is called automatically during `stepper_init_axis()` so
registers can be written safely before the driver is enabled.

`STEPPER_MAX_AXES` defaults to **4**. Override at compile time:
```makefile
CFLAGS += -DSTEPPER_MAX_AXES=2
```

---

## 6. Motion API

All motion functions require a successful `stepper_init_axis()` call first.
SPI-based functions (`move_to`, `run`, etc.) are no-ops on step/dir-only axes.

```c
/* --- absolute / relative position (microsteps) --- */
void stepper_move_to(uint8_t axis, int32_t position);
void stepper_move_relative(uint8_t axis, int32_t delta);

/* --- velocity mode --- */
void stepper_run(uint8_t axis, uint32_t vmax, bool reverse);
void stepper_stop(uint8_t axis);   /* decelerates using DMAX → VSTOP */

/* --- zero position --- */
void stepper_set_position_zero(uint8_t axis);   /* sets XACTUAL = XTARGET = 0 */
```

---

## 7. Linear Unit Helpers — mm and inch

Require `steps_per_mm` to be set in `TMC5160_Config_t`.

```c
/* Relative moves */
void stepper_move_mm(uint8_t axis, float mm);
void stepper_move_inch(uint8_t axis, float inch);

/* Absolute moves */
void stepper_move_to_mm(uint8_t axis, float mm);
void stepper_move_to_inch(uint8_t axis, float inch);

/* Velocity mode */
void stepper_run_mmps(uint8_t axis, float mm_per_sec, bool reverse);
void stepper_run_ips(uint8_t axis, float inch_per_sec, bool reverse);

/* Position getters — always in mm (or inch if units=INCH) */
float stepper_get_position_mm(uint8_t axis);
float stepper_get_position_inch(uint8_t axis);

/* Position setters — sync software counter + XACTUAL */
void stepper_set_position_mm(uint8_t axis, float mm);

/* Raw conversions */
uint32_t stepper_mmps_to_vmax(uint8_t axis, float mm_per_sec);
int32_t  stepper_pos_to_usteps(uint8_t axis, float mm);
float    stepper_usteps_to_mm(uint8_t axis, int32_t usteps);
```

Conversion formula used internally:

$$V_{reg} = v_{mm/s} \times steps\_per\_mm \times microsteps \times \frac{2^{23}}{f_{CLK}}$$

---

## 8. Rotation Axis API

Enabled when `cfg.steps_per_deg > 0`. The rotation API and linear API coexist
on the same axis — use whichever is appropriate.

```c
/* Relative moves */
void stepper_move_deg(uint8_t axis, float deg);

/* Absolute moves */
void stepper_move_to_deg(uint8_t axis, float deg);

/* Velocity mode */
void stepper_run_dps(uint8_t axis, float deg_per_sec, bool reverse);

/* Position getter */
float stepper_get_position_deg(uint8_t axis);

/* Position setter — sync software counter + XACTUAL */
void stepper_set_position_deg(uint8_t axis, float deg);

/* Raw conversion */
uint32_t stepper_dps_to_vmax(uint8_t axis, float deg_per_sec);
```

### Example — 360° rotation loop

```c
static const TMC5160_Config_t cfg_rot = {
    .drive_mode    = STEPPER_MODE_RAMP,
    .microsteps    = 16U,
    .irun          = 20U,
    .ihold         = 8U,
    .iholddelay    = 6U,
    .steps_per_mm  = 0.0f,     /* unused for pure rotation axis */
    .steps_per_deg = 8.889f,   /* 200 × 16 / 360               */
    /* ... other fields ... */
};

stepper_init_axis(0U, &cfg_rot, &hal);
stepper_set_ramp(0U, &ramp);
stepper_enable(0U);

stepper_move_to_deg(0U, 90.0f);    /* absolute 90°  */
stepper_move_deg(0U, -45.0f);      /* relative -45° */
stepper_run_dps(0U, 60.0f, false); /* 60 °/s forward */
```

---

## 9. Step/Dir Position Counter

In `STEPPER_MODE_STEPDIR`, the TMC5160 `XACTUAL` register only moves when
the chip generates steps internally (DCStep). MCU-generated pulses are
invisible to it. The library maintains a software counter `s_pos_usteps`
that you must drive by calling `stepper_step_tick()` once per pulse.

```c
/* Call after EVERY step pulse — in your ISR or in move_steps() */
void stepper_step_tick(uint8_t axis, bool forward);
```

### Integration pattern

```c
static void step_pulse(uint8_t axis, bool forward)
{
    if (forward) { hal.dir_set(); } else { hal.dir_clear(); }
    hal.step_set();
    hal.delay_us(5U);
    hal.step_clear();
    hal.delay_us(5U);
    stepper_step_tick(axis, forward);   /* keep counter in sync */
}
```

### Reading position in step/dir mode

```c
float pos_mm  = stepper_get_position_mm(axis);   /* uses software counter */
float pos_deg = stepper_get_position_deg(axis);  /* uses software counter */
```

`stepper_poll()` also writes `st.xactual` from the software counter in
step/dir mode, so status-based code does not need to be changed.

### Zeroing

```c
stepper_set_position_zero(axis);  /* resets software counter + XACTUAL */
stepper_set_position_mm(axis, 0.0f);  /* equivalent, via mm API */
```

---

## 10. Status Polling

Call `stepper_poll()` regularly from your main loop. It performs three SPI
reads (XACTUAL, VACTUAL, DRV_STATUS) and updates the internal status cache.
It also fires stall and home callbacks.

```c
TMC5160_MotorStatus_t st;
stepper_poll(axis, &st);

st.xactual;       /* current position in microsteps                    */
st.vactual;       /* current velocity (raw internal units)             */
st.sg_result;     /* StallGuard result 0–1023 (lower = higher load)    */
st.stalled;       /* StallGuard threshold exceeded                     */
st.standstill;    /* motor at standstill                               */
st.pos_reached;   /* XTARGET reached (from SPI status byte)            */
st.vel_reached;   /* VMAX reached                                      */
st.overtemp;      /* overtemperature shutdown (OT)                     */
st.overtemp_warn; /* overtemperature pre-warning (OTPW)                */
st.open_load_a;   /* open load detected on coil A                     */
st.open_load_b;   /* open load detected on coil B                     */
st.spi_status;    /* raw TMC5160_Status_t from last SPI transaction    */
```

Lightweight non-blocking helpers (use cached state — no SPI):

```c
bool stalled     = stepper_is_stalled(axis);
bool at_target   = stepper_pos_reached(axis);
```

In `STEPPER_MODE_STEPDIR`, `st.xactual` is filled from the software counter
(not an SPI read), so it is always valid even without an SPI link.

---

## 11. Runtime Current Control

Adjust run or hold current without re-initialising the axis. Useful for
torque boost during press-fit operations or reduced current while waiting
for a vision system.

```c
void stepper_set_irun(uint8_t axis, uint8_t irun);    /* 0-31 */
void stepper_set_ihold(uint8_t axis, uint8_t ihold);  /* 0-31 */
```

Both write `IHOLD_IRUN` immediately and update the cached config so a
subsequent `stepper_init_axis()` call does not revert the change.

```c
/* Example: boost current for a press-fit, then restore */
stepper_set_irun(0U, 28U);        /* near-max current */
stepper_move_to_mm(0U, target);   /* press move       */
while (!stepper_pos_reached(0U)) { stepper_poll(0U, NULL); }
stepper_set_irun(0U, 20U);        /* back to normal   */
```

---

## 12. StallGuard

StallGuard measures motor load by monitoring back-EMF. The result
(`sg_result`) ranges 0–1023: **lower = higher load / closer to stall**.

```c
/*
 * threshold : -64 to +63
 *   Lower value = more sensitive (fires sooner).
 *   Positive values reduce sensitivity for high-speed applications.
 * filter_en : true = 4-sample filtered result (more stable, slower)
 */
void stepper_stallguard_config(uint8_t axis, int8_t threshold, bool filter_en);
```

StallGuard is only active when motor speed is **below** `TCOOLTHRS`. This
threshold is written automatically when chopper mode is `AUTO` or
`AUTO_DCSTEP`, or can be set manually via `stepper_set_velocity_bands()`.

Register a callback to react to stall events asynchronously:

```c
void on_stall(void) { stepper_stop(axis); }
stepper_set_stall_callback(0U, on_stall);   /* called from stepper_poll() */
stepper_set_stall_callback(0U, NULL);       /* disable */
```

---

## 13. CoolStep

CoolStep uses the StallGuard measurement as a feedback signal to
automatically reduce run current when the motor is lightly loaded,
saving power and reducing heat without sacrificing torque.

```c
void stepper_coolstep_config(uint8_t axis,
                             uint8_t semin,   /* 1-15 lower SG bound; 0 = disable  */
                             uint8_t semax,   /* 0-15 upper SG band width          */
                             uint8_t seup,    /* 0-3  current increment step       */
                             uint8_t sedn,    /* 0-3  current decrement speed      */
                             bool    seimin); /* false=½ IRUN floor, true=¼ IRUN   */
```

### Field guide

| Field | Range | Effect |
|---|---|---|
| `semin` | 1–15 | SG floor. When `sg_result ≤ semin × 32`, current increases. `0` = CoolStep disabled. |
| `semax` | 0–15 | Window width. When `sg_result > (semin + semax + 1) × 32`, current decreases. |
| `seup` | 0–3 | Current increment per SG sample: 0=+1, 1=+2, 2=+4, 3=+8 (in 1/32 IRUN steps). Higher = faster response to load. |
| `sedn` | 0–3 | Current decrement speed: 0=every 32 samples (slow), 1=8, 2=2, 3=every sample (fast). Slow is safer. |
| `seimin` | bool | Minimum current floor. `false` = ½ IRUN, `true` = ¼ IRUN. Use `false` unless maximum power saving is needed. |

### Prerequisites

- SPI mode only
- Call `stepper_stallguard_config()` first so SG produces meaningful values
- `TCOOLTHRS` must be non-zero (set by `AUTO`/`AUTO_DCSTEP` modes or manually
  via `stepper_set_velocity_bands()`)

### Typical settings

```c
/* Conservative starting point */
stepper_coolstep_config(0U,
    3,      /* semin  */
    6,      /* semax  */
    0,      /* seup  — gentle increment */
    0,      /* sedn  — slow decrement */
    false); /* seimin — ½ IRUN floor */
```

---

## 14. Sensorless Homing

Uses StallGuard to detect the end-stop by stall, then zeros the position.
Requires SPI mode. Call `stepper_stallguard_config()` first.

```c
/* Blocking — returns only after stall detected and position zeroed */
stepper_home(axis, reverse, creep_vmax, NULL);

/* Non-blocking — starts motion, returns immediately.
   on_homed(axis) is called from stepper_poll() on stall. */
void on_homed(uint8_t axis) { /* axis position is now zero */ }
stepper_home(axis, reverse, creep_vmax, on_homed);

/* Pump from main loop: */
while (!done) { stepper_poll(axis, &st); }
```

Both modes stop the motor and call `stepper_set_position_zero()` when the
stall is detected. A 50 ms debounce delay (via `hal->delay_us`) is applied
before zeroing.

---

## 15. Hardware Stop Switches

Maps to the TMC5160 `SW_MODE` register. Ramp mode only. Zero-initialise
`TMC5160_SwMode_t` to start with all stops disabled.

```c
TMC5160_SwMode_t sw = { 0 };   /* all disabled */

sw.stop_l_enable    = true;    /* enable left  stop switch               */
sw.stop_r_enable    = true;    /* enable right stop switch               */
sw.pol_stop_l       = false;   /* false = active-low (default)           */
sw.pol_stop_r       = false;
sw.swap_lr          = false;   /* swap physical left/right pin mapping   */
sw.latch_l_active   = true;    /* latch XACTUAL on active   edge — left  */
sw.latch_l_inactive = false;   /* latch XACTUAL on inactive edge — left  */
sw.latch_r_active   = true;
sw.latch_r_inactive = false;
sw.en_latch_encoder = false;   /* also latch encoder position on stop    */
sw.sg_stop          = false;   /* treat stall as a stop event            */
sw.en_softstop      = true;    /* soft stop (ramp to 0) vs hard cut-off  */

stepper_sw_mode_config(axis, &sw);

/* Read back results after a stop event */
int32_t  pos       = stepper_get_xlatch(axis);     /* latched position       */
uint32_t ramp_stat = stepper_get_ramp_stat(axis);  /* RAMP_STAT status bits  */
```

---

## 16. Velocity Band Tuning

Adjusts the three TSTEP-based velocity thresholds at runtime without
re-initialising the axis. All three registers use **inverse velocity**
units (larger value = lower speed threshold).

```c
/*
 * tpwmthrs  — TSTEP threshold: StealthChop → SpreadCycle crossover
 * tcoolthrs — TSTEP threshold: enables StallGuard / CoolStep
 * thigh     — TSTEP threshold: enables fullstep / DCStep (0 = disabled)
 */
void stepper_set_velocity_bands(uint8_t axis,
                                uint32_t tpwmthrs,
                                uint32_t tcoolthrs,
                                uint32_t thigh);
```

These correspond directly to TMC5160 registers `TPWMTHRS`, `TCOOLTHRS`,
and `THIGH`. Typical starting values used by `STEPPER_CHOP_AUTO` at init:
`tpwmthrs = 500`, `tcoolthrs = 2000`, `thigh = 0`.

---

## 17. Position Compare Output

Fires a single pulse on the `DIAG1` pin when `XACTUAL` equals the target
position. Useful for triggering external hardware (camera, sensor, etc.)
at a precise position.

```c
stepper_set_position_compare(axis, target_usteps);
```

**Requires** `.diag1_index = true` in `TMC5160_Config_t` (routes the
position-compare signal to DIAG1 via GCONF).

---

## 18. Raw Register Access

For advanced configuration, debugging, or features not yet wrapped by
the library.

```c
stepper_write_reg(axis, addr, value);
uint32_t val = stepper_read_reg(axis, addr);
```

Register addresses are defined in `stepper_reg.h`. Example — read chip
version from `IOIN`:

```c
uint32_t ioin    = stepper_read_reg(0U, 0x04U);
uint8_t  version = (uint8_t)(ioin >> 24U);
/* version == 0x30 → TMC5160 SPI OK */
```

---

## 19. SPI Verification

Always verify the SPI link after `stepper_init_axis()` before enabling the motor:

```c
uint32_t ioin    = stepper_read_reg(0U, TMC5160_REG_IOIN);
uint8_t  version = (uint8_t)(ioin >> 24U);
if (version != 0x30U)
{
    /* wiring fault — CS, MOSI, MISO, SCK or VSPI supply issue */
}
```

Bits 31:24 of `IOIN` contain the chip version. `0x30` = TMC5160.

---

## 20. Multi-Axis

Up to `STEPPER_MAX_AXES` (default 4) independent axes. Each axis has its own
HAL, config, ramp, status, and callback state. Axes are completely independent.

```c
stepper_init_axis(0U, &cfg_x, &hal_x);
stepper_init_axis(1U, &cfg_y, &hal_y);
stepper_init_axis(2U, &cfg_z, &hal_z);

stepper_enable(0U);
stepper_enable(1U);
stepper_enable(2U);

/* Kick off three simultaneous moves */
stepper_move_mm(0U,  50.0f);
stepper_move_mm(1U, 100.0f);
stepper_move_mm(2U,  25.0f);

/* Poll all axes from main loop */
TMC5160_MotorStatus_t st;
while (true)
{
    stepper_poll(0U, &st);
    stepper_poll(1U, &st);
    stepper_poll(2U, &st);
}
```

Override the axis count at compile time:
```makefile
CFLAGS += -DSTEPPER_MAX_AXES=3
```

---

## 21. Common Pitfalls

**`VSTOP` must be ≥ 10**  
The TMC5160 will never reach a complete stop if `VSTOP < 10`. The chip
ignores position-reached if `VSTOP` is too small.

**StallGuard only works below `TCOOLTHRS`**  
If `TCOOLTHRS` is 0, StallGuard, CoolStep, and sensorless homing will
not function. Set it via `chopper_mode = AUTO`/`AUTO_DCSTEP` at init,
or call `stepper_set_velocity_bands()` manually.

**DCStep requires both `thigh` and `dcstep_vmin`**  
Setting only one of these is not enough for DCStep to engage. Both must
be non-zero and `THIGH > TCOOLTHRS` in the velocity ordering.

**CoolStep needs a tuned StallGuard threshold first**  
If `sg_result` is always near 0 or always near 1023, CoolStep will
either never reduce current or never increase it. Run `stallguard_config()`
and verify `sg_result` is dynamic before enabling CoolStep.

**`stepper_move_mm()` is relative, not absolute**  
It calls `stepper_move_relative()` internally. Use `stepper_move_to_mm()`
for absolute position moves.

**Step/dir position counter requires `stepper_step_tick()` calls**
`stepper_get_position_mm()` and `stepper_poll()` return the software
counter in step/dir mode. If you don't call `stepper_step_tick()` after
each pulse the position will always read 0. The counter is also not
aupdated by the chip — it is purely software.

**Rotation axis functions return 0 if `steps_per_deg == 0`**
All `_deg` functions are no-ops or return 0 if `steps_per_deg` is not
set in the config. There is no error — double-check your config struct.
