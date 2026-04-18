# tmc5160-ClickerMZ

Hardware-abstracted stepper motor library for the **TMC5160** targeting the
**MIKROE ClickerMZ (PIC32MZ EF)** board, built with **Microchip XC32 v5**.

The library compiles to a static archive (`libstepper.a`) and has no MCU
headers of its own — all peripherals are wired in through a `Stepper_HAL_t`
function-pointer struct, making it straightforward to reuse on any other MCU.

---

## Repository Layout

```
tmc5160-ClickerMZ/          ← workspace / build root
├── incs/stepper/
│   ├── stepper.h           ← public API
│   ├── stepper_hal.h       ← HAL interface (fill in for your MCU)
│   ├── stepper_transport.h ← TMC5160_Status_t (SPI frame status)
│   └── tmc5160_reg.h       ← chip register map (advanced / debug)
├── srcs/
│   ├── main.c              ← test harness (ClickerMZ-specific)
│   └── stepper/
│       ├── stepper.c
│       └── stepper_transport.c
├── libs/
│   └── libstepper.a        ← built by `make lib`
├── bins/
│   └── tmc5160-ClickerMZ.hex
└── Makefile
```

---

## Prerequisites

| Tool | Version tested |
|------|---------------|
| Microchip XC32 | v5.00 |
| MPLABX DFP (PIC32MZ-EF) | 1.4.168 |
| GNU Make (via XC Project Importer) | — |

Add `xc32/v5.00/bin` to your `PATH` or let the VS Code task / XC Project
Importer extension resolve it automatically.

---

## Building

```bash
# Full rebuild — produces bins/tmc5160-ClickerMZ.hex
make rebuild

# Incremental build
make

# Build archive only — produces libs/libstepper.a
make lib

# Clean build artefacts (objs/, other/)
make clean
```

---

## Using the Library in Another Project

1. Point the compiler at the headers:

```makefile
CFLAGS += -I/path/to/tmc5160-ClickerMZ/tmc5160-ClickerMZ/incs/stepper
```

2. Link against the archive:

```makefile
LDFLAGS += -L/path/to/tmc5160-ClickerMZ/tmc5160-ClickerMZ/libs -lstepper
```

3. Include in your source:

```c
#include "stepper.h"      /* full API          */
#include "stepper_hal.h"  /* Stepper_HAL_t     */
```

> `tmc5160_reg.h` is only needed for direct register-level access. Normal
> applications do not need to include it.

---

## Driver Compatibility

The library supports multiple driver families through the HAL:

| Driver | SPI fields | Step/Dir fields | Notes |
|--------|-----------|-----------------|-------|
| TMC5160 — ramp mode | all four | optional | Internal ramp generator, StallGuard homing |
| TMC5160 — step/dir | all four | all four | MCU generates pulses, SPI used for config + StallGuard |
| DRV8825 / A4988 / TMC2208 | `NULL` | all four | No SPI — homing requires a physical switch |

---

## HAL Wiring

Fill in `Stepper_HAL_t` once per axis, then pass it to `stepper_init_axis()`.
The struct has 11 function pointers:

```c
typedef struct {
    /* SPI — set all four NULL for step/dir-only drivers */
    void  (*spi_write_read)(const uint8_t *tx, uint8_t *rx, uint8_t len);
    bool  (*spi_is_busy)(void);
    void  (*cs_assert)(void);
    void  (*cs_deassert)(void);

    /* Enable pin — required for all drivers */
    void  (*en_assert)(void);    /* ENN low  = driver enabled  */
    void  (*en_deassert)(void);  /* ENN high = driver disabled */

    /* Step / Dir — set all four NULL for pure ramp mode */
    void  (*step_set)(void);
    void  (*step_clear)(void);
    void  (*dir_set)(void);
    void  (*dir_clear)(void);

    /* Timing — required for all drivers */
    void  (*delay_us)(uint32_t us);
} Stepper_HAL_t;
```

**ClickerMZ example** (PIC32MZ Harmony-generated peripherals):

```c
static void hal_spi_wr(const uint8_t *tx, uint8_t *rx, uint8_t len)
    { SPI2_WriteRead((uint8_t *)tx, len, rx, len); }
static bool hal_spi_busy(void)    { return SPI2_IsBusy(); }
static void hal_cs_lo(void)       { CS_Clear(); }
static void hal_cs_hi(void)       { CS_Set(); }
static void hal_en_lo(void)       { EN_Clear(); }
static void hal_en_hi(void)       { EN_Set(); }
static void hal_step_hi(void)     { Step_Set(); }
static void hal_step_lo(void)     { Step_Clear(); }
static void hal_dir_hi(void)      { Dir_Set(); }
static void hal_dir_lo(void)      { Dir_Clear(); }
static void hal_delay(uint32_t u) { CORETIMER_DelayUs(u); }

static const Stepper_HAL_t hal = {
    hal_spi_wr, hal_spi_busy,
    hal_cs_lo,  hal_cs_hi,
    hal_en_lo,  hal_en_hi,
    hal_step_hi, hal_step_lo,
    hal_dir_hi,  hal_dir_lo,
    hal_delay
};
```

---

## API Reference

### Initialisation

```c
/*
 * Initialise one axis (0 … STEPPER_MAX_AXES-1).
 * Returns false if axis index is out of range.
 * Safe to call again to re-configure.
 */
bool stepper_init_axis(uint8_t axis,
                       const TMC5160_Config_t *cfg,
                       const Stepper_HAL_t    *hal);

void stepper_enable(uint8_t axis);   /* Assert ENN — motor holds */
void stepper_disable(uint8_t axis);  /* Release ENN — motor free */
```

`STEPPER_MAX_AXES` defaults to **4**. Override at compile time:
```makefile
CFLAGS += -DSTEPPER_MAX_AXES=2
```

### Motor Configuration

```c
TMC5160_Config_t cfg = {
    .drive_mode     = STEPPER_MODE_STEPDIR,    /* or STEPPER_MODE_RAMP     */
    .chopper_mode   = STEPPER_CHOP_SPREADCYCLE,/* or STEALTHCHOP / AUTO    */
    .microsteps     = 16U,                     /* 1–256                     */
    .irun           = 20U,                     /* 0–31  (31 = 100% Imax)   */
    .ihold          = 8U,
    .iholddelay     = 6U,
    .steps_per_mm   = 80.0f,                   /* full steps / mm           */
    .fclk_hz        = 12000000U,               /* 12 MHz internal osc       */
    .rsense_mohm    = 75U,                     /* BTT PRO = 75 mΩ           */
    .invert_dir     = false,
    .encoder_enable = false,
};
```

### Ramp Configuration (TMC5160 ramp mode)

```c
TMC5160_RampConfig_t ramp = {
    .VSTART = 0U,
    .A1     = 1000U,
    .V1     = 50000U,
    .AMAX   = 5000U,
    .VMAX   = 200000U,
    .DMAX   = 5000U,
    .D1     = 1000U,
    .VSTOP  = 10U,    /* must be >= 10 */
};
stepper_set_ramp(0U, &ramp);
```

### Motion

```c
/* Absolute / relative moves (microsteps) */
stepper_move_to(axis, position);
stepper_move_relative(axis, delta);

/* Velocity mode */
stepper_run(axis, vmax, reverse);
stepper_stop(axis);

/* Engineering-unit helpers (requires steps_per_mm) */
stepper_move_mm(axis, mm);
stepper_run_mmps(axis, mm_per_sec, reverse);
uint32_t v = stepper_mmps_to_vmax(axis, mm_per_sec);

/* Reset XACTUAL to zero */
stepper_set_position_zero(axis);
```

### Status Polling

```c
TMC5160_MotorStatus_t st;
stepper_poll(axis, &st);   /* updates st from SPI registers */

st.xactual;     /* current position (microsteps) */
st.vactual;     /* current velocity (raw)        */
st.sg_result;   /* StallGuard result  0–1023      */
st.stalled;     /* stall detected                */
st.standstill;
st.pos_reached;
st.overtemp;
```

### StallGuard & Homing

```c
/* Configure threshold (-64 … +63; lower = more sensitive) */
stepper_stallguard_config(axis, threshold, filter_en);

/* Optional: CoolStep automatic current reduction */
stepper_coolstep_config(axis, semin, semax);

/* Sensorless homing — blocking */
stepper_home(axis, reverse, creep_vmax, NULL);

/* Sensorless homing — non-blocking; fires cb from stepper_poll() */
void on_homed(uint8_t axis) { /* axis is now zeroed */ }
stepper_home(axis, reverse, creep_vmax, on_homed);
/* pump from main loop: */
while (!done) { stepper_poll(axis, &st); }
```

---

## SPI Verification

Read `IOIN` (register `0x04`) immediately after init. Bits 31:24 contain the
chip version — `0x30` for TMC5160. Any other value indicates a wiring fault.

```c
uint32_t ioin    = stepper_read_reg(0U, 0x04U);
uint8_t  version = (uint8_t)(ioin >> 24U);
/* version == 0x30 → SPI OK */
```

---

## Target Hardware

- **MCU**: PIC32MZ1024EFH064 on MIKROE ClickerMZ
- **Driver**: BTT TMC5160 PRO (Rsense = 75 mΩ)
- **SPI**: SPI2 (Harmony plib)
- **UART**: UART2 @ 115200 8N1 for debug output
- **Bootloader**: MIKROE USB-UART bootloader (`USE_MIKROE_BOOTLOADER`)
