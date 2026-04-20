# tmc5160-ClickerMZ

Hardware-abstracted stepper motor library for the **TMC5160/DRV8825** targeting the
**MIKROE ClickerMZ (PIC32MZ EF)** board, built with **Microchip XC32 v5**.

The library compiles to a static archive (`libstepper.a`) and has no MCU
headers of its own — all peripherals are wired in through a `Stepper_HAL_t`
function-pointer struct, making it straightforward to reuse on any other MCU.

For the full API reference, configuration guide, and feature documentation see
**[USING_TMC5160_LIB.md](USING_TMC5160_LIB.md)**.

---

## Repository Layout

```
tmc5160-ClickerMZ/          ← workspace / build root
├── incs/stepper/
│   ├── stepper.h           ← public API
│   ├── stepper_hal.h       ← HAL interface (fill in for your MCU)
│   ├── stepper_transport.h ← TMC5160_Status_t (SPI frame status)
│   └── stepper_reg.h       ← chip register map (advanced / debug)
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

## Target Hardware

| Item | Detail |
|------|--------|
| MCU | PIC32MZ1024EFH064 @ 200 MHz |
| Board | MikroE Clicker 2 for PIC32MZ |
| Driver | BigTreeTech TMC5160 PRO (Click module) |
| Rsense | 75 mΩ (BTT PRO — not the standard 110 mΩ) |
| TMC5160 clock | 12 MHz internal oscillator |
| SPI | SPI2 (Harmony plib) |
| UART | UART2 @ 115200 8N1 for debug output |
| Bootloader | MIKROE USB-UART bootloader (`USE_MIKROE_BOOTLOADER`) |

### Pin Mapping (MikroBUS socket 1)

| Signal | PIC32MZ pin | Direction | Notes |
|--------|-------------|-----------|-------|
| SPI2 SCK | RG6 | Out | Dedicated SPI pin |
| SPI2 MOSI (SDO2) | RG8 | Out | PPS RPG8R=6 |
| SPI2 MISO (SDI2) | RG7 | In | PPS SDI2R=1 |
| CS (soft) | RG9 | Out | Idle HIGH; asserted LOW for full 5-byte frame |
| UART2 TX | RB2 | Out | PPS RPB2R=2 |
| UART2 RX | RB0 | In | PPS U2RXR=5 |
| EN | RE4 | Out | HIGH = driver disabled (safe default) |
| DIR | RB5 | Out | Direction |
| STEP | RB3 | Out | Step pulse (STEPDIR mode only) |
| DIAG0 | RB12 | In | StallGuard / fault output from TMC5160 |
| DIAG1 | RB13 | In | Second diagnostic output |

> **MSSEN is disabled.** Hardware SS would deassert between bytes, breaking the
> required 5-byte atomic SPI frame. CS is driven manually on RG9.

---

## Prerequisites

| Tool | Version tested |
|------|---------------|
| MPLAB X IDE | v6.25 |
| Microchip XC32 | v5.00 |
| PIC32MZ-EF DFP | 1.4.168 |
| VS Code extension | `davidcoetzee.xc-project-importer` 2.5.51 |
| GNU Make | via XC Project Importer |

Add `xc32/v5.00/bin` to your `PATH` or let the VS Code task / XC Project
Importer extension resolve it automatically.

Build paths are set for the above versions. Override on the command line:

```powershell
make COMPILER_LOCATION="C:/Program Files/Microchip/xc32/v5.00/bin" `
     DFP="C:/Program Files/Microchip/MPLABX/v6.25/packs/Microchip/PIC32MZ-EF_DFP/1.4.168"
```

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

### VS Code Tasks (`Ctrl+Shift+B`)

| Task | Command |
|------|---------|
| Build XC32 Project | `make` |
| Clean Build Artifacts | `make clean` |
| Flash Device | `make flash` |

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

> `stepper_reg.h` is only needed for direct register-level access. Normal
> applications do not need to include it.

---

## Getting Started

### 1. Build

```powershell
cd tmc5160-ClickerMZ
make
# Output: bins/tmc5160-ClickerMZ.hex
```

### 2. Flash

Flash `bins/tmc5160-ClickerMZ.hex` via the MikroE USB bootloader or a PICkit.

### 3. Verify over UART

Connect a USB-UART adapter to **RB2 (TX)** and **RB0 (RX)**, open a terminal at **115200 8N1**.

Expected output on first boot:

```
--- TMC5160 Test Start ---
IOIN = 0x30xxxxxx  VERSION = 0x30 [OK]
stepper_init [OK]
Move started to XTARGET=1000
Done. pos=1000  sg=xxx  stall=0  ot=0  [pos_reached]
```

If `VERSION` reads `0x00` or `0xFF`, SPI wiring is wrong — scope SCK / MOSI / CS (RG9).

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

## Library Feature Summary

| Feature | Detail |
|---------|--------|
| **Ramp mode** | TMC5160 internal trapezoidal / S-curve generator (RAMPMODE=0) |
| **Velocity mode** | Constant velocity (RAMPMODE=1/2) |
| **Step/Dir mode** | MCU-generated STEP pulses, DIR pin for direction |
| **Chopper modes** | StealthChop, SpreadCycle, AUTO, DCStep, AUTO+DCStep |
| **StallGuard** | Configurable SGT threshold; stall callback function pointer |
| **CoolStep** | Full SEMIN/SEMAX/SEUP/SEDN/SEIMIN control |
| **Homing** | Sensorless blocking or non-blocking via StallGuard |
| **Hardware stops** | Full SW_MODE register — latching, polarity, soft stop |
| **Encoder** | ABN encoder interface enable + ENC_CONST scaling |
| **DCStep** | VDCMIN + THIGH configuration for loss-free high-speed |
| **Linear units — mm** | `stepper_move_mm()`, `stepper_move_to_mm()`, `stepper_run_mmps()` |
| **Linear units — inch** | `stepper_move_inch()`, `stepper_move_to_inch()`, `stepper_run_ips()` |
| **Rotation axis** | `stepper_move_deg()`, `stepper_move_to_deg()`, `stepper_run_dps()`, `stepper_get_position_deg()` |
| **Position getters/setters** | `stepper_get_position_mm/inch/deg()`, `stepper_set_position_mm/deg()` |
| **Software position counter** | `stepper_step_tick()` keeps position accurate in step/dir mode |
| **Runtime current control** | `stepper_set_irun()` / `stepper_set_ihold()` — no re-init required |
| **Emergency stop** | `stepper_stop_all()` — stops every initialised axis at once |
| **Configurable TPOWERDOWN** | `cfg.tpowerdown` — coil hold-current delay, 0 = chip default |
| **Multi-axis** | Up to `STEPPER_MAX_AXES` (default 4) independent axes |

For the complete API, configuration field reference, and usage examples see
**[USING_TMC5160_LIB.md](USING_TMC5160_LIB.md)**.

---

## Roadmap

- [ ] Hardware verification — flash and confirm IOIN=0x30
- [ ] StallGuard / SGT calibration on the bench
- [ ] CoolStep SEMIN/SEMAX tuning
- [ ] STEPDIR mode via PIC32MZ Output Compare (OC4=RB3)
- [ ] DIAG0/DIAG1 interrupt wiring to EVIC
- [ ] Multi-axis coordinated moves
