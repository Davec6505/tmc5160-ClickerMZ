# tmc5160-ClickerMZ

TMC5160 stepper motor driver library for the **MikroE Clicker 2 for PIC32MZ** board with a **BigTreeTech TMC5160 PRO** click module.

Targets the **PIC32MZ1024EFH064** at 200 MHz.  The library uses the TMC5160 internal ramp generator over SPI2, with StallGuard2 stall detection, CoolStep load-adaptive current control, and SpreadCycle / StealthChop chopper selection.  A UART2 debug channel (115200 baud) is wired to the ClickerMZ USB-UART bridge.

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | PIC32MZ1024EFH064 @ 200 MHz |
| Board | MikroE Clicker 2 for PIC32MZ |
| Driver | BigTreeTech TMC5160 PRO (Click module) |
| Rsense | 75 mΩ (BTT PRO — not the standard 110 mΩ) |
| TMC5160 clock | 12 MHz internal oscillator |

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

> **MSSEN is disabled.** Hardware SS would deassert between bytes, breaking the required 5-byte atomic SPI frame.  CS is driven manually on RG9.

---

## Software Architecture

```
srcs/
├── main.c                          # Test harness / application entry
├── stepper/
│   ├── tmc5160_spi.c               # SPI transport — 5-byte burst, read double-tx
│   └── stepper.c                   # Motion library (ramp, velocity, homing, StallGuard)
└── config/default/                 # MCC-generated peripheral drivers (XC32 / Harmony Lite)
    ├── peripheral/clk/             # plib_clk — oscillator & PLL
    ├── peripheral/coretimer/       # plib_coretimer — CORETIMER_DelayUs / _DelayMs
    ├── peripheral/evic/            # plib_evic — interrupt controller
    ├── peripheral/gpio/            # plib_gpio — pin directions, PPS
    ├── peripheral/spi/spi_master/  # plib_spi2_master — polling SPI
    └── peripheral/uart/            # plib_uart2 — ring-buffer UART

incs/
├── stepper/
│   ├── tmc5160_reg.h               # Full TMC5160 register address & bit-field map
│   ├── tmc5160_spi.h               # SPI layer public API
│   └── stepper.h                   # Motion library public API
└── config/default/                 # MCC-generated headers
```

### SPI Transport (`tmc5160_spi.c`)

- All frames are **5-bytes wide** (40-bit): `[addr] [D31] [D23] [D15] [D7]`.
- CS is asserted for the entire burst, not per-byte.
- **Reads require two transactions**: first frame latches the address into the TMC5160 output register; second frame clocks out the data.
- Status byte decoded on every transaction into `TMC5160_Status_t` (reset, driver error, SG2, standstill, vel_reached, pos_reached, stop_L/R).

### Motion Library (`stepper.c`)

| Feature | Detail |
|---------|--------|
| **Ramp mode** | TMC5160 internal trapezoidal / S-curve generator (RAMPMODE=0) |
| **Velocity mode** | TMC5160 constant velocity (RAMPMODE=1/2) |
| **Step/Dir mode** | MCU-generated step pulses via Output Compare — placeholder in source |
| **Chopper** | SpreadCycle, StealthChop, or AUTO (chip switches on TPWMTHRS / TCOOLTHRS) |
| **StallGuard** | Configurable SGT threshold; stall callback function pointer |
| **CoolStep** | SEMIN/SEMAX/SEUP/SEDN configured via `stepper_coolstep_config()` |
| **Homing** | `stepper_home()` — runs at low velocity until stall, then zeros XACTUAL |
| **Engineering units** | `stepper_move_mm()`, `stepper_run_mmps()` using `steps_per_mm` |

VSENSE bit is set automatically from `rsense_mohm`:
- `rsense_mohm < 110` → `VSENSE=0` (full-scale 0.325 V, suits BTT PRO 75 mΩ)
- `rsense_mohm >= 110` → `VSENSE=1` (half-scale 0.180 V, suits standard 110 mΩ boards)

---

## Build Requirements

| Tool | Version |
|------|---------|
| MPLAB X IDE | v6.25 |
| XC32 compiler | v5.00 |
| PIC32MZ-EF DFP | 1.4.168 |
| VS Code extension | `davidcoetzee.xc-project-importer` 2.5.51 |

Build paths are hardcoded in `Makefile` for the above installation.  Override on the command line:

```powershell
make COMPILER_LOCATION="C:/Program Files/Microchip/xc32/v5.00/bin" `
     DFP="C:/Program Files/Microchip/MPLABX/v6.25/packs/Microchip/PIC32MZ-EF_DFP/1.4.168"
```

### VS Code Tasks (`Ctrl+Shift+B`)

| Task | Command |
|------|---------|
| Build XC32 Project | `make` |
| Clean Build Artifacts | `make clean` |
| Flash Device | `make flash` |

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

## Configuration Quick Reference

```c
TMC5160_Config_t cfg = {
    .drive_mode    = STEPPER_MODE_RAMP,
    .chopper_mode  = STEPPER_CHOP_AUTO,
    .microsteps    = 16U,
    .irun          = 20U,       /* 0-31; ~1.9 A for BTT PRO 75 mΩ  */
    .ihold         = 8U,
    .iholddelay    = 6U,
    .steps_per_mm  = 80.0f,     /* tune to your mechanics           */
    .fclk_hz       = 12000000U, /* 12 MHz internal clock            */
    .rsense_mohm   = 75U,       /* 75 mΩ for BTT TMC5160 PRO        */
    .invert_dir    = false,
    .encoder_enable= false,
};
stepper_init(&cfg);
```

### S-curve ramp

```c
TMC5160_RampConfig_t ramp = {
    .VSTART = 0,
    .A1     = 1000,   /* acceleration phase 1           */
    .V1     = 10000,  /* velocity threshold              */
    .AMAX   = 5000,   /* acceleration phase 2            */
    .VMAX   = 50000,  /* peak velocity                   */
    .DMAX   = 5000,
    .D1     = 1000,
    .VSTOP  = 10,     /* must be >= 10                   */
};
stepper_set_ramp(&ramp);
stepper_enable();
stepper_move_to(1000);          /* absolute microsteps */
stepper_move_mm(12.5f);         /* or in millimetres   */
```

### Polling

```c
TMC5160_MotorStatus_t status;
stepper_poll(&status);
if (status.pos_reached)  { /* done */ }
if (status.stalled)      { /* handle stall */ }
if (status.overtemp)     { /* shutdown */ }
```

---

## Roadmap

- [ ] Hardware verification — flash and confirm IOIN=0x30
- [ ] StallGuard / SGT calibration on the bench
- [ ] CoolStep SEMIN/SEMAX tuning
- [ ] STEPDIR mode via PIC32MZ Output Compare (OC4=RB3)
- [ ] HAL abstraction layer (replace `definitions.h` dependency for portability)
- [ ] Extract stepper library to standalone repo
