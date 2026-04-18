#ifndef STEPPER_HAL_H
#define STEPPER_HAL_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Stepper_HAL_t — hardware abstraction interface
 *
 * The consumer fills this struct with concrete function pointers that map to
 * their MCU's SPI, GPIO, and timer peripherals then passes it to
 * stepper_init_axis().  The library never includes any MCU-specific headers.
 *
 * SPI fields (first 4): set NULL for step/dir-only drivers (DRV8825, A4988,
 *   TMC2208 in UART mode, etc.).  The entire SPI configuration block in
 *   stepper_init_axis() is skipped when spi_write_read == NULL.
 *
 * Step/Dir fields: set NULL when using the TMC5160 internal ramp generator
 *   (STEPPER_MODE_RAMP).  StallGuard-based homing requires SPI.
 *
 * Minimum required fields for each driver class:
 *   TMC5160 ramp mode  — all SPI fields + en + delay
 *   TMC5160 step/dir   — all SPI fields + en + step + dir + delay
 *   DRV8825 / A4988    — en + step + dir + delay  (SPI = NULL)
 */
typedef struct
{
    /* SPI — required for TMC5160 register access; NULL for step/dir-only */
    void     (*spi_write_read)(const uint8_t *tx, uint8_t *rx, uint8_t len);
    bool     (*spi_is_busy)(void);
    void     (*cs_assert)(void);    /* CS low  */
    void     (*cs_deassert)(void);  /* CS high */

    /* Enable pin — required for all drivers */
    void     (*en_assert)(void);    /* ENN low  = driver enabled  */
    void     (*en_deassert)(void);  /* ENN high = driver disabled */

    /* Step/Dir — required for STEPPER_MODE_STEPDIR; NULL for ramp mode */
    void     (*step_set)(void);
    void     (*step_clear)(void);
    void     (*dir_set)(void);
    void     (*dir_clear)(void);

    /* Delay — required for all drivers */
    void     (*delay_us)(uint32_t us);
} Stepper_HAL_t;

#endif /* STEPPER_HAL_H */
