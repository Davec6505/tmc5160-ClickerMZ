/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all moduake helples in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "definitions.h"
#include "stepper.h"
#include "plib_gpio.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

/* ---------------------------------------------------------------
 * BTT TMC5160 PRO on ClickerMZ — test harness
 *
 * UART2 @ 115200 for debug output.
 * Test sequence:
 *   1. Read IOIN (0x04) — should return chip version 0x30
 *   2. Init stepper with ramp mode
 *   3. Set ramp and move 1000 microsteps forward
 *   4. Poll until position reached, print status
 * --------------------------------------------------------------- */

static char uart_buf[128];

static void debug_print(const char *msg)
{
    UART2_Write((uint8_t *)msg, strlen(msg));
}

/* Block until the TX ring buffer has fully drained to the hardware */
static void debug_flush(void)
{
    /* Wait for software ring buffer to drain into HW FIFO */
    while (UART2_WriteCountGet() > 0U) { ; }
    /* Then wait for HW shift register to finish */
    while (!UART2_TransmitComplete()) { ; }
}

static void hal_spi_write_read(const uint8_t *tx, uint8_t *rx, uint8_t len)
    { SPI2_WriteRead((uint8_t *)tx, len, rx, len); }
static bool hal_spi_is_busy(void)   { return SPI2_IsBusy(); }
static void hal_cs_assert(void)     { CS_Clear(); }
static void hal_cs_deassert(void)   { CS_Set(); }
static void hal_en_assert(void)     { EN_Clear(); }
static void hal_en_deassert(void)   { EN_Set(); }
static void hal_step_set(void)      { Step_Set(); }
static void hal_step_clear(void)    { Step_Clear(); }
static void hal_dir_set(void)       { Dir_Set(); }
static void hal_dir_clear(void)     { Dir_Clear(); }
static void hal_delay_us(uint32_t u){ CORETIMER_DelayUs(u); }


//uncomment this if using DRV8825 (no SPI, STEP/DIR only)
//#define STEPPER_USE_DRV8825


#ifndef STEPPER_USE_DRV8825
static const Stepper_HAL_t hal = {
    hal_spi_write_read, hal_spi_is_busy,
    hal_cs_assert, hal_cs_deassert,
    hal_en_assert, hal_en_deassert,
    hal_step_set, hal_step_clear,
    hal_dir_set, hal_dir_clear,
    hal_delay_us
};
#else
static const Stepper_HAL_t hal = {
    NULL, NULL, NULL, NULL,        /* spi_write_read, spi_is_busy, cs_assert, cs_deassert */
    hal_en_assert, hal_en_deassert,
    hal_step_set, hal_step_clear,
    hal_dir_set,  hal_dir_clear,
    hal_delay_us
};
#endif

/* Block until at least one RX byte arrives, discard it */
static void wait_for_keypress(void)
{
    uint8_t dummy;
    while (UART2_ReadCountGet() == 0U) { ; }
    UART2_Read(&dummy, 1U);
}

static void step_pulse(void)
{
    hal.step_set();
    hal.delay_us(100U);
    hal.step_clear();
    hal.delay_us(10U);
}

static void move_steps(int32_t steps)
{
    bool fwd = (steps >= 0);
    uint32_t count = fwd ? (uint32_t)steps : (uint32_t)(-steps);
    if (fwd) { hal.dir_set(); } else { hal.dir_clear(); }
    hal.delay_us(10U);
    for (uint32_t i = 0U; i < count; i++) { step_pulse(); }
}

int main(void)
{
    SYS_Initialize(NULL);
    
    LED1_Clear();
    LED2_Clear();

    debug_print("\r\n--- TMC5160 Test Harness ---\r\n");
    debug_print("Press any key to start...\r\n");
    debug_flush();

    wait_for_keypress();

    /* --- Step 1 & 2: Init stepper (HAL + config), then verify SPI comms --- */
    TMC5160_Config_t cfg = {
        .drive_mode    = STEPPER_MODE_STEPDIR,
        .chopper_mode  = STEPPER_CHOP_SPREADCYCLE,
        .microsteps    = 16U,
        .irun          = 20U,          /* ~65% of 3A = ~1.9A  */
        .ihold         = 8U,           /* ~25% hold current    */
        .iholddelay    = 6U,
        .steps_per_mm  = 80.0f,
        .fclk_hz       = 12000000U,    /* internal oscillator  */
        .rsense_mohm   = 75U,          /* BTT TMC5160 PRO 75mΩ */
        .invert_dir    = false,
        .encoder_enable= false,
    };

    if (stepper_init_axis(0U, &cfg, &hal) == false)
    {
        debug_print("stepper_init_axis failed\r\n");
        while (true) { ; }
    }
    debug_print("stepper_init_axis [OK]\r\n");
    debug_flush();

    /* IOIN: chip version in bits 31:24. TMC5160 = 0x30.
     * 0x00 or 0xFF = SPI wiring fault. */
    uint32_t ioin = stepper_read_reg(0U, 0x04U);
    uint8_t  version = (uint8_t)(ioin >> 24U);

    snprintf(uart_buf, sizeof(uart_buf),
             "IOIN = 0x%08lX  VERSION = 0x%02X %s\r\n",
             (unsigned long)ioin, version,
             (version == 0x30U) ? "[OK]" : "[FAIL - check SPI wiring]");
    debug_print(uart_buf);
    debug_flush();

    if (version != 0x30U)
    {
        debug_print("Halted - fix SPI before continuing.\r\n");
        while (true) { ; }
    }

    /* --- Step 3: Move in Step/Dir mode --- */
    stepper_enable(0U);
    debug_print("Moving 16000 steps forward (~5 revolutions at 200spr/16ustep)...\r\n");
    debug_flush();

    move_steps(16000);

    /* Read DRV_STATUS after move */
    uint32_t drv = stepper_read_reg(0U, 0x6FU);
    snprintf(uart_buf, sizeof(uart_buf),
             "Done. DRV_STATUS=0x%08lX  OLA=%d OLB=%d STST=%d\r\n",
             (unsigned long)drv,
             (int)((drv >> 29) & 1U),
             (int)((drv >> 30) & 1U),
             (int)((drv >> 31) & 1U));
    debug_print(uart_buf);
    debug_flush();

    stepper_disable(0U);
    debug_print("--- Test Complete ---\r\n");

    while (true)
    {
        LED1_Toggle();
        CORETIMER_DelayMs(500U);
    }

    return EXIT_FAILURE;
}



