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
#include "definitions.h"
#include "stepper.h"


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

int main(void)
{
    SYS_Initialize(NULL);

    debug_print("\r\n--- TMC5160 Test Start ---\r\n");

    /* --- Step 1: SPI comms check --- */
    /* IOIN register returns chip version in bits 31:24.
     * TMC5160 = 0x30, TMC5160A = 0x30
     * If you read 0x00 or 0xFF the SPI wiring is wrong. */
    uint32_t ioin = stepper_read_reg(0x04U);
    uint8_t  version = (uint8_t)(ioin >> 24U);

    snprintf(uart_buf, sizeof(uart_buf),
             "IOIN = 0x%08lX  VERSION = 0x%02X %s\r\n",
             (unsigned long)ioin, version,
             (version == 0x30U) ? "[OK]" : "[FAIL - check SPI wiring]");
    debug_print(uart_buf);

    if (version != 0x30U)
    {
        debug_print("Halted — fix SPI before continuing.\r\n");
        while (true) { ; }
    }

    /* --- Step 2: Initialise stepper --- */
    TMC5160_Config_t cfg = {
        .drive_mode    = STEPPER_MODE_RAMP,
        .chopper_mode  = STEPPER_CHOP_AUTO,
        .microsteps    = 16U,
        .irun          = 20U,          /* ~65% of 3A = ~1.9A  */
        .ihold         = 8U,           /* ~25% hold current    */
        .iholddelay    = 6U,
        .steps_per_mm  = 80.0f,        /* adjust for your mechanics */
        .fclk_hz       = 12000000U,    /* internal oscillator       */
        .rsense_mohm   = 75U,          /* BTT TMC5160 PRO           */
        .invert_dir    = false,
        .encoder_enable= false,
    };

    if (!stepper_init(&cfg))
    {
        debug_print("stepper_init failed\r\n");
        while (true) { ; }
    }
    debug_print("stepper_init [OK]\r\n");

    /* --- Step 3: Ramp config and move --- */
    TMC5160_RampConfig_t ramp = {
        .VSTART = 0U,
        .A1     = 1000U,
        .V1     = 10000U,
        .AMAX   = 5000U,
        .VMAX   = 50000U,
        .DMAX   = 5000U,
        .D1     = 1000U,
        .VSTOP  = 10U,
    };

    stepper_set_ramp(&ramp);
    stepper_enable();
    stepper_move_to(1000);

    debug_print("Moving to 1000 usteps...\r\n");

    /* --- Step 4: Poll until done --- */
    TMC5160_MotorStatus_t st;
    uint32_t timeout = 0U;

    do {
        stepper_poll(&st);
        timeout++;
        CORETIMER_DelayMs(10U);
    } while (!st.pos_reached && timeout < 1000U);

    snprintf(uart_buf, sizeof(uart_buf),
             "Done. pos=%ld  sg=%u  stall=%d  ot=%d  %s\r\n",
             (long)st.xactual, st.sg_result, (int)st.stalled,
             (int)st.overtemp,
             st.pos_reached ? "[pos_reached]" : "[TIMEOUT]");
    debug_print(uart_buf);

    stepper_disable();
    debug_print("--- Test Complete ---\r\n");

    while (true)
    {
        SYS_Tasks();
    }

    return EXIT_FAILURE;
}

