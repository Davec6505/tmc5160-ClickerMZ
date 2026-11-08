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

/* Drain the TX ring buffer — timeout prevents infinite hang if UART is misconfigured */
static void debug_flush(void)
{
    uint32_t t = 200000U;
    while (UART2_WriteCountGet() > 0U && t-- > 0U) { ; }
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

/* ---------------------------------------------------------------
 * DDS axis — one Axis_t per motor.  Caller owns the storage.
 * --------------------------------------------------------------- */
typedef struct {
    Stepper_t   motor;
    uint32_t    phase_acc;    /* DDS phase accumulator              */
    bool        step_pending; /* step pin is HIGH — clear next tick */
} Axis_t;

static Axis_t x_axis;

/* Current StallGuard threshold — adjusted at runtime via SW1/SW2 */
static int8_t s_sgt = 20;





/* ---------------------------------------------------------------
 * TMR2 callback — 10 µs tick (100 kHz)
 * No SPI, no blocking calls — GPIO and accumulator math only.
 * --------------------------------------------------------------- */
static void timer_isr(uint32_t status, uintptr_t context)
{
    (void)status; (void)context;

    /* De-assert step pulse raised in the previous tick */
    if (x_axis.step_pending)
    {
        x_axis.motor.hal->step_clear();
        x_axis.step_pending = false;
    }

    /* S-curve ramp tick: returns DDS tuning word and current direction */
    bool fwd;
    uint32_t prev = x_axis.phase_acc;
    x_axis.phase_acc += stepper_swramp_tick(&x_axis.motor, &fwd);

    /* 32-bit overflow = one microstep */
    if (x_axis.phase_acc < prev)
    {
        if (fwd) { x_axis.motor.hal->dir_set();   }
        else     { x_axis.motor.hal->dir_clear(); }
        x_axis.motor.hal->step_set();
        x_axis.step_pending = true;
        stepper_step_tick(&x_axis.motor, fwd);
    }
}

int main(void)
{
    SYS_Initialize(NULL);

    /* Immediate pulse — if you see this blink, code is running */
    LED1_Set();
    CORETIMER_DelayMs(200U);
    LED1_Clear();
    LED2_Set();
    CORETIMER_DelayMs(200U);
    LED2_Clear();

    /* --- Init stepper FIRST — before timer fires --- */
    TMC5160_Config_t cfg = {
        .drive_mode    = STEPPER_MODE_STEPDIR,
        .chopper_mode  = STEPPER_CHOP_SPREADCYCLE,
        .microsteps    = 64U,
        .irun          = 3U,           /* (3+1)/32  × 0.325/0.075/√2 ≈ 0.38A RMS — stoppable by hand */
        .ihold         = 1U,           /* minimal hold current */
        .iholddelay    = 6U,
        .steps_per_mm  = 320.0f,       /* 64 usteps × 200 steps/rev ÷ 40mm/rev */
        .fclk_hz       = 12000000U,    /* internal oscillator  */
        .rsense_mohm   = 75U,          /* BTT TMC5160 PRO 75 mΩ */
        .invert_dir    = false,
        .encoder_enable= false,
    };

    if (stepper_init(&x_axis.motor, &cfg, &hal) == false)
    {
        debug_print("stepper_init failed\r\n");
        while (true) { ; }
    }
/* SGT initial = +20: higher threshold needed at low current (irun=3 = 0.38A).
 * SW1 = SGT+1 (less sensitive), SW2 = SGT-1 (more sensitive).
 * Decrease toward -64 until stall fires under hand load. */
stepper_stallguard_config(&x_axis.motor, s_sgt, false);

/* TCOOLTHRS = 500000: keeps SG active at any speed above standstill. */
stepper_set_velocity_bands(&x_axis.motor, 0U, 500000U, 0U);
/* NOTE: SG_STOP (SW_MODE bit 10) is NOT set — it only gates the internal ramp
 * generator and has no effect in STEP/DIR mode. Stall detection is software-only. */

    /* S-curve ramp: 100 kHz tick, 80 000 usteps/s ceiling,
     * 200 000 usteps/s² acceleration, 2 000 000 usteps/s³ jerk.
     * Tune accel/jerk to taste: more jerk → sharper S, less jerk → smoother. */
    stepper_swramp_config(&x_axis.motor,
                          100000.0f,    /* tick_hz   — must match TMR2 period */
                          80000.0f,     /* v_max_ups                          */
                          200000.0f,    /* accel_ups2                         */
                          2000000.0f);  /* jerk_ups3 (0 = trapezoid)          */



    debug_print("stepper_init [OK]\r\n");
    debug_flush();

    /* Verify SPI: IOIN bits 31:24 = chip version. TMC5160 = 0x30 */
    uint32_t ioin    = stepper_read_reg(&x_axis.motor, 0x04U);
    uint8_t  version = (uint8_t)(ioin >> 24U);

    snprintf(uart_buf, sizeof(uart_buf),
             "IOIN = 0x%08lX  VERSION = 0x%02X %s\r\n",
             (unsigned long)ioin, version,
             (version == 0x30U) ? "[OK]" : "[FAIL - check SPI wiring]");
    debug_print(uart_buf);
    debug_flush();

    if (version != 0x30U)
    {
        /* SPI fault — blink LED2 rapidly and continue so LED1 still toggles */
        debug_print("SPI FAIL - check wiring. Continuing without motor.\r\n");
        debug_flush();
        for (uint8_t i = 0U; i < 10U; i++)
        {
            LED2_Toggle();
            CORETIMER_DelayMs(100U);
        }
    }

    /* Configure motion, enable driver, then start DDS timer */
    stepper_swramp_set_velocity(&x_axis.motor, 40000.0f);  /* +fwd, will ramp up */
    stepper_enable(&x_axis.motor);

    TMR2_CallbackRegister(timer_isr, (uintptr_t)0);
    TMR2_Start();

    /* Grace period: let motor reach speed before stall detection activates.
     * SG result is invalid during acceleration — ignore first 500 ms. */
    CORETIMER_DelayMs(500U);

    debug_print("DDS running\r\n");
    debug_flush();

    /* Main loop: SPI status + SW1/SW2 SGT tuning.
     * Not stalled: SW1 = SGT+1 (less sensitive), SW2 = SGT-1 (more sensitive).
     * Stalled:     SW1 = resume motor (ramp back up). */
    bool sw1_prev  = true;
    bool sw2_prev  = true;
    bool stalled   = false;
    while (true)
    {
        bool sw1 = (SW1_Get() != 0U);
        bool sw2 = (SW2_Get() != 0U);

        if (stalled)
        {
            /* SW1 falling edge — resume */
            if (!sw1 && sw1_prev)
            {
                stalled = false;
                LED1_Clear();
                stepper_swramp_set_velocity(&x_axis.motor, 40000.0f);
                debug_print("Resumed [SW1] — grace period...\r\n");
                debug_flush();
                CORETIMER_DelayMs(500U);  /* wait for speed before SG is valid */
            }
        }
        else
        {
            /* SW1: SGT+1 */
            if (!sw1 && sw1_prev && s_sgt < 63)
            {
                s_sgt++;
                stepper_stallguard_config(&x_axis.motor, s_sgt, false);
                snprintf(uart_buf, sizeof(uart_buf), "SGT -> %d  [SW1+]\r\n", (int)s_sgt);
                debug_print(uart_buf);
                debug_flush();
            }

            /* SW2: SGT-1 */
            if (!sw2 && sw2_prev && s_sgt > -64)
            {
                s_sgt--;
                stepper_stallguard_config(&x_axis.motor, s_sgt, false);
                snprintf(uart_buf, sizeof(uart_buf), "SGT -> %d  [SW2-]\r\n", (int)s_sgt);
                debug_print(uart_buf);
                debug_flush();
            }

            /* DIAG0 active low = stall */
            if (!DIAG0_Get())
            {
                stalled = true;
                LED1_Set();
                stepper_swramp_set_velocity(&x_axis.motor, 0.0f);
                snprintf(uart_buf, sizeof(uart_buf),
                         "*** STALL! SGT=%d — press SW1 to resume ***\r\n", (int)s_sgt);
                debug_print(uart_buf);
                debug_flush();
            }
        }

        sw1_prev = sw1;
        sw2_prev = sw2;

        TMC5160_MotorStatus_t st;
        stepper_poll(&x_axis.motor, &st);

        snprintf(uart_buf, sizeof(uart_buf),
                 "SGT=%d  sg=%u  stall=%d  stst=%d\r\n",
                 (int)s_sgt, st.sg_result, (int)st.stalled, (int)st.standstill);
        debug_print(uart_buf);
        debug_flush();
    }

    return EXIT_FAILURE;
}



