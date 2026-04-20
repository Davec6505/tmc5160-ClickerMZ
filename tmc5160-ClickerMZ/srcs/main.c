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
    uint32_t    tuning_word;  /* written by motion planner          */
    bool        forward;      /* current direction                  */
    bool        step_pending; /* step pin is HIGH — clear next tick */
} Axis_t;

static Axis_t x_axis;

/* Current StallGuard threshold — adjusted at runtime via SW1/SW2 */
static int8_t s_sgt = 20;

/* Forward declaration — defined after do_home() */
static void set_velocity_steps_per_sec(Axis_t *ax, float v, bool fwd);

/* ---------------------------------------------------------------
 * Sensorless homing
 *
 * 1. Lower SGT to homing sensitivity (more sensitive).
 * 2. Run reverse at creep speed.
 * 3. Stall fires → stop, zero position.
 * 4. Restore SGT and return.
 *
 * Call from main loop only — blocking, uses CORETIMER delays.
 * --------------------------------------------------------------- */
#define HOME_SGT        -10     /* more sensitive than run SGT — tune as needed */
#define HOME_USTEPS_SEC 8000.0f /* slow creep: ~37 RPM at 64 usteps             */
#define HOME_TIMEOUT_MS 5000U   /* abort if no stall within this time           */

static void do_home(void)
{
    debug_print("Homing: start\r\n");
    debug_flush();

    /* 1. Save run SGT, apply homing SGT */
    int8_t run_sgt = s_sgt;
    stepper_stallguard_config(&x_axis.motor, (int8_t)HOME_SGT, false);

    /* 2. Run reverse at creep speed */
    set_velocity_steps_per_sec(&x_axis, HOME_USTEPS_SEC, false);

    /* Grace period — let motor start before SG is sampled */
    CORETIMER_DelayMs(300U);

    /* 3. Poll for stall with timeout */
    uint32_t elapsed = 0U;
    bool     homed   = false;
    while (elapsed < HOME_TIMEOUT_MS)
    {
        TMC5160_MotorStatus_t st;
        stepper_poll(&x_axis.motor, &st);
        if (st.stalled)
        {
            homed = true;
            break;
        }
        CORETIMER_DelayMs(10U);
        elapsed += 10U;
    }

    /* 4. Stop, zero position */
    set_velocity_steps_per_sec(&x_axis, 0.0f, true);
    x_axis.motor.pos_usteps = 0;

    if (homed)
    {
        debug_print("Homing: done — position zeroed\r\n");
    }
    else
    {
        debug_print("Homing: TIMEOUT — no stall detected\r\n");
    }
    debug_flush();

    /* 5. Restore run SGT */
    s_sgt = run_sgt;
    stepper_stallguard_config(&x_axis.motor, s_sgt, false);

    /* Grace period before resuming normal motion */
    CORETIMER_DelayMs(500U);
}

/* Set step rate. Call before or after TMR2_Start(); safe from main loop. */
static void set_velocity_steps_per_sec(Axis_t *ax, float v, bool fwd)
{
    ax->forward     = fwd;
    ax->tuning_word = (uint32_t)(v * (4294967296.0f / 100000.0f)); /* 10 µs tick */
}

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

    /* DDS: 32-bit accumulator overflow = one step event */
    uint32_t prev = x_axis.phase_acc;
    x_axis.phase_acc += x_axis.tuning_word;

    if (x_axis.phase_acc < prev)
    {
        if (x_axis.forward) { x_axis.motor.hal->dir_set();   }
        else                { x_axis.motor.hal->dir_clear(); }
        x_axis.motor.hal->step_set();
        x_axis.step_pending = true;
        stepper_step_tick(&x_axis.motor, x_axis.forward);
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
    /* 64 usteps: DDS ceiling = 100000 usteps/sec = ~469 RPM.
     * 80000 usteps/sec = ~375 RPM — good speed with 10% headroom. */
    set_velocity_steps_per_sec(&x_axis, 40000.0f, true);
    stepper_enable(&x_axis.motor);

    TMR2_CallbackRegister(timer_isr, (uintptr_t)0);
    TMR2_Start();

    /* Grace period: let motor reach speed before stall detection activates.
     * SG result is invalid during acceleration — ignore first 500 ms. */
    CORETIMER_DelayMs(500U);

    debug_print("DDS running\r\n");
    debug_flush();

    /* Main loop: SPI status + SW1/SW2 SGT tuning.
     * SW1 (RB8)        = SGT+1 (less sensitive) OR resume after stall.
     * SW2 (RB11) short = SGT-1 (more sensitive).
     * SW2 (RB11) long  = trigger sensorless home (hold ~1s). */
    bool     sw1_prev   = true;
    bool     sw2_prev   = true;
    bool     stalled    = false;
    uint32_t sw2_held   = 0U;   /* counts 200ms ticks SW2 is held */
    while (true)
    {
        bool sw1 = (SW1_Get() != 0U);
        bool sw2 = (SW2_Get() != 0U);

        if (stalled)
        {
            /* Waiting for SW1 press to resume */
            if (!sw1 && sw1_prev)
            {
                stalled = false;
                LED2_Clear();
                stepper_read_reg(&x_axis.motor, 0x35U); /* read RAMPSTAT clears event flags */
                set_velocity_steps_per_sec(&x_axis, 40000.0f, true);
                debug_print("Resumed [SW1] — grace period...\r\n");
                debug_flush();
                /* Same grace period as startup: motor needs to reach speed before
                 * SG is valid — without this it immediately re-stalls from standstill. */
                CORETIMER_DelayMs(500U);
            }
        }
        else
        {
            if (!sw1 && sw1_prev && s_sgt < 63)
            {
                s_sgt++;
                stepper_stallguard_config(&x_axis.motor, s_sgt, false);
                snprintf(uart_buf, sizeof(uart_buf), "SGT -> %d  [SW1+]\r\n", (int)s_sgt);
                debug_print(uart_buf);
                debug_flush();
            }

            /* SW2: count held ticks; on release decide short vs long press */
            if (!sw2) { sw2_held++; }
            if (sw2 && !sw2_prev)
            {
                if (sw2_held >= 5U) /* 5 × 200ms = 1s hold → home */
                {
                    set_velocity_steps_per_sec(&x_axis, 0.0f, true);
                    do_home();
                    set_velocity_steps_per_sec(&x_axis, 40000.0f, true);
                    debug_print("Resuming normal run\r\n");
                    debug_flush();
                }
                else if (sw2_held > 0U && s_sgt > -64) /* short press → SGT-1 */
                {
                    s_sgt--;
                    stepper_stallguard_config(&x_axis.motor, s_sgt, false);
                    snprintf(uart_buf, sizeof(uart_buf), "SGT -> %d  [SW2-]\r\n", (int)s_sgt);
                    debug_print(uart_buf);
                    debug_flush();
                }
                sw2_held = 0U;
            }
        }
        sw1_prev = sw1;
        sw2_prev = sw2;

        TMC5160_MotorStatus_t st;
        stepper_poll(&x_axis.motor, &st);

        if (!stalled && st.stalled)
        {
            stalled = true;
            set_velocity_steps_per_sec(&x_axis, 0.0f, true); /* stop DDS */
            snprintf(uart_buf, sizeof(uart_buf),
                     "*** STALL detected! SGT=%d sg=%u — press SW1 to resume ***\r\n",
                     (int)s_sgt, st.sg_result);
            debug_print(uart_buf);
            debug_flush();
            LED2_Set();
        }
        else if (stalled && !st.stalled)
        {
            /* chip cleared SG_STOP latch automatically (e.g. shaft released) — keep DDS stopped */
        }
        else
        {
            LED2_Clear();
            snprintf(uart_buf, sizeof(uart_buf),
                     "SGT=%d  sg=%u  stall=%d  stst=%d\r\n",
                     (int)s_sgt, st.sg_result, (int)st.stalled, (int)st.standstill);
            debug_print(uart_buf);
            debug_flush();
        }

        LED1_Toggle();
        CORETIMER_DelayMs(200U);
    }

    return EXIT_FAILURE;
}



