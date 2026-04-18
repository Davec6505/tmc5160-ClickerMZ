#include "tmc5160_spi.h"
#include "tmc5160_reg.h"
#include "definitions.h"

/* Minimum CS high time between transactions (tSCSH = 100ns min) */
#define TMC5160_CS_GUARD_US     1U

/* ---------------------------------------------------------------
 * Decode the status byte returned as byte 0 of every SPI frame
 *
 * Bit 0: reset_flag   - IC had a reset, clear by reading GSTAT
 * Bit 1: driver_error - check GSTAT for details
 * Bit 2: sg2          - StallGuard2 active
 * Bit 3: standstill   - motor at standstill
 * Bit 4: vel_reached  - VMAX reached
 * Bit 5: pos_reached  - XTARGET reached
 * Bit 6: stop_l       - left stop switch
 * Bit 7: stop_r       - right stop switch
 * --------------------------------------------------------------- */
static void TMC5160_DecodeStatus(uint8_t byte, TMC5160_Status_t *s)
{
    if (s == NULL) { return; }
    s->reset_flag   = (byte & 0x01U) != 0U;
    s->driver_error = (byte & 0x02U) != 0U;
    s->sg2          = (byte & 0x04U) != 0U;
    s->standstill   = (byte & 0x08U) != 0U;
    s->vel_reached  = (byte & 0x10U) != 0U;
    s->pos_reached  = (byte & 0x20U) != 0U;
    s->stop_l       = (byte & 0x40U) != 0U;
    s->stop_r       = (byte & 0x80U) != 0U;
}

/* ---------------------------------------------------------------
 * Core 5-byte SPI burst
 *
 * CS is held LOW for the entire burst then released HIGH.
 * SPI2_WriteRead() is interrupt-driven; we poll IsBusy() to block
 * until complete before releasing CS — this is safe and simple
 * since we are not in an ISR context.
 *
 * Returns the status byte (byte 0 of RX).
 * If rx_data is non-NULL, assembles bytes 1-4 into a uint32_t.
 * --------------------------------------------------------------- */
static uint8_t TMC5160_Transfer5(uint8_t addr, uint32_t data, uint32_t *rx_data)
{
    uint8_t tx[5];
    uint8_t rx[5] = {0U, 0U, 0U, 0U, 0U};

    /* Pack frame - MSB first */
    tx[0] = addr;
    tx[1] = (uint8_t)((data >> 24U) & 0xFFU);
    tx[2] = (uint8_t)((data >> 16U) & 0xFFU);
    tx[3] = (uint8_t)((data >>  8U) & 0xFFU);
    tx[4] = (uint8_t)( data         & 0xFFU);

    CS_Clear();                          /* Assert CS (active low) */

    SPI2_WriteRead(tx, 5U, rx, 5U);
    while (SPI2_IsBusy()) { ; }          /* Wait for ISR completion */

    CS_Set();                            /* Deassert CS             */
    CORETIMER_DelayUs(TMC5160_CS_GUARD_US);  /* tSCSH guard         */

    if (rx_data != NULL)
    {
        *rx_data = ((uint32_t)rx[1] << 24U) |
                   ((uint32_t)rx[2] << 16U) |
                   ((uint32_t)rx[3] <<  8U) |
                    (uint32_t)rx[4];
    }

    return rx[0];   /* status byte */
}

/* ---------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------- */
void TMC5160_WriteReg(uint8_t addr, uint32_t data, TMC5160_Status_t *status)
{
    uint8_t sb = TMC5160_Transfer5((uint8_t)(addr | TMC5160_WRITE_FLAG), data, NULL);
    TMC5160_DecodeStatus(sb, status);
}

uint32_t TMC5160_ReadReg(uint8_t addr, TMC5160_Status_t *status)
{
    uint32_t value = 0U;
    uint8_t  sb;

    /* Transaction 1: latch the address into the TMC5160 output register */
    TMC5160_Transfer5((uint8_t)(addr & 0x7FU), 0U, NULL);

    /* Transaction 2: clock out the latched register value */
    sb = TMC5160_Transfer5((uint8_t)(addr & 0x7FU), 0U, &value);
    TMC5160_DecodeStatus(sb, status);

    return value;
}