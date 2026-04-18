#include "stepper_transport.h"
#include <stddef.h>

/* Minimum CS high time between back-to-back transactions (tSCSH >= 100 ns) */
#define CS_GUARD_US  1U

/* -------------------------------------------------------------------
 * Decode the status byte returned as byte 0 of every TMC5160 frame
 *
 * Bit 0 : reset_flag   — IC was reset; clear by reading GSTAT
 * Bit 1 : driver_error — error present; read GSTAT for detail
 * Bit 2 : sg2          — StallGuard2 active
 * Bit 3 : standstill   — motor at standstill
 * Bit 4 : vel_reached  — VMAX reached
 * Bit 5 : pos_reached  — XTARGET reached
 * Bit 6 : stop_l       — left stop switch
 * Bit 7 : stop_r       — right stop switch
 * ------------------------------------------------------------------- */
static void decode_status(uint8_t byte, TMC5160_Status_t *s)
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

/* -------------------------------------------------------------------
 * Send one 5-byte SPI frame.
 *
 * CS is asserted for the full burst then released.
 * spi_write_read() is polled via spi_is_busy() before CS deassert.
 * Returns the status byte (RX[0]).
 * If rx_data is non-NULL the 32-bit payload (RX[1..4]) is assembled.
 * ------------------------------------------------------------------- */
static uint8_t transfer5(const Stepper_HAL_t *hal,
                         uint8_t              addr,
                         uint32_t             data,
                         uint32_t            *rx_data)
{
    uint8_t tx[5];
    uint8_t rx[5] = {0U, 0U, 0U, 0U, 0U};

    tx[0] = addr;
    tx[1] = (uint8_t)((data >> 24U) & 0xFFU);
    tx[2] = (uint8_t)((data >> 16U) & 0xFFU);
    tx[3] = (uint8_t)((data >>  8U) & 0xFFU);
    tx[4] = (uint8_t)( data         & 0xFFU);

    hal->cs_assert();
    hal->spi_write_read(tx, rx, 5U);
    while (hal->spi_is_busy()) { ; }
    hal->cs_deassert();
    hal->delay_us(CS_GUARD_US);

    if (rx_data != NULL)
    {
        *rx_data = ((uint32_t)rx[1] << 24U)
                 | ((uint32_t)rx[2] << 16U)
                 | ((uint32_t)rx[3] <<  8U)
                 |  (uint32_t)rx[4];
    }

    return rx[0];
}

/* ------------------------------------------------------------------- */

void STEPPER_WriteReg(const Stepper_HAL_t *hal,
                      uint8_t              addr,
                      uint32_t             data,
                      TMC5160_Status_t    *status)
{
    uint8_t sb = transfer5(hal, (uint8_t)(addr | 0x80U), data, NULL);
    decode_status(sb, status);
}

uint32_t STEPPER_ReadReg(const Stepper_HAL_t *hal,
                         uint8_t              addr,
                         TMC5160_Status_t    *status)
{
    uint32_t rx_data = 0U;
    uint8_t  clean   = addr & 0x7FU;   /* ensure write bit clear */

    /* Transaction 1: issue read request — response is stale, discard */
    transfer5(hal, clean, 0U, NULL);

    /* Transaction 2: clock out — response contains the requested data */
    uint8_t sb = transfer5(hal, clean, 0U, &rx_data);
    decode_status(sb, status);

    return rx_data;
}
