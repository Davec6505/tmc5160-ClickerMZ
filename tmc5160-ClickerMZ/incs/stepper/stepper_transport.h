#ifndef STEPPER_TRANSPORT_H
#define STEPPER_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include "stepper_hal.h"

/*
 * TMC5160 SPI transport layer — stateless framing functions.
 *
 * The caller (stepper.c) owns the Stepper_HAL_t and passes it on every call.
 * This layer has no global state; it only packages/unpackages 40-bit frames.
 *
 * Frame format (40 bits, MSB first):
 *   TX[0] : addr | 0x80 for write, addr for read
 *   TX[1..4] : 32-bit data, MSB first
 *   RX[0] : status byte
 *   RX[1..4] : register value (read only; requires two transactions)
 *
 * READ protocol: TMC5160 returns data one frame behind the request.
 *   STEPPER_ReadReg() issues two transactions and returns the second result.
 */

/* Decoded SPI status byte returned as byte 0 of every TMC5160 frame */
typedef struct
{
    bool reset_flag;    /* IC reset since last GSTAT read    */
    bool driver_error;  /* Driver error present — read GSTAT */
    bool sg2;           /* StallGuard2 triggered             */
    bool standstill;    /* Motor at standstill               */
    bool vel_reached;   /* VMAX velocity reached             */
    bool pos_reached;   /* XTARGET position reached          */
    bool stop_l;        /* Left stop switch active           */
    bool stop_r;        /* Right stop switch active          */
} TMC5160_Status_t;

/*
 * Write 32-bit data to register addr.
 * status may be NULL if the caller does not need the status byte.
 */
void STEPPER_WriteReg(const Stepper_HAL_t *hal,
                      uint8_t              addr,
                      uint32_t             data,
                      TMC5160_Status_t    *status);

/*
 * Read 32-bit data from register addr.
 * Performs two SPI transactions internally (TMC5160 read protocol).
 * status may be NULL if the caller does not need the status byte.
 */
uint32_t STEPPER_ReadReg(const Stepper_HAL_t *hal,
                         uint8_t              addr,
                         TMC5160_Status_t    *status);

#endif /* STEPPER_TRANSPORT_H */
