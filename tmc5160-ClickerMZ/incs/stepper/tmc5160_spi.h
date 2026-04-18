#ifndef TMC5160_SPI_H
#define TMC5160_SPI_H

#include <stdint.h>
#include <stdbool.h>

/*
 * TMC5160 SPI Transport Layer
 *
 * Frame format (40 bits, MSB first):
 *   TX: [addr | 0x80 for write] [D31..D24] [D23..D16] [D15..D8] [D7..D0]
 *   RX: [status_byte           ] [D31..D24] [D23..D16] [D15..D8] [D7..D0]
 *
 * CS (RG9) is asserted LOW for the full 5-byte burst then deasserted HIGH.
 *
 * READ NOTE: TMC5160 read requires two transactions.
 *   Transaction 1: send address (bit7=0) - response data is invalid
 *   Transaction 2: send anything        - response contains register value
 *   TMC5160_ReadReg() handles this automatically.
 */

/* Decoded status byte - returned on every SPI transaction */
typedef struct {
    bool reset_flag;    /* IC reset since last GSTAT read            */
    bool driver_error;  /* Driver error present - read GSTAT         */
    bool sg2;           /* StallGuard2 triggered                     */
    bool standstill;    /* Motor at standstill                       */
    bool vel_reached;   /* VMAX velocity reached                     */
    bool pos_reached;   /* XTARGET position reached                  */
    bool stop_l;        /* Left stop switch active                   */
    bool stop_r;        /* Right stop switch active                  */
} TMC5160_Status_t;

/*
 * Write 32-bit value to a register.
 * status may be NULL if caller does not need status byte.
 */
void TMC5160_WriteReg(uint8_t addr, uint32_t data, TMC5160_Status_t *status);

/*
 * Read 32-bit value from a register.
 * Performs two SPI transactions internally.
 * status reflects the status byte from the second transaction.
 * status may be NULL if caller does not need status byte.
 */
uint32_t TMC5160_ReadReg(uint8_t addr, TMC5160_Status_t *status);

#endif /* TMC5160_SPI_H */