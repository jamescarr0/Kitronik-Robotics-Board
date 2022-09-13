#include <stdio.h>
#include "hardware/i2c.h"
#include "regwrite.h"
#include "Kitronik_Robotics_Board.h"


int reg_write(KitronikRoboticsBoard_t *self,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes)
{

    int num_bytes_read = 0;
    uint8_t msg[nbytes+1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1)
    {
        return 0;
    }

    // Createa data packets, prepend the register address to the front of the data packet
    // then append 8 byte data packets after the address.
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++)
    {
        msg[i + 1] = buf[i];
    }

    // The amount of bytes written.
    int res = 0;
    
    // Write data to the register over i2c.
    res = i2c_write_blocking(self->I2C_PORT, self->CHIP_ADDR, msg, (nbytes + 1), false);

    return res;
}