#include <stdio.h>
#include "hardware/i2c.h"
#include "inc/regwrite.h"


int reg_write(i2c_inst_t *i2c,
              const uint addr,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes)
{

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1)
    {
        return 0;
    }

    // Prepend the register address to the front of the data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++)
    {
        msg[i + 1] = buf[i];
    }

    // Write data to the register over i2c.
    int res = i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return res;
}