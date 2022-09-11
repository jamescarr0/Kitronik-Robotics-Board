#ifndef REG_WRITE_H
#define REG_WRITE_H

int reg_write(i2c_inst_t *i2c,
              const uint addr,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes);

#endif