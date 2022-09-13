#ifndef REG_WRITE_H
#define REG_WRITE_H

#include "Kitronik_Robotics_Board.h"

int reg_write(KitronikRoboticsBoard_t *self,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes);

#endif