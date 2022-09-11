#include <stdio.h>

#ifndef KITRONIK_ROBOTICS_BOARD_H
#define KITRONIK_ROBOTICS_BOARD_H

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

typedef struct KitronikRoboticsBoard
{
    uint8_t CHIP_ADDR;
    uint8_t SRV_REG_BASE;
    uint8_t MOT_REG_BASE;
    uint8_t REG_OFFSET;
    void (*motor_on) (struct KitronikRoboticsBoard *self, uint8_t motor, char dir, uint8_t speed);
    void (*motor_off) (struct KitronikRoboticsBoard *self, uint8_t motor);
    void (*software_reset) (struct KitronikRoboticsBoard *self);
    void (*outputs_off) (struct KitronikRoboticsBoard *self);
    void (*destroy) (struct KitronikRoboticsBoard *self);
} KitronikRoboticsBoard_t;

KitronikRoboticsBoard_t *krb_init();

void _krb_software_reset(KitronikRoboticsBoard_t *self);
void _krb_zero_outputs(KitronikRoboticsBoard_t *self);
void _krb_zero_motor_output(KitronikRoboticsBoard_t *self, uint8_t MOT_REG);
void _krb_motor_on(KitronikRoboticsBoard_t *self, uint8_t motor, char dir, uint8_t speed);
void _krb_motor_off(KitronikRoboticsBoard_t *self, uint8_t motor);
void _krb_destroy(KitronikRoboticsBoard_t *self);

#endif /* KITRONIK_ROBOTICS_BOARD_H */