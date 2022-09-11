#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/regwrite.h"
#include "inc/Kitronik_Robotics_Board.h"

KitronikRoboticsBoard_t *krb_init()
{
    KitronikRoboticsBoard_t *krb = malloc(sizeof(KitronikRoboticsBoard_t));
    krb->CHIP_ADDR = 0x6c;
    krb->SRV_REG_BASE = 0x08;
    krb->MOT_REG_BASE = 0x28;
    krb->REG_OFFSET = 0x4;
    krb->motor_on = &_krb_motor_on;
    krb->software_reset = &_krb_software_reset;
    krb->destroy = &_krb_destroy;
    krb->outputs_off = &_krb_zero_outputs;

    // I2C Initialisation. Frequency 100000.
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    krb->software_reset(krb);

    // Setup the prescale to have 20mS pulse repetition - this is dictated by the servos.
    reg_write(I2C_PORT, krb->CHIP_ADDR, 0xfe, "\x78", 1);

    // Block write outputs to off
    krb->outputs_off(krb);

    // Come out of sleep
    reg_write(I2C_PORT, krb->CHIP_ADDR, 0x00, "\x01", 1);

    return krb;
}

void _krb_software_reset(KitronikRoboticsBoard_t *self)
{
    // Software Reset
    i2c_write_raw_blocking(I2C_PORT, "\x06", 1);
}

void _krb_motor_on(KitronikRoboticsBoard_t *self, uint8_t motor, const char dir, uint8_t speed)
{

    // Cap speed to 100%
    if (speed < 0)
    {
        speed = 0;
    }
    else if (speed > 100)
    {
        speed = 100;
    }

    // Turn on a motor
    // Driving the motor. Convert 0-100% to 0-4095 and push it to the correct registers.
    // each motor has 4 writes - low and high bytes for a pair of registers.

    uint8_t MOT_REG = self->MOT_REG_BASE + (2 * (motor - 1) * self->REG_OFFSET);
    uint16_t PWM_Val = speed * 40.95;
    uint8_t low_byte = PWM_Val & 0xff;
    uint8_t high_byte = (PWM_Val >> 8) & 0xff;
    uint8_t zero = 0;

    if (dir == 'f')
    {
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG, &low_byte, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 1, &high_byte, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 4, &zero, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 5, &zero, 1);
    }
}

void _krb_zero_outputs(KitronikRoboticsBoard_t *self)
{
    // Block write outputs to off
    reg_write(I2C_PORT, self->CHIP_ADDR, 0xfa, "\x00", 1);
    reg_write(I2C_PORT, self->CHIP_ADDR, 0xfb, "\x00", 1);
    reg_write(I2C_PORT, self->CHIP_ADDR, 0xfc, "\x00", 1);
    reg_write(I2C_PORT, self->CHIP_ADDR, 0xfd, "\x00", 1);
}

void _krb_destroy(KitronikRoboticsBoard_t *self)
{
    // Turn all outputs off and free allocated memory.
    self->outputs_off(self);
    free(self);
}