#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/regwrite.h"
#include "inc/Kitronik_Robotics_Board.h"

/* Datasheet for the onboard PCA Chip: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf */
KitronikRoboticsBoard_t *krb_init()
{
    KitronikRoboticsBoard_t *krb = malloc(sizeof(KitronikRoboticsBoard_t));
    krb->CHIP_ADDR = 0x6c;
    krb->SRV_REG_BASE = 0x08;
    krb->MOT_REG_BASE = 0x28;
    krb->REG_OFFSET = 0x4;
    krb->motor_on = &_krb_motor_on;
    krb->motor_off = &_krb_motor_off;
    krb->destroy = &_krb_destroy;
    krb->servo = &_krb_servo_write;

    // I2C Initialisation. Frequency 100000.
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Reset the PCA.  This is performed on power up but may be useful for debugging.
    _krb_software_reset(krb);

    // Setup the prescale to have 20mS (50Hz) pulse repetition - this is dictated by the servos.
    _set_prescaler(krb);

    // Block write outputs to off
    _krb_zero_outputs(krb);

    // Come out of sleep.  It takes 500uS max for the oscillator to be up and running once SLEEP logic bit (bit4) is set to 1.
    // Timings are not guranteed if registers are accessed with the 500uS window.
    reg_write(I2C_PORT, krb->CHIP_ADDR, 0x00, "\x01", 1);
    sleep_us(500);  // Delay to gurantee PWM timings.

    return krb;
}

void _set_prescaler(KitronikRoboticsBoard_t *self)
{
    /*
        set PWM Frequency Pre Scale.  The prescale value is determined with the formunla:
        presscale value = round(osc clock / (4096 * update rate))
        Where update rate is the output modulation frequency required.
        For example, the output frequency of 50Hz with the internal oscillator clock frequency of
        25 Mhz:

        prescale value = round( 25MHZ / (4096*50Hz) ) - 1 
        prescale value = round (25000000 / (4096 * 50)) - 1 
        presscale value = 121 = 79h = 0x79
    */
    reg_write(I2C_PORT, self->CHIP_ADDR, 0xfe, "\x79", 1);
}

void _krb_software_reset(KitronikRoboticsBoard_t *self)
{
    /*
        byte 0: Call Address 0000000 = 0x0;
        byte 1: Software Reset Data Byte 00000110 = 0x6;
        If byte 1 does NOT equal 0x6, or more than 1 byte of data is sent, the PCA chip does not acknowledge it.
    */

    // Software Reset on the PCA Chip.
    i2c_write_blocking(I2C_PORT, 0x0, "\x06", 1, false);
}

void _krb_motor_on(KitronikRoboticsBoard_t *self, const uint8_t motor, const char dir, uint8_t speed)
{

    // Speed constraints - Cap speed to 100%
    if (speed < 0)
    {
        speed = 0;
    }
    else if (speed > 100)
    {
        speed = 100;
    }

    // Check motor number is within range. 1 - 4.
    if ((motor < 1) | (motor > 4))
    {
        printf("Error: Invalid motor selection.\n");
        return;
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
    else if (dir == 'r')
    {
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG, &zero, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 1, &zero, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 4, &low_byte, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 5, &high_byte, 1);
    }
    else
    {
        printf("Error: Invalid Direction.  Stopping motors.\n");
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG, &zero, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 1, &zero, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 4, &zero, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, MOT_REG + 5, &zero, 1);
    }
}

void _krb_motor_off(KitronikRoboticsBoard_t *self, const uint8_t motor)
{
    _krb_motor_on(self, motor, 'f', 0);
}

void _krb_zero_outputs(KitronikRoboticsBoard_t *self)
{
    // Block write all outputs to off: 0xfa, 0xfb, 0xfc, 0xfd.
    // Control registers allow four I2C bus write sequences to fill all the output registers
    // with the same pattern.
    for (int i = 0; i < 4; i++)
    {
        reg_write(I2C_PORT, self->CHIP_ADDR, 0xfa + i, "\x00", 1);
    }
}

void _krb_destroy(KitronikRoboticsBoard_t *self)
{
    // Turn all outputs off and free allocated memory.
    _krb_zero_outputs(self);
    free(self);
}

/*
    To get the PWM pulses to the correct size and zero
    offset these are the default numbers.
    Servo multiplier is calcualted as follows:
    4096 pulses / 20mS = count of 204.8
    1mS is 90 degrees of travel, so each degree is a count of 204.8 / 90 =  2.2755
    servo pulses always have a minimum value - so there is guarentees to be a pulse.
    in the servos examined this is 0.5ms or a count of 102.
    To calculate the count for the corect pulse is simply:
    (degrees x count per degree ) + offset
*/
void _krb_servo_write(KitronikRoboticsBoard_t *self, const uint8_t servo, uint8_t degrees)
{
    // Check the degrees is a realistic number, cap between 0-180.
    if (degrees > 180)
    {
        degrees = 180;
    }
    else if (degrees < 0)
    {
        degrees = 0;
    }

    if ((servo < 1) || (servo > 8))
    {
        printf("Invalid Servo Number, ensure servo number is between 1 and 8 only.\n");
    }
    else
    {
        uint8_t servo_reg = self->SRV_REG_BASE + ((servo - 1) * self->REG_OFFSET);
        uint16_t PWM_val = (degrees * 2.2755) + 102;
        uint8_t low_byte = (PWM_val)&0xff;
        uint8_t high_byte = (PWM_val >> 8) & 0x1; // Cap high byte at 1, should never be more than 2.5ms.
        reg_write(I2C_PORT, self->CHIP_ADDR, servo_reg, &low_byte, 1);
        reg_write(I2C_PORT, self->CHIP_ADDR, servo_reg + 1, &high_byte, 1);
    }
}
