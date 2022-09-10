#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

const uint8_t CHIP_ADDR = 0x6C;
const uint8_t SRV_REG_BASE = 0x08;
const uint8_t MOT_REG_BASE = 0x28;
const uint8_t REG_OFFSET = 0x4;

int reg_write(i2c_inst_t *i2c,
              const uint addr,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes);

void motor_on(uint8_t motor, const char dir, uint8_t speed);

int main()
{
    stdio_init_all();

    // I2C Initialisation. Frequency 100000.
    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Software Reset
    // i2c_write_raw_blocking(I2C_PORT, "\x06", 1);
    if (reg_write(I2C_PORT, CHIP_ADDR, 0, "\x06", 1) == PICO_ERROR_GENERIC)
    {
        while (1)
        {
            printf("PICO GENERIC ERROR!\n");
            sleep_ms(1000);
        }
    }

    // Setup the prescale to have 20mS pulse repetition - this is dictated by the servos.
    reg_write(I2C_PORT, CHIP_ADDR, 0xfe, "\x78", 1);

    // Block write outputs to off
    reg_write(I2C_PORT, CHIP_ADDR, 0xfa, "\x00", 1);
    reg_write(I2C_PORT, CHIP_ADDR, 0xfb, "\x00", 1);
    reg_write(I2C_PORT, CHIP_ADDR, 0xfc, "\x00", 1);
    reg_write(I2C_PORT, CHIP_ADDR, 0xfd, "\x00", 1);

    // Come out of sleep
    reg_write(I2C_PORT, CHIP_ADDR, 0x00, "\x01", 1);

    while (1)
    {
        motor_on(1, 'f', 100);
        sleep_ms(2000);
        motor_on(1, 'f', 10);
        sleep_ms(2000);
    }

    return 0;
}

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

void motor_on(uint8_t motor, const char dir, uint8_t speed)
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

    uint8_t MOT_REG = MOT_REG_BASE + (2 * (motor - 1) * REG_OFFSET);
    uint16_t PWM_Val = speed * 40.95;
    uint8_t low_byte = PWM_Val & 0xff;
    uint8_t high_byte = (PWM_Val >> 8) & 0xff;
    uint8_t zero = 0;

    if (dir == 'f')
    {
        reg_write(I2C_PORT, CHIP_ADDR, MOT_REG, &low_byte, 1);
        reg_write(I2C_PORT, CHIP_ADDR, MOT_REG + 1, &high_byte, 1);
        reg_write(I2C_PORT, CHIP_ADDR, MOT_REG + 4, &zero, 1);
        reg_write(I2C_PORT, CHIP_ADDR, MOT_REG + 5, &zero, 1);
    }
}