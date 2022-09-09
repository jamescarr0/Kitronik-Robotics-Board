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

int main()
{
    stdio_init_all();
    printf("Starting!\n");
    sleep_ms(3000);

    // I2C Initialisation. Using it at frequency 100000.
    i2c_init(I2C_PORT, 100000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Software Reset
    i2c_write_raw_blocking(I2C_PORT, "\x06", 1);

    // Setup the prescale to have 20mS pulse repetition - this is dictated by the servos.
    reg_write(I2C_PORT, CHIP_ADDR, 0xfe, "\x78", 1);

    // Block write outputs to off
    reg_write(I2C_PORT, CHIP_ADDR, 0xfa, "\x00", 1);
    reg_write(I2C_PORT, CHIP_ADDR, 0xfb, "\x00", 1);
    reg_write(I2C_PORT, CHIP_ADDR, 0xfc, "\x00", 1);
    reg_write(I2C_PORT, CHIP_ADDR, 0xfd, "\x00", 1);

    // Come out of sleep
    reg_write(I2C_PORT, CHIP_ADDR, 0x00, "\x01", 1);

    // Turn on a motor
    uint8_t motor = 1;
    uint8_t speed = 100;
    const uint8_t MOT_REG = MOT_REG_BASE + (2 * (motor - 1) * REG_OFFSET);
    uint16_t PWM_Val = (uint16_t ) speed * 40.95;
    uint8_t low_byte = PWM_Val & 0xff;
    uint8_t high_byte = (PWM_Val >> 8) & 0xff;

    reg_write(I2C_PORT, CHIP_ADDR, MOT_REG, &low_byte, 1);
    reg_write(I2C_PORT, CHIP_ADDR, MOT_REG+1, &high_byte, 1);
    reg_write(I2C_PORT, CHIP_ADDR, MOT_REG+4, 0, 1);
    reg_write(I2C_PORT, CHIP_ADDR, MOT_REG+5, 0, 1);

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
        msg[i+1] = buf[i];
    }

    // Write data to the register over i2c.
    i2c_write_blocking(i2c, addr, msg, nbytes+1, false);

    return num_bytes_read;
}