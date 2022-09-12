# C Library for the 5329 V1 Kitronik Robotics Board for Raspberry Pi Pio

Ported from the original Microptyhon code by Kitronik: https://github.com/KitronikLtd/Kitronik-Pico-Robotics-Board-MicroPython

Currently under development, work in progess..

Kitronik Robotics Board for Raspberry Pi Pico: https://kitronik.co.uk/products/5329-kitronik-compact-robotics-board-for-raspberry-pi-pico
Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

## Library

board_init() - init.
board_destroy() - clean up

### Motors
motor_on(struct KitronikRoboticsBoard *self, const uint8_t motor, const char dir, uint8_t speed)
motor_off(struct KitronikRoboticsBoard *self, const uint8_t motor)

### Servos
servo(struct KitronikRoboticsBoard *self, uint8_t servo, uint8_t degrees)

### Stepper Motors
