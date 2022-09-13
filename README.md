# C Library for the 5329 V1 Kitronik Robotics Board for Raspberry Pi Pico

Ported from the original Microptyhon code by Kitronik: https://github.com/KitronikLtd/Kitronik-Pico-Robotics-Board-MicroPython

Currently under development, work in progess..

Kitronik Robotics Board for Raspberry Pi Pico: https://kitronik.co.uk/products/5329-kitronik-compact-robotics-board-for-raspberry-pi-pico
Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

## Functions
---
### Construct and destruct board instance.
    KitronikRoboticsBoard_t *krb_init();
    void destroy (struct KitronikRoboticsBoard *self);

### Motor control.
    motor_on(struct KitronikRoboticsBoard *self, const uint8_t motor, const char dir, uint8_t speed)
    motor_off(struct KitronikRoboticsBoard *self, const uint8_t motor)

### Servo control.
    servo(struct KitronikRoboticsBoard *self, uint8_t servo, uint8_t degrees)

### Stepper Motor control.

---

## Function documentation and usage.
| krb_init()                          |
|:---------- |
| `KitronikRoboticsBoard_t *krb_init()` |
| Create and return a new board instance.  |
| **Parameters** <br> None |
| **Returns** <br> New board instance. |
| **Example**
| `KitronikRoboticsBoard_t *board = krb_init();` |
---
| destroy()                         |
|:---------- |
| `void destroy (struct KitronikRoboticsBoard *self)` |
| Clean up and destroy a board instance  |
| **Parameters** <br>  **self** The board instance used to make the call. |
| **Returns** <br> None. |
| **Example**
| `board->destroy(board);` |

