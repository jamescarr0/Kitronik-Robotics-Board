#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Kitronik_Robotics_Board.h"
#include "regwrite.h"

int main()
{
    stdio_init_all();
    KitronikRoboticsBoard_t *board = krb_init();

    uint8_t n = 0;
    while (n < 4)
    {
        board->motor_on(board, 1, 'f', 100);
        sleep_ms(2000);

        board->motor_off(board, 1);
        sleep_ms(2000);

        n++;
    }

    // Clean up. Turn all outputs off and free allocated memory
    board->destroy(board);

    return 0;
}