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
    // while (n < 2)
    // {
    //     board->motor_on(board, 1, 'f', 100);
    //     sleep_ms(2000);

    //     board->motor_off(board, 1);
    //     sleep_ms(500);

    //     board->motor_on(board, 1, 'r', 100);
    //     sleep_ms(2000);

    //     board->motor_off(board, 1);
    //     sleep_ms(500);

    //     n++;
    // }

    while (1)
    {
        board->servo(board, 1, 0);
        sleep_ms(1000);
        board->servo(board, 1, 180);
        sleep_ms(1000);
        board->servo(board, 1, 0);
        sleep_ms(1000);
        board->servo(board, 1, 90);
        sleep_ms(1000);
        board->servo(board, 1, 0);
        sleep_ms(1000);
        board->servo(board, 1, 20);
        sleep_ms(1000);
        n++;
    }

    // Clean up. Turn all outputs off and free allocated memory
    board->destroy(board);

    return 0;
}