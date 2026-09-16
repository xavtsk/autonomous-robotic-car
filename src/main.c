/*
 * File: main.c
 * Purpose: Verify the Pico build and USB serial connection.
 */

#include <stdio.h>

#include "pico/stdlib.h"

#define HELLO_INTERVAL_MS (1000U)

int main(void)
{
    stdio_init_all();

    for (;;)
    {
        printf("Hello from Raspberry Pi Pico!\n");
        sleep_ms(HELLO_INTERVAL_MS);
    }
}
