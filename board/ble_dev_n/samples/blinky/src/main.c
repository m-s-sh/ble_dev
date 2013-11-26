/*
 *  main.c - blinking the green LED.
 *
 *  Copyright (c) 2013 Mommosoft Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

/** @file
* @brief    Example project showing how blinking of the green LED.
*           GPIO Example Application.
*/

#include <stdbool.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"


#include "ble_dev_board_profile.h"

int main(void)
{
        // Configure green LED-pin as output
        nrf_gpio_cfg_output(BLE_DEV_BOARD_LED_GREEN);

        while(true)
        {
                nrf_gpio_pin_toggle(BLE_DEV_BOARD_LED_GREEN);
                nrf_delay_ms(100);
        }
}
