/*
 *  main.c - BLE Thermometer
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
* @brief    Example project showing how to use trace library and
*           simple interfacing with DHT22.
*/


#include "nrf_error.h"
#include "nrf_assert.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"

#include "ble_dev_board_profile.h"
#include "dht22.h"
#include "mk_trace.h"

#define STARTED_LED_PIN BLE_DEV_BOARD_LED_GREEN
#define ASSERT_LED_PIN BLE_DEV_BOARD_LED_RED
#define DHT22_PIN 8

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(ASSERT_LED_PIN);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development / debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    while (1) ;

    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}

int main(void)
{
    uint32_t err_code;
    struct dht_data buffer;
    float t;
    float h;

    nrf_gpio_cfg_output(ASSERT_LED_PIN);
    nrf_gpio_cfg_output(STARTED_LED_PIN);

    nrf_gpio_pin_set(STARTED_LED_PIN);

    mk_trace_init(256,256);
    nrf_delay_ms(2000);

    dht_init(DHT22_PIN);

    while(1) {
        err_code = dht_read(DHT22_PIN, &buffer);
        APP_ERROR_CHECK(err_code);
        t = buffer.data[2] & 0x7F;
        t *= 256;
        t += buffer.data[3];
        t /= 10;
        if (buffer.data[2] & 0x80)
                t *= -1;
        h = buffer.data[0];
        h *= 256;
        h += buffer.data[1];
        h /= 10;

        mk_trace("T %0.2fC H %0.2f%%\n\r", t,h);

        nrf_delay_ms(2000);
    }
}
