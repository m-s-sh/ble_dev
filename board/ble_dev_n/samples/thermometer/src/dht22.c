/*
 *  dht22.c - DHT22 sensor library
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

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_gpiote.h"
#include "dht22.h"

#define TIMEOUT 99999

static uint32_t wait_pin(uint32_t pin_number,uint32_t pin_state, uint32_t pin_timeout)
{
        while(nrf_gpio_pin_read(pin_number) == pin_state)
                if (pin_timeout-- == 0) return NRF_ERROR_TIMEOUT;
        return NRF_SUCCESS;
}

static uint32_t dht_read_byte(uint32_t pin_number, uint8_t* byte)
{
        uint8_t i;
        *byte = 0;

        for(i = 0; i < 8; i++) {
                if(wait_pin(pin_number, LOW, TIMEOUT) == NRF_ERROR_TIMEOUT)
                        return NRF_ERROR_TIMEOUT;

        nrf_delay_us(45);

        if (nrf_gpio_pin_read(pin_number) == HIGH)
                *byte |= (1 << (7 - i));

        if(wait_pin(pin_number, HIGH, TIMEOUT) == NRF_ERROR_TIMEOUT)
                return NRF_ERROR_TIMEOUT;
        }

        return NRF_SUCCESS;
}

uint32_t dht_read(uint32_t pin_number, struct dht_data *dht_data)
{
        uint8_t checksum;
        uint8_t i;
        uint32_t error;


        // request sample
        nrf_gpio_cfg_output(pin_number);
        nrf_gpio_pin_clear(pin_number);
        nrf_delay_ms(1);
        nrf_gpio_pin_set(pin_number);
        nrf_delay_us(40);

        GPIO_PIN_CONFIG(pin_number,
                    GPIO_PIN_CNF_DIR_Input,
                    GPIO_PIN_CNF_INPUT_Connect,
                    GPIO_PIN_CNF_PULL_Disabled,
                    GPIO_PIN_CNF_DRIVE_H0H1,
                    GPIO_PIN_CNF_SENSE_Low);

        // get ACT or TIMEOUT
        if(wait_pin(pin_number, LOW, TIMEOUT) == NRF_ERROR_TIMEOUT)
        return NRF_ERROR_TIMEOUT;

        if(wait_pin(pin_number, HIGH, TIMEOUT) == NRF_ERROR_TIMEOUT)
        return NRF_ERROR_TIMEOUT;

        // read the bytes.
        for(i = 0; i < 5; i++) {
        error = dht_read_byte(pin_number, dht_data->data + i);
        if(NRF_SUCCESS != error)
            return error;
        }
        // checking if the data is correct
        checksum = dht_data->data[0] + dht_data->data[1] + dht_data->data[2] + dht_data->data[3];
        if(checksum == dht_data->data[4]) {
        return NRF_SUCCESS;
        }
        return NRF_ERROR_INVALID_DATA;

}

/**@brief Initialize DHT22 temperature and humidity sensor.
 */
void dht_init(uint32_t pin_number) {
        // seems dht22 needs some time to start if init is called just before
        // chip is powered up.
        nrf_delay_ms(2000);
        nrf_gpio_cfg_output(pin_number);
        nrf_gpio_pin_set(pin_number);
}

