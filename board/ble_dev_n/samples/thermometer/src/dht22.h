/*  
 *  dht22.h - DHT22 sensor library
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
 
#ifndef _DHT22_H_
#define _DHT22_H_

#include <inttypes.h>

#define HIGH 1
#define LOW 0

 
struct dht_data{
    uint8_t data[5];
};

void dht_init(uint32_t pin);
uint32_t dht_read(uint32_t pin, struct dht_data *data);


#endif /*_DHT22_H_*/
