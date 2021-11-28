/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef HWINIT_H_INCLUDED
#define HWINIT_H_INCLUDED

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/crc.h>
#include "stm32_can_light.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PINDEF_BLKNUM    3  //3rd to last flash page
#define PINDEF_BLKSIZE   1024
#define NUM_PIN_COMMANDS 10
#define PIN_IN 0
#define PIN_OUT 1

struct pindef
{
   uint32_t port;
   uint16_t pin;
   uint8_t inout;
   uint8_t level;
};

struct pincommands
{
   struct pindef pindef[NUM_PIN_COMMANDS];
   uint32_t crc;
};

#define PINDEF_NUMWORDS (sizeof(struct pindef) * NUM_PIN_COMMANDS / 4)

void clock_setup(void);
void nvic_setup(void);
void rtc_setup(void);

void rcc_clock_setup_in_hse_8mhz_out_64mhz(void);

void initialize_pins();

#ifdef __cplusplus
}
#endif

#endif // HWINIT_H_INCLUDED
