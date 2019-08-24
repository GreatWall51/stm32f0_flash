/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2015, RT-Thread Develop Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-04-14     ArdaFu      first version
 */

#include <rtthread.h>
#include "gpio.h"
#include "led.h"

//ASM9260T EVK pin 16-7 LED0, 0: ON, 1 : OFF
void led_init(void)
{
    *(volatile rt_uint32_t*)(0x08000084) &= ~(1 << 19);
}

void led_on(int num)
{
    hw_gpio_set_level(51, GPIO_LEVEL_HIGH);
}

void led_off(int num)
{
    hw_gpio_set_level(51, GPIO_LEVEL_LOW);
}
