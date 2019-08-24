/*
 * File      : interrupt.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2015, RT-Thread Development Team
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
#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include <rtthread.h>

#define INT_IRQ     0x00
#define INT_FIQ     0x01

// IRQ Source
typedef enum 
{
    INT_VECTOR_LCD, 
    INT_VECTOR_CAMERA,
    INT_VECTOR_MOTION,
    INT_VECTOR_JPEG,
    INT_VECTOR_DAC,
    INT_VECTOR_ADC,
    INT_VECTOR_L2,
    INT_VECTOR_UART2,
    INT_VECTOR_UART1,
    INT_VECTOR_UART0,
    INT_VECTOR_SPI,    
    INT_VECTOR_MAC,    
    INT_VECTOR_IRDA,    
    INT_VECTOR_SDIO,
    INT_VECTOR_MMC,    
    INT_VECTOR_USB,
    INT_VECTOR_USB_DMA,
    INT_VECTOR_GPIO,
    INT_VECTOR_ASICCLK,
    INT_VECTOR_WGPIO,
    INT_VECTOR_TIMER1,
    INT_VECTOR_TIMER2,
    INT_VECTOR_TIMER3,
    INT_VECTOR_TIMER4,
    INT_VECTOR_TIMER5,
    INT_VECTOR_MAX
}INT_VECTOR;

#define INT_VECTOR_RESERVE  INT_VECTOR_MAX
#define INT_VECTOR_INVALID  INT_VECTOR_MAX
#define INT_VECTOR_INTL2    INT_VECTOR_GPIO

rt_uint32_t rt_hw_interrupt_get_level1_status(void);

rt_uint32_t rt_hw_interrupt_get_mask(rt_uint8_t irq_frq);

void rt_hw_interrupt_level1_handle(rt_uint8_t vector);

void rt_hw_interrupt_level2_handle(void);

rt_uint8_t rt_hw_interrupt_get_vector(rt_uint8_t id);

#endif
