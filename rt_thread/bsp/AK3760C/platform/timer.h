/*
 * File      : timer.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2015, RT-Thread Development Team
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
 * 2015-04-29     ArdaFu       first version
 */

#ifndef __TIMER_H__
#define __TIMER_H__
////////////////////////////////////////////////////////////////////////////////
#include "stdint.h"


#define TIMER_FREQ              (25000000)
#define BASE_TIMER_CLK          1000000
#define ERROR_TIMER             -1

#define PWM_MAX_FREQ            6000000
#define PWM_MIN_FREQ            92
#define PWM_MAX_DUTY_CYCLE      100
 
typedef enum
{
    AK_TIMER1 = 0,           ///< TIMER1
    AK_TIMER2,               ///< TIMER2
    AK_TIMER3,               ///< TIMER3
    AK_TIMER4,               ///< TIMER4
    AK_TIMER5,               ///< TIMER5
    
    AK_TIMER_NUM             ///< MAX TIMER number
} T_TIMER_ID;

typedef enum
{
    MODE_AUTO_RELOAD_TIMER=0,
    MODE_ONE_SHOT_TIMER,
    MODE_PWM
}T_TIMER_MODE;

typedef void (*timer_int_cb)(void);

typedef union
{
    uint32_t count;
    struct
    {
        /* little endian */
        uint16_t low_level;
        uint16_t high_level;
    }pwm;
}T_TIMER_COUNT;
    
//timer config struct
typedef struct
{
    T_TIMER_MODE mode;
    uint8_t clk_pre_div;
    T_TIMER_COUNT timer_count;
}T_TIMER_CFG; 


int hw_timer_init(int timer_id, T_TIMER_CFG timer_cfg);

void hw_timer_int_clear(int timer_id);
////////////////////////////////////////////////////////////////////////////////
#endif /* __TIMER_H__ */

