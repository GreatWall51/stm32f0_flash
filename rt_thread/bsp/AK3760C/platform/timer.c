/*
 * File      : timer.c
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
#include "anyka_cpu.h"
#include "rtthread.h"
#include "timer.h"
#include <rthw.h>
#include "interrupt.h"

#define TIMER_CLEAR_BIT                 (1<<30)
#define TIMER_FEED_BIT                  (1<<29)
#define TIMER_ENABLE_BIT                (1<<28)
#define TIMER_STATUS_BIT                (1<<27)
#define TIMER_READ_SEL_BIT              (1<<26) 
 
/*----------------------- Type Declarations ---------------------------------*/
 
 
/*----------------------- Variable Declarations -----------------------------*/
static rt_uint8_t timer_run[AK_TIMER_NUM] = {0};
static const rt_uint32_t timer_ctrl_reg1_grp[AK_TIMER_NUM] = {PWM_TIMER1_CTRL_REG1, PWM_TIMER2_CTRL_REG1, PWM_TIMER3_CTRL_REG1, PWM_TIMER4_CTRL_REG1, PWM_TIMER5_CTRL_REG1};
static const rt_uint32_t timer_ctrl_reg2_grp[AK_TIMER_NUM] = {PWM_TIMER1_CTRL_REG2, PWM_TIMER2_CTRL_REG2, PWM_TIMER3_CTRL_REG2, PWM_TIMER4_CTRL_REG2, PWM_TIMER5_CTRL_REG2};

int hw_timer_init(int timer_id, T_TIMER_CFG timer_cfg)
{
    if((timer_id >=  AK_TIMER_NUM) || (timer_id < AK_TIMER1))
        return 0;

    if(timer_run[timer_id] == 1) return 0;
    
    if(timer_cfg.mode == MODE_PWM)
    {
        if(timer_id == AK_TIMER1)
        {
            //gpio_pin_group_cfg(ePIN_AS_PWM1);
        }
        else if(timer_id == AK_TIMER2)
        {
            //gpio_pin_group_cfg(ePIN_AS_PWM2);
        }
        else
        {
            return 0;
        }
        REG32(timer_ctrl_reg1_grp[timer_id]) = timer_cfg.timer_count.count;
        REG32(timer_ctrl_reg2_grp[timer_id]) = ((0x2<<24) | (1<<28));
    }
    else if(timer_cfg.mode == MODE_ONE_SHOT_TIMER)
    {
        REG32(timer_ctrl_reg1_grp[timer_id]) = timer_cfg.timer_count.count;
        REG32(timer_ctrl_reg2_grp[timer_id]) = TIMER_ENABLE_BIT | TIMER_FEED_BIT | (timer_cfg.mode << 24) | \
                                                    ((timer_cfg.clk_pre_div-1) << 16); 
    }
    else if(timer_cfg.mode == MODE_AUTO_RELOAD_TIMER)
    {
        REG32(timer_ctrl_reg1_grp[timer_id]) = timer_cfg.timer_count.count;
        REG32(timer_ctrl_reg2_grp[timer_id]) = TIMER_ENABLE_BIT | TIMER_FEED_BIT | (timer_cfg.mode << 24) | \
                                                    ((timer_cfg.clk_pre_div-1) << 16);
    }
    
    timer_run[timer_id] = 1;
    
    return 1;
}

void hw_timer_int_clear(int timer_id)
{
    switch(timer_id)
    {
        case AK_TIMER1:
            REG32(PWM_TIMER1_CTRL_REG2) |= TIMER_CLEAR_BIT;
            break;
        case AK_TIMER2:
            REG32(PWM_TIMER2_CTRL_REG2) |= TIMER_CLEAR_BIT;
            break;
        case AK_TIMER3:
            REG32(PWM_TIMER3_CTRL_REG2) |= TIMER_CLEAR_BIT;
            break;
        case AK_TIMER4:
            REG32(PWM_TIMER4_CTRL_REG2) |= TIMER_CLEAR_BIT;
            break;
        case AK_TIMER5:
            REG32(PWM_TIMER5_CTRL_REG2) |= TIMER_CLEAR_BIT;
            break;
        default:
            break;
    }
    
}
