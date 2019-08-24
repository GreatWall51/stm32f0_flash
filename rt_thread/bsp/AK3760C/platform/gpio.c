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
 
#include "anyka_cpu.h"
#include <rthw.h>
#include "gpio.h"

#define GPIO_MAX 86

/******************************************************************************
* Name: 	 hw_gpio_check 
*
* Desc: 	 检查pin值是否合法
* Param: 	 
* Return: 	 RT_EOK - 合法, >0-不合法
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/15, Create this function by zhuangwei
 ******************************************************************************/
static rt_err_t hw_gpio_check(rt_uint8_t pin)
{
    if((pin>86)|((pin>=56)&&(pin<=58))|(pin == 78)|(pin == 81))
        return -RT_ERROR;
    else
        return RT_EOK;
}

/******************************************************************************
* Name: 	 hw_gpio_set_dir 
*
* Desc: 	 设置GPIO方向
* Param: 	 
* Return: 	 参考rt_err_t
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_err_t hw_gpio_set_dir(rt_uint8_t pin, rt_uint8_t dir)
{
    rt_uint32_t val;
    rt_uint8_t offset, shift;
    rt_base_t level;
    
    if(hw_gpio_check(pin) < 0) return -RT_ERROR;
    
    offset = pin/32;
    shift = pin%32;
    
    level = rt_hw_interrupt_disable();
    val = REG32(GPIO_DIR_REG1+offset*4);
    if(dir == GPIO_DIR_OUTPUT)
    {
        val &= ~(1<<shift);
    }
    else
    {
        val |= 1<<shift;
    }
    REG32(GPIO_DIR_REG1+offset*4) = val;
    rt_hw_interrupt_enable(level);
    
    return RT_EOK;
}

/******************************************************************************
* Name: 	 hw_gpio_set_level 
*
* Desc: 	 设置GPIO输出值
* Param: 	 
* Return: 	 参考rt_err_t
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_err_t hw_gpio_set_level(rt_uint8_t pin, rt_uint8_t level)
{
    rt_uint32_t val;
    rt_uint8_t offset, shift;
    rt_base_t int_level;
    
    if(hw_gpio_check(pin) < 0) return -RT_ERROR;
    
    offset = pin/32;
    shift = pin%32;
    
    int_level = rt_hw_interrupt_disable();
    val = REG32(GPIO_OUT_REG1+offset*4);
    if(level == GPIO_LEVEL_LOW)
    {
        val &= ~(1<<shift);
    }
    else
    {
        val |= 1<<shift;
    }
    REG32(GPIO_OUT_REG1+offset*4) = val;
    rt_hw_interrupt_enable(int_level);
    
    return RT_EOK;
}

/******************************************************************************
* Name: 	 hw_gpio_read_value 
*
* Desc: 	 读取gpio输入
* Param: 	 
* Return: 	 输入电平
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_uint8_t hw_gpio_read_value(rt_uint8_t pin)
{
    rt_uint8_t offset, shift;
    rt_uint32_t val;
    
    offset = pin/32;
    shift = pin%32;
    
    val = REG32(GPIO_IN_REG1+offset*4);
    if(val & (1<<shift))
        return GPIO_LEVEL_HIGH;
    else
        return GPIO_LEVEL_LOW;
}
