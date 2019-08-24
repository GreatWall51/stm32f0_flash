/*
 * File      : clock.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
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
 * 2011-01-13     weety       first version
 */

#include <rtthread.h>
#include "anyka_cpu.h"
#include "system_clock.h"

#define AK_FREQ_MIN             225
#define AK_FREQ_MAX             400

#define CPU_3X_ENABLE           (1 << 30)
#define MARK_3X_CFG             (1 << 28)
#define CPU_2X_CFG              (1 << 15)

/******************************************************************************
* Name: 	 is_cpu_3x 
*
* Desc: 	 是否cpu频率是3倍ASIC频率
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint8_t is_cpu_3x(void)
{
    return (REG32(CLOCK3X_CTRL_REG) & MARK_3X_CFG) ? (RT_TRUE) : (RT_FALSE);
}


/******************************************************************************
* Name: 	 is_cpu_2x 
*
* Desc: 	 是否cpu频率是2倍ASIC频率
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint8_t is_cpu_2x(void)
{
    if (RT_TRUE == is_cpu_3x()){
        return RT_FALSE;
    }
    
    return (REG32(CLOCK_DIV_REG) & CPU_2X_CFG) ? (RT_TRUE) : (RT_FALSE);
}

/******************************************************************************
* Name: 	 hw_clock_set_cpu_2x 
*
* Desc: 	 设置CPU时钟为ASIC时钟的2倍
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_uint8_t hw_clock_set_cpu_2x(rt_uint8_t enable)
{
    rt_uint8_t runing_at_2x = RT_FALSE;

    if (RT_TRUE == is_cpu_3x())
    {
        return RT_FALSE;
    }
        
    runing_at_2x = is_cpu_2x();

    if (enable) //set cpu 2x
    {
        if (RT_FALSE == runing_at_2x)
        {
            REG32(CLOCK_DIV_REG) |= CPU_2X_CFG;
        }
    }
    else //exit cpu 2x
    {                 
        if (RT_TRUE == runing_at_2x)
        {
            REG32(CLOCK_DIV_REG) &= ~CPU_2X_CFG;
        }
    }

    return RT_TRUE;
}

/******************************************************************************
* Name: 	 hw_clock_set_asic_half_pll 
*
* Desc: 	 设置ASIC时钟为PLL时钟一半
* Param: 	 mhz-CLK168M时钟频率，225-400，必须被5整除
* Return: 	 参考rt_errno
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
void hw_clock_set_asic_half_pll(void)
{
    rt_uint32_t val;
    
    val = REG32(CLOCK_DIV_REG);
    val &= (~(0xf<<17)); //disable CLK168M_DIV
    val &= ~(0x1f << 21); //disable ASIC_PRE_DIV
    val &= (~(0x7<<6)); //ASIC_DIV=2
    val |= (1<<14);//enable ASIC CLK
    REG32(CLOCK_DIV_REG) = val;
}

/******************************************************************************
* Name: 	 hw_clock_set_pll_clock 
*
* Desc: 	 设置PLL
* Param: 	 mhz-CLK168M时钟频率，225-400，必须被5整除
* Return: 	 参考rt_errno
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_err_t hw_clock_set_pll_clock(rt_uint32_t mhz)
{
    rt_uint8_t pll_sel;
    rt_uint32_t val;
    
    if((mhz<AK_FREQ_MIN) | (mhz>AK_FREQ_MAX))
        return -RT_ERROR;
    
    pll_sel = (mhz-AK_FREQ_MIN)/5;
    val = REG32(CLOCK_DIV_REG);
    val &= ~(0x1F);//clear pll_sel
    val |= pll_sel;
    val |= (1<<12);//enable pll
    REG32(CLOCK_DIV_REG) = val;
    while(((REG32(CLOCK_DIV_REG))&(1<<12))==1);
    
    return RT_EOK;
}

/******************************************************************************
* Name: 	 hw_clock_get_pll_mhz 
*
* Desc: 	 获取PLL频率
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint32_t hw_clock_get_pll_mhz(void)
{
    rt_uint32_t ratio;
    rt_uint32_t pll_sel;
    rt_uint32_t ret;
    
    ratio = REG32(CLOCK_DIV_REG);
    pll_sel = (ratio & 0x3f);
    
    ret = pll_sel*5 + AK_FREQ_MIN;

    return ret;        
}

/******************************************************************************
* Name: 	 hw_clock_get_clk168_mhz 
*
* Desc: 	 获取CLK168频率
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint32_t hw_clock_get_clk168_mhz(void)
{
    rt_uint32_t ratio;
    rt_uint8_t clk168_div = 0;
    rt_uint32_t clk168_mhz;

    ratio = REG32(CLOCK_DIV_REG);
    clk168_div = (ratio >> 17) & 0xF;

    clk168_mhz = hw_clock_get_pll_mhz() / (clk168_div+1);

    return clk168_mhz;
}

/******************************************************************************
* Name: 	 hw_clock_get_asic_mhz 
*
* Desc: 	 获取ASIC频率
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_uint32_t hw_clock_get_asic_mhz(void)
{
    rt_uint32_t ratio;
    rt_uint32_t pre_div = 0, asic_div = 0;
    rt_uint32_t asic_mhz;
    rt_uint32_t clk168_mhz;

    //get CLK168 freq
    clk168_mhz = hw_clock_get_clk168_mhz();

    //just return clk168 if bit[31] is set
    if(REG32(CLOCK3X_CTRL_REG) & (1U<<31))
    {
        return clk168_mhz;
    }

    //get ASIC_DIV
    ratio = REG32(CLOCK_DIV_REG);

    //get asic div
    asic_div = (ratio >> 6) & 0x07;
    if(asic_div == 0) asic_div = 1;

    //get pre div
    if(ratio & (1<<21)) pre_div = (ratio >> 22) & 0xF;

    if(is_cpu_3x())         //cpu 3X mode
    {
        asic_mhz = clk168_mhz / 3;
    }
    else                        //not special clock mode
    {
        asic_mhz = (clk168_mhz >> asic_div) / (pre_div + 1);
    }

    return asic_mhz;
}

/******************************************************************************
* Name: 	 rt_hw_clock_init 
*
* Desc: 	 时钟初始化
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_clock_init(void)
{
	hw_clock_set_pll_clock(280);
    hw_clock_set_asic_half_pll();
    hw_clock_set_cpu_2x(RT_TRUE);
}

