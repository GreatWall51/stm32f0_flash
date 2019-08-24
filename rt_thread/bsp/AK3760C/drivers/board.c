/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2009 RT-Thread Develop Team
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
 * 2011-01-13     weety      first version
 * 2015-05-02     ArdaFu     Port from AT91SAM9260 BSP
 */

#include <rtthread.h>
#include <rthw.h>
#include <timer.h>
#include "anyka_cpu.h"
#include "board.h"
#include <mmu.h>
#include "interrupt.h"
#include "system_clock.h"
#include "l2.h"

#define RAM_SIZE    (16 << 20)

extern void rt_hw_interrupt_init(void);
extern void rt_hw_uart_init(void);

static struct mem_desc hw_mem_desc[] =
{
    { 0x00000000, 0xFFFFFFFF, 0x00000000, RW_NCNB },/* None cached for 4G memory */
//  visual start, visual end, phy start , props
    { 0x00000000, 0x000FFFFF, 0x00000000, RW_NCNB },  /* on-chip ROM */
    { 0x08000000, 0x080FFFFF, 0x08000000, RW_NCNB },  /* system control */
    { 0x20000000, 0x203FFFFF, 0x20000000, RW_NCNB },  /* SFR !! */
    { 0x30000000, 0x30000000+RAM_SIZE-1, 0x30000000, RW_CB },  /* DRAM */
    { 0x48000000, 0x480FFFFF, 0x48000000, RW_NCNB },  /* L2 */
    { 0x70000000, 0x700FFFFF, 0x70000000, RW_NCNB },  /* USB */
};

/**
 * This function will handle rtos timer
 */
static void rt_systick_handler(int vector, void *param)
{
    hw_timer_int_clear(AK_TIMER1);
    rt_tick_increase();  
}

/**
 * This function will init pit for system ticks
 */
static void rt_hw_timer_init()
{
    T_TIMER_CFG cfg;
    
    cfg.mode = MODE_AUTO_RELOAD_TIMER;
    cfg.clk_pre_div = TIMER_FREQ/BASE_TIMER_CLK;
    cfg.timer_count.count = BASE_TIMER_CLK/RT_TICK_PER_SECOND;
    
    hw_timer_init(AK_TIMER1, cfg);
    /* install interrupt handler */
    rt_hw_interrupt_install(INT_VECTOR_TIMER1, rt_systick_handler, RT_NULL, "SysTick");
}

/******************************************************************************
* Name: 	 rt_hw_dma_init 
*
* Desc: 	 设置dma优先级
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2015/12/15, Create this function by zhuangwei
 ******************************************************************************/
static void rt_hw_dma_init(void)
{
    //set l2 dma priority to 3rd, only lower than lcd/camera
    REG32(DMA_PRIORITY_CTRL_REG1) = 0xffffffff;
    REG32(DMA_PRIORITY_CTRL_REG2) = 0xff03ffff;

    //set arm priority to lowest
    REG32(AHB_PRIORITY_CTRL_REG) = 0xff00;
} 

/**
 * This function will init ak3760c board
 */
void rt_hw_board_init(void)
{
    /* initialize mmu */
    rt_hw_mmu_init(hw_mem_desc, sizeof(hw_mem_desc)/sizeof(hw_mem_desc[0]));
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();

    /* initialize the system clock */
    rt_hw_clock_init();

    /* initialize dma */
    rt_hw_dma_init();
    
    /* initialize l2 buf */
    rt_hw_l2_init();
    
    /* initialize uart */
    rt_hw_uart_init();
#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* initialize timer0 */
    rt_hw_timer_init();
}
