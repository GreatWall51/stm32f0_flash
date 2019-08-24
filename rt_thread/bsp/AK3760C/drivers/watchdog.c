/******************************************************************************
* Copyright 2016-2021 Leelen.co
* FileName: 	 watchdog.c 
* Desc:
* 
* 
* Author: 	 zhuangwei
* Date: 	 2016/12/01
* Notes: 
* 
* -----------------------------------------------------------------
* Histroy: v1.0   2016/12/01, zhuangwei create this file
* 
******************************************************************************/
 
 
/*------------------------------- Includes ----------------------------------*/
#include <timer.h>
#include <rthw.h>
#include "anyka_cpu.h"
#include "interrupt.h"
#include "watchdog.h"
 
/*------------------- Global Definitions and Declarations -------------------*/
#define WATCHDOG_FEED_BIT      (1<<29) 
 
/*---------------------- Constant / Macro Definitions -----------------------*/
 
 
/*----------------------- Type Declarations ---------------------------------*/
 
 
/*----------------------- Variable Declarations -----------------------------*/
 
 
/*----------------------- Function Prototype --------------------------------*/
 
 
/*----------------------- Function Implement --------------------------------*/
/******************************************************************************
* Name: 	 rt_watchdog_handler 
*
* Desc: 	 看门狗中断
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/12/1, Create this function by zhuangwei
 ******************************************************************************/
static void rt_watchdog_handler(int vector, void *param)
{
    
}

/******************************************************************************
* Name: 	 rt_hw_watchdog_init 
*
* Desc: 	 看门狗初始化
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/12/1, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_watchdog_init(rt_uint8_t ms)
{
    T_TIMER_CFG cfg;
    
    cfg.mode = MODE_ONE_SHOT_TIMER;
    cfg.clk_pre_div = TIMER_FREQ/BASE_TIMER_CLK;
    cfg.timer_count.count = ms * 1000;
    
    hw_timer_init(AK_TIMER4, cfg);
    /* install interrupt handler */
    rt_hw_interrupt_install(INT_VECTOR_TIMER4, rt_watchdog_handler, RT_NULL, "Watchdog");
    REG32(CPU_TIMER_REG) = (1<<3);
} 

/******************************************************************************
* Name: 	 rt_hw_watchdog_init 
*
* Desc: 	 喂狗
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/12/1, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_watchdog_feed(void)
{
    REG32(PWM_TIMER4_CTRL_REG2) |= WATCHDOG_FEED_BIT;
}

/*---------------------------------------------------------------------------*/

