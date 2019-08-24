/*
 * File      : interrupt.c
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
 * 2011-01-13     weety      first version
 * 2015-04-27     ArdaFu     Port bsp from at91sam9260 to asm9260t
 * 2016-11-7      zhuangwei  Port to AK3760,not support fiq!!!
 */

#include <rthw.h>
#include "anyka_cpu.h"
#include "interrupt.h"

extern rt_uint32_t rt_interrupt_nest;

/* exception and interrupt handler table */
struct rt_irq_desc irq_desc[INT_VECTOR_MAX];

rt_uint32_t rt_interrupt_from_thread;
rt_uint32_t rt_interrupt_to_thread;
rt_uint32_t rt_thread_switch_interrupt_flag;


/* --------------------------------------------------------------------
 *  Interrupt initialization
 * -------------------------------------------------------------------- */
rt_uint8_t real2map(rt_uint8_t id)
{
    const rt_uint8_t vector_table[] = {
        INT_VECTOR_RESERVE,/* 0 */
        INT_VECTOR_LCD,/* 1 */
        INT_VECTOR_CAMERA,/* 2 */
        INT_VECTOR_MOTION,/* 3 */
        INT_VECTOR_JPEG,/* 4 */
        INT_VECTOR_RESERVE,/* 5 */
        INT_VECTOR_RESERVE,/* 6 */
        INT_VECTOR_RESERVE,/* 7 */
        INT_VECTOR_DAC,/* 8 */
        INT_VECTOR_ADC,/* 9 */
        INT_VECTOR_L2,/* 10 */
        INT_VECTOR_RESERVE,/* 11 */
        INT_VECTOR_RESERVE,/* 12 */
        INT_VECTOR_RESERVE,/* 13 */
        INT_VECTOR_UART2,/* 14 */
        INT_VECTOR_UART1,/* 15 */
        INT_VECTOR_UART0,/* 16 */
        INT_VECTOR_SPI,/* 17 */
        INT_VECTOR_MAC,/* 18 */
        INT_VECTOR_IRDA,/* 19 */
        INT_VECTOR_RESERVE,/* 21 */
        INT_VECTOR_SDIO,/* 21 */
        INT_VECTOR_MMC,/* 22 */
        INT_VECTOR_RESERVE,/* 23 */
        INT_VECTOR_RESERVE,/* 24 */
        INT_VECTOR_USB,/* 25 */
        INT_VECTOR_USB_DMA,/* 26 */
        INT_VECTOR_INTL2,/* 27 */
    };
    
    if(id > (sizeof(vector_table)-1))
    {
        return INT_VECTOR_INVALID;
    }
    
    return vector_table[id];
}

/******************************************************************************
* Name: 	 map2mask_bit 
*
* Desc: 	 转换到中断屏蔽寄存器位
* Param: 	 
* Return: 	 中断屏蔽寄存器的位
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/8/29, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint32_t map2mask_bit(int v)
{

    const rt_uint32_t mask_table[INT_VECTOR_MAX] = {
        IRQ_MASK_LCD_BIT/*LCD*/, 
        IRQ_MASK_CAMERA_BIT/*camera*/, 
        IRQ_MASK_MOTION/*motion*/, 
        IRQ_MASK_JPEG/*jpeg*/,
        IRQ_MASK_DAC_BIT/*dac*/, 
        IRQ_MASK_SIGDELTA_ADC_BIT/*adc*/,
        IRQ_MASK_L2_BIT/*l2*/,
        IRQ_MASK_UART2_BIT/*UART2*/,
        IRQ_MASK_UART1_BIT/*UART1*/, 
        IRQ_MASK_UART0_BIT/*UART0*/,
        IRQ_MASK_SPI_BIT/*spi*/, 
        IRQ_MASK_MAC_BIT/*mac*/,
        IRQ_MASK_IRDA_BIT/*irda*/, 
        IRQ_MASK_MCI2_BIT/*mci2*/, 
        IRQ_MASK_MCI1_BIT/*mci1*/,
        IRQ_MASK_USB_BIT/*usbotg*/, 
        IRQ_MASK_USBDMA_BIT/*usbdma*/,
        IRQ_MASK_SYS_MODULE_BIT/*gpio*/, 
        IRQ_MASK_SYS_MODULE_BIT/*asicclk*/, 
        IRQ_MASK_SYS_MODULE_BIT/*wgpio*/,
        IRQ_MASK_SYS_MODULE_BIT/*timer1*/, 
        IRQ_MASK_SYS_MODULE_BIT/*timer2*/,
        IRQ_MASK_SYS_MODULE_BIT/*timer3*/, 
        IRQ_MASK_SYS_MODULE_BIT/*timer4*/,
        IRQ_MASK_SYS_MODULE_BIT/*timer5*/
        };

    return mask_table[v];
}

/******************************************************************************
* Name: 	 map2mask_bit 
*
* Desc: 	 转换到中断屏蔽寄存器位
* Param: 	 
* Return: 	 中断屏蔽寄存器的位
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/8/29, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint32_t map2mask_bit_level2(int v)
{
    const rt_uint32_t mask_table[] = {
        IRQ_MASK_GPIO_BIT/*gpio*/,
        IRQ_MASK_ASICCLK_BIT/*asicclk*/,
        IRQ_MASK_WGPIO_BIT/*wgpio*/,
        IRQ_MASK_TIMER1_BIT/*timer1*/,
        IRQ_MASK_TIMER2_BIT/*timer2*/,
        IRQ_MASK_TIMER3_BIT/*timer3*/,
        IRQ_MASK_TIMER4_BIT/*timer4*/,
        IRQ_MASK_TIMER5_BIT/*timer5*/
        };

    v -= INT_VECTOR_INTL2;
        
    return mask_table[v];
}

rt_isr_handler_t rt_hw_interrupt_handle(rt_uint32_t vector, void *param)
{
    rt_kprintf("UN-handled interrupt %d occurred!!!\n", vector);
    return RT_NULL;
}

/**
 * This function will initialize hardware interrupt
 */
void rt_hw_interrupt_init(void)
{
    register rt_uint32_t idx;

    /* init exceptions table */
    for(idx=0; idx < INT_VECTOR_MAX; idx++)
    {
        irq_desc[idx].handler = (rt_isr_handler_t)rt_hw_interrupt_handle;
        irq_desc[idx].param = RT_NULL;
#ifdef RT_USING_INTERRUPT_INFO
        rt_snprintf(irq_desc[idx].name, RT_NAME_MAX - 1, "default");
        irq_desc[idx].counter = 0;
#endif
    }

    /* init interrupt nest, and context in thread sp */
    rt_interrupt_nest = 0;
    rt_interrupt_from_thread = 0;
    rt_interrupt_to_thread = 0;
    rt_thread_switch_interrupt_flag = 0;
}


/**
 * This function will disable a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_mask(int vector)
{
    register rt_base_t level;
    rt_uint32_t bit;
    
    level = rt_hw_interrupt_disable();
    bit = map2mask_bit(vector);
    REG32(IRQINT_MASK_REG) |= bit;
    if(bit == IRQ_MASK_SYS_MODULE_BIT)
    {
        REG32(INT_SYS_MODULE_REG) &= ~(map2mask_bit_level2(vector));
    }
    else
    {
        REG32(IRQINT_MASK_REG) &= ~(bit);
    }
    rt_hw_interrupt_enable(level);
}

/**
 * This function will enable a interrupt.
 * @param vector the interrupt number
 */
void rt_hw_interrupt_umask(int vector)
{
    register rt_base_t level;
    rt_uint32_t bit;
    
    level = rt_hw_interrupt_disable();
    bit = map2mask_bit(vector);
    REG32(IRQINT_MASK_REG) |= bit;
    if(bit == IRQ_MASK_SYS_MODULE_BIT)
    {
        REG32(INT_SYS_MODULE_REG) |= map2mask_bit_level2(vector);
    }
    rt_hw_interrupt_enable(level);
}

/**
 * This function will install a interrupt service routine to a interrupt.
 * @param vector the interrupt number
 * @param handler the interrupt service routine to be installed
 * @param param the interrupt service function parameter
 * @param name the interrupt name
 * @return old handler
 */
rt_isr_handler_t rt_hw_interrupt_install(int vector, rt_isr_handler_t handler,void *param,const char *name)
{
    rt_isr_handler_t old_handler = RT_NULL;

    if(vector < INT_VECTOR_MAX)
    {
        old_handler = irq_desc[vector].handler;
        if (handler != RT_NULL)
        {
            irq_desc[vector].handler = (rt_isr_handler_t)handler;
            irq_desc[vector].param = param;
#ifdef RT_USING_INTERRUPT_INFO
            rt_snprintf(irq_desc[vector].name, RT_NAME_MAX - 1, "%s", name);
            irq_desc[vector].counter = 0;
#endif
            /* enable interrupt */
            rt_hw_interrupt_umask(vector);
        }
    }

    return old_handler;
}

/******************************************************************************
* Name: 	 rt_hw_interrupt_get_level1_status 
*
* Desc: 	 获取level1中断状态寄存器
* Param: 	 
* Return: 	 level1中断状态寄存器的值
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/5, Create this function by zhuangwei
 ******************************************************************************/
rt_uint32_t rt_hw_interrupt_get_level1_status(void)
{
    return REG32(INT_STATUS_REG);
}

/******************************************************************************
* Name: 	 rt_hw_interrupt_get_mask 
*
* Desc: 	 获取level1中断屏蔽寄存器值
* Param: 	 irq_frq - irq还是frq
* Return: 	 level1中断屏蔽寄存器值
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/7, Create this function by zhuangwei
 ******************************************************************************/
rt_uint32_t rt_hw_interrupt_get_mask(rt_uint8_t irq_frq)
{
    if(irq_frq == INT_IRQ)
    {
        return REG32(IRQINT_MASK_REG);
    }
    else
    {
        return REG32(FRQINT_MASK_REG);
    }
}

/******************************************************************************
* Name: 	 rt_hw_interrupt_level1_handle 
*
* Desc: 	 处理level1中断
* Param: 	 中断向量
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/5, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_interrupt_level1_handle(rt_uint8_t vector)
{
    rt_isr_handler_t isr_func;
    void *param;
    
    isr_func = irq_desc[vector].handler;
    param = irq_desc[vector].param;
    isr_func(vector, param);
    
#ifdef RT_USING_INTERRUPT_INFO
    irq_desc[vector].counter ++;
#endif
}

/******************************************************************************
* Name: 	 rt_hw_interrupt_level1_handle 
*
* Desc: 	 处理level2中断
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/5, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_interrupt_level2_handle(void)
{
    rt_isr_handler_t isr_func;
    void *param;
    
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_TIMER1_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_TIMER1].handler;
        param = irq_desc[INT_VECTOR_TIMER1].param;
        isr_func(INT_VECTOR_TIMER1, param);
        
#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_TIMER1].counter ++;
#endif
    }
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_TIMER2_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_TIMER2].handler;
        param = irq_desc[INT_VECTOR_TIMER2].param;
        isr_func(INT_VECTOR_TIMER2, param);
        
#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_TIMER2].counter ++;
#endif
    }
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_TIMER3_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_TIMER3].handler;
        param = irq_desc[INT_VECTOR_TIMER3].param;
        isr_func(INT_VECTOR_TIMER3, param);

#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_TIMER3].counter ++;
#endif
    }
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_TIMER4_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_TIMER4].handler;
        param = irq_desc[INT_VECTOR_TIMER4].param;
        isr_func(INT_VECTOR_TIMER4, param);

#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_TIMER4].counter ++;
#endif
    }
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_TIMER5_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_TIMER5].handler;
        param = irq_desc[INT_VECTOR_TIMER5].param;
        isr_func(INT_VECTOR_TIMER5, param);
        
#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_TIMER5].counter ++;
#endif
    }
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_GPIO_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_GPIO].handler;
        param = irq_desc[INT_VECTOR_GPIO].param;
        isr_func(INT_VECTOR_GPIO, param);
        
#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_GPIO].counter ++;
#endif
    }
    if( *( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_WGPIO_BIT )
    {
        isr_func = irq_desc[INT_VECTOR_WGPIO].handler;
        param = irq_desc[INT_VECTOR_WGPIO].param;
        isr_func(INT_VECTOR_WGPIO, param);
        
#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_WGPIO].counter ++;
#endif
    }
    if(*( volatile rt_uint32_t* )INT_SYS_MODULE_REG & INT_STATUS_ASICCLK_BIT)
    {
        isr_func = irq_desc[INT_VECTOR_ASICCLK].handler;
        param = irq_desc[INT_VECTOR_ASICCLK].param;
        isr_func(INT_VECTOR_ASICCLK, param);

#ifdef RT_USING_INTERRUPT_INFO
        irq_desc[INT_VECTOR_ASICCLK].counter ++;
#endif
    }
}

/******************************************************************************
* Name: 	 rt_hw_interrupt_get_vector 
*
* Desc: 	 获取level1中断源
* Param: 	 id - 状态寄存器的bit号
* Return: 	 获取level1中断源
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/5, Create this function by zhuangwei
 ******************************************************************************/
rt_uint8_t rt_hw_interrupt_get_vector(rt_uint8_t id)
{   
    return real2map(id);
}

#ifdef RT_USING_FINSH
void list_irq(void)
{
    int irq;
    rt_kprintf("number\tcount\tname\n");
    for (irq = 0; irq < INT_VECTOR_MAX; irq++)
    {
        if (rt_strncmp(irq_desc[irq].name, "default", sizeof("default")))
        {
            rt_kprintf("%02ld: %10ld  %s\n",
                       irq, irq_desc[irq].counter, irq_desc[irq].name);
        }
    }
}

#include <finsh.h>
FINSH_FUNCTION_EXPORT(list_irq, list system irq);

#endif
