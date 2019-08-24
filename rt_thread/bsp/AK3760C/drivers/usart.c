/*
 * File      : usart.c
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
 * 2011-01-13     weety       first version
 * 2013-07-21     weety       using serial component
 * 2015-05-02     ArdaFu      Port from AT91SAM9260 BSP
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include "interrupt.h"
#include "anyka_cpu.h"
#include "system_clock.h"
#include "l2.h"
#include "led.h"

//#include "uart.h"

//REG_UART_CONFIG_1
#define RX_STA_CLR                      (1 << 29)
#define TX_STA_CLR                      (1 << 28)
#define RX_TIMEOUT_EN                   (1 << 23)
#define BAUD_RATE_ADJ_EN                (1 << 22)
#define UART_INTERFACE_EN               (1 << 21)

//REG_UART_CONFIG_2
#define TX_TH_INT_STA                   (1 << 31)
#define RX_TH_INT_STA                   (1 << 30)
#define TX_TH_INT_EN                    (1 << 29)
#define RX_TH_INT_EN                    (1 << 28)
#define TX_END_INT_EN                   (1 << 27)
#define RX_BUF_FULL_INT_EN              (1 << 26)
#define TX_BUF_EMP_INT_EN               (1 << 24)
#define RX_ERROR_INT_EN                 (1 << 23)
#define TIMEOUT_INT_EN                  (1 << 22)
#define RX_READY_INT_EN                 (1 << 21)
#define TX_END                          (1 << 19)
#define RX_TIMEROUT_STA                 (1 << 18)
#define RX_READY                        (1 << 17)
#define TX_BYT_CNT_VLD                  (1 << 16)
#define TX_BYT_CNT                      (4)
#define RX_ERROR                        (1 << 3)
#define RX_TIMEROUT                     (1 << 2)
#define RX_BUF_FULL                     (1 << 1)
#define TX_FIFO_EMPTY                   (1 << 0)

#define DRV_REG_OPS (RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM)
#define UART_GET_BASE_ADDR(uart_id) \
    (UART0_BASE_ADDR+(rt_uint32_t)(uart_id)*0x1000)
    
#define USART_STATUS_RX                 0
#define USART_STATUS_TX                 1

#define UART_RX_FIFO_SIZE               (128)    //1//128 bytes
#define UART2_RX_FIFO_SIZE              (512)    //1//128 bytes

enum
{
    USART0=0,
    USART1,
    USART2
};

typedef struct
{
    rt_uint8_t uart_id;
    int irq;
    rt_uint32_t rx_offset;
}hw_uart_t;

#if defined(RT_USING_UART0)
static struct rt_serial_device serial0;
hw_uart_t uart0 =
{
    USART0,
    INT_VECTOR_UART0,
    0
};
#endif

#if defined(RT_USING_UART1)
static struct rt_serial_device serial1;
hw_uart_t uart1 =
{
    USART1,
    INT_VECTOR_UART1,
    0
};
#endif

#if defined(RT_USING_UART2)
static struct rt_serial_device serial2;
hw_uart_t uart2 =
{
    USART2,
    INT_VECTOR_UART2,
    0
};
#endif


static void hw_usart_set_rx_th_int(rt_uint8_t uart_id, rt_uint8_t irq_byte);


/******************************************************************************
* Name: 	 hw_usart_clear_status 
*
* Desc: 	 清除串口发送或接收状态
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static void hw_usart_clear_status(rt_uint8_t uart_id, rt_uint8_t txrx_status)
{
    rt_uint32_t base_addr;
    rt_uint32_t reg_value;

    base_addr = UART_GET_BASE_ADDR(uart_id);
    reg_value = REG32(base_addr+UART_CFG_REG1);
    if (txrx_status)
        reg_value |= (1<<28);
    else
        reg_value |= (1<<29);
    REG32(base_addr+UART_CFG_REG1) = reg_value;  
}

/******************************************************************************
* Name: 	 hw_usart_tran_cpu_l2 
*
* Desc: 	 cpu写入或读取l2数据
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_uint8_t hw_usart_tran_cpu_l2(rt_uint32_t ram_addr, rt_uint8_t uart_id, rt_uint32_t tran_byte, rt_uint8_t tran_dir)
{
    rt_uint32_t buf_id;
    
    if(uart_id >= USART2)
    {
        return RT_FALSE;
    }
    
    if (MEM2BUF == tran_dir)
        buf_id = 6 + uart_id*2;
    else
        buf_id = 6 + uart_id*2 + 1;

    return rt_hw_l2_cpu(ram_addr, buf_id, 0, tran_byte, tran_dir);
}

/******************************************************************************
* Name: 	 hw_usart_write_cpu 
*
* Desc: 	 串口轮询发送
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static rt_uint8_t hw_usart_write_cpu(rt_uint8_t uart_id, rt_uint8_t *chr, rt_uint32_t byte_nbr)
{
    rt_uint32_t baseAddress;
    rt_uint32_t status;
    rt_uint32_t reg_value;
    rt_uint8_t  buf_id = 6+(uart_id<<1);
        
    if (byte_nbr >= 64)
    {
        return RT_FALSE;
    }
    
    //get base address
    baseAddress = UART_GET_BASE_ADDR(uart_id);

    //clear tx status
    hw_usart_clear_status(uart_id, USART_STATUS_TX); //clear tx status
    
    //transport datas to l2 buf
    rt_hw_l2_clr_status(buf_id);
    if (!hw_usart_tran_cpu_l2((rt_uint32_t)chr, uart_id,  byte_nbr, MEM2BUF))
    {
        return RT_FALSE;
    }

    //start to trans
    reg_value = REG32(baseAddress+UART_CFG_REG2);
    reg_value &= 0x3fe00000;
    reg_value |= (byte_nbr<<4); 
    reg_value |= (1<<16);
    REG32(baseAddress + UART_CFG_REG2) = reg_value;
    
    //wait for tx end
    while (1)
    {
        status = REG32(baseAddress + UART_CFG_REG2);
      
        if (status & TX_END)
        { 
            break;
        }
    }
    
    return RT_TRUE;
}

/******************************************************************************
* Name: 	 hw_usart_l2_cpy 
*
* Desc: 	 从L2读取数据
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static void hw_usart_l2_cpy(rt_uint8_t *data, rt_uint32_t base_addr, rt_uint32_t offset, rt_uint32_t len)
{
    rt_uint32_t offset_adj = offset & 0xfffffffc;
    rt_uint32_t tmp, i;
    rt_uint32_t cnt = 0;

    i = offset - offset_adj;
    tmp = REG32(base_addr + offset_adj);
    while(cnt < len)
    {
        data[cnt++] = (tmp >> (i*8)) & 0xff;

        i = (i+1)&0x3;
        if(i == 0)
        {
            offset_adj += 4;
            tmp = REG32(base_addr + offset_adj);
        }
    }
}

/**
 * This function will handle serial
 */
void rt_hw_usart_handler(int vector, void *param)
{
    rt_uint32_t baseAddress,reg;
    rt_uint32_t status;
    hw_uart_t *uart;
    rt_device_t dev = (rt_device_t)param;
    uart = (hw_uart_t *)dev->user_data;
    
    baseAddress = UART_GET_BASE_ADDR(uart->uart_id);
    status = REG32(baseAddress+UART_CFG_REG2);

    if(status & TX_END_INT_EN)
    {
        reg = REG32(baseAddress+UART_CFG_REG2);
        reg &= ~TX_END_INT_EN;
        REG32(baseAddress+UART_CFG_REG2) = reg; 
        rt_hw_serial_isr((struct rt_serial_device *)dev, RT_SERIAL_EVENT_TX_DONE);
    }
    else if(status & RX_TH_INT_EN)
    {
        reg = REG32(baseAddress+UART_CFG_REG2);
        reg |= RX_TH_INT_EN;//clear
        REG32(baseAddress+UART_CFG_REG2) = reg;
        rt_hw_serial_isr((struct rt_serial_device *)dev, RT_SERIAL_EVENT_RX_IND);
    }
    else if(status & TX_BUF_EMP_INT_EN)
    {
        
    }
}

static void hw_usart_config(rt_uint8_t uart_id,int baudRate, int dataBits, int stopBits,int parity)
{
    rt_uint32_t reg_value;
    rt_uint32_t base_addr;
    rt_uint32_t br_value,sys_clk;
    
    RT_ASSERT(dataBits == DATA_BITS_8);
    RT_ASSERT(stopBits == STOP_BITS_1);

    base_addr = UART_GET_BASE_ADDR(uart_id);
    
    //set baudrate
    sys_clk = hw_clock_get_asic_mhz()*1000000;
    br_value = ((sys_clk<<2)+baudRate)/(baudRate<<1);

    if(br_value%2)
		reg_value = ((br_value/2-1)&0xffff)|UART_INTERFACE_EN|BAUD_RATE_ADJ_EN|RX_TIMEOUT_EN|TX_STA_CLR|RX_STA_CLR;
	else
		reg_value = ((br_value/2-1)&0xffff)|UART_INTERFACE_EN|RX_TIMEOUT_EN|TX_STA_CLR|RX_STA_CLR;	
    REG32(base_addr+UART_CFG_REG1) = reg_value;   
    
    //set parity
    reg_value = REG32(base_addr+UART_CFG_REG1);
    if(parity == PARITY_NONE)
    {       
        reg_value &= ~(1 << 26);         
    } 
    else
    {
        reg_value |= (1 << 26); 
        if(parity == PARITY_ODD)
        {
            reg_value &= ~(1 << 25);
        }
        else if(parity == PARITY_EVEN)
        {
             reg_value |= (1 << 25);
        }
    }
    REG32(base_addr+UART_CFG_REG1) = reg_value; 
}

/**
* UART device in RT-Thread
*/
static rt_err_t hw_usart_configure(struct rt_serial_device *serial,
                                    struct serial_configure *cfg)
{
    hw_uart_t *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);
    uart = (hw_uart_t *)serial->parent.user_data;
    
    hw_usart_config(uart->uart_id, cfg->baud_rate, cfg->data_bits, 
                  cfg->stop_bits, cfg->parity);

    return RT_EOK;
}

static rt_err_t hw_usart_control(struct rt_serial_device *serial,
                                  int cmd, void *arg)
{
    hw_uart_t* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (hw_uart_t *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        rt_hw_interrupt_mask(uart->irq);
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        rt_hw_interrupt_umask(uart->irq);
        break;
    }

    return RT_EOK;
}

static int hw_usart_putc(struct rt_serial_device *serial, char c)
{
    hw_uart_t *uart = serial->parent.user_data;

    if(hw_usart_write_cpu(uart->uart_id, (rt_uint8_t *)&c, 1) == RT_TRUE)
        return 1;
    else
        return 0;
}

static int hw_usart_getc(struct rt_serial_device *serial)
{
    int ret = -1;
    rt_uint8_t ch;
    hw_uart_t *uart = serial->parent.user_data;
    rt_uint32_t baseAddress = UART_GET_BASE_ADDR(uart->uart_id);
    rt_uint32_t nbr_to_read,bufferAddress;
    rt_uint8_t  buf_id = 7+(uart->uart_id<<1);
    
    bufferAddress = rt_hw_l2_get_addr(buf_id);
    
    nbr_to_read = (REG32(baseAddress+UART_DATA_CFG)>>13)&0x7f;
    
    /* check fifo empty or not */
    if (nbr_to_read != uart->rx_offset)
    {
        hw_usart_l2_cpy(&ch, bufferAddress, uart->rx_offset, 1);
        uart->rx_offset++;
        uart->rx_offset %= UART_RX_FIFO_SIZE;
        
        ret = ch & 0xff;
    }
    
    return ret;
}

static const struct rt_uart_ops hw_usart_ops =
{
    hw_usart_configure,
    hw_usart_control,
    hw_usart_putc,
    hw_usart_getc,
};

static void hw_usart_set_rx_th_int(rt_uint8_t uart_id, rt_uint8_t irq_byte)
{
    rt_uint32_t base_addr, reg_value;
    
    base_addr = UART_GET_BASE_ADDR(uart_id);
    
    //umask rx th int
    reg_value = REG32(base_addr+UART_CFG_REG2);
    reg_value |= RX_TH_INT_EN;
	REG32(base_addr+UART_CFG_REG2) = reg_value;
        
    /* set threshold to 64bytes or 256B(uart2) */
    reg_value = REG32(base_addr+UART_DATA_CFG);
    reg_value &= ~(0x7FU << 25);
    REG32(base_addr+UART_DATA_CFG) = reg_value;    

    reg_value = REG32(base_addr+UART_RX_THREINT);
    reg_value &= ~0x1f;
    reg_value |= (irq_byte-1);     
    REG32(base_addr+UART_RX_THREINT) = reg_value;
      
    hw_usart_clear_status(uart_id, USART_STATUS_RX);   
    rt_hw_l2_clr_status(6+(uart_id<<1));

    /* clear count */
    reg_value = REG32(base_addr+UART_RX_THREINT);
    reg_value &= ~(1<<5);
    REG32(base_addr+UART_RX_THREINT) = reg_value;
    
	REG32(base_addr + UART_TIME_OUT) &= ~(0x1<<11);
}

/******************************************************************************
* Name: 	 hw_usart_interrput_init 
*
* Desc: 	 初始化中断
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
static void hw_usart_interrput_init(rt_uint8_t uart_id)
{
    rt_uint32_t base_addr;
    
    base_addr = UART_GET_BASE_ADDR(uart_id);
	REG32(base_addr+UART_CFG_REG2) = 0;	//mask all interrupt
}
/******************************************************************************
* Name: 	 hw_usart_clock_init 
*
* Desc: 	 使能串口模块时钟
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
void hw_usart_clock_init(rt_uint8_t uart_id)
{
    rt_uint8_t bit = 6;
    bit = (rt_uint8_t)uart_id + bit;

    REG32(CLOCK_CTRL_REG) &= ~(1<<bit);
}

/******************************************************************************
* Name: 	 hw_usart_gpio_init 
*
* Desc: 	 设置串口引脚复用
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
//USART0 PINS TX=GPIO2, RX=GPIO14
//USART1 PINS TX=GPIO15, RX=GPIO19
//USART2 PINS TX=GPIO16, RX=GPIO20
void hw_usart_gpio_init(void)
{
    rt_uint32_t reg_value1 = 0;
    rt_uint32_t reg_value2 = 0;

    reg_value1 = REG32(GPIO_SHAREPIN_CONTROL1);
    reg_value2 = REG32(GPIO_SHAREPIN_CONTROL2);
    
#ifdef RT_USING_UART0
    reg_value1 &= ~(3<<23);//GPIO14
    reg_value1 |= (2<<23);//GPIO14
    reg_value1 &= ~(1<<2);//GPIO2
    reg_value1 |= (1<<2);//GPIO2
    reg_value1 &= ~(1<<4);//GPIO2
#endif

#ifdef RT_USING_UART1
    reg_value1 &= ~(1<<25);//GPIO15
    reg_value1 |= (1<<25);//GPIO15
    reg_value2 &= ~(3<<2);//GPIO19
    reg_value2 |= (2<<2);//GPIO19
#endif

#ifdef RT_USING_UART2
    reg_value1 &= ~(3<<27);//GPIO16
    reg_value1 |= (1<<27);//GPIO16
    reg_value2 &= ~(3<<4);//GPIO20
    reg_value2 |= (2<<4);//GPIO20
#endif

    REG32(GPIO_SHAREPIN_CONTROL1) = reg_value1;
    REG32(GPIO_SHAREPIN_CONTROL2) = reg_value2;
}

/******************************************************************************
* Name: 	 hw_serial_config_set_default 
*
* Desc: 	 初始化串口配置
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
void hw_serial_config_set_default(struct rt_serial_device* serial)
{
    serial->ops = &hw_usart_ops;

    serial->config.baud_rate = BAUD_RATE_115200;
    serial->config.bit_order = BIT_ORDER_LSB;
    serial->config.data_bits = DATA_BITS_8;
    serial->config.parity    = PARITY_NONE;
    serial->config.stop_bits = STOP_BITS_1;
    serial->config.invert    = NRZ_NORMAL;
    serial->config.bufsz     = RT_SERIAL_RB_BUFSZ;
}

/******************************************************************************
* Name: 	 rt_hw_uart_init 
*
* Desc: 	 初始化串口
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
int rt_hw_uart_init(void)
{
    hw_usart_gpio_init();

#if defined(RT_USING_UART0)
    hw_usart_clock_init(USART0);
    hw_serial_config_set_default(&serial0);
    hw_usart_interrput_init(USART0);
    hw_usart_set_rx_th_int(USART0, 1);
    
    /* register uart device */
    rt_hw_serial_register(&serial0, "uart0", DRV_REG_OPS, &uart0);
    rt_hw_interrupt_install(uart0.irq, rt_hw_usart_handler,
                            (void *)&(serial0.parent), "UART0");
    rt_hw_interrupt_umask(uart0.irq);
#endif

#if defined(RT_USING_UART1)
    hw_usart_clock_init(USART1);
    hw_serial_config_set_default(&serial1);
    hw_usart_interrput_init(USART1);
    
    /* register uart device */
    rt_hw_serial_register(&serial1, "uart1",  DRV_REG_OPS, &uart1);
    rt_hw_interrupt_install(uart1.irq, rt_hw_usart_handler,
                            (void *)&(serial1.parent), "UART1");
    rt_hw_interrupt_umask(uart1.irq);
#endif

#if defined(RT_USING_UART2)
    hw_usart_clock_init(USART2);
    hw_serial_config_set_default(&serial2);
    hw_usart_interrput_init(USART2);
    
    /* register uart device */
    rt_hw_serial_register(&serial2, "uart2", DRV_REG_OPS, &uart2);
    rt_hw_interrupt_install(uart2.irq, rt_hw_usart_handler,
                            (void *)&(serial2.parent), "UART2");
    rt_hw_interrupt_umask(uart2.irq);
#endif

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_uart_init);
