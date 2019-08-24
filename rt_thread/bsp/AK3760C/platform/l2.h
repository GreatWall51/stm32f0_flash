/******************************************************************************
* Copyright 2016-2021 Leelen.co
* FileName: 	 l2.h 
* Desc:
* 
* 
* Author: 	 zhuangwei
* Date: 	 2016/11/17
* Notes: 
* 
* -----------------------------------------------------------------
* Histroy: v1.0   2016/11/17, zhuangwei create this file
* 
******************************************************************************/
#ifndef _L2_H_     
#define _L2_H_    
 
 
/*------------------------------- Includes ----------------------------------*/
#include <rtthread.h> 
 
/*----------------------------- Global Defines ------------------------------*/
#define BUF2MEM             0
#define MEM2BUF             1 

#define L2_STATUS_IDLE      0
#define L2_STATUS_USED      1

#define BUF_NULL            0xff

#define L2_COM_BUF_SIZE     512
/*----------------------------- Global Typedefs -----------------------------*/
enum
{
    L2_DEV_USB_EP1 = 0,           ///< usb ep1        0
    L2_DEV_USB_EP2 = 1,           ///< usb ep2        1
    L2_DEV_MCI0 = 4,              ///< mci1           4
    L2_DEV_MCI1 = 5,              ///< mci2           5
    L2_DEV_SPI1_RX = 7,           ///< spi1 rx        7
    L2_DEV_SPI1_TX = 8,           ///< spi1 tx        8
    L2_DEV_DAC = 9,               ///< dac            9
    L2_DEV_UART3_RX = 12,         ///< uart3 rx       12
    L2_DEV_UART3_TX = 13,         ///< uart3 tx       13
    L2_DEV_ADC = 14,              ///< adc            14
    L2_DEV_RESERVE = 15           ///< reserve
}; 
 
/*----------------------------- External Variables --------------------------*/
 
 
/*------------------------ Global Function Prototypes -----------------------*/
void rt_hw_l2_init(void);

rt_uint32_t rt_hw_l2_get_addr(rt_uint8_t buf_id);
 
void rt_hw_l2_clr_status(rt_uint8_t buf_id);

rt_uint8_t rt_hw_l2_cpu(rt_uint32_t ram_addr, rt_uint8_t buf_id, rt_uint32_t buf_offset, \
    rt_uint32_t tran_byte, rt_uint8_t tran_dir);

rt_uint8_t rt_hw_l2_alloc(rt_uint8_t dev);


#endif //_L2_H_
