/******************************************************************************
* Copyright 2016-2021 Leelen.co
* FileName: 	 gpio.h 
* Desc:
* 
* 
* Author: 	 zhuangwei
* Date: 	 2016/11/15
* Notes: 
* 
* -----------------------------------------------------------------
* Histroy: v1.0   2016/11/15, zhuangwei create this file
* 
******************************************************************************/
#ifndef _GPIO_H_     
#define _GPIO_H_    
 
 
/*------------------------------- Includes ----------------------------------*/
#include "rtthread.h"
 
/*----------------------------- Global Defines ------------------------------*/
#define GPIO_DIR_OUTPUT     0
#define GPIO_DIR_INPUT      1

#define GPIO_LEVEL_LOW      0
#define GPIO_LEVEL_HIGH     1
 
/*----------------------------- Global Typedefs -----------------------------*/
 
 
/*----------------------------- External Variables --------------------------*/
 
 
/*------------------------ Global Function Prototypes -----------------------*/
rt_err_t hw_gpio_set_dir(rt_uint8_t pin, rt_uint8_t dir);

rt_err_t hw_gpio_set_level(rt_uint8_t pin, rt_uint8_t level);
 
rt_uint8_t hw_gpio_read_value(rt_uint8_t pin);

#endif //_GPIO_H_
