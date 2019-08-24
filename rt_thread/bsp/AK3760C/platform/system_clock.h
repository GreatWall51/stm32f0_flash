/******************************************************************************
* Copyright 2016-2021 Leelen.co
* FileName: 	 system_clock.h 
* Desc:
* 
* 
* Author: 	 zhuangwei
* Date: 	 2016/11/16
* Notes: 
* 
* -----------------------------------------------------------------
* Histroy: v1.0   2016/11/16, zhuangwei create this file
* 
******************************************************************************/
#ifndef _SYSTEM_CLOCK_H_     
#define _SYSTEM_CLOCK_H_    
 
 
/*------------------------------- Includes ----------------------------------*/
#include <rtthread.h> 
 
/*----------------------------- Global Defines ------------------------------*/
 
 
/*----------------------------- Global Typedefs -----------------------------*/
 
 
/*----------------------------- External Variables --------------------------*/
 
 
/*------------------------ Global Function Prototypes -----------------------*/
void rt_hw_clock_init(void); 

rt_uint32_t hw_clock_get_asic_mhz(void);

#endif //_SYSTEM_CLOCK_H_
