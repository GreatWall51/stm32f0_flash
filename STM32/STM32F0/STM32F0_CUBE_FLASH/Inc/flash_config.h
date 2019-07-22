/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 flash_config.h
* Desc:
*
*
* Author: 	 LiuWeiQiang
* Date: 	 2019/06/03
* Notes:
*
* -----------------------------------------------------------------
* Histroy: v1.0   2019/06/03, LiuWeiQiang create this file
*
******************************************************************************/
#ifndef _FLASH_CONFIG_H_
#define _FLASH_CONFIG_H_


/*------------------------------- Includes ----------------------------------*/


/*----------------------------- Global Defines ------------------------------*/
#ifdef FLASH_BASE
#undef FLASH_BASE
#define FLASH_BASE 0x08000000U
#endif 

#ifdef FLASH_PAGE_SIZE
#undef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 1024  //byte
#endif


//flash�����ַ(���Ե�ַ)���������ת��
//����0x8006000ת����24
#define FLASH_PAGE_TO_ADDR(page)          ((uint32_t)(FLASH_BASE+(FLASH_PAGE_SIZE)*(page))) /* Base @ of page address*/
#define ADDR_TO_FLASH_PAGE(addr)          (((addr)-FLASH_BASE)/(FLASH_PAGE_SIZE))

#define FLASH_USER_START_ADDR       FLASH_PAGE_TO_ADDR(24)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR         FLASH_PAGE_TO_ADDR(34)   /* End @ of user Flash area */



/*----------------------------- Global Typedefs -----------------------------*/


/*----------------------------- External Variables --------------------------*/


/*------------------------ Global Function Prototypes -----------------------*/



#endif //_FLASH_CONFIG_H_
