/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 flash.h
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
#ifndef _FLASH_H_
#define _FLASH_H_


/*------------------------------- Includes ----------------------------------*/
#include <stdint.h>
#include "flash_config.h"
/*----------------------------- Global Defines ------------------------------*/
typedef enum
{
	FLASH_SUCCESS = 0,
	FLASH_PARAM_ERROR,
	FLASH_ADDR_ERROR,
	FLASH_WRITE_WORD_ERROR,
	FLASH_WRITE_HALF_WORD_ERROR,
	FLASH_WRITE_BYTE_ERROR,
	FLASH_READ_ERROR,
	FLASH_ERASE_ERROR,
} FLASH_ERROR_CODE_E;

/*----------------------------- Global Typedefs -----------------------------*/
//flash用户数据地址与扇区编号转换
//例如:0x800600转换成0
#define ADDR_TO_PAGE(addr)    (((addr)-FLASH_USER_START_ADDR)/FLASH_PAGE_SIZE)
#define PAGE_TO_ADDR(pag_no)  ((pag_no)*FLASH_PAGE_SIZE+FLASH_USER_START_ADDR)

/*----------------------------- External Variables --------------------------*/


/*------------------------ Global Function Prototypes -----------------------*/
FLASH_ERROR_CODE_E flash_write(uint32_t address, const uint8_t* pdata, uint32_t size);
uint32_t flash_read ( uint32_t address, uint8_t* pdata, uint32_t size );
FLASH_ERROR_CODE_E flash_erase( uint32_t start_addr,uint32_t end_addr);

uint32_t flash_read_page (uint8_t sec_no, uint32_t offect, uint8_t* pdata, uint32_t size );
uint32_t flash_write_page (uint8_t sec_no, uint32_t offect,const uint8_t* pdata, uint32_t size );
FLASH_ERROR_CODE_E flash_erase_page ( uint32_t start_page, uint16_t page_cnt );

#endif //_FLASH_H_
