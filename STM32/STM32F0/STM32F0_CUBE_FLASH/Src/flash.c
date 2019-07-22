/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 flash.c
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


/*------------------------------- Includes ----------------------------------*/
#include "stm32f0xx_hal.h"
#include "flash.h"
#include "debug.h"

/*------------------- Global Definitions and Declarations -------------------*/
#ifndef FLASH_BASE
#define FLASH_BASE 0x08000000U
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE 1024  //byte
#endif


//flash物理地址(绝对地址)与扇区编号转换
//例如0x8006000转换成24
#ifndef FLASH_PAGE_TO_ADDR
#define FLASH_PAGE_TO_ADDR(page)          ((uint32_t)(FLASH_BASE+(FLASH_PAGE_SIZE)*(page))) /* Base @ of page address*/
#endif

#ifndef ADDR_TO_FLASH_PAGE
#define ADDR_TO_FLASH_PAGE(addr)          (((addr)-FLASH_BASE)/(FLASH_PAGE_SIZE))
#endif

#ifndef FLASH_USER_START_ADDR
#define FLASH_USER_START_ADDR       FLASH_PAGE_TO_ADDR(24)   /* Start @ of user Flash area */
#endif

#ifndef FLASH_USER_END_ADDR
#define FLASH_USER_END_ADDR         FLASH_PAGE_TO_ADDR(34)   /* End @ of user Flash area */
#endif

/*---------------------- Constant / Macro Definitions -----------------------*/
#define BUILD_UINT16(loByte, hiByte) \
          ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))
					
#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
          ((uint32_t)((uint32_t)((Byte0) & 0x00FF) \
          + ((uint32_t)((Byte1) & 0x00FF) << 8) \
          + ((uint32_t)((Byte2) & 0x00FF) << 16) \
          + ((uint32_t)((Byte3) & 0x00FF) << 24)))

/*----------------------- Type Declarations ---------------------------------*/


/*----------------------- Variable Declarations -----------------------------*/


/*----------------------- Function Prototype --------------------------------*/


/*----------------------- Function Implement --------------------------------*/


/*---------------------------------------------------------------------------*/
/******************************************************************************
* Name: 	 flash_read
*
* Desc:
* Param(in):  	 address  -------读取FLASH的起始地址
                 size     -------欲读取的长度(单位:字节)
* Param(out): 	 pdata    -------读取的实际数据
* Return:     	 实际读到的字节数
* Global:
* Note:      pdata的空间由调用者申请，空间不得小于size个字节
						 address为flash的绝对地址，如0x8006000
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
uint32_t flash_read ( uint32_t address, uint8_t* pdata, uint32_t size )
{
	uint32_t read_index = 0;
	uint8_t  value;
	uint32_t start_addr;
	uint32_t end_addr;


	/*参数有效性检查 */
	if ( !pdata || size < 1 )
	{
		return 0;//FLASH_PARAM_ERROR;
	}
	/*地址有效性检查 */
//    start_addr = address+FLASH_BASE_ADDRESS;
	start_addr = address;
	end_addr = start_addr + size;
	if ( start_addr < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR )
	{
		return 0;//FLASH_ADDR_ERROR;
	}

	/*按字节方式读取数据*/
	read_index = 0;
	while ( read_index < size )
	{
		value = * ( __IO uint8_t* ) start_addr;
		start_addr = start_addr + 1;
		* ( pdata + read_index ) = value;
		read_index++;
	}
	return read_index;
}

/******************************************************************************
* Name: 	 flash_write
*
* Desc:       	 在FLASH的指定位置写入数据
* Param(in):  	 address  -------写入FLASH的起始地址(必须为4的整数倍)
                 pdata    -------写入的实际数据
                 size     -------写入数据的长度
* Param(out):
* Return:
* Global:
* Note:			 pdata由调用者传入，大小至少size字节
						 address为flash的绝对地址，如0x8006000
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
FLASH_ERROR_CODE_E flash_write(uint32_t address, const uint8_t* pdata, uint32_t size)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
	uint32_t end_addr = 0;
	uint32_t start_addr;
//	uint16_t page_num;       //计算需要写入的数据所占用的页数
	uint32_t word_num;       //计算按字写入的数据长度
	uint8_t  half_word_num;  //计算按半字写入的数据长度
	uint8_t  byte_num;       //计算按字节写入的数据长度
	uint32_t write_index = 0;
	
	//parameter check
	if((!pdata) || (size<1))
	{
		return FLASH_PARAM_ERROR;
	}
//	/*计算使用的页数*/
//	page_num = (size % FLASH_PAGE_SIZE)?(size / FLASH_PAGE_SIZE+1):(size / FLASH_PAGE_SIZE);
	/*计算需按字，半字，字节方式写入的数据长度*/
	word_num = (size >> 2);             // size/4
	half_word_num = (size % 4)>>1;      // (size%4)>>1
	byte_num = (size % 2);              // size % 2
	/*flash地址有效性检测*/
	start_addr = address;
	end_addr = ( start_addr + size );
	if ( start_addr < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR )
	{
		return FLASH_ADDR_ERROR;
	}
	/*写入数据*/
	/* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();
	write_index = 0;
	while(write_index < word_num)
	{
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_addr, BUILD_UINT32 ( * ( pdata ), * ( pdata + 1 ), * ( pdata + 2 ), * ( pdata + 3 ) ));
		if(HAL_OK == result)
		{
			start_addr = start_addr + 4;
			pdata = pdata + 4;
			write_index++;
		}
		else
		{
			return FLASH_WRITE_WORD_ERROR;
		}
	}
	write_index = 0;
	while(write_index < half_word_num)
	{
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,start_addr,BUILD_UINT16 ( * ( pdata ), * ( pdata + 1 ) ));
		if ( HAL_OK == result )
		{
			start_addr = start_addr + 2;
			pdata = pdata + 2;
			write_index++;
		}
		else
		{
			return FLASH_WRITE_HALF_WORD_ERROR;
		}
	}
	write_index = 0;
	while(write_index < byte_num)
	{
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,start_addr,BUILD_UINT16 ( * ( pdata ), 0xFFFF ));
		if ( HAL_OK == result )
		{
			start_addr = start_addr + 2;
			pdata = pdata + 2;
			write_index++;
		}
		else
		{
			return FLASH_WRITE_BYTE_ERROR;
		}
	}
	/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	return FLASH_SUCCESS;
}

/******************************************************************************
* Name: 	 flash_erase
*
* Desc:				   按指定地址擦除一页flash
* Param(in):  	 start_addr  -------起始绝对地址
						     end_addr    -------结束绝对地址
* Param(out): 	 
* Return:     	 
* Global:
* Note:      如果起始地址不是扇区的起始地址，会擦除整个扇区
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
FLASH_ERROR_CODE_E flash_erase( uint32_t start_addr,uint32_t end_addr)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	
	debug("start_addr = %x,end_addr=%x,FLASH_USER_START_ADDR = %x,FLASH_USER_END_ADDR = %x\r\n",start_addr,end_addr,FLASH_USER_START_ADDR,FLASH_USER_END_ADDR);
	if((start_addr > end_addr) || (start_addr < FLASH_USER_START_ADDR) || (end_addr > FLASH_USER_END_ADDR))
	{
		return FLASH_ADDR_ERROR;
	}
	/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
	
	/* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = start_addr;
  EraseInitStruct.NbPages = (end_addr - start_addr+(FLASH_PAGE_SIZE-1))/FLASH_PAGE_SIZE;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
		HAL_FLASH_Lock();
		return FLASH_ERASE_ERROR;
	}
	/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
	return FLASH_SUCCESS;
}







/******************************************************************************
* Name: 	 flash_read_page
*
* Desc:
* Param(in):  	 page_no  -------扇区编号（相对编号(相对于用户起始flash编号)，从0开始）
                 size     -------欲读取的长度(单位:字节)
* Param(out): 	 pdata    -------读取的实际数据
* Return:     	 实际读到的字节数
* Global:
* Note:      pdata的空间由调用者申请，空间不得小于size个字节
						 address为flash的绝对地址，如0x8006000
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
uint32_t flash_read_page (uint8_t page_no, uint32_t offect, uint8_t* pdata, uint32_t size )
{
	uint32_t result = 0;
	uint32_t addr;
	
	addr = PAGE_TO_ADDR(page_no);
	addr += offect;
	result = flash_read(addr,pdata,size);
	return result;
}
/******************************************************************************
* Name: 	 flash_write_page
*
* Desc:       	 在FLASH的指定页，指定偏移地址写入数据
* Param(in):  	 page_no  -------扇区编号(相对编号(相对于用户起始flash编号)，从0开始)
								 offect   -------本扇区的偏移地址(必须为4的整数倍)
                 pdata    -------写入的实际数据
                 size     -------写入数据的长度
* Param(out):
* Return:
* Global:
* Note:			 pdata由调用者传入，大小至少size字节
						 address为flash的绝对地址，如0x8006000
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
uint32_t flash_write_page (uint8_t page_no, uint32_t offect,const uint8_t* pdata, uint32_t size )
{
	uint32_t result = 0;
	uint32_t addr;
	
	addr = PAGE_TO_ADDR(page_no);
	addr += offect;
	result = flash_write(addr,pdata,size);
	return result;
}

/******************************************************************************
* Name: 	 flash_erase_page
*
* Desc:				   按指定扇区擦除flash
* Param(in):  	 start_page  -------起始相对扇区
						     page_cnt    -------擦除的扇区个数
* Param(out): 	 
* Return:     	 
* Global:
* Note:      如果起始地址不是扇区的起始地址，会擦除整个扇区
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
FLASH_ERROR_CODE_E flash_erase_page ( uint32_t start_page, uint16_t page_cnt )
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	
	/* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();
	
	/* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = PAGE_TO_ADDR(start_page);
  EraseInitStruct.NbPages = page_cnt;
	
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
		HAL_FLASH_Lock();
		return FLASH_ERASE_ERROR;
	}
	
	/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

	return FLASH_SUCCESS;
}


void print_flash_info(void)
{
	printf("flash information:\r\n");
	printf("user flash start address : 0x%x\r\n",FLASH_USER_START_ADDR);
	printf("user flash end address   : 0x%x\r\n",FLASH_USER_END_ADDR);
	printf("user flash page size     : %x\r\n",FLASH_PAGE_SIZE);
}

