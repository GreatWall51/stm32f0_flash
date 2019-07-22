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


//flash�����ַ(���Ե�ַ)���������ת��
//����0x8006000ת����24
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
* Param(in):  	 address  -------��ȡFLASH����ʼ��ַ
                 size     -------����ȡ�ĳ���(��λ:�ֽ�)
* Param(out): 	 pdata    -------��ȡ��ʵ������
* Return:     	 ʵ�ʶ������ֽ���
* Global:
* Note:      pdata�Ŀռ��ɵ��������룬�ռ䲻��С��size���ֽ�
						 addressΪflash�ľ��Ե�ַ����0x8006000
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


	/*������Ч�Լ�� */
	if ( !pdata || size < 1 )
	{
		return 0;//FLASH_PARAM_ERROR;
	}
	/*��ַ��Ч�Լ�� */
//    start_addr = address+FLASH_BASE_ADDRESS;
	start_addr = address;
	end_addr = start_addr + size;
	if ( start_addr < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR )
	{
		return 0;//FLASH_ADDR_ERROR;
	}

	/*���ֽڷ�ʽ��ȡ����*/
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
* Desc:       	 ��FLASH��ָ��λ��д������
* Param(in):  	 address  -------д��FLASH����ʼ��ַ(����Ϊ4��������)
                 pdata    -------д���ʵ������
                 size     -------д�����ݵĳ���
* Param(out):
* Return:
* Global:
* Note:			 pdata�ɵ����ߴ��룬��С����size�ֽ�
						 addressΪflash�ľ��Ե�ַ����0x8006000
* Author: 	 Liuwq
* -------------------------------------
* Log: 	 2019/06/03, Create this function by Liuwq
 ******************************************************************************/
FLASH_ERROR_CODE_E flash_write(uint32_t address, const uint8_t* pdata, uint32_t size)
{
	HAL_StatusTypeDef result = HAL_ERROR;
	
	uint32_t end_addr = 0;
	uint32_t start_addr;
//	uint16_t page_num;       //������Ҫд���������ռ�õ�ҳ��
	uint32_t word_num;       //���㰴��д������ݳ���
	uint8_t  half_word_num;  //���㰴����д������ݳ���
	uint8_t  byte_num;       //���㰴�ֽ�д������ݳ���
	uint32_t write_index = 0;
	
	//parameter check
	if((!pdata) || (size<1))
	{
		return FLASH_PARAM_ERROR;
	}
//	/*����ʹ�õ�ҳ��*/
//	page_num = (size % FLASH_PAGE_SIZE)?(size / FLASH_PAGE_SIZE+1):(size / FLASH_PAGE_SIZE);
	/*�����谴�֣����֣��ֽڷ�ʽд������ݳ���*/
	word_num = (size >> 2);             // size/4
	half_word_num = (size % 4)>>1;      // (size%4)>>1
	byte_num = (size % 2);              // size % 2
	/*flash��ַ��Ч�Լ��*/
	start_addr = address;
	end_addr = ( start_addr + size );
	if ( start_addr < FLASH_USER_START_ADDR || end_addr > FLASH_USER_END_ADDR )
	{
		return FLASH_ADDR_ERROR;
	}
	/*д������*/
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
* Desc:				   ��ָ����ַ����һҳflash
* Param(in):  	 start_addr  -------��ʼ���Ե�ַ
						     end_addr    -------�������Ե�ַ
* Param(out): 	 
* Return:     	 
* Global:
* Note:      �����ʼ��ַ������������ʼ��ַ���������������
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
* Param(in):  	 page_no  -------������ţ���Ա��(������û���ʼflash���)����0��ʼ��
                 size     -------����ȡ�ĳ���(��λ:�ֽ�)
* Param(out): 	 pdata    -------��ȡ��ʵ������
* Return:     	 ʵ�ʶ������ֽ���
* Global:
* Note:      pdata�Ŀռ��ɵ��������룬�ռ䲻��С��size���ֽ�
						 addressΪflash�ľ��Ե�ַ����0x8006000
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
* Desc:       	 ��FLASH��ָ��ҳ��ָ��ƫ�Ƶ�ַд������
* Param(in):  	 page_no  -------�������(��Ա��(������û���ʼflash���)����0��ʼ)
								 offect   -------��������ƫ�Ƶ�ַ(����Ϊ4��������)
                 pdata    -------д���ʵ������
                 size     -------д�����ݵĳ���
* Param(out):
* Return:
* Global:
* Note:			 pdata�ɵ����ߴ��룬��С����size�ֽ�
						 addressΪflash�ľ��Ե�ַ����0x8006000
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
* Desc:				   ��ָ����������flash
* Param(in):  	 start_page  -------��ʼ�������
						     page_cnt    -------��������������
* Param(out): 	 
* Return:     	 
* Global:
* Note:      �����ʼ��ַ������������ʼ��ַ���������������
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

