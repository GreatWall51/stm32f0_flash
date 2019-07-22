/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 debug_flash.c
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
#include <stdlib.h>
#include "debug.h"
#include "flash.h"
/*------------------- Global Definitions and Declarations -------------------*/

extern void print_flash_info(void);
/*---------------------- Constant / Macro Definitions -----------------------*/


/*----------------------- Type Declarations ---------------------------------*/


/*----------------------- Variable Declarations -----------------------------*/


/*----------------------- Function Prototype --------------------------------*/


/*----------------------- Function Implement --------------------------------*/
CMD_FN(flash)
{
	cmd_spliter_t *cmd_spliter;

	cmd_spliter = get_cmd_spliter();

	//参数个数
//	printf("param cnt = %d\r\n",cmd_spliter->arg_cnt-1);
	//解析第一个参数
	if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "info") == 0))
	{
		print_flash_info();
	}
	//flash read 134242304 20
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "read") == 0))
	{
		//解析第二个参数
		if (cmd_spliter->arg_cnt >= 4)
		{
			uint32_t read_addr = 0;
			uint32_t read_cnt = 0;
			uint8_t *read_buf = NULL;
			
			read_addr = atoi(cmd_spliter->arg[2]);
			read_cnt  = atoi(cmd_spliter->arg[3]);
			read_buf = (uint8_t *)malloc(read_cnt);
			
			if(!read_buf) 
			{
				debug("malloc read_buf fail!\r\n");
				return ;
			}
			read_cnt = flash_read(read_addr,read_buf,read_cnt);
			printf("read %d data from 0x%x:\r\n",read_cnt,read_addr);
			debug_hex(read_buf,read_cnt);
			free(read_buf);
		}
		else
		{
			printf("flash_read function parameter count error!\r\n");
		}
	}
	//flash write 134242304 20
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "write") == 0))
	{
		if (cmd_spliter->arg_cnt >= 4)
		{
			uint8_t *write_buf = NULL;
			uint32_t write_cnt;
			uint32_t write_addr = 0;
			
			
			write_cnt = atoi(cmd_spliter->arg[3]);
			write_buf = (uint8_t *)malloc(write_cnt);
			if(!write_buf) 
			{
				debug("malloc write_buf fail!\r\n");
				return ;
			}
			for(int i=0;i < write_cnt;i++) write_buf[i] = i;
			write_addr = atoi(cmd_spliter->arg[2]);
			debug("prepare to write!\r\n");
			if(flash_write(write_addr,write_buf,write_cnt) == FLASH_SUCCESS)
			{
				printf("write %d byte data to %x success!\r\n",write_cnt,write_addr);
			}
			else
			{
				printf("write fail!\r\n");
			}
			free(write_buf);
		}
		else
		{
			printf("flash_write function parameter count error!\r\n");
		}
	}
	//flash erase 134242304 134242324
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "erase") == 0))
	{
		//解析第二个参数
		if (cmd_spliter->arg_cnt >= 4)
		{
			uint32_t start_addr = 0;
			uint32_t end_addr = 0;
			
			start_addr = atoi(cmd_spliter->arg[2]);
			end_addr = atoi(cmd_spliter->arg[3]);
			if(flash_erase(start_addr,end_addr) == FLASH_SUCCESS)
			{
				printf("erase from start_addr %d to end_addr %d success!\r\n",start_addr,end_addr);
			}
			else
			{
				printf("erase fail!\r\n");
			}
		}
	}
	//flash read_page 0 2 10
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "read_page") == 0))
	{
		if(cmd_spliter->arg_cnt >= 5)
		{
			uint8_t sec_no;
			uint8_t offect;
			uint8_t read_cnt;
			uint8_t *read_buf;
			
			sec_no = atoi(cmd_spliter->arg[2]);
			offect = atoi(cmd_spliter->arg[3]);
			read_cnt = atoi(cmd_spliter->arg[4]);
			
			read_buf = (uint8_t *)malloc(read_cnt);
			if(!read_buf) 
			{
				debug("malloc read_buf fail!\r\n");
				return ;
			}
			flash_read_page(sec_no,offect,read_buf,read_cnt);
			printf("read %d data from page 0x%x:\r\n",read_cnt,sec_no);
			debug_hex(read_buf,read_cnt);
			free(read_buf);
		}
		else
		{
			debug("flash_read_page function parameter count error!\r\n");
		}
	}
	//flash write_page 0 2 10
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "write_page") == 0))
	{
		if(cmd_spliter->arg_cnt >= 5)
		{
			uint8_t *write_buf = NULL;
			uint32_t write_cnt;
			uint32_t write_page_no = 0;
			uint32_t write_offset;

			write_page_no = atoi(cmd_spliter->arg[2]);
			write_offset = atoi(cmd_spliter->arg[3]);
			write_cnt = atoi(cmd_spliter->arg[4]);
			write_buf = (uint8_t *)malloc(write_cnt);
			if(!write_buf) 
			{
				debug("malloc write_buf fail!\r\n");
				return ;
			}
			for(int i=0;i < write_cnt;i++) write_buf[i] = i;
			if(flash_write_page(write_page_no,write_offset,write_buf,write_cnt) == FLASH_SUCCESS)
			{
				printf("write %d byte data to page %d success!\r\n",write_cnt,write_page_no);
			}
			else
			{
				printf("write fail!\r\n");
			}
			free(write_buf);
		}
		else
		{
			printf("flash_write function parameter count error!\r\n");
		}
	}
	//flash write_page 0 2 10
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "erase_page") == 0))
	{
		if (cmd_spliter->arg_cnt >= 4)
		{
			uint8_t start_page = 0;
			uint8_t page_cnt = 0;
			
			start_page = atoi(cmd_spliter->arg[2]);
			page_cnt = atoi(cmd_spliter->arg[3]);
			if(flash_erase_page(start_page,page_cnt) == FLASH_SUCCESS)
			{
				printf("erase from page %d to page %d success!\r\n",start_page,start_page+page_cnt);
			}
			else
			{
				printf("erase fail!\r\n");
			}
		}
		else
		{
			printf("flash_erase_page function parameter count error!\r\n");
		}
	}
	//测试sector擦写特性
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "clean_write") == 0))
	{
		if (cmd_spliter->arg_cnt >= 4)
		{
			uint8_t *write_buf = NULL;
			uint32_t write_cnt;
			uint32_t write_addr = 0;
			
			
			write_cnt = atoi(cmd_spliter->arg[3]);
			write_buf = (uint8_t *)malloc(write_cnt);
			if(!write_buf) 
			{
				debug("malloc write_buf fail!\r\n");
				return ;
			}
			for(int i=0;i < write_cnt;i++) write_buf[i] = i;
			write_buf[3] = 0;
			write_addr = atoi(cmd_spliter->arg[2]);
			debug("prepare to write!\r\n");
			if(flash_write(write_addr,write_buf,write_cnt) == FLASH_SUCCESS)
			{
				printf("write %d byte data to %x success!\r\n",write_cnt,write_addr);
			}
			else
			{
				printf("write fail!\r\n");
			}
			free(write_buf);
		}
		else
		{
			printf("flash_write function parameter count error!\r\n");
		}
	}
	else
	{
		printf("parameter count error!\r\n");
	}
	
	return;
}


/*---------------------------------------------------------------------------*/

