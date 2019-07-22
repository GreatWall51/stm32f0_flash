/******************************************************************************
* Copyright 2018-2023 Renyucjs@163.com
* FileName: 	 debug_cmd_io.c 
* Desc:
* 
* 
* Author: 	 Liuwq
* Date: 	 2018/04/02
* Notes: 
* 
* -----------------------------------------------------------------
* Histroy: v1.0   2018/04/02, Liuwq create this file
* 
******************************************************************************/
 
 
/*------------------------------- Includes ----------------------------------*/
#include "debug.h"

/*------------------- Global Definitions and Declarations -------------------*/
 
 
/*---------------------- Constant / Macro Definitions -----------------------*/
 
 
/*----------------------- Type Declarations ---------------------------------*/
 
 
/*----------------------- Variable Declarations -----------------------------*/
 
 
/*----------------------- Function Prototype --------------------------------*/
 
 
/*----------------------- Function Implement --------------------------------*/
CMD_FN(test)
{
	cmd_spliter_t *cmd_spliter;

	cmd_spliter = get_cmd_spliter();

	//参数个数
	printf("param cnt = %d\r\n",cmd_spliter->arg_cnt-1);
	//解析第一个参数
	if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "parameter1") == 0))
	{
		printf("param 1 success!\r\n");
		//解析第二个参数
		if ((cmd_spliter->arg_cnt >= 3) && (strcmp(cmd_spliter->arg[2], "parameter2") == 0))
		{
			printf("param 2 success!\r\n");
			//解析第三个参数
			if ((cmd_spliter->arg_cnt >= 4) && (strcmp(cmd_spliter->arg[3], "parameter3") == 0))
			{
				printf("param 3 success!\r\n");
			}
		}
	}
	else
	{
		printf("parameter count error!\r\n");
	}
	return;
}
/*---------------------------------------------------------------------------*/

