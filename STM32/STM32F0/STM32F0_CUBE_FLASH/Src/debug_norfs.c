/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 debug_norfs.c
* Desc:
*
*
* Author: 	 LiuWeiQiang
* Date: 	 2019/07/01
* Notes:
*
* -----------------------------------------------------------------
* Histroy: v1.0   2019/07/01, LiuWeiQiang create this file
*
******************************************************************************/


/*------------------------------- Includes ----------------------------------*/
#include "debug.h"
#include "norfs.h"
#include "media.h"
#include "norls.h"
/*------------------- Global Definitions and Declarations -------------------*/


/*---------------------- Constant / Macro Definitions -----------------------*/


/*----------------------- Type Declarations ---------------------------------*/


/*----------------------- Variable Declarations -----------------------------*/

static HLS *hls = NULL;
static struct norfs_fd fd;
static char *file_name =  "picture/20190102_081413.jpg";
/*----------------------- Function Prototype --------------------------------*/


/*----------------------- Function Implement --------------------------------*/
CMD_FN(norfs)
{
	cmd_spliter_t *cmd_spliter;

	cmd_spliter = get_cmd_spliter();

	//参数个数
//	printf("param cnt = %d\r\n",cmd_spliter->arg_cnt-1);
	//解析第一个参数
	if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "init") == 0))
	{
		int result;
		
		media_init();
		
		result = media_config(NORFS,MEDIA_PIC,"picture",-1,1024);
		if(result == 0) debug("init success!\r\n");
		else debug("init fail!\r\n");
	}
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "norfs_open") == 0))
	{
		if(norfs_open(&fd,file_name,50,NOR_MODE_RDONLY) == 0)
		{
			debug("open file %s success!\r\n",file_name);
		}
		else
		{
			debug("open file fail!\r\n");
		}
	}
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "norfs_close") == 0))
	{
		if(norfs_close(&fd) == 0)
		{
			debug("close file %s success!\r\n",file_name);
		}
		else
		{
			debug("close file fail!\r\n");
		}
	}
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "norfs_remove") == 0))
	{
		if(norfs_remove(file_name) == 0)
		{
			debug("remove file %s success!\r\n",file_name);
		}
		else
		{
			debug("remove file fail!\r\n");
		}
	}
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "openls") == 0))
	{
		hls = openls(MEDIA_PIC,NORFS);
		if(hls != NULL)
		{
			debug("openls success!\r\n");
		}
		else
		{
			debug("openls fail!\r\n");
		}
	}
	else if ((cmd_spliter->arg_cnt >= 2) && (strcmp(cmd_spliter->arg[1], "getnewls") == 0))
	{
		
		if(getnewls(hls,&file_name,file_name,1,FATTR_PROTE_BIT,FATTR_READ_BIT) != NULL)
		{
			debug("get block : %d\r\n",hls->current);
			debug("getnewls success!\r\n");
		}
		else
		{
			debug("getnewls fail!\r\n");
		}
	}
	else
	{
		debug("unknow function,please register!\r\n");
	}
}

/*---------------------------------------------------------------------------*/

