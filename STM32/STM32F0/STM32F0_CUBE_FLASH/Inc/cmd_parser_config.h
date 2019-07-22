/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 cmd_parser_config.h
* Desc:
*
*
* Author: 	 LiuWeiQiang
* Date: 	 2019/05/25
* Notes:
*
* -----------------------------------------------------------------
* Histroy: v1.0   2019/05/25, LiuWeiQiang create this file
*
******************************************************************************/
#ifndef _CMD_PARSER_CONFIG_H_
#define _CMD_PARSER_CONFIG_H_


/*------------------------------- Includes ----------------------------------*/
#include "debug.h"

/*----------------------------- Global Defines ------------------------------*/
//system function configure
#define C_STRCMP strcmp
#define C_PRINTF debug
#define C_MEMSET memset
#define C_STRTOK strtok_safe
#define C_MEMCMP memcmp
#define C_STRCHR strchr

//command parser buffer configure
#define CMD_MAX_LEN 200 //���������������������ַ�����
#define ARG_MAX_CNT 10  //debug���������������

/*----------------------------- Global Typedefs -----------------------------*/
#define CMD_DEFINE(cmd,dsc) {NULL,#cmd,dsc,_cmd_fn_##cmd}
#define CMD_FN_DECLARE(cmd) extern void _cmd_fn_##cmd(void)

//�ڴ˴��������ڵ��Եĺ���
#define CMD_DECLARE() \
CMD_FN_DECLARE(test);\
CMD_FN_DECLARE(flash);\


//�ڴ˴�ע�ᴮ�ڵ��Եĺ���
#define CMD_TABLE()  \
CMD_DEFINE(test,          "test [parameter1] [parameter2] [parameter3]"),\
CMD_DEFINE(flash,          "flash [info] [read] [write] [erase]"),\
/*----------------------------- External Variables --------------------------*/


/*------------------------ Global Function Prototypes -----------------------*/



#endif //_CMD_PARSER_CONFIG_H_
