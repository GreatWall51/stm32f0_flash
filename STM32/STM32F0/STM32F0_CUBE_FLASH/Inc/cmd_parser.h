/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 cmd_praser.h
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
#ifndef _CMD_PARSER_H_
#define _CMD_PARSER_H_


/*------------------------------- Includes ----------------------------------*/
#include <stdint.h>
#include "cmd_parser_config.h"

/*----------------------------- Global Defines ------------------------------*/


/*----------------------------- Global Typedefs -----------------------------*/
#define CMD_FN(cmd) void _cmd_fn_##cmd(void)
typedef struct
{
    uint8_t  arg_cnt;             //命令参数个数
    char*    arg[ARG_MAX_CNT+1]; //命令地址
}cmd_spliter_t;//串口命令分离器

/*----------------------------- External Variables --------------------------*/


/*------------------------ Global Function Prototypes -----------------------*/
void receive_cmd_callback(uint8_t cmd_char);
void cmd_parser_real_time(void);
cmd_spliter_t *get_cmd_spliter(void);

#endif //_CMD_PARSER_H_
