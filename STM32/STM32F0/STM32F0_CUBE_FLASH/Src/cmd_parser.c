/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 cmd_parser.c
* Desc:			命令行解析器
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


/*------------------------------- Includes ----------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "cmd_parser.h"
/*------------------- Global Definitions and Declarations -------------------*/


/*---------------------- Constant / Macro Definitions -----------------------*/


/*----------------------- Type Declarations ---------------------------------*/
//命令数据接收器
typedef struct
{
    uint8_t  finish;                   //命令接收完成标志
    uint32_t cmd_len;                  //命令行接收到的数据总长度
    char     cmd_buf[CMD_MAX_LEN];     //命令行数据缓冲区
} cmd_receiver_t;

//命令数据解析器
typedef struct CMD_PARSER
{
    struct CMD_PARSER *next;
    char *cmd;                  //命令名称
    char *cmd_describe;         //命令描述
    void (*cmd_action)(void);   //响应动作
//    uint32_t time_evt_mask;     //定时事件
} cmd_parser_t;

CMD_DECLARE();
static cmd_parser_t cmd_table[]=
{
	CMD_TABLE()
};

static cmd_receiver_t cmd_receiver = {0};     //串口数据接收器
cmd_spliter_t cmd_spliter = {0};              //串口命令分离器

/*----------------------- Variable Declarations -----------------------------*/


/*----------------------- Function Prototype --------------------------------*/


/*----------------------- Function Implement --------------------------------*/
static int _strspn(const char *s,const char *accept)
{
	const char *p;
	const char *a;
	int count=0;
	if(!s || !accept) return 0;
	for (p=s;*p!='\0';p++)
	{
		for (a=accept;*a!='\0';a++)
		{
            if (*p==*a)
            {
				break;
            }
		}
		if (*a=='\0')
		{
			return count;
		}
		++count;
	}
	return count;
 
}
static char *_strpbrk(const char *str1, const char *str2)
{
	char *pstr1= (char *)str1;
  if(!str1 || !str2) return 0;
	while (*pstr1)
	{
		char *pstr2 = (char *)str2;
 
		while (*pstr2 && (*pstr2 != *pstr1))
			++pstr2;
		if (*pstr2 == *pstr1)
			break;
		++pstr1;
	}
	return pstr1;
}
//根据函数原型实现strtok()函数
static char* _strtok(char* str_arr,const char* delimiters,char**temp_str)
{
    //定义一个指针来指向待分解串
    char *b_temp;
    /*
    * 1、判断参数str_arr是否为空，如果是NULL就以传递进来的temp_str作为起始位置；
    * 若不是NULL，则以str为起始位置开始切分。
    */
    if(str_arr == NULL)
    {
        str_arr =*temp_str;
    }
    //2、跳过待分解字符串
    //扫描delimiters字符开始的所有分解符
    str_arr += _strspn(str_arr, delimiters);
    //3、判断当前待分解的位置是否为'\0'，若是则返回NULL，否则继续
    if(*str_arr =='\0')
    {
        return NULL;
    }
    /*
    * 4、保存当前的待分解串的指针b_temp，调用strpbrk()在b_temp中找分解符，
    * 如果找不到，则将temp_str赋值为待分解字符串末尾部'\0'的位置，
    * b_temp没有发生变化；若找到则将分解符所在位置赋值为'\0',
    * b_temp相当于被截断了，temp_str指向分解符的下一位置。
    */
    b_temp = str_arr;
    str_arr = _strpbrk(str_arr, delimiters);
    if(str_arr == NULL)
    {
        *temp_str = strchr(b_temp,'\0');
    }
    else
    {
        *str_arr ='\0';
        *temp_str = str_arr +1;
    }
    //5、函数最后部分无论找没找到分解符，都将b_temp返回。
    return b_temp;
}
//使用myStrtok来简化myStrtok_origin函数
char* strtok_safe(char* str_arr,const char* delimiters)
{
    static char*last;
    return _strtok(str_arr,delimiters,&last);
}
/******************************************************************************
* Name: 	 get_cmd_spliter 
*
* Desc:       	 获取命令分离器
* Param(in):  	 
* Param(out): 	 
* Return:     	 
* Global:     	 
* Note:       	 
* Author: 	 LiuWeiQiang
* -------------------------------------
* Log: 	 2019/05/25, Create this function by LiuWeiQiang
 ******************************************************************************/
cmd_spliter_t *get_cmd_spliter(void)
{
    return &cmd_spliter;
}
/******************************************************************************
* Name: 	 _cmd_parse
*
* Desc:       	 命令解析
* Param(in):  	 
* Param(out): 	 
* Return:     	 0->解析成功，1->解析失败
* Global:     	 
* Note:       	 
* Author: 	 LiuWeiQiang
* -------------------------------------
* Log: 	 2019/05/25, Create this function by LiuWeiQiang
 ******************************************************************************/
static uint8_t _cmd_parse(char *cmd)
{
	uint16_t cmd_index;

	if (cmd == NULL) return 0;
	//从命令注册表中查找匹配命令
	for(cmd_index = 0;cmd_index < sizeof(cmd_table)/sizeof(cmd_parser_t);cmd_index++)
	{
		//查找到注册的匹配命令，执行回调函数，并返回
		if(0 == C_STRCMP(cmd , cmd_table[cmd_index].cmd))
		{
			(cmd_table[cmd_index].cmd_action)();
			return 1;
		}
	}
	return 0;
}

/******************************************************************************
* Name: 	 _cmd_help 
*
* Desc:       	 打印帮助信息
* Param(in):  	 
* Param(out): 	 
* Return:     	 
* Global:     	 
* Note:       	 
* Author: 	 LiuWeiQiang
* -------------------------------------
* Log: 	 2019/05/25, Create this function by LiuWeiQiang
 ******************************************************************************/
static void _cmd_help(void)
{
	cmd_parser_t *cmd_parser = NULL;
	uint16_t cmd_index;

	C_PRINTF("\r\ncommand register table information:\r\n");
	C_PRINTF("---------------------------------------------------------------\r\n");
	C_PRINTF("    command     |                 tamplate\r\n");
	C_PRINTF("---------------------------------------------------------------\r\n");
	for(cmd_index = 0;cmd_index < sizeof(cmd_table)/sizeof(cmd_parser_t);cmd_index++)
	{
		cmd_parser = &cmd_table[cmd_index];
		C_PRINTF("%-15s |   %s\r\n", cmd_parser->cmd, cmd_parser->cmd_describe);
	}
	C_PRINTF("---------------------------------------------------------------\r\n");
}

/******************************************************************************
* Name: 	 receive_cmd_callback 
*
* Desc:       	 命令行接收数据回调函数
* Param(in):  	 
* Param(out): 	 
* Return:     	 
* Global:     	 
* Note:       	 每接收完一个字节进行回调
* Author: 	 LiuWeiQiang
* -------------------------------------
* Log: 	 2019/05/25, Create this function by LiuWeiQiang
 ******************************************************************************/
void receive_cmd_callback(uint8_t cmd_char)
{
	if(cmd_receiver.finish != 0) return;

	if(cmd_receiver.cmd_len <= (CMD_MAX_LEN-1))
	{
		cmd_receiver.cmd_buf[cmd_receiver.cmd_len++] = cmd_char;
		cmd_receiver.cmd_buf[cmd_receiver.cmd_len] = '\0';
	}
	else
	{
		return ;
	}

	switch (cmd_char)
	{
		case '\n':
		case '\r':
		{
			cmd_receiver.finish = 1;
			if(cmd_receiver.cmd_len > 1) cmd_receiver.cmd_len -= 1;
		}
		break;
		case 0x08: //退格
		case 0x7F:
		//case 0x53: //DEL
		{
			if (cmd_receiver.cmd_len > 0)
			{
				cmd_receiver.cmd_len--;
				cmd_receiver.cmd_buf[cmd_receiver.cmd_len] = '\0';
			}
		}
		break;
		//将命令行的, / - :全部转化成空格
		case ',':
		case '/':
		case '-':
		case ':':
		{
			cmd_receiver.cmd_buf[cmd_receiver.cmd_len-1] = ' ';
		}
		break;
		default:
		break;
	}
}

/******************************************************************************
* Name: 	 cmd_parser_real_time 
*
* Desc:       	 命令解释器线程
* Param(in):  	 
* Param(out): 	 
* Return:     	 
* Global:     	 
* Note:       	 
* Author: 	 LiuWeiQiang
* -------------------------------------
* Log: 	 2019/05/25, Create this function by LiuWeiQiang
 ******************************************************************************/
void cmd_parser_real_time(void)
{
    if(cmd_receiver.finish == 1)
    {
        char * ptr;
        if(cmd_receiver.cmd_len == 0) return;
    	//解析输入的命令
    	C_MEMSET(&cmd_spliter,0,sizeof(cmd_spliter_t));
        cmd_receiver.cmd_buf[cmd_receiver.cmd_len] = '\0';
        ptr = C_STRTOK(cmd_receiver.cmd_buf," ");
        while(ptr)
        {
            cmd_spliter.arg[cmd_spliter.arg_cnt++] = ptr;
            if(cmd_spliter.arg_cnt >= ARG_MAX_CNT+1)
            {
                break;
            }
            ptr = C_STRTOK(NULL," ");
        }
		//第一个参数为命令，进行解析
        if(_cmd_parse(cmd_spliter.arg[0]))
        {
//            debug("cmd parse success!\r\n");
        }
		//输入换行符，以# >进行回显
//		else if(C_MEMCMP(cmd_spliter.arg[0],"\r\n",2)==0 \
//			    || C_MEMCMP(cmd_spliter.arg[0],"\n",1)==0 \
//			    || C_MEMCMP(cmd_spliter.arg[0],"\r",1)==0)
				else if(C_STRCMP(cmd_spliter.arg[0],"\r\n")==0 \
			    || C_STRCMP(cmd_spliter.arg[0],"\n")==0 \
			    || C_STRCMP(cmd_spliter.arg[0],"\r")==0)
		{
			C_PRINTF("\r\n# >");
		}
		//输入'?',打印帮助信息
		else if(*cmd_spliter.arg[0] == '?')
		{			
			_cmd_help();
		}
		//未知命令
        else
        {
            C_PRINTF("Unknown command!\r\n");
        }
        C_MEMSET(&cmd_receiver,0,sizeof(cmd_receiver_t));
    }
}
/*---------------------------------------------------------------------------*/

