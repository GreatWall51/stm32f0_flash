/******************************************************************************
* Copyright 2019-2024 liuweiqiang@leelen.cn
* FileName: 	 cmd_parser.c
* Desc:			�����н�����
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
//�������ݽ�����
typedef struct
{
    uint8_t  finish;                   //���������ɱ�־
    uint32_t cmd_len;                  //�����н��յ��������ܳ���
    char     cmd_buf[CMD_MAX_LEN];     //���������ݻ�����
} cmd_receiver_t;

//�������ݽ�����
typedef struct CMD_PARSER
{
    struct CMD_PARSER *next;
    char *cmd;                  //��������
    char *cmd_describe;         //��������
    void (*cmd_action)(void);   //��Ӧ����
//    uint32_t time_evt_mask;     //��ʱ�¼�
} cmd_parser_t;

CMD_DECLARE();
static cmd_parser_t cmd_table[]=
{
	CMD_TABLE()
};

static cmd_receiver_t cmd_receiver = {0};     //�������ݽ�����
cmd_spliter_t cmd_spliter = {0};              //�������������

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
//���ݺ���ԭ��ʵ��strtok()����
static char* _strtok(char* str_arr,const char* delimiters,char**temp_str)
{
    //����һ��ָ����ָ����ֽ⴮
    char *b_temp;
    /*
    * 1���жϲ���str_arr�Ƿ�Ϊ�գ������NULL���Դ��ݽ�����temp_str��Ϊ��ʼλ�ã�
    * ������NULL������strΪ��ʼλ�ÿ�ʼ�з֡�
    */
    if(str_arr == NULL)
    {
        str_arr =*temp_str;
    }
    //2���������ֽ��ַ���
    //ɨ��delimiters�ַ���ʼ�����зֽ��
    str_arr += _strspn(str_arr, delimiters);
    //3���жϵ�ǰ���ֽ��λ���Ƿ�Ϊ'\0'�������򷵻�NULL���������
    if(*str_arr =='\0')
    {
        return NULL;
    }
    /*
    * 4�����浱ǰ�Ĵ��ֽ⴮��ָ��b_temp������strpbrk()��b_temp���ҷֽ����
    * ����Ҳ�������temp_str��ֵΪ���ֽ��ַ���ĩβ��'\0'��λ�ã�
    * b_tempû�з����仯�����ҵ��򽫷ֽ������λ�ø�ֵΪ'\0',
    * b_temp�൱�ڱ��ض��ˣ�temp_strָ��ֽ������һλ�á�
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
    //5��������󲿷�������û�ҵ��ֽ��������b_temp���ء�
    return b_temp;
}
//ʹ��myStrtok����myStrtok_origin����
char* strtok_safe(char* str_arr,const char* delimiters)
{
    static char*last;
    return _strtok(str_arr,delimiters,&last);
}
/******************************************************************************
* Name: 	 get_cmd_spliter 
*
* Desc:       	 ��ȡ���������
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
* Desc:       	 �������
* Param(in):  	 
* Param(out): 	 
* Return:     	 0->�����ɹ���1->����ʧ��
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
	//������ע����в���ƥ������
	for(cmd_index = 0;cmd_index < sizeof(cmd_table)/sizeof(cmd_parser_t);cmd_index++)
	{
		//���ҵ�ע���ƥ�����ִ�лص�������������
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
* Desc:       	 ��ӡ������Ϣ
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
* Desc:       	 �����н������ݻص�����
* Param(in):  	 
* Param(out): 	 
* Return:     	 
* Global:     	 
* Note:       	 ÿ������һ���ֽڽ��лص�
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
		case 0x08: //�˸�
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
		//�������е�, / - :ȫ��ת���ɿո�
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
* Desc:       	 ����������߳�
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
    	//�������������
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
		//��һ������Ϊ������н���
        if(_cmd_parse(cmd_spliter.arg[0]))
        {
//            debug("cmd parse success!\r\n");
        }
		//���뻻�з�����# >���л���
//		else if(C_MEMCMP(cmd_spliter.arg[0],"\r\n",2)==0 \
//			    || C_MEMCMP(cmd_spliter.arg[0],"\n",1)==0 \
//			    || C_MEMCMP(cmd_spliter.arg[0],"\r",1)==0)
				else if(C_STRCMP(cmd_spliter.arg[0],"\r\n")==0 \
			    || C_STRCMP(cmd_spliter.arg[0],"\n")==0 \
			    || C_STRCMP(cmd_spliter.arg[0],"\r")==0)
		{
			C_PRINTF("\r\n# >");
		}
		//����'?',��ӡ������Ϣ
		else if(*cmd_spliter.arg[0] == '?')
		{			
			_cmd_help();
		}
		//δ֪����
        else
        {
            C_PRINTF("Unknown command!\r\n");
        }
        C_MEMSET(&cmd_receiver,0,sizeof(cmd_receiver_t));
    }
}
/*---------------------------------------------------------------------------*/

