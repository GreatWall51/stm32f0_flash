/******************************************************************************
* Copyright 2017-2022 545122859@qq.com
* FileName:      debug.c
* Desc:      用串口打印单个字节数据
*
*
* Author:    LiuWeiQiang
* Date:      2017/05/18
* Notes:
*
* -----------------------------------------------------------------------------
* Histroy: v1.0   2017/05/18, LiuWeiQiang create this file
*
******************************************************************************/

/*-------------------------------- Includes ----------------------------------*/
#include "debug.h"

/*----------------------- Constant / Macro Definitions -----------------------*/
typedef void assert_hook_t ( const char* ex, const char* func, size_t line );

/*------------------------ Variable Define/Declarations ----------------------*/
uint8_t g_fm23_init_flg;

int g_dbg_last_line = 0;
int g_dbg_pre_line = 0;
char* g_dbg_last_file = NULL;
char* g_dbg_pre_file = NULL;
uint32_t g_dbg_chk_timeout = 0;
size_t g_dbg_chk_val = 0;

/*----------------------------- External Variables ---------------------------*/
extern void putCh ( uint8_t ch );

//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;
	/* Whatever you require here. If the only file you are using is */
	/* standard output using printf() for debugging, no file handling */
	/* is required. */
};
/* FILE is typedef’ d in stdio.h. */
FILE __stdout;
int _sys_exit ( int x )
{
	x = x;
	return 0;
}
void _ttywrch ( int ch )
{
	ch = ch;
}

//重定义fputc函数
int fputc ( int ch, FILE* f )
{
	putCh ( ch );
	return ch;
}

/*------------------------ Function Prototype --------------------------------*/


/*------------------------ Variable Define/Declarations ----------------------*/


/*------------------------ Function Implement --------------------------------*/
void assert_handle ( const char* ex_string, const char* func, size_t line,assert_hook_t* error_handle )
{
	volatile char dummy = 0;
	assert_hook_t* handle;

	handle = error_handle;
	if ( handle )
	{
		handle ( ex_string, func, line );
	}
	else
	{
		debug ( "assertion failed:(%s) at function:%s, line number:%d \r\n", ex_string, func, line );
		while ( dummy == 0 );
	}
}


/*---------------------------------------------------------------------------*/
void test_assert_hanhle ( const char* ex_string, const char* func, size_t line )
{
	debug ( "[%s] assertion failed at function:%s, line number:%d \r\n", ex_string, func, line );
}
void test_assert_hanhle1 ( const char* ex_string, const char* func, size_t line )
{
	debug ( "%s assertion failed at function:%s, line number:%d \r\n", ex_string, func, line );
}

void debug_init()
{
	extern void debug_uart_init(void);
	
	debug_uart_init();
	debug ( "..........COMMAND_PARSER..........\r\n" );
}
void debug_real_time ( void )
{
	cmd_parser_real_time();
}

/*---------------------------------------------------------------------------*/
