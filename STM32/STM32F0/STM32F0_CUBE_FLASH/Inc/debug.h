/******************************************************************************
* Copyright 2017-2022 545122859@qq.com
* FileName:      debug.h
* Desc:      �ô��ڴ�ӡ�����ֽ�����
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
#ifndef _DEBUG_H_
#define  _DEBUG_H_
/*-------------------------------- Includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "cmd_parser.h"
#include "usart.h"
/*----------------------- Constant / Macro Definitions -----------------------*/
#define CFG_ENABLE_DEBUG            1      //�ܿ���
#define CFG_ENABLE_PRINTF_DBG       1      //ʹ�ܴ�ӡ
#define CFG_ENABLE_TIMEOUT_DBG      0      //��ʱ��ӡ
#define TIMROUT_PRINT_TIME          2000   //��ʱ��ӡ�ĳ�ʱʱ��ms

extern int g_dbg_last_line;
extern char* g_dbg_last_file;

extern int g_dbg_pre_line;
extern char* g_dbg_pre_file;

extern uint32_t g_dbg_chk_timeout;
extern size_t g_dbg_chk_val;


/*------------------------ Variable Define/Declarations ----------------------*/
#if (CFG_ENABLE_DEBUG == 0)
#undef CFG_ENABLE_PRINTF_DBG
#define CFG_ENABLE_PRINTF_DBG 0
#undef CFG_ENABLE_TIMEOUT_DBG
#define CFG_ENABLE_TIMEOUT_DBG 0
#endif

/*----------------------------- External Variables ---------------------------*/


/*------------------------ Function Prototype --------------------------------*/


/*------------------------ Variable Define/Declarations ----------------------*/


/*------------------------ Function Implement --------------------------------*/



#if CFG_ENABLE_PRINTF_DBG
#define debug(P...)  do{       \
                     printf(P);\
                     }while(0)

#define debug_hex(data,len) do{ const int l=(len); int x;        \
                    for(x=0 ; x<l ; x++) debug("0x%02x ",*((data)+x));   \
                    debug("\r\n");}while(0)

#define debug_char(data,len) do{ const int l=(len); int x;     \
                    for(x=0 ; x<l ; x++) debug("%c",*((data)+x));      \
                    debug("\r\n");}while(0)

#define debug_dec(data,len) do{ const int l=(len); int x;     \
                     for(x=0 ; x<l ; x++) debug("%d ",*((data)+x));    \
                     debug("\r\n");}while(0)
//���ԣ�error_handleΪ���������ʱ�Ĵ�����(����Ϊassert_hook_t)���������������NULL(Ĭ��ֻ�д�ӡ)
#define ASSERT(expression,error_handle)                              \
if(!(expression))                                                    \
{                                                                    \
    assert_handle(#expression,__FUNCTION__,__LINE__,error_handle);   \
}
#else
#define debug(P...)                      do{}while(0)
#define debug_hex(data,len)         do{}while(0)
#define debug_char(data,len)        do{}while(0)
#define debug_dec(data,len)        do{}while(0)
#define ASSERT(expression,error_handle)  do{}while(0)
#endif



#if CFG_ENABLE_PRINTF_DBG
/* ��ʱ���ӱ���ֵ */
#define DEBUG_CHK_LINE_VAL(VAL) do{g_dbg_chk_val = (size_t)(VAL);}while(0)

/* ��Ҫ���Ӵ�ִ�� */
#define DEBUG_CHK_LINE() do{               \
    g_dbg_pre_file= g_dbg_last_file;       \
    g_dbg_pre_line = g_dbg_last_line;      \
    g_dbg_last_file = __FILE__;            \
    g_dbg_last_line = __LINE__;            \
    g_dbg_chk_timeout = 0;                 \
}while(0)

/* ��Timer�ж���ִ�У�TIME_OUTΪ��ʱ��ӡ����*ms */
#define DEBUG_CHK_LINE_TIMER_INT(TIME_OUT) do{\
    if(g_dbg_chk_timeout > TIME_OUT)\
    {\
        debug("![%x]! %s:%d -> %s:%d ->!\r\n", g_dbg_chk_val, g_dbg_pre_file, g_dbg_pre_line, g_dbg_last_file, g_dbg_last_line);\
        g_dbg_chk_timeout = 0;\
    }\
    g_dbg_chk_timeout++;\
}while(0)
#else
#define DEBUG_CHK_LINE() do{}while(0)
#define DEBUG_CHK_LINE_VAL(VAL) do{}while(0)
#define DEBUG_CHK_LINE_TIMER_INT(TIME_OUT) do{}while(0)
#endif


void debug_init ( void );
void debug_real_time ( void );


#endif
/*---------------------------------------------------------------------------*/
