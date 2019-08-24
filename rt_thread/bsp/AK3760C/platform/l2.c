/******************************************************************************
* Copyright 2016-2021 Leelen.co
* FileName: 	 l2.c 
* Desc:
* 
* 
* Author: 	 zhuangwei
* Date: 	 2016/11/17
* Notes: 
* 
* -----------------------------------------------------------------
* Histroy: v1.0   2016/11/17, zhuangwei create this file
* 
******************************************************************************/
 
 
/*------------------------------- Includes ----------------------------------*/
#include <rthw.h>
#include <rtdevice.h>
#include "interrupt.h"
#include "anyka_cpu.h"
#include "l2.h" 
 
/*------------------- Global Definitions and Declarations -------------------*/
//define DMA Request Register 's bit map
#define DMA_EN                  0

//define Fraction DMA Address Information Register 's bit map
#define AHB_FLAG_EN             31
#define LDMA_FLAG_EN            30
 
/*---------------------- Constant / Macro Definitions -----------------------*/




/*----------------------- Type Declarations ---------------------------------*/
 
 
/*----------------------- Variable Declarations -----------------------------*/
static rt_uint8_t l2_info_table[6] = {L2_STATUS_IDLE};
static rt_uint8_t device_info_table[L2_DEV_RESERVE] = {BUF_NULL};



/*----------------------- Function Prototype --------------------------------*/
static void rt_hw_l2_interrupt_handler(int vector, void *param);
 
/*----------------------- Function Implement --------------------------------*/
/******************************************************************************
* Name: 	 rt_hw_l2_init 
*
* Desc: 	 初始化L2
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_l2_init(void)
{
    //open l2 clock
    REG32(CLOCK_CTRL_REG) &= ~(1<<15);

    //enable l2 dma
    REG32(L2_DMA_REQ) = (1 << DMA_EN);

    //use auto cpu-controlling of buffer status
    REG32(L2_FRAC_ADDR) = (1U << LDMA_FLAG_EN)|(1U << AHB_FLAG_EN);

    //disable all common buffer
    REG32(L2_COMBUF_CFG) = 0x00000000;

    //enable uart buffer and clear uart buffer status
    REG32(L2_UARTBUF_CFG) = 0x0;//0x90c30000;

    //enable l2 irq interrupt, but disable the interrupt of all the buffers
    rt_hw_interrupt_install(INT_VECTOR_L2, rt_hw_l2_interrupt_handler,
                            RT_NULL, "L2");
    REG32(L2_INT_ENA) = 0x0;
}

static void rt_hw_l2_interrupt_handler(int vector, void *param)
{
    
    
}

/******************************************************************************
* Name: 	 hw_l2_get_addr 
*
* Desc: 	 根据buf id获取L2 BUF的地址
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/16, Create this function by zhuangwei
 ******************************************************************************/
rt_uint32_t rt_hw_l2_get_addr(rt_uint8_t buf_id)
{
    rt_uint32_t buf_addr = 0;

	if(buf_id < 6)
	{
		buf_addr = L2_BUF_MEM_BASE_ADDR + buf_id*512;
	}
	else if(buf_id <= 9)
	{
		buf_addr = L2_BUF_MEM_BASE_ADDR + 0xc00 + (buf_id-6)*128;
	}
	else
	{
        return 0;
	}

    return buf_addr;
}

/******************************************************************************
* Name: 	 rt_hw_l2_clr_status 
*
* Desc: 	 清L2状态
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/17, Create this function by zhuangwei
 ******************************************************************************/
void rt_hw_l2_clr_status(rt_uint8_t buf_id)
{
    rt_uint32_t reg_value;
    rt_base_t level;

    level = rt_hw_interrupt_disable();

    if (buf_id < 6)
    {
        reg_value = REG32(L2_COMBUF_CFG);
        reg_value |= 1<<(buf_id + 24);
        REG32(L2_COMBUF_CFG) = reg_value;
    }
    else
    {
        reg_value = REG32(L2_UARTBUF_CFG);
        reg_value |= (1<<(buf_id + 10));
        REG32(L2_UARTBUF_CFG) = reg_value;
    }

    rt_hw_interrupt_enable(level);
}

/******************************************************************************
* Name: 	 rt_hw_l2_cpu 
*
* Desc: 	 l2与cpu数据传输
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/17, Create this function by zhuangwei
 ******************************************************************************/
rt_uint8_t rt_hw_l2_cpu(rt_uint32_t ram_addr, rt_uint8_t buf_id, rt_uint32_t buf_offset, \
    rt_uint32_t tran_byte, rt_uint8_t tran_dir)
{
    rt_uint32_t tran_nbr, frac_nbr;
    rt_uint32_t buf_cnt, buf_remain;
    rt_uint32_t temp_ram = 0, temp_buf = 0;
    rt_uint32_t i,j;
    rt_uint32_t buf_addr;

    if(0 == ram_addr) return RT_FALSE;
    
    buf_addr = rt_hw_l2_get_addr(buf_id);
    if(0 == buf_addr) return RT_FALSE;

    buf_addr += buf_offset;
    tran_nbr = tran_byte >> 2;//4Byte align
    frac_nbr = tran_byte & 0x3;
    
    buf_cnt = (buf_offset+tran_byte) / 64;
    buf_remain = (buf_offset+tran_byte) % 64;

    if (tran_dir) 
    {
        //memory to buffer
        if (ram_addr & 0x3)
        {
            for (i=0; i<tran_nbr; i++)
            {
                temp_ram = 0;
                for (j=0; j<4; j++)
                    temp_ram |= ((ReadRamb(ram_addr+i*4+j))<<(j*8));
                WriteBuf(temp_ram, (buf_addr+i*4));
            }
            if (frac_nbr)
            {
                temp_ram = 0;
                for (j=0; j<frac_nbr; j++)
                    temp_ram |= ((ReadRamb(ram_addr+tran_nbr*4+j))<<(j*8));    
                WriteBuf(temp_ram, (buf_addr+tran_nbr*4));
            }
        }
        else
        {
            for (i=0; i<tran_nbr; i++)
            {
                WriteBuf(ReadRaml(ram_addr+i*4), (buf_addr+i*4));
            }
            if (frac_nbr)
            {
                WriteBuf(ReadRaml(ram_addr+tran_nbr*4), (buf_addr+tran_nbr*4));
            }
        }

        //set buffer status
        if((buf_remain > 0) && (buf_remain <= 60))
        {
            WriteBuf(0, (buf_addr-buf_offset+buf_cnt*64+60));
        }
    }
    else
    {
        //buffer to memory
        if (ram_addr%4)
        {
            for (i=0; i<tran_nbr; i++)
            {
                temp_buf = ReadBuf(buf_addr+i*4);            
                for (j=0; j<4; j++)
                {
                    WriteRamb((rt_uint8_t)((temp_buf>>j*8)&0xff), (ram_addr+i*4+j));
                }
            }
            if (frac_nbr)
            {
                temp_buf = ReadBuf(buf_addr+tran_nbr*4);    
                for (j=0; j<frac_nbr; j++)
                {
                    WriteRamb((rt_uint8_t)((temp_buf>>j*8)&0xff), (ram_addr+tran_nbr*4+j));                
                }
            }
        }
        else
        {
            for (i=0; i<tran_nbr; i++)
            {    
                WriteRaml(ReadBuf(buf_addr+i*4), (ram_addr+i*4));
            }
            if (frac_nbr)
            {
                temp_buf = ReadBuf(buf_addr+tran_nbr*4);    
                temp_ram = ReadRaml(ram_addr+tran_nbr*4);
                temp_buf &= ((1<<(frac_nbr*8+1))-1);
                temp_ram &= ~((1<<(frac_nbr*8+1))-1);
                temp_ram |= temp_buf;
                WriteRaml(temp_ram, (ram_addr+tran_nbr*4));
            }
        }

        //clr buffer status
        if((buf_remain > 0) && (buf_remain <= 60))
        {
            temp_buf = ReadBuf(buf_addr-buf_offset+buf_cnt*64+60);
        }
    }
    return RT_TRUE;
}

/******************************************************************************
* Name: 	 rt_hw_l2_combuf_ctrl 
*
* Desc: 	 控制L2 buf 使能
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/17, Create this function by zhuangwei
 ******************************************************************************/
static void rt_hw_l2_combuf_ctrl(rt_uint8_t buf_id, rt_uint8_t bEnable)
{
    if(bEnable)
    {
        //enable buffer and dma
        REG32(L2_COMBUF_CFG) |= (1 << buf_id) | (1 << (buf_id+16));
    }
    else
    {
        //disable buffer and dma
        REG32(L2_COMBUF_CFG) &= ~((1 << buf_id) | (1 << (buf_id+16)));
    }
}

/******************************************************************************
* Name: 	 rt_hw_l2_select_combuf 
*
* Desc: 	 关联L2和设备模块
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/17, Create this function by zhuangwei
 ******************************************************************************/
static void rt_hw_l2_select_combuf(rt_uint8_t dev, rt_uint8_t buf_id)
{
    rt_uint32_t reg_id;
    rt_uint32_t base_bit;
    rt_uint32_t reg_value;

    if (dev > 9)
    {
        reg_id = L2_ASSIGN_REG2;
        base_bit = (dev-10)*3;
    }
    else
    {
        reg_id = L2_ASSIGN_REG1;
        base_bit = dev*3;
    }

    reg_value = REG32(reg_id);
    reg_value &= ~(0x7<<base_bit);
    reg_value |= ((buf_id&0x7)<<base_bit);
    REG32(reg_id) = reg_value;
}
/******************************************************************************
* Name: 	 rt_hw_l2_alloc 
*
* Desc: 	 给设备分配L2
* Param: 	 
* Return: 	 
* Global: 	 
* Note: 	 
* Author: 	 zhuangwei
* -------------------------------------
* Log: 	 2016/11/17, Create this function by zhuangwei
 ******************************************************************************/
rt_uint8_t rt_hw_l2_alloc(rt_uint8_t dev)
{
    rt_uint32_t buffer_nbr = sizeof(l2_info_table);    
    rt_uint32_t i;    
    rt_uint8_t slct_id=BUF_NULL;
    rt_base_t level;

    //check if device already owns a common buffer
    if(device_info_table[dev] != BUF_NULL)
    {
        return device_info_table[dev];
    }

    level = rt_hw_interrupt_disable();
    
    for (i = 1; i < buffer_nbr; i++)
    {
        if (l2_info_table[i] == L2_STATUS_IDLE)
        {
            slct_id = i;
        } 
    }

    if(slct_id != BUF_NULL)
    {
        l2_info_table[slct_id] = L2_STATUS_USED;
        
        //enable buffer
        rt_hw_l2_combuf_ctrl(slct_id, RT_TRUE);

        //change device info
        device_info_table[dev] = slct_id;

        //select buffer for the device
        rt_hw_l2_select_combuf(dev, slct_id);

        //clear buffer status
        rt_hw_l2_clr_status(slct_id);
    }
    
    rt_hw_interrupt_enable(level);
    
    return slct_id;
}
/*---------------------------------------------------------------------------*/

