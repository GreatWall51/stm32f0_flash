/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author		Notes
 * 2011-01-13     weety		 first version
 * 2015-04-27     ArdaFu     Port bsp from at91sam9260 to asm9260t
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>

//#define RAMFS_SIZE              1024*1024
//#define FLASH_NAME              "flash0"

//typedef struct
//{
//	rt_uint32_t file_length;
//    rt_uint32_t ld_addr;
//    rt_uint32_t start_page;
//    rt_uint32_t backup_page;        //backup data start page
//    rt_uint8_t file_name[16];
//}T_FILE_CONFIG; 

//ALIGN(4)
//static rt_uint8_t ramfs_pool[RAMFS_SIZE];

#ifdef RT_USING_FINSH
extern void finsh_system_init(void);
extern void finsh_set_device(const char* device);
#endif

#ifdef RT_USING_LWIP
extern void eth_system_device_init();
extern void rt_hw_stm32_eth_init();
extern void lwip_sys_init();
#endif

#ifdef RT_USING_DFS
extern void dfs_init(void);
extern int dfs_ramfs_init(void);
#endif

void rt_init_thread_entry(void* parameter)
{
    /* Initialization RT-Thread Components */
//    struct dfs_ramfs* ramfs_root;
    
#ifdef RT_USING_SPI
    rt_hw_spiflash_init();
#endif
//    gd_init(FLASH_NAME, "SPI01");

#ifdef RT_USING_DFS    
    dfs_init();
    
    dfs_ramfs_init();
    
    ramfs_root = dfs_ramfs_create(ramfs_pool, RAMFS_SIZE);
    if(ramfs_root != RT_NULL)
    {
        if (dfs_mount(RT_NULL, "/", "ram", 0, ramfs_root) == 0)
        {
            rt_kprintf("RAM File System initialized!\n");
        }
        else
        {
            rt_kprintf("RAM File System initialzation failed!\n");
        }
    }
#endif

#ifdef RT_USING_LWIP
    {

        /* register ethernetif device */
        eth_system_device_init();
        rt_hw_stm32_eth_init();
        /* init lwip system */
        lwip_sys_init();
    }
#endif
    
#ifdef RT_USING_FINSH
    /* init finsh */
    finsh_system_init();
    finsh_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

}

#ifdef RT_USING_LED
#include "led.h"

void rt_led_thread_entry(void* parameter)
{
    rt_uint8_t cnt = 0;
    led_init();
    while(1)
    {
        /* light on leds for one second */
        rt_thread_delay(1000);
        cnt++;
        if(cnt & 0x01)
            led_on(1);
        else
            led_off(1);
    }
}

#define FILE_CFG_SIZE           256
#define UPDATE_FILE_SECTOR      (16)


//void update(int argc, char **argv)
//{
//    struct dfs_fd fd;
//    struct stat file_stat;
//    rt_device_t flash_dev;
//    T_FILE_CONFIG *file_cfg;
//    rt_uint8_t *buffer;
//    rt_uint32_t sector_cnt;

//    if(argc == 2)
//    {
//        /* get new firmware */
//        if (dfs_file_open(&fd, argv[1], DFS_O_RDONLY) < 0)
//        {
//            rt_kprintf("Open %s failed\n", argv[1]);

//            return;
//        }
//        if (dfs_file_stat(argv[1], &file_stat) != 0)
//        {
//            dfs_file_close(&fd);
//            return;
//        }
//        
//        buffer = (rt_uint8_t *)rt_malloc(file_stat.st_size+FILE_CFG_SIZE*2);

//        flash_dev = rt_device_find(FLASH_NAME);
//        if(flash_dev == RT_NULL)
//        {
//            rt_kprintf("cannot find device %s\n", FLASH_NAME);
//            goto exit;
//        }
//        
//        if (rt_device_open(flash_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
//        {
//            rt_kprintf("cannot open device %s\n", FLASH_NAME);
//            return;
//        }
//        
//        rt_device_read(flash_dev, UPDATE_FILE_SECTOR, buffer, 1);
//        file_cfg = (T_FILE_CONFIG *)(buffer+4+256);
//        
//        dfs_file_read(&fd, buffer+FILE_CFG_SIZE*2, file_stat.st_size);
//        file_cfg->file_length = file_stat.st_size;
//        file_cfg->start_page = UPDATE_FILE_SECTOR*16+2;
//        
//        sector_cnt = (file_stat.st_size+FILE_CFG_SIZE*2+4095)>>12;
//        rt_device_write(flash_dev, UPDATE_FILE_SECTOR, buffer, sector_cnt);

//exit:
//        rt_free(buffer);
//        dfs_file_close(&fd);
//        rt_device_close(flash_dev);
//       
//    }
//    else
//    {
//        rt_kprintf("bad parameter! e.g: update rtthread.bin\n");
//    }
//}
//MSH_CMD_EXPORT(update, update firmware);

static void start_led_thread(void)
{
    rt_thread_t led_thread;
    led_thread = rt_thread_create("led", rt_led_thread_entry, RT_NULL, 512,
                                  (RT_THREAD_PRIORITY_MAX / 8 * 5), 20);
    if(led_thread != RT_NULL)
        rt_thread_startup(led_thread);
}
#endif

int rt_application_init()
{
    rt_thread_t init_thread;
    init_thread = rt_thread_create("init", rt_init_thread_entry, RT_NULL, 2048,
                                   (RT_THREAD_PRIORITY_MAX / 8 * 2), 20);
    if(init_thread != RT_NULL)
        rt_thread_startup(init_thread);

#ifdef RT_USING_LED
    start_led_thread();
#endif
    return 0;
}
