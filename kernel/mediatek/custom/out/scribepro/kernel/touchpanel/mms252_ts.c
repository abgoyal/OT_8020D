/*
 * Touchscreen driver for Melfas MMS-200 series
 *
 * Copyright (C) 2013 Melfas Inc.
 * Author: DVK team <dvk@melfas.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
#include <linux/proc_fs.h>
#include <linux/jiffies.h>
#include <linux/firmware.h>
#include <linux/earlysuspend.h>
#include <linux/irq.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/init.h>

#include <asm/uaccess.h>
#include <cust_eint.h>
#include <asm/unaligned.h>
#include <mach/eint.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_pm_ldo.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>

#include "tpd.h"
#include "mms252_ts.h"


//DiabloX has touch key 
/* Flag to enable touch key */
#define MMS_HAS_TOUCH_KEY	1
#define TPD_HAVE_BUTTON MMS_HAS_TOUCH_KEY

#define TP_DEV_NAME "mms252"
#define I2C_RETRY_CNT 5 //Fixed value
#define DOWNLOAD_RETRY_CNT 5 //Fixed value
#define MELFAS_DOWNLOAD 1 //Fixed value

#define PRESS_KEY 1 //Fixed value
#define RELEASE_KEY 0 //Fixed value

#define TS_READ_LEN_ADDR 0x0F //Fixed value
#define TS_READ_START_ADDR 0x10 //Fixed value
#define TS_READ_REGS_LEN 66 //Fixed value
#define TS_WRITE_REGS_LEN 16 //Fixed value

#define TS_MAX_TOUCH 	10 //Model Dependent
#define TS_READ_HW_VER_ADDR 0xF1 //Model Dependent
#define TS_READ_SW_VER_ADDR 0xF5 //Model Dependent

#define MELFAS_HW_REVISON 0x01 //Model Dependent
#define MELFAS_FW_VERSION 0x0a //Model Dependent

#define MELFAS_MAX_TRANSACTION_LENGTH 66
#define MELFAS_MAX_I2C_TRANSFER_SIZE  7
#define MELFAS_I2C_DEVICE_ADDRESS_LEN 1
//#define I2C_MASTER_CLOCK       400
#define MELFAS_I2C_MASTER_CLOCK       100
#define MELFAS_I2C_ADDRESS   0x20

//add by xingxing.luo
#define MAIN_VERSION			1
#define BUILD_VERSION			6
#define MELFAS_UPDATE_FW		1
#define SYS_DEBUG				1
#define INPUT_PROTOCAL_A		0
#define UPGRADE_NOWAIT			1

#define TOUCH_TYPE_ADD			0x05
#define TOUCH_TYPE_BITS		0x03
#define TOUCH_INDEX_BITS		0x0F
#define TOUCH_ACTION_BITS		0x80

#define MELFAS_ESD_ERROR 		0x0F
#define FINGER_EVENT_SZ			6	

#define FIRMWARE_SIZE		64*1024


#if PROTOCAL_A
#define REPORT_MT_DOWN(touch_number, x, y, width, strength) \
do {     \	
	input_report_abs(tpd->dev, ABS_PRESSURE, strength);  \
	input_report_key(tpd->dev, BTN_TOUCH, 1);\
	input_report_abs(tpd->dev, ABS_PRESSURE, strength);  \
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, width);         \
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, strength); \
	input_mt_sync(tpd->dev);\
} while (0)

#else

#define REPORT_MT_DOWN(touch_number, x, y, width, strength) \
do {     \	
	input_mt_slot(tpd->dev, touch_number);\
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_key(tpd->dev, BTN_TOUCH, 1);\
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, width);         \
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, strength); \
} while (0)

#endif
/*
	input_report_abs(tpd->dev, ABS_PRESSURE, strength);  \
	input_report_key(tpd->dev, BTN_TOUCH, 1);\
	input_report_abs(tpd->dev, ABS_PRESSURE, strength);  \
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, touch_number);\
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);             \
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);             \
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, width);         \
	input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, strength); \
*/

#define REPORT_MT_UP() \
do {   \
    input_report_key(tpd->dev, BTN_TOUCH, 0);  \
    input_mt_sync(tpd->dev);                \
} while (0)

//static int tsp_keycodes[4] = {KEY_MENU, KEY_HOME, KEY_SEARCH, KEY_BACK};

#ifdef TPD_HAVE_BUTTON 
#define TPD_KEY_COUNT	3
#define TPD_KEYS  {  KEY_MENU,KEY_HOMEPAGE,KEY_BACK,}
#define TPD_KEYS_DIM	{{60,850,120,100},{180,850,120,100},{300,850,120,100}}//FIXME
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

/**********************************************************/
/*                                 GLOBAL VARIALES DECLARE                               */
/**********************************************************/
extern struct tpd_device *tpd;

typedef struct muti_touch_info
{
	uint16_t posX;
	uint16_t posY;
	uint16_t width;
	uint16_t pressure;
	unsigned int pressed;
	
	bool update;
};

struct mms_ts_info {
	struct i2c_client 	*client;

	char *fw_name;
};

#if 1
const u8 MELFAS_binary[] = {
#include "MCH_TSCRIBE6_R02_V10.mfsb.h"
};
#define MELFAS_binary_length sizeof(MELFAS_binary)
#endif

static struct muti_touch_info g_Mtouch_info[TS_MAX_TOUCH];
struct completion mms200_init_done;
struct i2c_client *melfas_i2c_client = NULL;
static const struct i2c_device_id melfas_tpd_id[] = {{TP_DEV_NAME,0},{}};
static struct i2c_board_info __initdata melfas_i2c_tpd={ I2C_BOARD_INFO(TP_DEV_NAME, MELFAS_I2C_ADDRESS)};

static int melfas_tpd_flag = 0;
static u8 * DMAbuffer_va = NULL;
static dma_addr_t DMAbuffer_pa = NULL;
kal_uint8 TPD_DBG = 0;

struct sys_fw_info
{
	int file_err_num;
	u8 local_buf[FIRMWARE_SIZE];
};
struct sys_fw_info *sys_fw_info = NULL;
static int sys_err_num = 0;
static int power_statue = 0;

/**********************************************************/

/**********************************************************/
/*					FUNCTION DECLARE                                              */
/**********************************************************/
static DECLARE_WAIT_QUEUE_HEAD(melfas_waiter);
static int melfas_tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int melfas_tpd_i2c_remove(struct i2c_client *client);
static int melfas_tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
extern int isc_fw_download(struct i2c_client *client, const U8 *data, size_t len);
static int melfas_i2c_read(struct i2c_client *client, U16 addr, U16 len, U8 *rxbuf);
int melfas_i2c_DMAread(struct i2c_client *client, U16 addr, U16 len, U8 *rxbuf);
int melfas_i2c_DMAwrite(struct i2c_client *client, U16 addr, U16 len, U8 *txbuf);

static void melfas_ts_release_all_finger(void);
static int melfas_report_data(int fingerID);
static int melfas_proc_touch_data(U8 *buf);
static int download_fw_online(struct i2c_client *client, const char *file_name);
static int update_fw_by_apk(struct i2c_client *client);

extern void mms_fw_update_controller(const struct firmware *fw, struct i2c_client *client);

/**************************************************************/
static struct i2c_driver melfas_tpd_i2c_driver =
{                       
    .probe = melfas_tpd_i2c_probe,                                   
    .remove = __devexit_p(melfas_tpd_i2c_remove),                           
    .detect = melfas_tpd_i2c_detect,                           
    .driver.name = "mtk-tpd", 
    .id_table = melfas_tpd_id,                             
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = melfas_ts_suspend, 
    .resume = melfas_ts_resume,
#endif    
}; 

/*****************************************************************/
/*				              	SYSFS                                                               */
/*****************************************************************/

#define TP_SYSFS_SUPPORT
#ifdef TP_SYSFS_SUPPORT

static ssize_t TP_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
	int ret;
	U8 val[5];

	//power on
//	if (power_statue == 0) 
	{
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(200);
	}
	
	ret = melfas_i2c_read(melfas_i2c_client, 0xc1, 1, &val[0]);
	ret = melfas_i2c_read(melfas_i2c_client, 0xc2, 1, &val[1]);
	ret = melfas_i2c_read(melfas_i2c_client, 0xc3, 1, &val[2]);
	ret = melfas_i2c_read(melfas_i2c_client, 0xc4, 1, &val[3]);
	ret = melfas_i2c_read(melfas_i2c_client, 0xc5, 1, &val[4]);	

	s += sprintf(s, "0xc1:   0x%x \n", val[0]);
	s += sprintf(s, "0xc2:   0x%x \n", val[1]);
	s += sprintf(s, "0xc3:   0x%x \n", val[2]);
	s += sprintf(s, "0xc4:   0x%x \n", val[3]);
	s += sprintf(s, "0xc5:   0x%x \n", val[4]);
	return (s - buf);
}

static ssize_t TP_value_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	char *s = buf;
	int save;
	ssize_t ret_val;
	char cmd_str[16] = {0};
	unsigned int val; 
	sscanf(buf, "%s %d", (char *)&cmd_str, &val);

	if(strcmp(cmd_str, "DEBUG") == 0)
	{
	    if (val == 1)
	        TPD_DBG = 1;
	    else
	        TPD_DBG = 0;
	}
   
	ret_val = (s - buf);
	return ret_val;
}


static ssize_t get_driver_version(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	int count = 0;
	
	count += snprintf(buf + count, PAGE_SIZE - count, "TP_DRIVER_VERSION: %d.%d\n",
		MAIN_VERSION,BUILD_VERSION);

	return count;
}

static ssize_t update_fw_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	int count = 0;
	return snprintf(buf, PAGE_SIZE, "sys_err_num: %d\n",sys_err_num);
}

static ssize_t update_fw_store(struct device *dev,
	struct device_attribute *attr, const char *buf,size_t count)
{
	int ret = 0;
	const char *file_name = buf;

	sys_err_num = 0;
	
	printk(KERN_ERR "tpd_sys:Start_update_firmware_online...\n");

	if (sscanf(buf, "%s", file_name) < 0){
		printk("tpd_sys:input_error!\n");
		return -EINVAL;
	}
	printk(KERN_ERR "tpd_sys:local_buf:*%s*\n",file_name);
	sys_err_num ++;     // 1
	
	sys_err_num++;	    // 2
	printk(KERN_ERR "tpd_sys:Enter_online_mode!\n");
	
#if 1	
	sys_fw_info = (struct sys_fw_info *) kzalloc(sizeof(struct sys_fw_info),GFP_KERNEL);
	if (sys_fw_info == NULL)
	{
		printk(KERN_ERR "******tpd_sys:malloc_failed!****\n");
		return count;
	}

	ret = download_fw_online(melfas_i2c_client,file_name);	
	if (ret < 0)
	{
		printk(KERN_ERR "tpd_sys:download_fw_online failed!\n");
	}
	kfree(sys_fw_info);
	printk(KERN_ERR "******tpd_sys:free!******");
#endif				
	return count;
}

static DEVICE_ATTR(TP_DEBUG, 0644,  TP_value_show, TP_value_store);  
static DEVICE_ATTR(DRIVER_VERSION, 0644 ,  get_driver_version, NULL); 
static DEVICE_ATTR(update_fw, 0644, update_fw_show, update_fw_store); 

static struct attribute *TP_sysfs_attrs[] = {
	&dev_attr_TP_DEBUG.attr,
	&dev_attr_DRIVER_VERSION.attr,
	&dev_attr_update_fw.attr,
	NULL,
};
static struct attribute_group TP_attr_group = {
        .attrs = TP_sysfs_attrs,
};

//add sysfs
struct kobject *TP_ctrl_kobj;
static int TP_sysfs_init(void)
{ 
	TP_ctrl_kobj = kobject_create_and_add("tp-info", NULL);
	if (!TP_ctrl_kobj)
		return -ENOMEM;

	return sysfs_create_group(TP_ctrl_kobj, &TP_attr_group);
}
//remove sysfs
static void TP_sysfs_exit(void)
{
	sysfs_remove_group(TP_ctrl_kobj, &TP_attr_group);

	kobject_put(TP_ctrl_kobj);
}

static int download_fw_online(struct i2c_client *client, const char *file_name)
{
	size_t size = 0;
	int i  = 0;
	struct file *fp = NULL;
	struct inode *inode = NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	off_t fsize; 
	unsigned long magic; 	

	printk(KERN_ERR "tpd_sys:download_fw_online!\n");
	sys_err_num++;	// 3
	
	//STEP 1: open the file
#if 1
	fp = filp_open(file_name, O_RDONLY , 0777);
	
	if (IS_ERR(fp))
	{
		printk(KERN_ERR "tpd_sys:open_firmware_file_failed!\n");
		return -1;
	}
	inode=fp->f_dentry->d_inode;
	fsize=inode->i_size;
//	magic=inode->i_sb->s_magic; 

	fs = get_fs();
//	set_fs(KERNEL_DS);
	set_fs(get_ds());

	sys_err_num++;	// 4
#endif 

#if 1
	//STEP 2:read the file to buffer	
	size = fp->f_op->read(fp,sys_fw_info->local_buf,fsize,&(fp->f_pos));
	if (size < 0)
	{
		printk(KERN_ERR "tpd_sys:read_firmware_file_failed!\n");
		return -2;		
	}
	
#if SYS_DEBUG
	printk(KERN_ERR "*****tpd_sys:fsize %d****\n", fsize);
	printk(KERN_ERR "*****tpd_sys:size %d****\n", size);
	printk(KERN_ERR "*****tpd_sys:MELFAS_binary_length %d****\n", MELFAS_binary_length);
#endif 

	filp_close(fp, NULL);
	set_fs(fs);
	
	sys_err_num++;	// 5

	//STEP 3: download the firmware		

	//power on
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(200);
	
	struct firmware fw_info = 
	{
		.size = size,
		.data = &sys_fw_info->local_buf[0],
	};
	mms_fw_update_controller(&fw_info,client);

#endif 
	return 0;	
}


#endif
/*****************************End of SYSFS*************************/


/*****************************************************************/
/*				              	ADD FOR APP                                                      */
/*****************************************************************/
static int update_fw_by_apk(struct i2c_client *client)
{
	int ret = 0;
	struct mms_ts_info *info;
	const char *fw_name = "mms_ts.fw";
	
	struct firmware fw_info = 
	{
//		.size = size,
//		.data = &sys_fw_info->local_buf[0],
	};
	printk(KERN_ERR "tpd_update:update_fw_by_apk\n");
	
	info = (struct mms_ts_info *)kzalloc(sizeof(*info), GFP_KERNEL);
	info->client = client;

	info->fw_name = kstrdup(fw_name, GFP_KERNEL);

#if UPGRADE_NOWAIT	
	ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &client->dev,
				GFP_KERNEL, info, mms_fw_update_controller);
#else
	ret = request_firmware(&fw_info,fw_name,&client->dev);
#endif

//	if(ret < 0)
	{
		printk(KERN_ERR "*****tpd_update: request_firmware:%d****\n",ret);
	}

	printk(KERN_ERR "tpd_update:%d,%d\n",fw_info.size,fw_info.data);
	kfree(info);
	return ret;
}

/***********************End of APP Interface***************************/

void touchkey_handler(u8 key, bool on)
{
	int i;
	switch(key)
	{
/*	
		case 1:
		    TPD_DEBUG("MMS_TOUCH_KEY_EVENT BACK, %d \n",on);
		    input_report_key(tpd->dev, KEY_BACK, on);
		break;

		case 2:
		    TPD_DEBUG("MMS_TOUCH_KEY_EVENT HOME, %d \n",on);
		    input_report_key(tpd->dev, KEY_HOMEPAGE, on);
		break;

		case 3:
		    TPD_DEBUG("MMS_TOUCH_KEY_EVENT MENU, %d \n",on);
		    input_report_key(tpd->dev, KEY_MENU, on);
		break;
*/

		case 1:
		    TPD_DEBUG("MMS_TOUCH_KEY_EVENT BACK, %d \n",on);
		    input_report_key(tpd->kpd, KEY_BACK, on);
		break;

		case 2:
		    TPD_DEBUG("MMS_TOUCH_KEY_EVENT HOME, %d \n",on);
		    input_report_key(tpd->kpd, KEY_HOMEPAGE, on);
		break;

		case 3:
		    TPD_DEBUG("MMS_TOUCH_KEY_EVENT MENU, %d \n",on);
		    input_report_key(tpd->kpd, KEY_MENU, on);
		break;

		default:
		break;
	}
	input_sync(tpd->kpd);
}



static void esd_rest_tp(void)
{
	TPD_DEBUG("==========tp have inter esd =============\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
   	msleep(50);
    	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
}

static int melfas_report_data(int fingerID)
{

	if(g_Mtouch_info[fingerID].pressure == 0)
	{
#if INPUT_PROTOCAL_A
		// release event  protocal A  
		REPORT_MT_UP();  
#else		
		input_mt_slot(tpd->dev, fingerID);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);
#endif
	}
	else
	{
		REPORT_MT_DOWN(fingerID, 
		g_Mtouch_info[fingerID].posX, 
		g_Mtouch_info[fingerID].posY, 
		g_Mtouch_info[fingerID].width, 
		g_Mtouch_info[fingerID].pressure);
	}
	printk("[MMS252]: Touch ID: %d, pressure : %d, x: %d, y: %d, area: %d pressure: %d\n",fingerID,  
		g_Mtouch_info[fingerID].posX, 
		g_Mtouch_info[fingerID].posY,
		g_Mtouch_info[fingerID].width,
		g_Mtouch_info[fingerID].pressure
		);
	return 0;
}

static int melfas_proc_touch_data(U8 *buf)
{	
	int i;
	int read_num; 
	int fingerID;
	int Touch_Type = 0;
	int Touch_Action = 0;
	
	//check the packet size
	melfas_i2c_read(melfas_i2c_client, TS_READ_LEN_ADDR, 1, buf);
	read_num = buf[0];
	TPD_DEBUG("melfas_touch_event_handler,read_num = %d  \n",read_num);
	
	if(read_num)
	{
		if (read_num <= 8)
    			melfas_i2c_read(melfas_i2c_client, TS_READ_START_ADDR, read_num, buf);
		else if (read_num > 8)
    			melfas_i2c_DMAread(melfas_i2c_client, TS_READ_START_ADDR, read_num, buf);

		if (((read_num % 6)!= 0) || (read_num > 60))
		{
			melfas_ts_release_all_finger();
			esd_rest_tp();
			return -1;
		}
		
          	if(MELFAS_ESD_ERROR == buf[0])
           	{
			esd_rest_tp();
			return -1;
	 	}

		for (i = 0; i < read_num; i = i + FINGER_EVENT_SZ)
            {
			/*get Touch_Type:(01:finger input, 10:touch key area)*/
			Touch_Type = (buf[i] >> TOUCH_TYPE_ADD) & TOUCH_TYPE_BITS;
			TPD_DEBUG("%s : touch type = %d, buf[i] = %x \n",__FUNCTION__,Touch_Type, buf[i]);

			/* touch type is panel(touch key area) */
			if (Touch_Type == MMS_TOUCH_KEY_EVENT)
			{
				touchkey_handler((buf[i] & 0x0f),(bool)(buf[i] & 0x80));
			}
			else
			{
				//get the touch ID
				fingerID = (buf[i] & TOUCH_INDEX_BITS) - 1;		

				//get the touch action	 
				Touch_Action = (buf[i] & TOUCH_ACTION_BITS);

				//fingerID check
				if ((fingerID < 0) || (fingerID >TS_MAX_TOUCH )) 
				{
					TPD_DMESG("illegal finger id (id: %d)\n", fingerID);
					continue;
				}

				//Touch_Action check
				if (!Touch_Action && !g_Mtouch_info[fingerID].pressed) 
				{
					TPD_DMESG("Wrong touch release (id: %d)\n", fingerID);
					continue;
				}
				
				g_Mtouch_info[fingerID].posX = (uint16_t)(buf[i + 2] | ((buf[i + 1] & 0xf) << 8));
				g_Mtouch_info[fingerID].posY = (uint16_t)(buf[i + 3] | (((buf[i + 1] >> 4 ) & 0xf) << 8));
				g_Mtouch_info[fingerID].width = buf[i + 4];
				g_Mtouch_info[fingerID].pressure = Touch_Action ? buf[i + 5] : 0;
				g_Mtouch_info[fingerID].pressed = Touch_Action;

				melfas_report_data(fingerID);
			}
                
		}
		if ( tpd != NULL && tpd->dev != NULL )
            		input_sync(tpd->dev);
	}

	return 0;	
}

static int melfas_touch_event_handler(void *unused)
{
#if 1
	
	int ret = 0;
	U8 buf[TS_READ_REGS_LEN] = {0};
	
//	int i, read_num;//, keyID = 0;
	//uint8_t buf_esd[2]={0};
	
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD }; 
    sched_setscheduler(current, SCHED_RR, &param); 

    do
    {
    		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE);

		wait_event_interruptible(melfas_waiter, melfas_tpd_flag != 0);

		melfas_tpd_flag = 0;
		set_current_state(TASK_RUNNING); 
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		
		ret = melfas_proc_touch_data(buf);
		if (ret < 0)
		{
			continue;
		}		
    } while ( !kthread_should_stop() ); 

    return 0;
#endif
}

static void melfas_i2c_tpd_eint_interrupt_handler(void)
{ 
    TPD_DEBUG_PRINT_INT;
    melfas_tpd_flag=1;
    wake_up_interruptible(&melfas_waiter);
} 

int melfas_i2c_write_bytes( struct i2c_client *client, U16 addr, int len, U32 *txbuf )
{
    u8 buffer[MELFAS_MAX_TRANSACTION_LENGTH]={0};
    u16 left = len;
    u8 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg = 
    {
        .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
        .flags = 0,
        .buf = buffer,
        .timing = MELFAS_I2C_MASTER_CLOCK,
    };


    if ( txbuf == NULL )
        return -1;

    TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        retry = 0;

        buffer[0] = (u8)addr+offset;

        if ( left > MELFAS_MAX_I2C_TRANSFER_SIZE )
        {
            memcpy( &buffer[MELFAS_I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MELFAS_MAX_I2C_TRANSFER_SIZE );
            msg.len = MELFAS_MAX_TRANSACTION_LENGTH;
            left -= MELFAS_MAX_I2C_TRANSFER_SIZE;
            offset += MELFAS_MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy( &buffer[MELFAS_I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
            msg.len = left + MELFAS_I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

        TPD_DEBUG("byte left %d offset %d\n", left, offset );

        while ( i2c_transfer( client->adapter, &msg, 1 ) != 1 )
        {
            retry++;

            if ( retry == I2C_RETRY_CNT )
            {
                TPD_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                TPD_DMESG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
            else
                 TPD_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);

        }
    }

    return 0;
}


static int melfas_i2c_read(struct i2c_client *client, U16 addr, U16 len, U8 *rxbuf)
{
    u8 buffer[MELFAS_I2C_DEVICE_ADDRESS_LEN]={0};
    u8 retry;
    u16 left = len;
    u8 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
            .flags = 0,
            .buf = buffer,
            .len = MELFAS_I2C_DEVICE_ADDRESS_LEN,
            .timing = MELFAS_I2C_MASTER_CLOCK
        },
        {
            .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
            .flags = I2C_M_RD,
            .timing = MELFAS_I2C_MASTER_CLOCK
        },
    };

    if ( rxbuf == NULL )
        return -1;

    TPD_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        buffer[0] = (u8)addr+offset;

        msg[1].buf = &rxbuf[offset];

        if ( left > MELFAS_MAX_TRANSACTION_LENGTH )
        {
            msg[1].len = MELFAS_MAX_TRANSACTION_LENGTH;
            left -= MELFAS_MAX_TRANSACTION_LENGTH;
            offset += MELFAS_MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while ( i2c_transfer( client->adapter, &msg[0], 2 ) != 2 )
        {
            retry++;

            if ( retry == I2C_RETRY_CNT )
            {
                TPD_DEBUG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}

int melfas_i2c_DMAread(struct i2c_client *client, U16 addr, U16 len, U8 *rxbuf)
{
	int retry,i;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG,
			.flags = I2C_M_RD,
			.len = len,
			.buf = DMAbuffer_pa,
		}
	};
	for (retry = 0; retry < I2C_RETRY_CNT; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(5);
	}
	if (retry == I2C_RETRY_CNT) {
		printk(KERN_ERR "i2c_read_block retry over %d\n",I2C_RETRY_CNT);
		return -EIO;
	}
	
	for(i=0;i<len;i++)
        rxbuf[i]=DMAbuffer_va[i];
	
	return 0;
}
int melfas_i2c_DMA_RW_isc(struct i2c_client *client, 
                            int rw,
                            U8 *rxbuf, U16 rlen,
                            U8 *txbuf, U16 tlen)
{
	int retry,i;
	
	struct i2c_msg msg[] = {
		{
			.addr = client->addr & I2C_MASK_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG,
			.flags = 0,
			.buf = DMAbuffer_pa,
		}
	};
	/*
	rw = 1 //write
	rw = 2 //read
	*/
	if (rw == 1)//write
	{
		for(i=0;i<tlen;i++)
		    DMAbuffer_va[i] = txbuf[i];
		msg[0].flags = 0;
		msg[0].len = tlen;
		for (retry = 0; retry < I2C_RETRY_CNT; retry++) 
		{
			if (i2c_transfer(client->adapter, msg, 1) == 1)
			    break;
			mdelay(5);
		}
		if (retry == I2C_RETRY_CNT) {
			printk(KERN_ERR "i2c_read_block retry over %d\n",I2C_RETRY_CNT);
			return -EIO;
		}
	}
	else if (rw == 2)//read
	{
	    //write from TX
		for(i=0;i<tlen;i++)
		    DMAbuffer_va[i] = txbuf[i];
		msg[0].flags = 0;
		msg[0].len = tlen;
		for (retry = 0; retry < I2C_RETRY_CNT; retry++) 
		{
			if (i2c_transfer(client->adapter, msg, 1) == 1)
			    break;
			mdelay(5);
		}
		if (retry == I2C_RETRY_CNT) {
			printk(KERN_ERR "i2c_read_block retry over %d\n",I2C_RETRY_CNT);
			return -EIO;
		}
		//read into RX
		msg[0].flags = I2C_M_RD;
		msg[0].len = rlen;
		for (retry = 0; retry < I2C_RETRY_CNT; retry++) 
		{
			if (i2c_transfer(client->adapter, msg, 1) == 1)
			    break;
			mdelay(5);
		}
		if (retry == I2C_RETRY_CNT) {
			printk(KERN_ERR "i2c_read_block retry over %d\n",I2C_RETRY_CNT);
			return -EIO;
		}
        
		for(i=0;i<rlen;i++)
		     rxbuf[i] = DMAbuffer_va[i];
    }
	
	return 0;
}

int melfas_check_firmware(struct i2c_client *client)
{
    int ret = 0;
    //uint8_t i = 0;
    u8 val[5];

  	ret = melfas_i2c_read(client, 0xc1, 1, &val[0]);
	ret = melfas_i2c_read(client, 0xc2, 1, &val[1]);
	ret = melfas_i2c_read(client, 0xc3, 1, &val[2]);
	ret = melfas_i2c_read(client, 0xc4, 1, &val[3]);
	ret = melfas_i2c_read(client, 0xc5, 1, &val[4]);	

    if (ret >= 0)
    {
        TPD_DMESG("[melfas_tpd]: 0XC1[0x%02x],0XC2[0x%02x],0Xc3[0x%02x],0XC4[0x%02x],0XC5[0x%02x]\n",
                    val[0], val[1],val[2],val[3],val[4]);
                    
        if (val[2] != MELFAS_FW_VERSION || 0xff == val[2])
            ret = 1;//need update
        else
            ret = 0;//need NOT update
            
        TPD_DMESG("[melfas_tpd]: MMS_FW_VERSION is 0x%02x",
                    val[2]);
        goto out;
    }
    else if (ret < 0)
    {
        TPD_DMESG("[melfas_tpd] %s,%d: i2c read fail[%d] \n", __FUNCTION__, __LINE__, ret);
        goto out;
    }

out:	
	return ret;	//-1
}



#if 1
//#define FW_NAME "MCH_TDIABLOX_R01_V02.mfsb"
//static const char *fw_name = FW_NAME;

struct firmware fw_info = 
{
	.size = MELFAS_binary_length,
	.data = &MELFAS_binary[0],
};

static int melfas_firmware_update(struct i2c_client *client)
{
    	int ret = 0;
	int ver = 0;	
	u8 val[5] ;
   	 ret = melfas_check_firmware(client);
    	if (ret == 1)
    	{
#if MELFAS_DOWNLOAD
		int ver;
		//       MELFAS_binary = kstrdup(fw_name,GFP_KERNEL);
		//    	 ret = request_firmware_nowait(THIS_MODULE, true, fw_name, &client->dev,
		//   		GFP_KERNEL, client, mms_fw_update_controller);
		TPD_DMESG("[melfas_tpd] MELFAS_binary_length = %x\n", MELFAS_binary_length);
		TPD_DMESG("[melfas_tpd] MELFAS_binary[0] = %x, addr = %x\n", fw_info.data[0],fw_info.data);
		mms_fw_update_controller(&fw_info,client);
		mdelay(50);
		
		ret = melfas_i2c_read(client, 0xc3, 1, &val[2]);
		if (ver >= 0)
		{
			if(val[2] == MELFAS_FW_VERSION)
				ret = 1;
			else
				ret = -2;
		}
	
#endif
    }
    return ret;
}
#endif 

void  mms_reboot(void)
{
	TPD_DMESG("mms_reboot\n");
	hwPowerDown(MT65XX_POWER_LDO_VGP4, "TP");
	//init GPIO pin
	//init CE //GPIO_CTP_RST_PIN, is CE
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	
	//init EINT, mask CTP EINT //GPIO_CTP_EINT_PIN, is RSTB
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_DOWN);
//	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	msleep(10);  //dummy delay here

	//turn on VDD33, LDP_VGP4
	hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_3300, "TP");
	mdelay(10);  //tce, min is 0, max is ?


	//set RST to HIGH
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	mdelay(3);  //tpor, min is 1, max is 5

	//set EINT to HIGH 
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	mdelay(20);//t boot_core, typicl is 20, max is 25ms
	msleep(300);
}



static int melfas_tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{             
	int err = 0;
	u8 chip_info = 0;
	int ret = 0;
    
	struct task_struct *thread = NULL;
	
	TPD_DMESG("[melfas_tpd] %s\n", __func__);
	
	//init_completion(&mms200_init_done);
	
#if 1//def TP_SYSFS_SUPPORT
    //TP sysfs debug support
	TP_sysfs_init();
#endif

    //TP DMA support
    if(DMAbuffer_va == NULL)
        DMAbuffer_va = (u8 *)dma_alloc_coherent(NULL, 4096,&DMAbuffer_pa, GFP_KERNEL);
    
    TPD_DMESG("dma_alloc_coherent va = 0x%8x, pa = 0x%8x \n",DMAbuffer_va,DMAbuffer_pa);
    if(!DMAbuffer_va)
    {
        TPD_DMESG("Allocate DMA I2C Buffer failed!\n");
        return -1;
    }
	
	
reset_proc:
    mms_reboot();
    
	melfas_i2c_client = client;        
//	update_fw_by_apk(client);
	
#if MELFAS_UPDATE_FW	
	ret = melfas_firmware_update(client);
	
	if (ret == -1)
	{
		//It was i2c transfor error
		printk(KERN_ERR "'tpd_update: No_TP_here!\n");
	}
	else if (ret == 0)
	{
		printk(KERN_ERR "'tpd_update: Needn't upgrade firmware.\n");
	}
	else if (ret == 1)
	{
		printk(KERN_ERR "'tpd_update: upgrade firmware.\n");
	}
	else if (ret == -2)
	{
		printk(KERN_ERR "'tpd_update: upgrade firmware failed.\n");
		//goto reset_proc;
	}
#endif

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, melfas_i2c_tpd_eint_interrupt_handler, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mdelay(5);
	 
	thread = kthread_run(melfas_touch_event_handler, 0, TPD_DEVICE);

	if (IS_ERR(thread))
	{ 
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE "[melfas_tpd] failed to create kernel thread: %d\n", err);
	}

	if ((melfas_i2c_read(melfas_i2c_client, MMS_CHIP_INFO, 1, &chip_info)) < 0)    
	{
		TPD_DMESG("I2C transfer error, line: %d %d\n", __LINE__ ,chip_info);
		return -1; 		   
	}
	
	tpd_load_status = 1;
	return 0;

}

static int melfas_tpd_i2c_remove(struct i2c_client *client)
{
	if(DMAbuffer_va)
	{
		dma_free_coherent(NULL, 4096, DMAbuffer_va, DMAbuffer_pa);
		DMAbuffer_va = NULL;
		DMAbuffer_pa = 0;
	}

    return 0;
}
static int melfas_tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	TPD_DMESG("[melfas_tpd] %s\n", __func__);
	strcpy(info->type, "mtk-tpd");
	return 0;
}
static int melfas_tpd_local_init(void) 
{

	TPD_DMESG("[melfas_tpd] end %s, %d\n", __FUNCTION__, __LINE__);  
	if(i2c_add_driver(&melfas_tpd_i2c_driver)!=0)
	{
	    TPD_DMESG("[melfas_tpd] unable to add i2c driver.\n");
	    return -1;
	}
	if(tpd_load_status == 0)
	{
		TPD_DMESG("[melfas_tpd] add error touch panel driver.\n");
		i2c_del_driver(&melfas_tpd_i2c_driver);
		return -1;
	}
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
//	tpd_button_init();

#endif   

	input_mt_init_slots(tpd->dev, TS_MAX_TOUCH);
	tpd_type_cap = 1;

	return 0;
}


#ifdef SLOT_TYPE
static void melfas_ts_release_all_finger(void)
{
	int i;
	TPD_DMESG("[melfas_tpd] %s\n", __func__);
	for (i = 0; i < TS_MAX_TOUCH; i++)
	{
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
	}
	input_sync(tpd->dev);
}
#else	
static void melfas_ts_release_all_finger(void)
{
	int i;
    TPD_DMESG("[melfas_tpd] %s\n", __func__);

	for(i=0; i<TS_MAX_TOUCH; i++)
	{
#if INPUT_PROTOCAL_A
		if(-1 == g_Mtouch_info[i].pressure)
			continue;

		if(g_Mtouch_info[i].pressure == 0)
			input_mt_sync(tpd->dev);

		if(0 == g_Mtouch_info[i].pressure)
			g_Mtouch_info[i].pressure = -1;
#else
		input_mt_slot(tpd->dev, i);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, -1);	
#endif
	}
	input_sync(tpd->dev);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_tpd_early_suspend(struct early_suspend *h)
{
	TPD_DMESG("[melfas_tpd] %s\n", __func__);

	melfas_ts_release_all_finger();

	//irq mask
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	//power down
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);

}

static void melfas_tpd_late_resume(struct early_suspend *h)
{
    //int ret;
    //struct melfas_ts_data *ts = i2c_get_clientdata(client);

	TPD_DMESG("[melfas_tpd] %s\n", __func__);

	//power on
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(200);
	//irq unmask
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

}
#endif

static struct tpd_driver_t melfas_tpd_device_driver =
{
    .tpd_device_name = "melfas_mms252",
    .tpd_local_init = melfas_tpd_local_init,
#ifdef CONFIG_HAS_EARLYSUSPEND    
    .suspend = melfas_tpd_early_suspend,
    .resume = melfas_tpd_late_resume,
#endif
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif		
};

/* called when loaded into kernel */
static int __init melfas_tpd_driver_init(void)
{
    TPD_DMESG("[melfas_tpd] %s\n", __func__);
	i2c_register_board_info(0, &melfas_i2c_tpd, 1);
    if ( tpd_driver_add(&melfas_tpd_device_driver) < 0)
        TPD_DMESG("[melfas_tpd] add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit melfas_tpd_driver_exit(void)
{
    TPD_DMESG("[melfas_tpd] %s\n", __func__);
    tpd_driver_remove(&melfas_tpd_device_driver);
}


module_init(melfas_tpd_driver_init);
module_exit(melfas_tpd_driver_exit);

