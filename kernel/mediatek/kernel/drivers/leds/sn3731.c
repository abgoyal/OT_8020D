#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <asm/unistd.h>
#include <linux/kthread.h>

#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/mutex.h>


#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <mach/mt_reg_base.h>
#include <mach/mt_boot.h>
#include <mtk_kpd.h>            
#include <mach/irqs.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>

#ifdef MT6577
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include <linux/aee.h>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/leds-mt65xx.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>

#include <cust_leds.h>
#include <mach/mt_pwm.h>
//#include <mach/mt_gpio.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
//#include <mach/mt_pmic_feature_api.h>
//#include <mach/mt_boot.h>

static int debug_enable = 1;
#define LEDS_DEBUG(format, args...) do{ \
	if(debug_enable) \
	{\
		printk(KERN_EMERG format,##args);\
	}\
}while(0)


#define NLED_BREATH 0
#define NLED_MUSIC 1
#define NLED_IMAGE 2

static struct sn3731_data *sn3731_private_data;
struct task_struct *sn3731_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static const struct i2c_device_id sn3731_i2c_id[] = {{"sn3731",0},{}};

static struct i2c_board_info __initdata i2c_sn3731={I2C_BOARD_INFO("sn3731", (0xE8>>1))};

static struct mt65xx_led_data *led;

static unsigned char mode = 0;
static unsigned char frame_count = 0;
static unsigned char loop_count = 0; 
static unsigned char time1 = 0;
static unsigned char time2 =0;
static unsigned int time[4];
static unsigned duration = 0;
static unsigned char image[8][11];
static struct work_struct *sn_eint_work;

struct sn3731_data {
        struct i2c_client *client;
	 struct mutex mutex;
        struct mt65xx_led_data *sn_led_data;
        atomic_t                early_suspend;
};

struct mt65xx_led_data {
	struct led_classdev cdev;
	struct cust_mt65xx_led cust;
	struct work_struct work;
	int level;
	int delay_on;
	int delay_off;
};

static int write_reg(u8 addr, u8 cmd)
{
        char buf[2];
        int ret = -1;

        buf[0] = addr;
        buf[1] = cmd;
	 mutex_lock(&sn3731_private_data->mutex);
	 //sn3731_private_data->client->ext_flag=((sn3731_private_data->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
	// g_pstI2Cclient->addr =(i2cId >> 1);
   	sn3731_private_data->client->ext_flag = (sn3731_private_data->client->ext_flag)&(~I2C_DMA_FLAG);
	//sn3731_private_data->client->ext_flag = (sn3731_private_data->client->ext_flag)&(~I2C_DMA_FLAG);
	 printk("sn3731 write reg %d------------------------------------\n",addr);
	 printk("sn3731 write reg addr:%d\n",sn3731_private_data->client->addr);
	 sn3731_private_data->client->addr&=I2C_MASK_FLAG;
        ret = i2c_master_send(sn3731_private_data->client, buf, 2);
        if (ret < 0){
			//sn3731_private_data->client->ext_flag = 0;
		  mutex_unlock(&sn3731_private_data->mutex);
                printk("sn3731: write reg reg %d failed! \n",addr);
                return -1;
        }
        else
        {
        printk("sn3731: write reg reg %d success! \n",addr);
		}
	// sn3731_private_data->client->ext_flag = 0;
	 mutex_unlock(&sn3731_private_data->mutex);
        return 0;
}

static kal_uint32 read_reg(u8 addr_read)
{
        int ret;
	char buf[1];
	buf[0] = addr_read;
	mutex_lock(&sn3731_private_data->mutex);
	//sn3731_private_data->client->ext_flag=((sn3731_private_data->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	sn3731_private_data->client->ext_flag = (sn3731_private_data->client->ext_flag)&(~I2C_DMA_FLAG);
	
	printk("sn3731 read reg-----------------------------------\n");
        i2c_master_send(sn3731_private_data->client, buf, 1);
	udelay(31);
	char buf_rec[1];
        ret = i2c_master_recv(sn3731_private_data->client, buf_rec, 1);
        if (ret < 0)
	{		//sn3731_private_data->client->ext_flag = 0;
		  mutex_unlock(&sn3731_private_data->mutex);
                printk("sn3731: i2c read error\n");
		return -1;
	}	
	//sn3731_private_data->client->ext_flag = 0;
	 mutex_unlock(&sn3731_private_data->mutex);
        return buf_rec[0];
}

kal_uint32 sn3731_read_interface (kal_uint8 RegNum)
{
	kal_uint32 i = 0;
	i = read_reg(RegNum);
	return i;
}

static int  sn3731_config_interface (kal_uint8 RegNum, kal_uint8 val)
{
	int ret;
	ret = write_reg(RegNum,val);
	return ret;
	
}

static int sn3731_init_frame(void)
{
	int i;
	int ret = 0;
	for(i = 0; i < 18; i++ )
	{
		ret = sn3731_config_interface(i,0x0);
		if(ret < 0)
			return ret;
	}
	for(i = 18; i < 172; i++ )
	{
		ret = sn3731_config_interface(i,0xFF);
		if(ret < 0)
			return ret;
	}
	
	return ret;
}

static int  sn3731_init(void)
{
	int ret;
	int i;
	for(i = 0; i < 8; i++)
	{
		ret = sn3731_config_interface(0xFD,i);
		if(ret < 0)
			return ret;
		ret = sn3731_init_frame();
		if(ret < 0)
			return ret;
	}
	ret = sn3731_config_interface(0xFD,0xB);
	if(ret < 0)
		return ret;
	for(i = 0;i < 13;i++)
	{
		ret = sn3731_config_interface(i,0);
		if(ret < 0)
			return ret;
	}
	return ret;
       
	
}

int set_led_time_AB(int FT)
{	
	int  AB = 0;
	if(FT <= 48)
	{
		AB = 0;
	}else
	if(FT > 48 && FT <=96)
	{
		AB = 1;
	}else
	if(FT > 96 && FT <=192)
	{
		AB = 2;
	}else
	if(FT > 192 && FT <=384)
	{
		AB = 3;
	}else
	if(FT > 384 && FT <=768)
	{
		AB = 4;
	}else
	if(FT > 768 && FT <=1536)
	{
		AB = 5;
	}else
	if(FT > 1536 && FT <=3072)
	{
		AB = 6;
	}else
	if(FT > 3072 )
	{
		AB = 7;
	}
	
	return AB;	
}

void set_led_breath()
{
	int i,j,k,a;
	int multiple = 0;
	int frame_num = 0;
	int loop_frame_num;
	int ret;
	int A,B;
	//sn3731_init();
	duration = time[0] + time[1] +time[2];
	multiple = time[3] / duration;
	ret = sn3731_config_interface(0xFD,0xB);
	if(ret >= 0)
	{
		ret = sn3731_config_interface(0x0,0x8);
		if(ret < 0)
			goto err;
		do{	
			loop_frame_num = multiple * frame_count + frame_count;
			if(loop_frame_num > 8)
				multiple = multiple/2;
		}while(loop_frame_num > 8 && multiple>0);
			
		ret = sn3731_config_interface(0x2,((loop_count << 4) + loop_frame_num));
		if(ret < 0)
			goto err;
		a = time[1] / 15;
		if(a>=64)
		{
			ret = sn3731_config_interface(0x3,0);
			if(ret < 0)
				goto err;
		}else
		{
			ret = sn3731_config_interface(0x3,a);
			if(ret < 0)
				goto err;
		}
		
		A = set_led_time_AB(time[0]);                                                                                                                   
		B = set_led_time_AB(time[2]); 
		printk("sn3731      A = %d,    B=%d    a=%d     multiple=%d    loop_frame_num=%d\n",A,B,a,multiple,loop_frame_num);
		ret = sn3731_config_interface(0x8,((A << 4)+B));
		if(ret < 0)
			goto err;
		if((time[0]+time[2]) == 0)
		{
			ret = sn3731_config_interface(0x9,0x0);
			if(ret < 0)
				goto err;
		}else
		{
			ret = sn3731_config_interface(0x9,0x10);
			if(ret < 0)
				goto err;
		}
		ret = sn3731_config_interface(0xa,0x1);
		if(ret < 0)
			goto err;
		
		for(i = 0;i <= frame_count;i++)
		{
			ret = sn3731_config_interface(0xFD,frame_num);
			if(ret < 0)
				goto err;
			for(j = 0; j < 9; j++)
			{
				ret = sn3731_config_interface((j * 2),image[i][j]);
				if(ret < 0)
					goto err;
			}
			ret = sn3731_config_interface(0x1,image[i][9]);
			if(ret < 0)
				goto err;
			ret = sn3731_config_interface(0x3,image[i][10]);
			if(ret < 0)
				goto err;
			for(k=0;k<multiple;k++)
				if((frame_num+k+1) < 8)
				{	printk("sn3731          frame NO. = %d\n",frame_num+k+1);
					ret = sn3731_config_interface(0xFD,(frame_num+k+1));
					if(ret < 0)
						goto err;
					for(j = 0; j < 9; j++)
					{
						ret = sn3731_config_interface((j * 2),0x0);
						if(ret < 0)
							goto err;
					}
					ret = sn3731_config_interface(0x1,0x0);
					if(ret < 0)
						goto err;
					ret = sn3731_config_interface(0x3,0x0);
					if(ret < 0)
						goto err;
				}
			frame_num++;
			frame_num += multiple;
		}
	}
err:
	printk("sn3731    set_led_breath     i2c     transfer    error!\n");
}

void set_led_music()
{	int i;
	int ret;
	//sn3731_init();
	printk("sn3731       set_led_music\n");
	ret = sn3731_config_interface(0xFD,0xB);
	if(ret < 0)
		goto err;		
	ret = sn3731_config_interface(0x00,0x0);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x01,0x0);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x09,0x0);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x0A,0x1);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0xFD,0x0);
	if(ret < 0)
		goto err;
	for(i = 0 ;i < 9;i++)
	{
		ret = sn3731_config_interface((i*2), image[0][i]);
		if(ret < 0)
			goto err;
	}
	ret = sn3731_config_interface(0x1,image[0][9]);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x3,image[0][10]);
	if(ret < 0)
		goto err;

	
err:
	printk("sn3731     set_led_music     i2c transfer   error!\n");
}

void set_led_image()
{	int i;
	int ret;
	//sn3731_init();
	printk("sn3731       set_led_image\n");
	ret = sn3731_config_interface(0xFD,0xB);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x00,0x0);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x01,0x0);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x09,0x0);
	if(ret < 0)
	goto err;
	ret = sn3731_config_interface(0xa,0x1);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0xFD,0x0);
	if(ret < 0)
		goto err;
	for(i = 0 ;i < 9;i++)
	{
		ret = sn3731_config_interface((i*2), image[0][i]);
		if(ret < 0)
			goto err;
	}
	ret = sn3731_config_interface(0x1,image[0][9]);
	if(ret < 0)
		goto err;
	ret = sn3731_config_interface(0x3,image[0][10]);
	if(ret < 0)
		goto err;

err:
	printk("sn3731      set_led_image     i2c   transfer    error!\n");
	
}
static int set_led_behavior(void)
{
	switch(mode)
	{
		case NLED_BREATH:
			set_led_breath();break;
		case NLED_MUSIC:
			set_led_music();break;
		case NLED_IMAGE:
			set_led_image();break;

	}

}



/******************************************************************************************************************************************/

/*static void sn3731_eint_work(struct work_struct *work)
{
	printk("sn3731             sn3731_eint_work\n");
       int ledcover_open = mt_get_gpio_in(GPIO34);
       if(ledcover_open == 1)
       {
               mt65xx_eint_set_polarity(0,CUST_EINT_POLARITY_LOW);
               printk("sn3731           ledcover_open=1\n");
               kobject_uevent(&sn3731_private_data->client->dev.kobj, KOBJ_REMOVE);
       }else
       {
               mt65xx_eint_set_polarity(0,CUST_EINT_POLARITY_HIGH);
               printk("sn3731           ledcover_open=0\n");
               kobject_uevent(&sn3731_private_data->client->dev.kobj, KOBJ_ADD);
       }
       
       mt65xx_eint_unmask(0);
}
void sn3731_eint_handler(void)
{
	printk("sn3731             sn3731_eint_handler\n");
       mt65xx_eint_mask(0);
       schedule_work(sn_eint_work);
}*/
static ssize_t store_led_image(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	printk("sn3731    store_led_image\n");
	int i,k;
	if(buf != NULL && size != 0)
	{
		mode = buf[0];
		if( mode == 0)
		{
			frame_count = buf[1];
			loop_count = buf[2];
			for(i=0;i < 4; i++)
			{
				time1 = buf[3+i*2];
				time2 = buf[4+i*2];
				printk("sn3731   time1 = %d,time2 = %d\n",time1,time2);
				time[i] = ((time1 << 8 )+ time2);
				printk("sn3731    time[%d] = %d\n",i,time[i]);
			}
			for(i = 0;i <= frame_count;i++)
				for (k = 0; k < 11 ;k++)
				{
					image[i][k] = buf[11+(i*11+k)];
					printk("sn3731   image[%d][%d]  = %d\n",i,k,image[i][k]);
				}
		}
		if(mode == 1)
		{	
			for( i= 0;i < 11;i++)
				{
					image[0][i] = buf[i+1];
					printk("sn3731    image[0][%d] = %d\n",i,image[0][i]);
		}
						
					
				}
		if(mode == 2)
		{
			time1 = buf[1];
			time2 = buf[2];

			duration = time1 << 8 + time2;
			printk("sn3731     duration = %d\n",duration);
			for( i= 0;i < 11;i++)
			{
				image[0][i] = buf[3+i];
				printk("sn3731    image[0][%d] = %d\n",i,image[0][i]);
			}

		}
			
		
	}
	set_led_behavior();
	return size;

}
static ssize_t show_led_image(struct device *dev,struct device_attribute *attr,char *buf)
{
	
	return 0;

}

static DEVICE_ATTR(led_image, 0664,show_led_image,store_led_image);

static ssize_t show_ledcover_open(struct device *dev,struct device_attribute *attr,char *buf)
{
       unsigned int ledcover_open = mt_get_gpio_in(GPIO34);
       printk("sn3731      show_ledcover_open           ledcover_open = %d\n ",ledcover_open);
       return sprintf(buf,"%u\n" ,ledcover_open);
       return 0;

}

static DEVICE_ATTR(ledcover_open,0664, show_ledcover_open,NULL);

/*******************************************************************************************************************************************************************/
static int sn3731_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int ret,rc;
	int retval = 1;
	printk("sn3731 driver probe addr:%d\n",client->addr);
       
	led = kzalloc(sizeof(struct mt65xx_led_data), GFP_KERNEL);
	if (!led) {
		ret = -ENOMEM;
		goto err;
	}


        sn3731_private_data = kzalloc(sizeof(struct sn3731_data), GFP_KERNEL);
        if(!sn3731_private_data)
	{
        	printk("sn3731: kzalloc sn3731_data fail!\n");
		return -1;
	}
	memset(sn3731_private_data, 0, sizeof(*sn3731_private_data));
	sn3731_private_data->client = client;
	
	mutex_init(&sn3731_private_data->mutex);
       sn3731_private_data->sn_led_data = led;
       
       //INIT_WORK(&(sn3731_private_data->sn_led_data)->work, sn3731_eint_work);

       sn_eint_work = &(sn3731_private_data->sn_led_data)->work;
	rc = device_create_file(&(client->dev), &dev_attr_led_image);
	if(rc)
	{
		printk("[led_image] device create file duty fail!\n");	
	}
       rc = device_create_file(&(client->dev), &dev_attr_ledcover_open);
       if(rc)
       {
               printk("[ledcover_open] device create file duty fail!\n");      
       }
	//set_led_call();
	
       mt_set_gpio_mode(GPIO34, GPIO_MODE_01);
       mt_set_gpio_dir(GPIO34, GPIO_DIR_IN);
       mt_set_gpio_pull_enable(GPIO34, GPIO_PULL_DISABLE);

       mt_set_gpio_mode(GPIO18, GPIO_MODE_00);
       mt_set_gpio_dir(GPIO18, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO18, GPIO_OUT_ONE);

	//ret = sn3731_init();
	//if(ret < 0)
		//printk("sn3731   init    error!\n");
		
       /*mt65xx_eint_set_sens(0, CUST_EINT_EDGE_SENSITIVE );
       mt65xx_eint_set_polarity(0,CUST_EINT_POLARITY_LOW);
       mt65xx_eint_set_hw_debounce(0, CUST_EINT_DEBOUNCE_ENABLE);
       mt65xx_eint_registration(0, CUST_EINT_DEBOUNCE_ENABLE, CUST_EINT_POLARITY_LOW, sn3731_eint_handler, 0); 
       mt65xx_eint_unmask(0);*/
       
       
	return 0;///for debug

err:

	return ret;

}

static int sn3731_driver_remove()
{
	mt_set_gpio_mode(GPIO18, GPIO_MODE_00);
       mt_set_gpio_dir(GPIO18, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO18, GPIO_OUT_ZERO);
	return 0;
}

static void sn3731_late_resume(struct early_suspend *h)
{
	atomic_set(&(sn3731_private_data->early_suspend), 0);

        printk("sn3731 resume------------------------------------------------\n");
}

static void sn3731_early_suspend(struct early_suspend *h)
{
	atomic_set(&(sn3731_private_data->early_suspend), 1);

        printk("sn3731 suspend------------------------------------------------\n");
}
static struct i2c_driver led_sn3731_driver = {
    .probe      = sn3731_driver_probe,
    .remove  = sn3731_driver_remove,
    .id_table   = sn3731_i2c_id,
    .driver     = {
    .name  = "sn3731",
    },
};


static int __init sn3731_driver_init(void)
{
	 struct i2c_adapter *adapter;
	 struct i2c_client *client;
	 int i;
        printk("sn3731 driver init\n");

        i2c_register_board_info(2, &i2c_sn3731, 1);
		
	//adapter = i2c_get_adapter(2);
	
	//client = i2c_new_device(adapter, &i2c_sn3731);
	
      // i2c_put_adapter(adapter);
	printk("sn3731 add driver1\n");
	if(i2c_add_driver(&led_sn3731_driver)!=0)
        {
		printk("failed to register led sn3731 driver\n");
        }
	else
	{
		printk("success to register led sn3731 driver\n");
	}
	
        return 0;	
}


static void __exit sn3731_driver_exit (void)
{
	i2c_del_driver(&led_sn3731_driver);
	i2c_unregister_device(sn3731_private_data->client);
	//i2c_unregister_device(sn3731_private_data->client2);
	kfree(sn3731_private_data);
}

module_init(sn3731_driver_init);
module_exit(sn3731_driver_exit);

MODULE_AUTHOR("guo yc");
MODULE_DESCRIPTION("Led sn3731 Driver");
MODULE_LICENSE("GPL");
