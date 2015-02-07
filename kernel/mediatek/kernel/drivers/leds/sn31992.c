#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
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


static struct sn3199_data *data;
struct task_struct *sn3199_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static const struct i2c_device_id sn3199_i2c_id[] = {{"sn3199",0},{}};
static struct i2c_board_info __initdata i2c_sn31991={ I2C_BOARD_INFO("sn3199", (0xce>>1))};
//static struct i2c_board_info __initdata i2c_sn31992={ I2C_BOARD_INFO("sn3199", (0x67>>1))};
static struct mt65xx_led_data *led;
static struct i2c_client *new_client = NULL;

#define NLED_ON 0
#define NLED_BLINK 1
#define NLED_BREATH 2
#define NLED_STREAMING 3


#define	T0A(i) (i)
#define	T0B(i) (i<<4)
#define	T1A(i) (i)
#define	T2B(i) (i<<4)
#define	T4A(i) (i)
#define	T4B(i) (i<<4)
#define	DT(i)  (i<<7)
#define	T1   20
#define	T2   10
#define	T3   150

struct sn3199_data {
        struct i2c_client *client;
        struct mt65xx_led_data *sn_led_data;
	 //struct early_suspend    early_drv;
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

struct nled_setting
{
	u8 nled_mode; //0, off; 1, on; 2, blink;3,breath
	u32 breath_start_time;
	u32 breath_rise_time;
	u32 breath_on_time;
	u32 breath_drop_time;
	u32 breath_off_time;
};

static int write_reg(u8 addr, u8 cmd)
{
        char buf[2];
        int ret = -1;

        buf[0] = addr;
        buf[1] = cmd;
	 
	new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
	printk("sn3199 write reg------------------------------------\n");
        ret = i2c_master_send(data->client, buf, 2);
        if (ret < 0){
		  new_client->ext_flag=0;
                printk("sn3199: write reg failed! \n");
		  data->client = new_client;
                return -1;
        }
	new_client->ext_flag=0;
	data->client = new_client;
        return 0;
}

static kal_uint32 read_reg(u8 addr_read)
{
        int ret;
	char buf[1];
	buf[0] = addr_read;
	new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	
	printk("sn3199 read reg-----------------------------------\n");
        i2c_master_send(data->client, buf, 1);
	udelay(31);
	char buf_rec[1];
        ret = i2c_master_recv(data->client, buf_rec, 1);
        if (ret < 0)
	{	new_client->ext_flag=0;
		data->client = new_client;
                printk("sn3199: i2c read error\n");
		return -1;
	}	

	new_client->ext_flag=0;
	data->client = new_client;
        return buf_rec[0];
}

kal_uint32 sn3199_read_interface (kal_uint8 RegNum)
{
	kal_uint32 i = 0;
	i = read_reg(RegNum);
	return i;
}

kal_uint32 sn3199_config_interface (kal_uint8 RegNum, kal_uint8 val)
{
	write_reg(RegNum,val);
}

/*void sn3199_init(void)
{
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x1,0x0);
	sn3199_config_interface(0x2,0x0);
	sn3199_config_interface(0x3,0x0);
	sn3199_config_interface(0x4,0x0);
	sn3199_config_interface(0x5,0x0);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x0,0x1);


}

*/

/*void sn3199_handler()
{

	printk("sn3199 handler---------------------------------------------\n");

	mt65xx_eint_mask(4);

	kal_uint32 reg1 = 0;

	reg1 = sn3199_read_interface(222222222222);

	printk("----------------------------------\n");
	

	mt65xx_eint_unmask(4);

}


static void sn3199_thread_handler(void)
{
        do
        {
                mt65xx_eint_unmask(4);
                set_current_state(TASK_INTERRUPTIBLE);
                wait_event_interruptible(waiter,qt_flag!=0);

                qt_flag = 0;

                set_current_state(TASK_RUNNING);
                mt65xx_eint_mask(4);

		  sn3199_handler();
        }while(!kthread_should_stop());

        return 0;

}

void 3199_eint_handler(void)
{
         qt_flag = 1;
         wake_up_interruptible(&waiter);
}
*/
/*void set_led_off(void)
{       sn3199_config_interface(0x0,0x1);

        sn3199_config_interface(0x1,0x0);  
        sn3199_config_interface(0x2,0x0);
        
        sn3199_config_interface(0x10,0x0);

}*/
void set_led_on(unsigned long status)
{  	/*int led1 = 0;
	int led2 = 0;
	int led3 = 0;

	led1 = status & 0x1;
	led2 = status & 0x2;
	led3 = status & 0x4;*/


	 sn3199_config_interface(0x0,0x1);

	/*if(led1 == 0 && led2 ==0)
        	sn3199_config_interface(0x1,0x0);
	if(led1 == 0 && led2 ==1)
		sn3199_config_interface(0x1,0x70)
	if(led1 == 1 && led2 ==0)
		sn3199_config_interface(0x1,0x07)
	if(led1 == 1 && led2 ==1)
		sn3199_config_interface(0x1,0x77)
	if(led3 ==0)	
        	sn3199_config_interface(0x2,0x0);
	if(led3 ==1)	
        	sn3199_config_interface(0x2,0x7);
        */
        sn3199_config_interface(0x3,0x0);
        sn3199_config_interface(0x4,0x10);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
        
        sn3199_config_interface(0x7,0xff);
        sn3199_config_interface(0x8,0xff);
        sn3199_config_interface(0x9,0xff);
        sn3199_config_interface(0xa,0xff);
        sn3199_config_interface(0xb,0xff);
        sn3199_config_interface(0xc,0xff);
        sn3199_config_interface(0xd,0xff);
        sn3199_config_interface(0xe,0xff);
        sn3199_config_interface(0xf,0xff);
        
        sn3199_config_interface(0x10,0x0);
        
}
void set_led_breath(unsigned long status)
{    

	printk("sn3199         set_led_breath\n");
	 sn3199_config_interface(0x0,0x1);
        //sn3199_config_interface(0x1,0x77);
        //sn3199_config_interface(0x2,0x7);
        sn3199_config_interface(0x3,0x70);
        sn3199_config_interface(0x4,0x00);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
        
        sn3199_config_interface(0x7,0xff);
        sn3199_config_interface(0x8,0xff);
        sn3199_config_interface(0x9,0xff);
        sn3199_config_interface(0xa,0xff);
        sn3199_config_interface(0xb,0xff);
        sn3199_config_interface(0xc,0xff);
        sn3199_config_interface(0xd,0xff);
        sn3199_config_interface(0xe,0xff);
        sn3199_config_interface(0xf,0xff);
        
        sn3199_config_interface(0x10,0x0);
        
        sn3199_config_interface(0x11,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(0)+T0B(0)));
		
        sn3199_config_interface(0x1a,(T1A(0)+T2B(2)));
        sn3199_config_interface(0x1b,(T1A(0)+T2B(2)));
        sn3199_config_interface(0x1c,(T1A(0)+T2B(2)));
		
        sn3199_config_interface(0x1d,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x1e,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x1f,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x20,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x21,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x22,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x23,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x24,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x25,(T4A(1)+T4B(1)));
		
        
        sn3199_config_interface(0x26,0x0);
}


void set_led_streaming(unsigned long status)
{    

	printk("sn3199         set_led_breath\n");
	 sn3199_config_interface(0x0,0x1);
        //sn3199_config_interface(0x1,0x77);
        //sn3199_config_interface(0x2,0x7);
        sn3199_config_interface(0x3,0x70);
        sn3199_config_interface(0x4,0x00);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
        
        sn3199_config_interface(0x7,0xff);
        sn3199_config_interface(0x8,0xff);
        sn3199_config_interface(0x9,0xff);
        sn3199_config_interface(0xa,0xff);
        sn3199_config_interface(0xb,0xff);
        sn3199_config_interface(0xc,0xff);
        sn3199_config_interface(0xd,0xff);
        sn3199_config_interface(0xe,0xff);
        sn3199_config_interface(0xf,0xff);
        
        sn3199_config_interface(0x10,0x0);
        
        sn3199_config_interface(0x11,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(0)+T0B(0)));
		
        sn3199_config_interface(0x1a,(T1A(0)+T2B(2)));
        sn3199_config_interface(0x1b,(T1A(0)+T2B(2)));
        sn3199_config_interface(0x1c,(T1A(0)+T2B(2)));
		
        sn3199_config_interface(0x1d,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x1e,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x1f,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x20,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x21,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x22,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x23,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x24,(T4A(1)+T4B(1)));
        sn3199_config_interface(0x25,(T4A(1)+T4B(1)));
		
        
        sn3199_config_interface(0x26,0x0);
}

static int led_set_pwm(unsigned long status, struct nled_setting* led)
{
	int led1 = 0;
	int led2 = 0;
	int led3 = 0;

	led1 = status & 0x1;
	led2 = (status & 0x2) >> 1;
	led3 = (status & 0x4) >> 2;
	printk("sn3199     led_set_pwm\n");
	if(led1 == 0 && led2 ==0)
        	sn3199_config_interface(0x1,0x0);
	if(led1 == 0 && led2 ==1)
		sn3199_config_interface(0x1,0x70);
	if(led1 == 1 && led2 ==0)
		sn3199_config_interface(0x1,0x07);
	if(led1 == 1 && led2 ==1)
		sn3199_config_interface(0x1,0x77);
	if(led3 ==0)	
        	sn3199_config_interface(0x2,0x0);
	if(led3 ==1)	
        	sn3199_config_interface(0x2,0x7);
	
	switch (led->nled_mode)
	{
		case NLED_ON :
			set_led_on(status);
            
		case NLED_BLINK :
		       set_led_breath(status);
			   
		case NLED_BREATH:
			set_led_breath(status);
			
		case NLED_STREAMING:
			set_led_streaming(status);
	}



	return 0;
	
}


static int mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level)
{
	struct nled_setting led_tmp_setting = {0,0,0,0,0,0};
	int tmp_level;
	tmp_level = level;
	if (level > LED_FULL)
		level = LED_FULL;
	else if (level < 0)
		level = 0;

    printk("mt65xx_leds_set_cust: set brightness, name:%s, mode:%d, level:%d\n", 
		cust->name, cust->mode, level);
	switch (cust->mode) {
		case MT65XX_LED_MODE_PWM:
			
			{
				if(level == 0)
				{
					led_tmp_setting.nled_mode = NLED_ON;
				}else
				{
					led_tmp_setting.nled_mode = NLED_ON;
				}
				led_set_pwm(cust->data,&led_tmp_setting);
			}
			return 1;
            
		default:
			break;
	}
	return -1;
}


static void mt65xx_led_work(struct work_struct *work)
{
	struct mt65xx_led_data *led_data =
		container_of(work, struct mt65xx_led_data, work);//最后的work是mt65xx_led_data的一个成员，第一个work是此work成员的地址，container_of函数是得到结构体mt65xx_led_data的首地址。 

	//LEDS_DEBUG("[LED]%s:%d\n", led_data->cust.name, led_data->level);

	mt65xx_led_set_cust(&led_data->cust, led_data->level);
}

static void mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct mt65xx_led_data *led_data =
		container_of(led_cdev, struct mt65xx_led_data, cdev);

	// do something only when level is changed

	if((led_data->level != level)||!strcmp(led_data->cust.name,"red")) {
		led_data->level = level;
		schedule_work(&led_data->work);
		
	}
}
static void mt65xx_blink_set();
/******************************************************************************************************************************************/
static unsigned int breath_led_enable;

static unsigned long startt0 = 0;
static unsigned long riset1 = 512;
static unsigned long dropt3 = 512;
static unsigned long ledont2 =1024;
static unsigned long ledofft4 =1024;

static unsigned long ledstatus = 0;

static ssize_t store_led_status(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned long led_status = 0;
	struct nled_setting led_tmp_seting;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			led_status = simple_strtoul(buf,&pvalue,10);
			ledstatus= led_status;
			
			led_tmp_seting.nled_mode = led_status >> 6;
			led_tmp_seting.breath_start_time = startt0;
			led_tmp_seting.breath_rise_time = riset1;
			led_tmp_seting.breath_on_time = ledont2;
			led_tmp_seting.breath_drop_time = dropt3;
			led_tmp_seting.breath_off_time =ledofft4;
			
			led_set_pwm(ledstatus, &led_tmp_seting);
			
		}
	return size;
}

static ssize_t show_led_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int led_status = ledstatus;
	return sprintf(buf,"%u\n" ,ledstatus);
	return 0;

}

static DEVICE_ATTR(led_status,0664, show_led_status,store_led_status);



static ssize_t store_breath_led(struct device *dev, struct device_attribute *attr ,const char *buf,size_t size)
{
	printk("sn3199    store_breath_led1\n");
	char *pvalue = NULL;
	unsigned int breath_led;
	//struct cust_mt65xx_led *cust_led_list =get_cust_led_list();
	
	struct nled_setting led_tmp_seting;
	led_tmp_seting.nled_mode = NLED_BREATH;
	led_tmp_seting.breath_start_time = startt0;
	led_tmp_seting.breath_rise_time = riset1;
	led_tmp_seting.breath_on_time = ledont2;
	led_tmp_seting.breath_drop_time = dropt3;
	led_tmp_seting.breath_off_time =ledofft4;
	
	if(buf != NULL && size != 0)
		{	printk("sn3199    store_breath_led2\n");
			breath_led = simple_strtoul(buf,&pvalue,10);
			if(breath_led == 1)
				{	printk("sn3199    store_breath_led        breath_led=1\n");
					//mt_set_gpio_out(GPIO194,GPIO_OUT_ONE);
					breath_led_enable = 1;
					led_set_pwm(ledstatus, &led_tmp_seting);

				}
			if(breath_led == 0)
				{	printk("sn3199    store_breath_led       breath_led=0\n");
					//mt_set_gpio_out(GPIO194,GPIO_OUT_ZERO);
					breath_led_enable = 0;
					led_tmp_seting.nled_mode = NLED_ON;
					led_set_pwm(ledstatus, &led_tmp_seting);
					//mt65xx_led_set_cust(&led_tmp_seting,0);

				}
		}
	return size;
	
}

static ssize_t show_breath_led(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf( buf,"%u\n",breath_led_enable);
	return 0;

}

static DEVICE_ATTR (breath_led, 0664, show_breath_led, store_breath_led);

static ssize_t store_start_t0(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int start_t0 = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			start_t0 = simple_strtoul(buf,&pvalue,10);
			startt0 = start_t0;
		}
	return size;
}

static ssize_t show_start_t0(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int start_t0 = startt0;
	return sprintf(buf,"%u\n" ,start_t0);
	return 0;

}

static DEVICE_ATTR(start_t0,0664, show_start_t0,store_start_t0);


static ssize_t store_rise_t1(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int rise_t1 = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			rise_t1 = simple_strtoul(buf,&pvalue,10);
			riset1 = rise_t1;
		}
	return size;
}

static ssize_t show_rise_t1(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int rise_t1 = riset1;
	return sprintf(buf,"%u\n" ,rise_t1);
	return 0;

}

static DEVICE_ATTR(rise_t1,0664, show_rise_t1,store_rise_t1);

static ssize_t store_ledon_t2(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int ledon_t2 = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			ledon_t2 = simple_strtoul(buf,&pvalue,10);
			ledont2 = ledon_t2;
		}
	return size;
}

static ssize_t show_ledon_t2(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int ledon_t2 = ledont2;
	return sprintf(buf,"%u\n" ,ledon_t2);
	return 0;

}

static DEVICE_ATTR(ledon_t2,0664, show_ledon_t2,store_ledon_t2);

static ssize_t store_drop_t3(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int drop_t3 = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			drop_t3 = simple_strtoul(buf,&pvalue,10);
			dropt3 = drop_t3;
		}
	return size;
}

static ssize_t show_drop_t3(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int drop_t3 = dropt3;
	return sprintf(buf,"%u\n" ,drop_t3);
	return 0;

}

static DEVICE_ATTR(drop_t3,0664, show_drop_t3,store_drop_t3);

static ssize_t store_ledoff_t4(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int ledoff_t4 = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			ledoff_t4 = simple_strtoul(buf,&pvalue,10);
			ledofft4 = ledoff_t4;
		}
	return size;
}

static ssize_t show_ledoff_t4(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int ledoff_t4 = ledofft4;
	return sprintf(buf,"%u\n" ,ledoff_t4);
	return 0;

}

static DEVICE_ATTR(ledoff_t4,0664, show_ledoff_t4,store_ledoff_t4);

/*******************************************************************************************************************************************************************/
static int sn3199_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int ret,rc;
	printk("sn3199_driver_probe\n");
	int retval = 1;
	printk("sn3199 driver probe addr:%d\n",client->addr);

	led = kzalloc(sizeof(struct mt65xx_led_data), GFP_KERNEL);
	if (!led) {
		ret = -ENOMEM;
		goto err;
	}



	hwPowerOn(MT65XX_POWER_LDO_VGP2,VOL_1800,"led");

        data = kzalloc(sizeof(struct sn3199_data), GFP_KERNEL);
        if(!data)
	{
        	printk("sn3199: kzalloc sn3199_data fail!\n");
		return -1;
	}
	memset(data, 0, sizeof(*data));

	    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        ret= -ENOMEM;
        goto err;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

	
	new_client = client;
	
	data->client = new_client;
	
	led->cust.mode = MT65XX_LED_MODE_PWM;
	led->cust.data = PWM1;
	led->cust.name = "red";
	//led->cust.config_data = {0};//bei add

	led->cdev.name = "red";
	led->cdev.brightness_set = mt65xx_led_set;
//	led->cdev.blink_set = mt65xx_blink_set;

	INIT_WORK(&led->work, mt65xx_led_work);
       data->sn_led_data = led;
       //ret = led_classdev_register(&dev->dev, &led->cdev);
        
	rc = device_create_file(&(client->dev), &dev_attr_breath_led);
	if(rc)
	{
		printk("[breath_led] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_start_t0);
	if(rc)
	{
		printk("[start_t0] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_rise_t1);
	if(rc)
	{
		printk("[rise_t1] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_ledon_t2);
	if(rc)
	{
		printk("[ledon_t2] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_drop_t3);
	if(rc)
	{
		printk("[drop_t3] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_ledoff_t4);
	if(rc)
	{
		printk("[ledoff_t4] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_led_status);
	if(rc)
	{
		printk("[led_status] device create file duty fail!\n");	
	}
	/*rc = device_create_file(led->cdev.dev, &dev_attr_delay_on);
	if(rc)
	{
		LEDS_DEBUG("[notify_on] device create file duty fail!\n");	
	}
	rc = device_create_file(led->cdev.dev, &dev_attr_delay_off);
	if(rc)
	{
		LEDS_DEBUG("[notify_off] device create file duty fail!\n");	
	}
*/




	mt_set_gpio_mode(GPIO18, GPIO_MODE_00);
       mt_set_gpio_dir(GPIO18, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO18, GPIO_OUT_ONE);
	//udelay(10);
	//mt_set_gpio_out(GPIO18, GPIO_OUT_ONE);


	mt_set_gpio_mode(GPIO19, GPIO_MODE_00);
       mt_set_gpio_dir(GPIO19, GPIO_DIR_OUT);
       mt_set_gpio_out(GPIO19, GPIO_OUT_ONE);
       //udelay(10);
       //mt_set_gpio_out(GPIO19,GPIO_OUT_ONE);


	/*mt_set_gpio_mode(GPIO18, 1);
	mt_set_gpio_dir(GPIO18, GPIO_DIR_OUT);

        mt_set_gpio_pull_select(GPIO18, GPIO_PULL_UP);
        mt_set_gpio_pull_enable(GPIO18, 1);
	

	
	mt_set_gpio_mode(GPIO19, 1);
	mt_set_gpio_dir(GPIO19, GPIO_DIR_OUT);

        mt_set_gpio_pull_select(GPIO19, GPIO_PULL_UP);
        mt_set_gpio_pull_enable(GPIO19,1);*/
	
	//set_led_blink();
	printk("sn3199_driver_probe   success");

	/*mt65xx_eint_set_sens(4, 1);
	mt65xx_eint_set_polarity(4,0);
	mt65xx_eint_set_hw_debounce(4, 0);
	mt65xx_eint_registration(4, 0, 0, sn3199_eint_handler, 0); 
	mt65xx_eint_unmask(4);


        sn3199_thread = kthread_run(sn3199_thread_handler, 0, "sn3199");
        if (IS_ERR(coreriver_thread))
        {
                 retval = PTR_ERR(coreriver_thread);
		 printk("coreriver kthread_run fail \n");
        }*/

	//atomic_set(&(data->early_suspend), 0);
	//data->sn_led_data = led;
       //data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
       //data->early_drv.suspend  = sn3199_early_suspend,
       //data->early_drv.resume   = sn3199_late_resume,
       //register_early_suspend(&data->early_drv);

	return 0;

err:
	/*if (i) {
		for (i = i-1; i >=0; i--) {
			if (!g_leds_data[i])
				continue;
			led_classdev_unregister(&g_leds_data[i]->cdev);
			cancel_work_sync(&g_leds_data[i]->work);
			kfree(g_leds_data[i]);
			g_leds_data[i] = NULL;
		}
	}*/

	return ret;

}

static void sn3199_late_resume(struct early_suspend *h)
{
	atomic_set(&(data->early_suspend), 0);

        printk("sn3199 resume------------------------------------------------\n");
}

static void sn3199_early_suspend(struct early_suspend *h)
{
	atomic_set(&(data->early_suspend), 1);

        printk("sn3199 suspend------------------------------------------------\n");
}
static struct i2c_driver led_sn3199_driver = {
    .probe      = sn3199_driver_probe,
   // .remove     = sn3199_driver_remove,
   // .resume     = sn3199_driver_resume,
    .id_table   = sn3199_i2c_id,
    .driver     = {
    .name  = "sn3199",
    },
};


static int __init sn3199_driver_init(void)
{
	 struct i2c_adapter *adapter;
	 struct i2c_client *client;
	 int i;
        printk("sn3199 driver init\n");

        //i2c_register_board_info(5, &i2c_sn3199, 1);
	adapter = i2c_get_adapter(5);
	
	client = i2c_new_device(adapter, &i2c_sn31991);
		
       i2c_put_adapter(adapter);
	printk("sn3199 add driver1\n");
	if(i2c_add_driver(&led_sn3199_driver)!=0)
        {
		printk("failed to register led sn3199 driver\n");
        }
	else
	{
		printk("success to register led sn3199 driver\n");
	}
	//set_led_blink();
        return 0;	
}


static void __exit sn3199_driver_exit (void)
{
	i2c_del_driver(&led_sn3199_driver);
	i2c_unregister_device(data->client);
	
	kfree(data);
}

module_init(sn3199_driver_init);
module_exit(sn3199_driver_exit);

MODULE_AUTHOR("guo yc");
MODULE_DESCRIPTION("Led SN3199 Driver");
MODULE_LICENSE("GPL");
