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
//#define	T1   20
//#define	T2   10
//#define	T3   150

static struct sn3199_data *sn3199_private_data;
struct task_struct *sn3199_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static const struct i2c_device_id sn3199_i2c_id[] = {{"sn3199",0},{"sn31991",0},{}};

static struct i2c_board_info __initdata i2c_sn3199[]={
	{I2C_BOARD_INFO("sn3199", (0xCE>>1))},{I2C_BOARD_INFO("sn31991", (0xC8>>1))},
};
static struct mt65xx_led_data *led;

static unsigned int REG4_1 = 0x00;
static unsigned int REG4_2 = 0x80;
static unsigned int breath_led_enable;
static unsigned int ledflash;
static unsigned long T0A = 0;
static unsigned long T0B = 0;
static unsigned long T1A = 4;
static unsigned long T2B =1;
static unsigned long T4A =0;
static unsigned long T4B =0;
static unsigned int DT =1;

static unsigned long electric_current = 0;
static unsigned long ledstatus = 0;

static unsigned long ledcolor[6]={0,0,0,0,0,0};
static long t4[70];
static int t4_num;



struct sn3199_data {
        struct i2c_client *client;
	 struct i2c_client *client1;
        struct i2c_client *client2;
	 struct mutex mutex;
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
	//u32 blink_on_time ;
	//u32 blink_off_time;
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
	 mutex_lock(&sn3199_private_data->mutex);
	 sn3199_private_data->client->ext_flag=((sn3199_private_data->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
	 printk("sn3199 write reg------------------------------------\n");
	 printk("sn3199 write reg addr:%d\n",sn3199_private_data->client->addr);
        ret = i2c_master_send(sn3199_private_data->client, buf, 2);
        if (ret < 0){
			sn3199_private_data->client->ext_flag = 0;
		  mutex_unlock(&sn3199_private_data->mutex);
                printk("sn3199: write reg failed! \n");
                return -1;
        }
	 sn3199_private_data->client->ext_flag = 0;
	 mutex_unlock(&sn3199_private_data->mutex);
        return 0;
}

static kal_uint32 read_reg(u8 addr_read)
{
       int ret;
	char buf[1];
	buf[0] = addr_read;
	mutex_lock(&sn3199_private_data->mutex);

	sn3199_private_data->client->ext_flag=((sn3199_private_data->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
	printk("sn3199 read reg-----------------------------------\n");
       ret = i2c_master_send(sn3199_private_data->client, buf, 1);
	if(ret < 0)
	{
		printk("sn3199   read  error i2c_master_send\n");
	}
	
	udelay(31);
	char buf_rec[1];
       ret = i2c_master_recv(sn3199_private_data->client, buf_rec, 1);
        if (ret < 0)
	{	sn3199_private_data->client->ext_flag = 0;
		mutex_unlock(&sn3199_private_data->mutex);
              printk("sn3199: i2c read error\n");
		return -1;
	}	
	sn3199_private_data->client->ext_flag = 0;
	 mutex_unlock(&sn3199_private_data->mutex);
        return buf_rec[0];
        //return ret;
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

void sn3199_init(void)
{
	sn3199_config_interface(0x0,0x1);
	sn3199_config_interface(0x1,0x0);
	sn3199_config_interface(0x2,0x0);
	sn3199_config_interface(0x3,0x0);
	//sn3199_config_interface(0x4,0x0);
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
       sn3199_config_interface(0x1a,(T1A(5)+T2B(2)));
       sn3199_config_interface(0x1b,(T1A(5)+T2B(2)));
       sn3199_config_interface(0x1c,(T1A(5)+T2B(2)));
       sn3199_config_interface(0x1d,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x1e,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x1f,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x20,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x21,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x22,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x23,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x24,(T4A(0)+T4B(1)));
       sn3199_config_interface(0x25,(T4A(0)+T4B(1)));

	sn3199_config_interface(0x26,0x0);


}


void set_led_on(unsigned long status)
{  

	printk("sn3199        set_led_on\n");
	 sn3199_config_interface(0x0,0x1);
        
        sn3199_config_interface(0x3,0x00);
        //sn3199_config_interface(0x4,0x80);
        sn3199_config_interface(0x5,0x70);
        sn3199_config_interface(0x6,0x0);

        //sn3199_config_interface(0x10,0x0);

	 sn3199_config_interface(0x11,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x1a,(T1A(5)+T2B(2)));
        sn3199_config_interface(0x1b,(T1A(5)+T2B(2)));
        sn3199_config_interface(0x1c,(T1A(5)+T2B(2)));
        sn3199_config_interface(0x1d,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x1e,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x1f,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x20,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x21,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x22,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x23,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x24,(T4A(0)+T4B(1)));
        sn3199_config_interface(0x25,(T4A(0)+T4B(1)));
        
        //sn3199_config_interface(0x26,0x0);
        
}
void set_led_breath(unsigned long status)
{    

	printk("sn3199         set_led_breath1\n");
	 sn3199_config_interface(0x0,0x1);

        sn3199_config_interface(0x3,0x70);
        //sn3199_config_interface(0x4,0x00);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
  
        //sn3199_config_interface(0x10,0x0);
        
        sn3199_config_interface(0x11,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x1a,(T1A(T1A)+T2B(T2B)));
        sn3199_config_interface(0x1b,(T1A(T1A)+T2B(T2B)));
        sn3199_config_interface(0x1c,(T1A(T1A)+T2B(T2B)));
        sn3199_config_interface(0x1d,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x1e,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x1f,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x20,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x21,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x22,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x23,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x24,(T4A(0)+T4B(0)));
        sn3199_config_interface(0x25,(T4A(0)+T4B(0)));
        
        //sn3199_config_interface(0x26,0x0);
}

void set_led_blink(unsigned long status)
{    

	printk("sn3199         set_led_breath1\n");
	 sn3199_config_interface(0x0,0x1);

        sn3199_config_interface(0x3,0x70);
        //sn3199_config_interface(0x4,0x00);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
        
        //sn3199_config_interface(0x10,0x0);
        
        sn3199_config_interface(0x11,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x1a,(T1A(7)+T2B(T2B)));
        sn3199_config_interface(0x1b,(T1A(7)+T2B(T2B)));
        sn3199_config_interface(0x1c,(T1A(7)+T2B(T2B)));
        sn3199_config_interface(0x1d,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x1e,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x1f,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x20,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x21,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x22,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x23,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x24,(T4A(T4A)+T4B(T4B)));
        sn3199_config_interface(0x25,(T4A(T4A)+T4B(T4B)));
        
       //sn3199_config_interface(0x26,0x0);
}

void set_led1_streaming(unsigned long status)
{    

	printk("sn3199         set_led_streaming\n");
	 sn3199_config_interface(0x0,0x1);
      
        sn3199_config_interface(0x3,0x70);
        //sn3199_config_interface(0x4,0x00);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
        
        //sn3199_config_interface(0x10,0x0);
        
        sn3199_config_interface(0x11,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(2)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(2)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(2)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(4)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(4)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(4)+T0B(0)));
		
        sn3199_config_interface(0x1a,(T1A(2)+T2B(1)));
        sn3199_config_interface(0x1b,(T1A(2)+T2B(1)));
        sn3199_config_interface(0x1c,(T1A(2)+T2B(1)));
		
        sn3199_config_interface(0x1d,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x1e,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x1f,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x20,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x21,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x22,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x23,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x24,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x25,(T4A(2)+T4B(0)));
		
        
        //sn3199_config_interface(0x26,0x0);
}

void set_led2_streaming(unsigned long status)
{    

	printk("sn3199         set_led_streaming\n");
	 sn3199_config_interface(0x0,0x1);
      
        sn3199_config_interface(0x3,0x70);
        //sn3199_config_interface(0x4,0x00);
        sn3199_config_interface(0x5,0x0);
        sn3199_config_interface(0x6,0x0);
        
        //sn3199_config_interface(0x10,0x0);
        
        sn3199_config_interface(0x11,(T0A(2)+T0B(0)));
        sn3199_config_interface(0x12,(T0A(2)+T0B(0)));
        sn3199_config_interface(0x13,(T0A(2)+T0B(0)));
        sn3199_config_interface(0x14,(T0A(4)+T0B(0)));
        sn3199_config_interface(0x15,(T0A(4)+T0B(0)));
        sn3199_config_interface(0x16,(T0A(4)+T0B(0)));
        sn3199_config_interface(0x17,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x18,(T0A(0)+T0B(0)));
        sn3199_config_interface(0x19,(T0A(0)+T0B(0)));
		
        sn3199_config_interface(0x1a,(T1A(2)+T2B(1)));
        sn3199_config_interface(0x1b,(T1A(2)+T2B(1)));
        sn3199_config_interface(0x1c,(T1A(2)+T2B(1)));
		
        sn3199_config_interface(0x1d,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x1e,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x1f,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x20,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x21,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x22,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x23,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x24,(T4A(2)+T4B(0)));
        sn3199_config_interface(0x25,(T4A(2)+T4B(0)));
		
        
        //sn3199_config_interface(0x26,0x0);
}
void set_led_start(void)
{
	sn3199_private_data->client = sn3199_private_data->client1;
	sn3199_config_interface(0x10,0x0);
	sn3199_config_interface(0x26,0x0);
	sn3199_private_data->client = sn3199_private_data->client2;
	sn3199_config_interface(0x10,0x0);
	sn3199_config_interface(0x26,0x0);
}

void set_led_electricity(void)
{	printk("sn3731           set_led_electricity\n");
	int current_data = 0;
	//int reg4;
	sn3199_private_data->client = sn3199_private_data->client1;
	//reg4 = sn3199_read_interface(0x4);
	current_data = REG4_1& 0x8f;
	REG4_1 = current_data | (electric_current << 4);
	sn3199_config_interface(0x4, REG4_1);
	sn3199_private_data->client = sn3199_private_data->client2;
	//reg4 = sn3199_read_interface(0x4);
	current_data = REG4_2& 0x8f;
	REG4_2 = current_data | (electric_current << 4);
	sn3199_config_interface(0x4, REG4_2);
	printk("sn3199           set_led_electricity       reg4_1=%d        reg4_2=%d\n",REG4_1,REG4_2);

}
void set_led_flash(void)
{    

	printk("sn3199         set_led_flash\n");
	sn3199_private_data->client = sn3199_private_data->client1;
	 sn3199_config_interface(0x0,0x1);

	 sn3199_config_interface(0x1,0x77);
        sn3199_config_interface(0x2,0x7);

	 sn3199_config_interface(0x3,0x00);
       // sn3199_config_interface(0x4,0x0);
        sn3199_config_interface(0x5,0x70);
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
	sn3199_private_data->client = sn3199_private_data->client2;
	 sn3199_config_interface(0x0,0x1);

	 sn3199_config_interface(0x1,0x77);
        sn3199_config_interface(0x2,0x7);

	 sn3199_config_interface(0x3,0x00);
        //sn3199_config_interface(0x4,0x10);
        sn3199_config_interface(0x5,0x70);
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

	//msleep(5);
	sn3199_private_data->client = sn3199_private_data->client1;
	 sn3199_config_interface(0x0,0x1);

	 sn3199_config_interface(0x1,0x0);
        sn3199_config_interface(0x2,0x0);

	 sn3199_config_interface(0x3,0x00);
        //sn3199_config_interface(0x4,0x0);
        sn3199_config_interface(0x5,0x70);
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

	sn3199_private_data->client = sn3199_private_data->client2;
	 sn3199_config_interface(0x0,0x1);

	 sn3199_config_interface(0x1,0x0);
        sn3199_config_interface(0x2,0x0);

	 sn3199_config_interface(0x3,0x00);
        //sn3199_config_interface(0x4,0x10);
        sn3199_config_interface(0x5,0x70);
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

void led_set_color()
{
	int i = 0;
	int rgb[3] = {0,0,0};
	sn3199_private_data->client = sn3199_private_data->client1;
	for(i = 0;i<3;i++)
	{	printk("client1");
		rgb[0] = (ledcolor[i] & 0xff0000) >> 16;
		rgb[1] = (ledcolor[i] & 0xff00) >> 8;
		rgb[2] = (ledcolor[i] & 0xff);
		sn3199_config_interface((0x7+i*3),rgb[0]);
	       sn3199_config_interface((0x8+i*3),rgb[1]);
	       sn3199_config_interface((0x9+i*3),rgb[2]);
		printk("led_color         i=%d,         rgb=%d,%d,%d\n",i,rgb[0],rgb[1],rgb[2]);
	}
	sn3199_config_interface(0x10,0x0);

	sn3199_private_data->client = sn3199_private_data->client2;
	for(i = 3;i<6;i++)
	{	printk("client2");
		rgb[0] = (ledcolor[i] & 0xff0000) >> 16;
		rgb[1] = (ledcolor[i] & 0xff00) >> 8;
		rgb[2] = (ledcolor[i] & 0xff);
		sn3199_config_interface((0x7+(i-3)*3),rgb[0]);
	       sn3199_config_interface((0x8+(i-3)*3),rgb[1]);
	       sn3199_config_interface((0x9+(i-3)*3),rgb[2]);
		printk("led_color         i=%d,         rgb=%d,%d,%d\n",i,rgb[0],rgb[1],rgb[2]);
	}
	sn3199_config_interface(0x10,0x0);
}

static int led_set_pwm(unsigned long status, struct nled_setting* led)
{
	int led1 = 0;
	int led2 = 0;
	int led3 = 0;
	int led4 = 0;
	int led5 = 0;
	int led6 = 0; 

	led1 = status & 0x1;
	led2 = (status & 0x2) >> 1;
	led3 = (status & 0x4) >> 2;
	led4 = (status & 0x8) >> 3;
	led5 = (status & 0x16) >> 4;
	led6 = (status & 0x32) >> 5;
	printk("sn3199     led_set_pwm     led->nled_mode=%d\n",led->nled_mode);

	sn3199_private_data->client = sn3199_private_data->client1;
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

	sn3199_config_interface(0x4,REG4_1);


	switch (led->nled_mode)
	{
		case NLED_ON :
			
			set_led_on(status);break;
            
		case NLED_BLINK :
		       set_led_blink(status);break;
			   
		case NLED_BREATH:
			set_led_breath(status);break;
			
		case NLED_STREAMING:
			set_led1_streaming(status);break;
	}

	sn3199_private_data->client = sn3199_private_data->client2;
	if(led4 == 0 && led5 ==0)
        	sn3199_config_interface(0x1,0x0);
	if(led4 == 0 && led5 ==1)
		sn3199_config_interface(0x1,0x70);
	if(led4 == 1 && led5 ==0)
		sn3199_config_interface(0x1,0x07);
	if(led4 == 1 && led5 ==1)
		sn3199_config_interface(0x1,0x77);
	if(led6 ==0)	
        	sn3199_config_interface(0x2,0x0);
	if(led6 ==1)	
        	sn3199_config_interface(0x2,0x7);
	//sn3199_config_interface(0x10,0x0);
	printk("sn3199     led1=%d,led2=%d,led3=%d,led4=%d,led5=%d,led=%d\n",led1,led2,led3,led4,led5,led6);
	sn3199_config_interface(0x4,REG4_2);
	switch (led->nled_mode)
	{
		case NLED_ON :
			
			set_led_on(status);break;
            
		case NLED_BLINK :
			//sn3199_config_interface(0x4,0x80);
		       set_led_blink(status);break;
			   
		case NLED_BREATH:
			//sn3199_config_interface(0x4,0x80);
			set_led_breath(status);break;
			
		case NLED_STREAMING:
			
			set_led2_streaming(status);break;
	}


	set_led_start();
	return 0;
	
}

void set_led_t4(void)
{
	int i,j,k;
	int t;
	long sum;
	i=0;
	j=0;
	k=0;
	sum = 1;
	t = 0;
	for(i=1;i<16;i++)
	{	sum = 1;
		sum = sum * i * 260;
		t4[(i -1) * 4] =((sum << 6) |i);
		printk("sn3199   guoyanchun         sum = %d\n",sum);
		for(j=1;j<4;j++)
		{
			sum = sum*2;
			t4[(i -1) * 4 + j] = ((sum << 6) |(j << 4) |i);
			printk("sn3199   guoyanchun         sum = %d\n",sum);
		}
	}
		
	for(i = 0;i < 60;i++)
	{
		for(j = 0; j < i; j++)
		{
			if((t4[i]>>6) < (t4[j]>>6))
			{
				t = t4[i];
				for( k= i;k > j; k--)
				{
					t4[k] = t4[k-1];
				}
				t4[j] = t;
			}
		}
	}		
	
	
	for(i = 0;i < 60;i++)
	{
		if(t4[i] < 0)
		{
			printk("sn3199      guoyanchun     i = %d\n",i);
			t4_num = i;
			break;
		}
			
		if((t4[i] >> 6)== (t4[i+1] >> 6))
		{
			for(j = i+1;j<60;j++)
			{
				t4[j] = t4[j+1];
				t4[60] = -1;
			}
				
			i--;
		}
	}
		
			
	for(i = 0;i < t4_num;i++)		
		printk("sn3199   guoyanchun       paixu        sum = %d , A = %d,B = %d\n",(t4[i]>>6),(t4[i] & 0xf),((t4[i] & 0x30) >> 4));
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
		container_of(work, struct mt65xx_led_data, work);

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

static ssize_t store_T1(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int t1 = 0;
	if(buf != NULL && size != 0)
	{
		LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
		t1 = simple_strtoul(buf,&pvalue,10);
		if(t1 < 130)
		{
			T1A = 7;
		}else if(130 <= t1 && t1 < 390)
		{
			T1A = 0;
		}else if(390 <= t1 && t1 < 780)
		{
			T1A = 1;
		}else if(780 <= t1 && t1 < 1560)
		{
			T1A = 2;
		}else if(1560 <= t1 && t1 < 3120)
		{
			T1A = 3;
		}else if(3120 <= t1)
		{
			T1A = 4;
		}
	}
	printk("sn3199              T1A = %d\n",T1A);
	return size;
}

static ssize_t show_T1(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int t1= T1A;
	return sprintf(buf,"%u\n" ,t1);
	return 0;

}

static DEVICE_ATTR(T1,0664, show_T1,store_T1);

static ssize_t store_T2(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int t2 = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			t2 = simple_strtoul(buf,&pvalue,10);
			if(t2 < 130)
			{
				T2B = 0;
			}else if(130 <= t2 && t2 < 390)
			{
				T2B = 1;
			}else if(390 <= t2 && t2 < 780)
			{
				T2B = 2;
			}else if(780 <= t2 && t2 < 1560)
			{
				T2B = 3;
			}else if(1560 <= t2 && t2 < 3120)
			{
				T2B = 4;
			}else if(3120 <= t2 && t2 < 6240)
			{
				T2B = 5;
			}
			else if(6240 <= t2 && t2 < 12480)
			{
				T2B = 6;
			}
			else if(12480 <= t2)
			{
				T2B = 7;
			}
		}
	printk("sn3199              T1A = %d\n",T1A);
	return size;
}

static ssize_t show_T2(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int t2= T2B;
	return sprintf(buf,"%u\n" ,t2);
	return 0;

}

static DEVICE_ATTR(T2,0664, show_T2,store_T2);

static ssize_t store_T4(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	int T4 = 0;
	unsigned int i = 0; 
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			T4 = simple_strtoul(buf,&pvalue,10);
			printk("sn3199                T4 = %d\n",T4);
			if(T4 < (t4[0]>>6))
			{
				T4A = 0;
				T4B = 0;
			}
			else if(T4 > (t4[t4_num-1]>>6))
			{
				T4A = 15;
				T4B = 3;
			}else
			{
				for(i = 0;i<t4_num - 1;i++ )
				{
					if(T4 >= (t4[i] >> 6) && T4 < (t4[i+1]>>6))
					{
						if((T4 - (t4[i]>>6)) <= ((t4[i+1]>>6) - T4))
						{
							T4A = (t4[i] & 0xf);
							T4B = ((t4[i] & 0x30) >> 4);
						}else
						{
							T4A = (t4[i+1] & 0xf);
							T4B = (( t4[i+1] & 0x30) >> 4);
						}
						break;
					}
							
				}
			}
			printk("sn3199           guoyanchun      T4A = %d,T4B = %d\n",T4A,T4B);
			
		}
	return size;
}

static ssize_t show_T4(struct device *dev,struct device_attribute *attr,char *buf)
{
	/*unsigned int AB = TB;
	return sprintf(buf,"%u\n" ,AB);*/
	return 0;

}

static DEVICE_ATTR(T4,0664, show_T4,store_T4);

static ssize_t store_electricity(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{	printk("sn3199    store_electricity\n ");
	char *pvalue = NULL;
	unsigned int electricity = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			electricity = simple_strtoul(buf,&pvalue,10);
			electric_current= electricity;
			switch(electricity)
			{
				case 1:
					electric_current = 3;break;
				case 2:
					electric_current = 2;break;
				case 3:
					electric_current = 1;break;
				case 4:
					electric_current = 0;break;
				case 5:
					electric_current = 7;break;
				case 6:
					electric_current = 6;break;
				case 7:
					electric_current = 5;break;
				case 8:
					electric_current = 4;break;
					
			}
			set_led_electricity();
		}
	return size;
}

static ssize_t show_electricity(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int electricity = electric_current;
	return sprintf(buf,"%u\n" ,electricity);
	return 0;

}

static DEVICE_ATTR(electricity,0664, show_electricity,store_electricity);

static ssize_t store_led_color(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int led_color[6] = {0,0,0,0,0,0};
	
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			led_color[0] = simple_strtoul(buf,&pvalue,16);
			ledcolor[0]= led_color[0];
			if(*pvalue && *pvalue == ',')
				{
					led_color[1] = simple_strtol(pvalue+1,&pvalue , 16);
					ledcolor[1] = led_color[1];
					if(*pvalue && *pvalue == ',')
						{
							led_color[2] = simple_strtol(pvalue+1,&pvalue , 16);
							ledcolor[2] = led_color[2];
							if(*pvalue && *pvalue == ',')
							{
								led_color[3] = simple_strtol(pvalue+1,&pvalue , 16);
								ledcolor[3] = led_color[3];
								if(*pvalue && *pvalue == ',')
								{
									led_color[4] = simple_strtol(pvalue+1,&pvalue , 16);
									ledcolor[4] = led_color[4];
									if(*pvalue && *pvalue == ',')
									{
										led_color[5] = simple_strtol(pvalue+1,NULL , 16);
										ledcolor[5] = led_color[5];
									}
								}
							}
						}
				}
		}
	printk("led_color=%d,%d,%d,%d,%d,%d",led_color[0],led_color[1],led_color[2],led_color[3],led_color[4],led_color[5]);
	led_set_color();
	return size;

}

static ssize_t show_led_color(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int led_color[3]= {led_color[0],led_color[1],led_color[2]};
	return sprintf(buf,"%d,%d,%d\n" ,led_color[0],led_color[1],led_color[2]);
	return 0;

}

static DEVICE_ATTR(led_color,0664, show_led_color,store_led_color);

static ssize_t store_led_flash(struct device *dev,struct device_attribute *attr,const char *buf,size_t size)
{
	char *pvalue = NULL;
	unsigned int led_flash = 0;
	if(buf != NULL && size != 0)
		{
			LEDS_DEBUG("store_pwm_register: size:%d,sddress :0x&s\n", size, buf);
			led_flash = simple_strtoul(buf,&pvalue,10);
			if(led_flash == 1)
				set_led_flash();
			ledflash = led_flash;
		}
	return size;
}

static ssize_t show_led_flash(struct device *dev,struct device_attribute *attr,char *buf)
{
	unsigned int led_flash = ledflash;
	return sprintf(buf,"%u\n" ,led_flash);
	return 0;

}

static DEVICE_ATTR(led_flash,0664, show_led_flash,store_led_flash);

/*******************************************************************************************************************************************************************/
static int sn3199_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	int ret,rc;
	int retval = 1;
	printk("sn3199 driver probe addr:%d\n",client->addr);
       if(client->addr ==i2c_sn3199[0].addr)
       {
		led = kzalloc(sizeof(struct mt65xx_led_data), GFP_KERNEL);
		if (!led)
		{
			ret = -ENOMEM;
			goto err;
		}

	        sn3199_private_data = kzalloc(sizeof(struct sn3199_data), GFP_KERNEL);
	        if(!sn3199_private_data)
		{
	        	printk("sn3199: kzalloc sn3199_data fail!\n");
			return -1;
		}
		memset(sn3199_private_data, 0, sizeof(*sn3199_private_data));
		sn3199_private_data->client = client;
		sn3199_private_data->client1 = client;
		
		mutex_init(&sn3199_private_data->mutex);
		led->cust.mode = MT65XX_LED_MODE_PWM;
		led->cust.data = PWM1;
		led->cust.name = "red";
		led->cdev.name = "red";
		led->cdev.brightness_set = mt65xx_led_set;


		INIT_WORK(&led->work, mt65xx_led_work);
		mt_set_gpio_mode(GPIO18, GPIO_MODE_00);
	       mt_set_gpio_dir(GPIO18, GPIO_DIR_OUT);
	       mt_set_gpio_out(GPIO18, GPIO_OUT_ONE);

		mt_set_gpio_mode((GPIO34 + GPIO_EXTEND_START), GPIO_MODE_00);
		mt_set_gpio_dir((GPIO34 + GPIO_EXTEND_START), GPIO_DIR_OUT);
		mt_set_gpio_out((GPIO34 + GPIO_EXTEND_START), GPIO_OUT_ONE);
	
		mt_set_gpio_mode(GPIO21, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO21, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO21, GPIO_OUT_ONE);
		
	       sn3199_private_data->sn_led_data = led;
		sn3199_private_data->client = sn3199_private_data->client1;
		sn3199_config_interface(0x04, 0x00);
		sn3199_init();
		set_led_t4();
	}
	else
	{
		mt_set_gpio_mode(GPIO19, GPIO_MODE_00);
	       mt_set_gpio_dir(GPIO19, GPIO_DIR_OUT);
	       mt_set_gpio_out(GPIO19, GPIO_OUT_ONE);
		sn3199_private_data->client2 = client;
		sn3199_private_data->client = sn3199_private_data->client2;
		sn3199_config_interface(0x04, 0x80);
		sn3199_init();
	}


	rc = device_create_file(&(client->dev), &dev_attr_led_flash);
	if(rc)
	{
		printk("[led_flash] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_led_color);
	if(rc)
	{
		printk("[led_color] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_T1);
	if(rc)
	{
		printk("[T1] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_T2);
	if(rc)
	{
		printk("[T2] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_T4);
	if(rc)
	{
		printk("[T4] device create file duty fail!\n");	
	}
	rc = device_create_file(&(client->dev), &dev_attr_led_status);
	if(rc)
	{
		printk("[led_status] device create file duty fail!\n");	
	}
	
	rc = device_create_file(&(client->dev), &dev_attr_electricity);
	if(rc)
	{
		printk("[electricity] device create file duty fail!\n");	
	}
	
 
			 
	return 0;

err:
	

	return ret;

}

static void sn3199_late_resume(struct early_suspend *h)
{
	atomic_set(&(sn3199_private_data->early_suspend), 0);

        printk("sn3199 resume------------------------------------------------\n");
}

static void sn3199_early_suspend(struct early_suspend *h)
{
	atomic_set(&(sn3199_private_data->early_suspend), 1);

        printk("sn3199 suspend------------------------------------------------\n");
}
static struct i2c_driver led_sn3199_driver = {
    .probe      = sn3199_driver_probe,
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

        i2c_register_board_info(5, i2c_sn3199, 2);
	//i2c_register_board_info(5, &i2c_sn3199[1], 1);
	/*adapter = i2c_get_adapter(5);
	for (i = 0; i < 2; i++) {
	client = i2c_new_device(adapter, &i2c_sn3199[i]);
		}
       i2c_put_adapter(adapter);
       */
	printk("sn3199 add driver1\n");
	if(i2c_add_driver(&led_sn3199_driver)!=0)
        {
		printk("failed to register led sn3199 driver\n");
        }
	else
	{
		printk("success to register led sn3199 driver\n");
	}
	
        return 0;	
}


static void __exit sn3199_driver_exit (void)
{
	i2c_del_driver(&led_sn3199_driver);
	i2c_unregister_device(sn3199_private_data->client1);
	i2c_unregister_device(sn3199_private_data->client2);
	kfree(sn3199_private_data);
}

module_init(sn3199_driver_init);
module_exit(sn3199_driver_exit);

MODULE_AUTHOR("guo yc");
MODULE_DESCRIPTION("Led SN3199 Driver");
MODULE_LICENSE("GPL");
