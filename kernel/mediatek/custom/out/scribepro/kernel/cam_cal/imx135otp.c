/*
 * Driver for CAM_CAL
 * zhfan for sunny module otp PR 450709
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "imx135otp.h"
#include <asm/system.h>  // for SMP

static bool IMX135OtpDebug = FALSE; //zhfan for control log pr458701
//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
//zhfan for control log pr458701
#define CAM_CALDB(fmt, arg...) if(IMX135OtpDebug){printk( "[JRDIMX135MIPIRawOTP] "  fmt, ##arg);}
#else
#define CAM_CALDB(x,...)
#endif
#define Sleep(ms) mdelay(ms)


static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 1


/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x6d>>1)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

/*=======================================================================
  * camera sysfs zhfan begin pr458701
  * zhfan PR459644
  * =>modify sysfs permission from all WR to other user cannot write
  *=======================================================================*/
#define IMX135OTP_SYSFS_SUPPORT
#ifdef IMX135OTP_SYSFS_SUPPORT
static ssize_t IMX135OTP_value_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    char *s = buf;
    int ret;
    U8 val[4];

    return (s - buf);
}


static ssize_t IMX135OTP_value_store(struct kobject *kobj, struct kobj_attribute *attr,
               const char *buf, size_t n)
{
    char *s = buf;
    int retval;
    ssize_t ret_val;
    char cmd_str[16] = {0};
    int val;
    kal_uint8 i;
    int ret;
    sscanf(buf, "%s %d", (char *)&cmd_str, &val);

    if(strcmp(cmd_str, "imx135otplog") == 0)
    {
        if(0 != val)
            IMX135OtpDebug = TRUE;
        else
            IMX135OtpDebug = FALSE;
    }
    return n;
}

static DEVICE_ATTR(IMX135OTP_DEBUG, 0644,  IMX135OTP_value_show, IMX135OTP_value_store);  //0666->0664 

static struct attribute *IMX135OTP_sysfs_attrs[] = {
    &dev_attr_IMX135OTP_DEBUG.attr,
    NULL,
};
static struct attribute_group IMX135OTP_attr_group = {
        .attrs = IMX135OTP_sysfs_attrs,
};

//add sysfs
struct kobject *IMX135OTP_ctrl_kobj;
static int IMX135OTP_sysfs_init(void)
{ 
    IMX135OTP_ctrl_kobj = kobject_create_and_add("IMX135OTP", NULL);
    if (!IMX135OTP_ctrl_kobj)
        return -ENOMEM;

    return sysfs_create_group(IMX135OTP_ctrl_kobj, &IMX135OTP_attr_group);
}
//remove sysfs
static void IMX135OTP_sysfs_exit(void)
{
    sysfs_remove_group(IMX135OTP_ctrl_kobj, &IMX135OTP_attr_group);

    kobject_put(IMX135OTP_ctrl_kobj);
}
#endif
//zhfan end

/*******************************************************************************
*
********************************************************************************/
//Address: 2Byte, Data: 1Byte
int iWriteCAM_CAL(u16 a_u2Addr  , u32 a_u4Bytes, u8 puDataInBytes)
{
    int  i4RetValue = 0;
    u32 u4Index = 0;
	int retry = 3;
    char puSendCmd[3] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,puDataInBytes};
	do {
        CAM_CALDB("[CAM_CAL][iWriteCAM_CAL] Write 0x%x=0x%x \n",a_u2Addr, puDataInBytes);
		i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, 3);
        if (i4RetValue != 3) {
            CAM_CALDB("[CAM_CAL] I2C send failed!!\n");
        }
        else {
            break;
    	}
        mdelay(10);
    } while ((retry--) > 0);    
   //CAM_CALDB("[CAM_CAL] iWriteCAM_CAL done!! \n");
    return 0;
}


//Address: 2Byte, Data: 1Byte
int iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};

    //CAM_CALDB("[CAM_CAL] iReadCAM_CAL!! \n");   
    //CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	
    if (i4RetValue != 2)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    //CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
	CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 

    //CAM_CALDB("[CAM_CAL] iReadCAM_CAL done!! \n");
    return 0;
}

kal_bool IMX135OTPWriteSensor(kal_uint16 address, kal_uint16 para)
{
    s8 ret;
    ret = iWriteCAM_CAL(address, 1, para);	
    if (ret != 0)
    {
        CAM_CALDB("[CAM_CAL] I2C iWriteData failed!! \n");
    }
}

kal_uint8 IMX135OTPReadSensor(kal_uint16 address)
{
    kal_uint16 getByte;
    s8 ret;
    ret = iReadCAM_CAL(address, 1, &getByte);
    CAM_CALDB("!!!!!address=0x%x, getByte= 0x%x!!!!!\n", address, getByte);
    return getByte;
}

//read data from the address of bank and save the data in iBuffer
kal_bool IMX135SunnyReadOtp(u8 Bank,u16 address,u8 *iBuffer,u16 buffersize)
{
    u16 reVal;
    u16 j = 0;
    u16 i = 0;
    u8 rdrdy_status;
    u16 k = buffersize;

    CAM_CALDB("IMX135Sunny_ReadOtp ENTER Bank:0x%x address:0x%x buffersize:%d\n ",Bank, address, buffersize);

    while (i<buffersize)
    {
        j=0;
        IMX135OTPWriteSensor(0x3b02, Bank);//select bank
        IMX135OTPWriteSensor(0x3b00, 0x01);//select bank
        Sleep(1);//10 -> 100
        rdrdy_status = IMX135OTPReadSensor(0x3b01);//select bank

        if(0x01 == rdrdy_status)
        {
            CAM_CALDB("Ready to read OTP!\n");
        }
        else
        {
            CAM_CALDB("Haven't been Ready to read OTP!\n");
            return KAL_FALSE;
        }
        while (j<64)
        {
            reVal= IMX135OTPReadSensor(address+j);
            *(iBuffer+i) =(u8)reVal;
            //*(iBuffer+k) =(u8)reVal;
            i++;
            j++;
            if (i>=buffersize)
            {
                break;
            }
        }
    }
    return KAL_TRUE;
}

kal_bool CheckIMX135SunnyOTPValidGroup(u8 *groupBank, u8 arrayLength, s8 *validIndex)
{
    u8 i, programFlag;
    u16 ProgramFlagReg = 0x3b04;
    kal_bool ret;

    *validIndex = -1;

    for (i = 0 ; i < arrayLength; i++)
    {
        IMX135SunnyReadOtp(groupBank[i], ProgramFlagReg, &programFlag, 1);
        if(1 == programFlag)
        {
            *validIndex = i;
            break;
        }
    }
    if(*validIndex != -1)
    {
        ret = KAL_TRUE;
    }
    else
    {
        ret = KAL_FALSE;
    }
    return ret;
}

//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
   int  i4ResidueDataLength;
   u32 u4IncOffset = 0;
   u32 u4CurrentOffset;
   u8 * pBuff;

   CAM_CALDB("[S24CAM_CAL] iWriteData\n" );

   i4ResidueDataLength = (int)ui4_length;
   u4CurrentOffset = ui4_offset;
   pBuff = pinputdata;   

   CAM_CALDB("[CAM_CAL] iWriteData u4CurrentOffset is %d \n",u4CurrentOffset);   

   do 
   {
       CAM_CALDB("[CAM_CAL][iWriteData] Write 0x%x=0x%x \n",u4CurrentOffset, pBuff[0]);
       i4RetValue = iWriteCAM_CAL((u16)u4CurrentOffset, 1, pBuff[0]);
       if (i4RetValue != 0)
       {
            CAM_CALDB("[CAM_CAL] I2C iWriteData failed!! \n");
            return -1;
       }           
       u4IncOffset ++;
       i4ResidueDataLength --;
       u4CurrentOffset = ui4_offset + u4IncOffset;
       pBuff = pinputdata + u4IncOffset;
   }while (i4ResidueDataLength > 0);
   CAM_CALDB("[S24CAM_CAL] iWriteData done\n" );
 
   return 0;
}

//Burst Read Data
static int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
    int  i4RetValue = 0;
    int  i4ResidueDataLength;
    u32 u4IncOffset = 0;
    u32 u4CurrentOffset;
    u8 * pBuff;
    u8 awbGroupBank[] = {0x02,0x01};
    u8 rdRdyStatus, arrayLength;
    s8 ValidIndex = -1;
    kal_bool ret;
	//   CAM_CALDB("[S24EEPORM] iReadData \n" );

	/*   if (ui4_offset + ui4_length >= 0x2000)
	{
	  CAM_CALDB("[S24CAM_CAL] Read Error!! S-24CS64A not supprt address >= 0x2000!! \n" );
	  return -1;
	}
	*/

    //1. check which group is valid
    arrayLength = sizeof(awbGroupBank) / sizeof(awbGroupBank[0]);
    ret = CheckIMX135SunnyOTPValidGroup(&awbGroupBank[0], arrayLength, &ValidIndex);
    if (KAL_TRUE == ret)
    {
        CAM_CALDB("AWB OTP exist! Valid group is awbGroupBank[%d]: 0x%x\n", ValidIndex, awbGroupBank[ValidIndex]);
    }
    else
    {
        CAM_CALDB("No AWB OTP Data!\n");
        return -1;
    }

    //2. read otp
    i4RetValue = IMX135SunnyReadOtp(awbGroupBank[ValidIndex], 0x3b04, pinputdata, ui4_length);
    //   CAM_CALDB("[S24EEPORM] iReadData finial address is %d length is %d buffer address is 0x%x\n",u4CurrentOffset, i4ResidueDataLength, pBuff);   
    //   CAM_CALDB("[S24EEPORM] iReadData done\n" );
    return 0;
}

static void Enb_OTP_Read(int enb){
	u8 * pOneByteBuff = NULL;
	pOneByteBuff = (u8 *)kmalloc(I2C_UNIT_SIZE, GFP_KERNEL);
	//Enable Reading OTP
	CAM_CALDB("[CAM_CAL]Enb_OTP_Read=%d\n",enb);
	iReadData(0x3d21, I2C_UNIT_SIZE, pOneByteBuff);
	CAM_CALDB("[CAM_CAL]0x3d21=0x%x\n",pOneByteBuff);
	if(enb){
		*pOneByteBuff = (*pOneByteBuff | 0x01);
	    iWriteData(0x3d21, I2C_UNIT_SIZE, pOneByteBuff );
	} else {
	    *pOneByteBuff = (*pOneByteBuff & 0xFE);
	    iWriteData(0x3d21, I2C_UNIT_SIZE, pOneByteBuff );
	}
}

static void Clear_OTP_Buff(){
	u8 AllZero[OTP_SIZE]={0};
	iWriteData(OTP_START_ADDR, OTP_SIZE, AllZero);
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB("[S24CAM_CAL] init Working buffer address 0x%8x  command is 0x%8x\n", (u32)pWorkingBuff, (u32)a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif 
            CAM_CALDB("[CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[CAM_CAL] length %d \n", ptempbuf->u4Length);
            CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%8x \n", (u32)pWorkingBuff);

            //Enb_OTP_Read(1); //Enable OTP Read
            i4RetValue = iReadData((u16)(ptempbuf->u4Offset+OTP_START_ADDR), ptempbuf->u4Length, pWorkingBuff);
			//Enb_OTP_Read(0); //Disable OTP Read
			//Clear_OTP_Buff(); //Clean OTP buff
            CAM_CALDB("[S24CAM_CAL] After read Working buffer data  0x%4x \n", *pWorkingBuff);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     CAM_CALDB("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%8x \n", (u32)pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

/*
    if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, CAM_CAL_DRVNAME))
    {
        CAM_CALDB("[S24CAM_CAL] Fail to enable analog gain\n");
        return -EIO;
    }
*/

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {CAM_CAL_I2C_GROUP_ID, OV5647OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, OV5647OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect,                           
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = OV5647OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S24CAM_CAL] Attached!! \n");
    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{
    i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    IMX135OTP_sysfs_init(); //zhfan for otp sys fs pr458701
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


