
#ifndef _OV8825SUNNYAF_H
#define _OV8825SUNNYAF_H

#include <linux/ioctl.h>
//#include "kd_imgsensor.h"

#define OV8825SUNNYAF_MAGIC 'A'
//IOCTRL(inode * ,file * ,cmd ,arg )


//Structures
typedef struct {
//current position
unsigned long u4CurrentPosition;
//macro position
unsigned long u4MacroPosition;
//Infiniti position
unsigned long u4InfPosition;
//Motor Status
bool          bIsMotorMoving;
//Motor Open?
bool          bIsMotorOpen;
} stOV8825SUNNYAF_MotorInfo;

//Control commnad
//S means "set through a ptr"
//T means "tell by a arg value"
//G means "get by a ptr"             
//Q means "get by return a value"
//X means "switch G and S atomically"
//H means "switch T and Q atomically"
#define OV8825SUNNYAFIOC_G_MOTORINFO _IOR(OV8825SUNNYAF_MAGIC,0,stOV8825SUNNYAF_MotorInfo)

#define OV8825SUNNYAFIOC_T_MOVETO _IOW(OV8825SUNNYAF_MAGIC,1,unsigned long)

#define OV8825SUNNYAFIOC_T_SETINFPOS _IOW(OV8825SUNNYAF_MAGIC,2,unsigned long)

#define OV8825SUNNYAFIOC_T_SETMACROPOS _IOW(OV8825SUNNYAF_MAGIC,3,unsigned long)

#else
#endif
