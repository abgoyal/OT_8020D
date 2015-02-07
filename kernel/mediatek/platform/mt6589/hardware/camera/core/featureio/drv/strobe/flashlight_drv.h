

#ifndef _FLASHLIGHT_DRV_H_
#define _FLASHLIGHT_DRV_H_

#include <utils/threads.h>
using namespace android;

class FlashlightDrv : public StrobeDrv
{
private:

    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    FlashlightDrv();


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual ~FlashlightDrv();


public:

    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    static StrobeDrv* getInstance();


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual void destroyInstance();


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual FLASHLIGHT_TYPE_ENUM getFlashlightType() const;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int init(unsigned long sensorDev);

    virtual int initTemp(unsigned long sensorDev);


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int uninit();
    int uninitNoLock();


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setFire(unsigned long a_fire) ;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setLevel(unsigned long a_level) ;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setTimeus(unsigned long a_timeus) ;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setStartTimeus(unsigned long a_timeus) ;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setState(unsigned long a_state) ;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setFlashlightModeConf(unsigned long a_strobeMode) ;


    /*******************************************************************************
       * Functionality :
       *
       ********************************************************************************/
    virtual int setCaptureFlashlightConf(unsigned long a_strobeWidth) ;


    /*******************************************************************************
       * Author : Cotta
       * Functionality : commadn control
       *
       ********************************************************************************/
    virtual int sendCommand(unsigned int cmd, unsigned int pArg1, unsigned int *pArg2, unsigned int *pArg3);

	virtual int isOn(int* a_isOn);
    virtual int setOnOff(int a_isOn);
    virtual int setStep(int step);
	virtual int setDuty(int duty);
	virtual int getFlashReg(int shift, unsigned short* reg);
	virtual int setFlashReg(int shift, unsigned short reg, unsigned short mask);
	virtual int getCoolDownTime(int* ms);


	virtual int lockSensedV();
	virtual int unlockSensedV();
	virtual int mapDutyStep(int peakI, int aveI, int* duty, int* step);

	virtual int getVBat(int* vbat);
	virtual int setTimeOutTime(int ms);


	virtual int hasFlashHw();

protected:

private:

    /*******************************************************************************
       * Author : Cotta
       * Functionality : set value of sensor capture delay
       *
       ********************************************************************************/
    int setCaptureDelay(unsigned int value);


    /*******************************************************************************
       * Author : Cotta
       * Functionality : get value of strobe WDT. unit : ms
       *
       ********************************************************************************/
    int getStrobeWDTValue(unsigned int *pValue);

    int m_sensorDev;
    int m_fdSTROBE;
    int m_flashType;
    int m_strobeMode;
    volatile int mUsers;
    mutable Mutex mLock;

    int m_isOn;
    int m_bTempInit;
};

#endif

