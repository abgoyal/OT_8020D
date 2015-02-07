
//modified by zhfan, pr450349, porting diablox main camera imx135 VCM driver dw9714a 
#include <utils/Log.h>
#include <fcntl.h>
#include <math.h>

//#include "msdk_nvram_camera_exp.h"
//#include "msdk_lens_exp.h"
#include "camera_custom_nvram.h"
#include "camera_custom_lens.h"

const NVRAM_LENS_PARA_STRUCT DW9714A_LENS_PARA_DEFAULT_VALUE =
{
    //Version
    NVRAM_CAMERA_LENS_FILE_VERSION,

    // Focus Range NVRAM
    {0, 1023},

    // AF NVRAM
    {
        // -------- AF ------------
        {110,  // i4Offset//Carter@2013-6-9[150==>110]
         21,  // i4NormalNum
         21,  // i4MacroNum
         0,  // i4InfIdxOffset
         0,  // i4MacroIdxOffset
        {
                       0,  10, 20,  30,  40,  50,  60,  70, 80, 90, 
                      105,125, 155, 190, 235, 285, 340,  405, 480, 560,   
                      645, 0,   0,   0,   0,   0,   0,   0,   0,   0, 
            },
        30,  // i4THRES_MAIN
        20,  // i4THRES_SUB
        3,  // i4INIT_WAIT
          {500, 500, 500, 500, 500}, // i4FRAME_WAIT
        0,  // i4DONE_WAIT
              
        0,  // i4FAIL_POS

        33,  // i4FRAME_TIME
        10,  // i4FIRST_FV_WAIT
            
        45,  // i4FV_CHANGE_THRES
        10000,  // i4FV_CHANGE_OFFSET
        12,  // i4FV_CHANGE_CNT
        40,  // i4GS_CHANGE_THRES
        10000,  // i4GS_CHANGE_OFFSET
        12,  // i4GS_CHANGE_CNT
        15,  // i4FV_STABLE_THRES
        10000,  // i4FV_STABLE_OFFSET
        4,  // i4FV_STABLE_NUM
        4,  // i4FV_STABLE_CNT
        12,  // i4FV_1ST_STABLE_THRES
        10000,  // i4FV_1ST_STABLE_OFFSET
        6,  // i4FV_1ST_STABLE_NUM
        6  // i4FV_1ST_STABLE_CNT
         },
         
         // -------- ZSD AF ------------
         {202, // i4Offset
           9, // i4NormalNum
           12, // i4MacroNum
            0, // i4InfIdxOffset
            0, //i4MacroIdxOffset           
           {
                 0,   20,  44,  72, 105, 144, 189, 242, 305, 379,
               453,  527,   0,   0,   0,   0,   0,   0,   0,   0,
               0,   0,   0,   0,   0,   0,   0,   0,   0,   0              
           },
           30, // i4THRES_MAIN;
           20, // i4THRES_SUB;            
           1,  // i4INIT_WAIT;
           {500, 500, 500, 500, 500}, // i4FRAME_WAIT
           0,  // i4DONE_WAIT;
                     
           0,  // i4FAIL_POS;

           66,  // i4FRAME_TIME                                  
           5,  // i4FIRST_FV_WAIT;
                     
           45,  // i4FV_CHANGE_THRES;
           10000,  // i4FV_CHANGE_OFFSET;        
           12,  // i4FV_CHANGE_CNT;
           40,  // i4GS_CHANGE_THRES;    
           10000,  // i4GS_CHANGE_OFFSET;    
           12,  // i4GS_CHANGE_CNT;            
           15,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
           10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
           4,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
           4,  // i4FV_STABLE_CNT;           // max = 9                                      
           12,  // i4FV_1ST_STABLE_THRES;        
           10000,  // i4FV_1ST_STABLE_OFFSET;
           6,  // i4FV_1ST_STABLE_NUM;                        
           6  // i4FV_1ST_STABLE_CNT;      
           }, 
           
           // -------- VAFC ------------
         {130, // i4Offset 		//Carter.chen@2013-6-7[200-->180-->130]
           16, // i4NormalNum //Carter.chen@2013-6-7
           23, // i4MacroNum
            0, // i4InfIdxOffset
            0, //i4MacroIdxOffset           
             {
                  0,  20,  40,  60,  80, 100, 120, 140, 160, 180,
                  200, 220, 240, 260, 280, 300, 320, 340, 360, 380,
                  410,   450,   500,   0,   0,   0,   0,   0,   0,   0              
             },
           30, // i4THRES_MAIN;
           20, // i4THRES_SUB;            
           1,  // i4INIT_WAIT;
           {500, 500, 500, 500, 500}, // i4FRAME_WAIT
           0,  // i4DONE_WAIT;
             
           0,  // i4FAIL_POS;

           33,  // i4FRAME_TIME                          
           5,  // i4FIRST_FV_WAIT;
             
           45,  // i4FV_CHANGE_THRES;
           10000,  // i4FV_CHANGE_OFFSET;        
           12,  // i4FV_CHANGE_CNT;
           40,  // i4GS_CHANGE_THRES;    
           10000,  // i4GS_CHANGE_OFFSET;    
           12,  // i4GS_CHANGE_CNT;            
           12,  // i4FV_STABLE_THRES;         // percentage -> 0 more stable  
           10000,  // i4FV_STABLE_OFFSET;        // value -> 0 more stable
           6,  // i4FV_STABLE_NUM;           // max = 9 (more stable), reset = 0
           6,  // i4FV_STABLE_CNT;           // max = 9                                      
           12,  // i4FV_1ST_STABLE_THRES;        
           10000,  // i4FV_1ST_STABLE_OFFSET;
           6,  // i4FV_1ST_STABLE_NUM;                        
           6  // i4FV_1ST_STABLE_CNT;      
          },

        // --- sAF_TH ---
         {
          8,   // i4ISONum;
          {100,150,200,300,400,600,800,1600},       // i4ISO[ISO_MAX_NUM];
                  
          6,   // i4GMeanNum;
          {20,55,105,150,180,205},        // i4GMean[GMEAN_MAX_NUM];

          { 31, 31, 31, 31, 31, 89, 89, 89,
            63, 63, 63, 63, 63,127,127,127,
           127,127,127,127,127,180,180,180},        // i4GMR[3][ISO_MAX_NUM];
          
// ------------------------------------------------------------------------                  
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},        // i4FV_DC[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
          {120000,120000,120000,120000,120000,100000,80000,80000,
           120000,120000,120000,120000,120000,100000,80000,80000,
           120000,120000,120000,120000,120000,100000,80000,80000,
           120000,120000,120000,120000,120000,100000,80000,80000,
           120000,120000,120000,120000,120000,100000,80000,80000,
           120000,120000,120000,120000,120000,100000,80000,80000},         // i4MIN_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];        

          {   5,5,5,5,5,5,0,0,
              5,5,5,5,5,5,0,0,
              5,5,5,5,5,5,0,0,
              5,5,5,5,5,5,0,0,
              5,5,5,5,5,5,0,0,
              5,5,5,5,5,5,0,0}, // i4HW_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},        // i4FV_DC2[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0},         // i4MIN_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];
          
          {0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0}          // i4HW_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];       
          
         },
// ------------------------------------------------------------------------

         // --- sZSDAF_TH ---
          {
           8,   // i4ISONum;
           {100,150,200,300,400,600,800,1600},       // i4ISO[ISO_MAX_NUM];
                   
           6,   // i4GMeanNum;
           {20,55,105,150,180,205},        // i4GMean[GMEAN_MAX_NUM];

           { 31, 31, 31, 31, 31, 31, 31, 31,
             63, 63, 63, 63, 63, 63, 63, 63,
            127,127,127,127,127,127,127,127},        // i4GMR[3][ISO_MAX_NUM];
           
// ------------------------------------------------------------------------                   
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},        // i4FV_DC[GMEAN_MAX_NUM][ISO_MAX_NUM];
            
           {50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000,
            50000,50000,50000,50000,50000,50000,50000,50000},         // i4MIN_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];        
         
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},       // i4HW_TH[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},        // i4FV_DC2[GMEAN_MAX_NUM][ISO_MAX_NUM];
            
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0},         // i4MIN_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];
           
           {0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0}          // i4HW_TH2[GMEAN_MAX_NUM][ISO_MAX_NUM];       
// ------------------------------------------------------------------------           
          },

          1, // i4VAFC_FAIL_CNT;
          8, // i4CHANGE_CNT_DELTA;//Carter@2013-6-9[0==>16==>8]

          0, // i4LV_THRES;

          18, // i4WIN_PERCENT_W;
          24, // i4WIN_PERCENT_H;                
          200,  // i4InfPos;
          20, //i4AFC_STEP_SIZE;

          {
              {50, 100, 150, 200, 250}, // back to bestpos step
              { 0,   0,   0,   0,   0}  // hysteresis compensate step
          },

          {0, 50, 150, 250, 350}, // back jump
          400,  //i4BackJumpPos

          80, // i4FDWinPercent;
          40, // i4FDSizeDiff;

          3,   //i4StatGain          

          {0,0,0,0,0,0,0,0,0,0,
           0,0,0,0,0,0,0,0,0,0}// i4Coef[20];
    },

    {0}
};


UINT32 DW9714A_getDefaultData(VOID *pDataBuf, UINT32 size)
{
    UINT32 dataSize = sizeof(NVRAM_LENS_PARA_STRUCT);

    if ((pDataBuf == NULL) || (size < dataSize))
    {
        return 1;
    }

    // copy from Buff to global struct
    memcpy(pDataBuf, &DW9714A_LENS_PARA_DEFAULT_VALUE, dataSize);

    return 0;
}

PFUNC_GETLENSDEFAULT pDW9714A_getDefaultData = DW9714A_getDefaultData;


