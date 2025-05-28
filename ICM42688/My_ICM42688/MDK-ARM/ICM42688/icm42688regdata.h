#ifndef ICM42688_REGDATA_H
#define ICM42688_REGDATA_H
#include "main.h"
//¼Ä´æÆ÷½á¹¹Ìå

/**
 * @struct sSignalPathReset_t
 * @brief  Register:SIGNAL_PATH_RESET  
 */
typedef struct {
	uint8_t   reserved0: 1; 
	uint8_t   FIFOFlush: 1; 
	uint8_t   TMSTStrobe: 1; 
	uint8_t   abortAndReset: 1; 
	uint8_t   reserved4:1; 
	uint8_t   DMPMemResetEn:1;
	uint8_t   DMPInitEn:1;
	uint8_t   reserved7:1;
} __attribute__ ((packed)) sSignalPathReset_t;


/**
 * @struct sAPEXConfig0_t
 * @brief  Register:APEX_Config0
 */
typedef struct 
{
	uint8_t   dmpODR: 2; 
	uint8_t   reserved: 1; 
	uint8_t   R2WEn: 1; 
	uint8_t   tiltEnable:1; 
	uint8_t   PEDEnable:1;
	uint8_t   tapEnable:1;
	uint8_t   DMPPowerSave:1;	
}sAPEXConfig0_t;

/**
 * @struct sAccelConfig0_t
 * @brief  Register:Accel_Config0  
 */
typedef struct {
	uint8_t   accelODR: 4; 
	uint8_t   reserved: 1; 
	uint8_t   accelFsSel: 3; 
} __attribute__ ((packed)) sAccelConfig0_t;

/**
 * @struct sGyroConfig0_t
 * @brief  Register:Gyro_Config0
 */
typedef struct {
	uint8_t   gyroODR: 4; 
	uint8_t   reserved: 1; 
	uint8_t   gyroFsSel: 3; 
} __attribute__ ((packed)) sGyroConfig0_t;

/**
 * @struct sGyroConfig1_t
 * @brief  Register:Gyro_Config1   
 */
typedef struct {
	uint8_t   gyroDec2M2ODR: 2; 
	uint8_t   gyroUIFiltODR: 2; 
	uint8_t   reserved: 1; 
	uint8_t   agyroFiltBW: 3; 
} __attribute__ ((packed)) sGyroConfig1_t;

/**
 * @struct sPWRMgmt0_t
 * @brief  Register:PWR_MGMT0   
 */
typedef struct {
	uint8_t   accelMode: 2; 
	uint8_t   gyroMode: 2; 
	uint8_t   idle: 1; 
	uint8_t   tempDis:1; 
	uint8_t   reserved:2;
} __attribute__ ((packed)) sPWRMgmt0_t;

/**
 * @struct sINTFConfig0_t
 * @brief  Register:INTF_Config0
 */
typedef struct {
	uint8_t   UISifsConfig: 2; 
	uint8_t   reserved:2;
	uint8_t   sensorDataEndian: 1; 
	uint8_t   FIFOCountEndian: 1; 
	uint8_t   FIFOCountRec:1; 
	uint8_t   FIFOInvalidSampleTreatment:1;
} __attribute__ ((packed)) sINTFConfig0_t;


/**
 * @struct sINTFConfig1_t
 * @brief  Register:INTF_Config1
 */
typedef struct {
	uint8_t   clksel: 2; 
	uint8_t   rtcMode: 1; 
	uint8_t   accelLpClkSel: 1; 
	uint8_t   reserved:4;
} __attribute__ ((packed)) sINTFConfig1_t;

/**
 * @struct sAccelConfig1_t
 * @brief  Register:Accel_Config1
*/
typedef struct {
	uint8_t   reserved: 1; 
	uint8_t   accelDec2M2ORD: 2; 
	uint8_t   accelUIFiltORD: 2; 
	uint8_t   reserved2: 3;
} __attribute__ ((packed)) sAccelConfig1_t;

/**
 * @struct sGyroAccelConfig0_t
 * @brief  Register:Gyro_Accel_Config0
 */
typedef struct {
	uint8_t   gyroUIFiltBW: 4; 
	uint8_t   accelUIFiltBW: 4; 
} __attribute__ ((packed)) sGyroAccelConfig0_t;

/**
 * @struct sAPEXConfig7_t
 * @brief  Register:APEX_Config7
 */
typedef struct {
	uint8_t   tapMaxPeakTol: 2; 
	uint8_t   tapMinJerkThr: 6; 
} __attribute__ ((packed)) sAPEXConfig7_t;

/**
 * @struct sAPEXConfig8_t
 * @brief  Register:APEX_Config8
 */
typedef struct {
	uint8_t   tapTmin: 3; 
	uint8_t   tapTavg: 2; 
	uint8_t   tapTmax: 2;
	uint8_t   reserved:1;
} __attribute__ ((packed)) sAPEXConfig8_t;

/**
 * @struct sAPEXConfig4_t
 * @brief  Register:APEX_Config4
 */
typedef struct {
	uint8_t   reserved: 3;
	uint8_t   sleepTimeOut: 3; 
	uint8_t   tiltWaitTimeSel: 2; 
} __attribute__ ((packed)) sAPEXConfig4_t;

/**
 * @struct sSMDConfig_t
 * @brief  Register:SMD_Config   
 */
typedef struct {
	uint8_t   SMDMode: 2; 
	uint8_t   WOMMode: 1; 
	uint8_t   WOMIntMode: 1;
	uint8_t   reserved: 4;
} __attribute__ ((packed)) sSMDConfig_t;


/**
 * @struct sGyroConfigStatic9_t
 * @brief  Register:Gyro_Config_Static9   
 */
typedef struct {
	uint8_t   gyroNFCoswzX8: 1; 
	uint8_t   gyroNFCoswzY8: 1; 
	uint8_t   gyroNFCoswzZ8: 1;
	uint8_t   gyroNFCoswzSelX: 1;
	uint8_t   gyroNFCoswzSelY: 1;
	uint8_t   gyroNFCoswzSelZ: 1;
	uint8_t   reserved:2;
} __attribute__ ((packed)) sGyroConfigStatic9_t;

/**
 * @struct sGyroConfigStatic2_t
 * @brief  Register:Gyro_Config_Static2
 */
typedef struct {
	uint8_t   gyroNFDis: 1; 
	uint8_t   gyroAAFDis: 1; 
	uint8_t   reserved: 6;
} __attribute__ ((packed)) sGyroConfigStatic2_t;

/**
 * @struct sGyroConfigStatic5_t
 * @brief  Register:Gyro_Config_Static5
 */
typedef struct {
	uint8_t   gyroAAFDeltsqr: 4; 
	uint8_t   gyroAAFBitshift: 4; 
} __attribute__ ((packed)) sGyroConfigStatic5_t;

/**
 * @struct sAccelConfigStatic2_t
 * @brief  Register:Accel_Config_Static2
 */
typedef struct {
	uint8_t   accelAAFDis: 1; 
	uint8_t   accelAAFDelt: 6;
	uint8_t   reserved: 1; 
} __attribute__ ((packed)) sAccelConfigStatic2_t;

/**
 * @struct sAccelConfigStatic4_t
 * @brief  Register:Accel_Config_Static4 
 */
typedef struct {
	uint8_t   accelAAFDeltsqr: 4; 
	uint8_t   accelAAFBitshift: 4; 
} __attribute__ ((packed)) sAccelConfigStatic4_t;


/**
 * @struct sFIFOConfig1_t
 * @brief  Register:FIFO_Config1
 */
typedef struct {
	uint8_t   FIFOAccelEn: 1; 
	uint8_t   FIFOGyroEn: 1; 
	uint8_t   FIFOTempEn: 1;
	uint8_t   FIFOTmstFsyncEn: 1;
	uint8_t   FIFOHiresEn: 1;
	uint8_t   FIFOWmGtTh: 1;
	uint8_t   FIFOResumeParialRd:1;
	uint8_t   reseved :1;
} __attribute__ ((packed)) sFIFOConfig1_t;

/**
 * @struct sTMSTConfig_t
 * @brief  Register:TMST_Config
 */
typedef struct {
	uint8_t   TimeStampEn: 1; 
	uint8_t   TimeStampFSYNCEn: 1; 
	uint8_t   TimeStampDeltaEn: 1;
	uint8_t   TimeStampRes: 1;
	uint8_t   TimeStampToRegsEn: 1;
	uint8_t   reseved :3;
} __attribute__ ((packed)) sTMSTConfig_t;

/**
 * @struct sINTConfig_t
 * @brief  Register:INT_Config
 */
typedef struct {
	uint8_t   INT1Polarity: 1;
	uint8_t   INT1DriveCirCuit: 1;
	uint8_t   INT1Mode: 1;
	uint8_t   INT2Polarity: 1;
	uint8_t   INT2DriveCirCuit: 1;
	uint8_t   INT2Mode :1;
	uint8_t   reversed :2;
} __attribute__ ((packed)) sINTConfig_t;

/**
 * @struct sINTSource_t
 * @brief  Register:INT_Source
 */
typedef struct {
	uint8_t   tapDetIntEn: 1;
	uint8_t   sleepDetIntEn: 1;
	uint8_t   wakeDetIntEn: 1;
	uint8_t   tiltDetIntEn: 1;
	uint8_t   stepCntOflIntEn :1;
	uint8_t   stepDetIntEn :1;
	uint8_t   reserved:2;
} __attribute__ ((packed)) sINTSource_t;

#endif