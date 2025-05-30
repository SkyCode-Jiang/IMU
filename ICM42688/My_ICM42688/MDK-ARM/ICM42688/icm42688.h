
#ifndef ICM42688_H
#define ICM42688_H

#include "main.h"
#include "icm42688regdata.h"
#define X_ 0
#define Y_ 1
#define Z_ 2
//ÈýÖáÔ­Ê¼Êý¾Ý
typedef struct
{
int16_t X;
int16_t Y;
int16_t Z;
}INT16_XYZ;
//ÈýÖá¸¡µãÀàÐÍÊý¾Ý
typedef struct
{
float X;
float Y;
float Z;
}FLOAT_XYZ;


typedef struct {
uint8_t _r, _g, _b;
uint8_t _mode;
uint8_t _tapNum ;
uint8_t _tapAxis;
uint8_t _tapDir ;
float _gyroRange;
float _accelRange;
bool FIFOMode;
int16_t _accelX;  //FIFO ACC
int16_t _accelY;  //FIFO ACC
int16_t _accelZ;  //FIFO ACC
int16_t _gyroX;   //FIFO GYRO
int16_t _gyroZ;   //FIFO GYRO
int16_t _gyroY;   //FIFO GYRO
int8_t _temp;     //FIFO temp

int8_t _INTPin;
sAccelConfig0_t accelConfig0;
sPWRMgmt0_t PWRMgmt0;
sINTFConfig1_t INTFConfig1;
sAccelConfig1_t accelConfig1;
sGyroAccelConfig0_t gyroAccelConfig0;
sAPEXConfig4_t APEXConfig4;
sAPEXConfig7_t APEXConfig7;
sAPEXConfig8_t APEXConfig8;
sSMDConfig_t SMDConfig;
sGyroConfig1_t  gyroConfig1;
sFIFOConfig1_t FIFOConfig1;
sINTConfig_t INTConfig;
sGyroConfig0_t gyroConfig0;
sAPEXConfig0_t APEXConfig0;
sGyroConfigStatic9_t gyroConfigStatic9;
sGyroConfigStatic2_t gyroConfigStatic2;
sGyroConfigStatic5_t gyroConfigStatic5;
sAccelConfigStatic2_t accelConfigStatic2;
sAccelConfigStatic4_t accelConfigStatic4;
sINTSource_t  INTSource;
sSignalPathReset_t SignalPathReset_t ;
}	ICM42688Set;

#define DFRobot_ICM42688_I2C_L_ADDR 0x68 
#define DFRobot_ICM42688_I2C_H_ADDR 0x69
// Ä£¿éÎª0x69
#define ICM42688_I2C_READ_ADDR   (DFRobot_ICM42688_I2C_H_ADDR<<1 | (0X01))
#define ICM42688_I2C_WRITE_ADDR  (DFRobot_ICM42688_I2C_H_ADDR<<1 &  ~(0X01) )

#define DFRobot_ICM42688_ID  0x47 

#define ICM42688_DEVICE_CONFIG          0x11
#define ICM42688_DRIVE_CONFIG           0x13

#define ICM42688_SIGNAL_PATH_RESET      0x4B

#define ICM42688_PWR_MGMT0              0x4E

#define ICM42688_INT_CONFIG             0x14
#define ICM42688_INT_STATUS             0x2D
#define ICM42688_INT_STATUS2            0x37
#define ICM42688_INT_STATUS3            0x38
#define ICM42688_INT_CONFIG0            0x63
#define ICM42688_INT_CONFIG1            0x64
#define ICM42688_INT_SOURCE0            0x65
#define ICM42688_INT_SOURCE1            0x66
#define ICM42688_INT_SOURCE3            0x68
#define ICM42688_INT_SOURCE4            0x69
#define ICM42688_INT_SOURCE6            0x4D
#define ICM42688_INT_SOURCE7            0x4E
#define ICM42688_INT_SOURCE8            0x4F
#define ICM42688_INT_SOURCE9            0x50
#define ICM42688_INT_SOURCE10           0x51

#define ICM42688_TEMP_DATA1             0x1D
#define ICM42688_TEMP_DATA0             0x1E
#define ICM42688_ACCEL_DATA_X1          0x1F
#define ICM42688_ACCEL_DATA_X0          0x20
#define ICM42688_ACCEL_DATA_Y1          0x21
#define ICM42688_ACCEL_DATA_Y0          0x22
#define ICM42688_ACCEL_DATA_Z1          0x23
#define ICM42688_ACCEL_DATA_Z0          0x24
#define ICM42688_GYRO_DATA_X1           0x25
#define ICM42688_GYRO_DATA_X0           0x26
#define ICM42688_GYRO_DATA_Y1           0x27
#define ICM42688_GYRO_DATA_Y0           0x28
#define ICM42688_GYRO_DATA_Z1           0x29
#define ICM42688_GYRO_DATA_Z0           0x30

#define ICM42688_TMST_FSYNCH            0x43
#define ICM42688_TMST_FSYNCL            0x44

#define ICM42688_GYRO_CONFIG_STATIC2    0x0B
#define ICM42688_GYRO_CONFIG_STATIC3    0x0C
#define ICM42688_GYRO_CONFIG_STATIC4    0x0D
#define ICM42688_GYRO_CONFIG_STATIC5    0x0E
#define ICM42688_GYRO_CONFIG_STATIC6    0x0F
#define ICM42688_GYRO_CONFIG_STATIC7    0x10
#define ICM42688_GYRO_CONFIG_STATIC8    0x11
#define ICM42688_GYRO_CONFIG_STATIC9    0x12
#define ICM42688_GYRO_CONFIG_STATIC10   0x13



#define ICM42688_GYRO_CONFIG0           0x4F
#define ICM42688_ACCEL_CONFIG0          0x50
#define ICM42688_GYRO_CONFIG1           0x51
#define ICM42688_GYRO_ACCEL_CONFIG0     0x52
#define ICM42688_ACCEL_CONFIG1          0x53

#define ICM42688_TMST_CONFIG            0x54


#define ICM42688_SMD_CONFIG             0x57

#define ICM42688_FIFO_CONFIG            0x16
#define ICM42688_FIFO_COUNTH            0x2E
#define ICM42688_FIFO_COUNTL            0x2F
#define ICM42688_FIFO_DATA              0x30
#define ICM42688_FIFO_CONFIG1           0x5F
#define ICM42688_FIFO_CONFIG2           0x60
#define ICM42688_FIFO_CONFIG3           0x61
#define ICM42688_FIFO_LOST_PKT0         0x6C
#define ICM42688_FIFO_LOST_PKT1         0x6D

#define ICM42688_FSYNC_CONFIG           0x62




#define ICM42688_SELF_TEST_CONFIG       0x70
#define ICM42688_WHO_AM_I               0x75
#define ICM42688_REG_BANK_SEL           0x76   
#define ICM42688_SENSOR_CONFIG0         0x03



#define ICM42688_XG_ST_DATA             0x5F
#define ICM42688_YG_ST_DATA             0x60
#define ICM42688_ZG_ST_DATA             0x61

#define ICM42688_TMSTVAL0               0x62
#define ICM42688_TMSTVAL1               0x63
#define ICM42688_TMSTVAL2               0x64

#define ICM42688_INTF_CONFIG0           0x4C
#define ICM42688_INTF_CONFIG1           0x4D
#define ICM42688_INTF_CONFIG4           0x7A
#define ICM42688_INTF_CONFIG5           0x7B
#define ICM42688_INTF_CONFIG6           0x7C

#define ICM42688_ACCEL_CONFIG_STATIC2   0x03
#define ICM42688_ACCEL_CONFIG_STATIC3   0x04
#define ICM42688_ACCEL_CONFIG_STATIC4   0x05

#define ICM42688_XA_ST_DATA             0x3B
#define ICM42688_YA_ST_DATA             0x3C
#define ICM42688_ZA_ST_DATA             0x3D

#define ICM42688_APEX_DATA0             0x31
#define ICM42688_APEX_DATA1             0x32
#define ICM42688_APEX_DATA2             0x33
#define ICM42688_APEX_DATA3             0x34
#define ICM42688_APEX_DATA4             0x35
#define ICM42688_APEX_DATA5             0x36
#define ICM42688_APEX_CONFIG0           0x56
#define ICM42688_APEX_CONFIG1           0x40 
#define ICM42688_APEX_CONFIG2           0x41
#define ICM42688_APEX_CONFIG3           0x42
#define ICM42688_APEX_CONFIG4           0x43
#define ICM42688_APEX_CONFIG5           0x44
#define ICM42688_APEX_CONFIG6           0x45
#define ICM42688_APEX_CONFIG7           0x46
#define ICM42688_APEX_CONFIG8           0x47
#define ICM42688_APEX_CONFIG9           0x48

#define ICM42688_ACCEL_WOM_X_THR        0x4A
#define ICM42688_ACCEL_WOM_Y_THR        0x4B
#define ICM42688_ACCEL_WOM_Z_THR        0x4C

#define ICM42688_OFFSET_USER0           0x77
#define ICM42688_OFFSET_USER1           0x78
#define ICM42688_OFFSET_USER2           0x79
#define ICM42688_OFFSET_USER3           0x7A
#define ICM42688_OFFSET_USER4           0x7B
#define ICM42688_OFFSET_USER5           0x7C
#define ICM42688_OFFSET_USER6           0x7D
#define ICM42688_OFFSET_USER7           0x7E
#define ICM42688_OFFSET_USER8           0x7F

#define ICM42688_STEP_DET_INT           1<<5
#define ICM42688_STEP_CNT_OVF_INT       1<<4
#define ICM42688_TILT_DET_INT           1<<3
#define ICM42688_WAKE_INT               1<<2
#define ICM42688_SLEEP_INT              1<<1
#define ICM42688_TAP_DET_INT            1

#define ICM42688_SMD_INT                1<<3
#define ICM42688_WOM_Z_INT              1<<2
#define ICM42688_WOM_Y_INT              1<<1
#define ICM42688_WOM_X_INT              1

#define ICM42688_STATUS_WALK 1
#define ICM42688_STATUS_RUN 2


#define ERR_OK             0      ///< No error
#define ERR_DATA_BUS      -1      ///< Data bus error
#define ERR_IC_VERSION    -2      ///< The chip version not match

#define GYRO     0
#define ACCEL    1
#define ALL      5



#define TMST_DEFAULT_CONFIG_START  0x23
#define TMST_VALUE_DIS             0<<4
#define TMST_VALUE_EN              1<<4
#define TMST_RES_EN_DIS            0<<3 
#define TMST_RES_EN                1<<3
#define TMST_FSYNC_EN              1<<1
#define TMST_FSYNC_DIS             0<<1
#define TMST_DELTA_EN              0<<2
#define TMST_DELTA_DIS             1<<2
#define TMST_EN                    1
#define TMST_DIS                   0

#define X_AXIS   0
#define Y_AXIS   2
#define Z_AXIS   4

#define X_AXIS_WOM   1
#define Y_AXIS_WOM   2
#define Z_AXIS_WOM   4

#define ODR_32KHZ         1
#define ODR_16KHZ         2
#define ODR_8KHZ          3
#define ODR_4KHZ          4
#define ODR_2KHZ          5
#define ODR_1KHZ          6
#define ODR_200HZ         7
#define ODR_100HZ         8
#define ODR_50HZ          9
#define ODR_25KHZ         10
#define ODR_12_5KHZ       11
#define ODR_6_25KHZ       12
#define ODR_3_125HZ       13
#define ODR_1_5625HZ      14
#define ODR_500HZ         15

#define FSR_0             0
#define FSR_1             1
#define FSR_2             2
#define FSR_3             3
#define FSR_4             4
#define FSR_5             5
#define FSR_6             6
#define FSR_7             7

#define LP_MODE_ONLY_ACCEL  2
#define LN_MODE  3
#define STANDBY_MODE_ONLY_GYRO 1 
#define OFF_MODE   0

#define TAP_SINGLE 8
#define TAP_DOUBLE 16




int ICM42688_Readme(void);
void getHEXAccelData(uint8_t where, uint8_t *value, size_t size );
void getHEXGyroData(uint8_t where, uint8_t *value, size_t size );
void samepleGetTest(FLOAT_XYZ ACCFloat,FLOAT_XYZ GRYOFloat);
int16_t getInt16AccelDataX(void);
int16_t getInt16AccelDataY(void);
int16_t getInt16AccelDataZ(void);
int16_t getInt16GyroDataX(void);
int16_t getInt16GyroDataY(void);
int16_t getInt16GyroDataZ(void);

void enableGETDATAInterrupt();
/**
*@author       JCLStill
*@brief         Des
*@param         range AD×ª»»
*@return        void
*/
float ADchange(int value,float range);


/**
* @fn getTemperature
* @brief Get measured temperature
* @return Temperature value, unit: ?
*/
float getTemperature(void);

/**
* @fn getAccelDataX
* @brief Get X-axis accelerometer value
* @return X-axis accelerometer value, unit: mg
*/
float getAccelDataX(void);

/**
* @fn getAccelDataY
* @brief Get Y-axis accelerometer value
* @return Y-axis accelerometer value, unit: mg
*/
float getAccelDataY(void);

/**
* @fn getAccelDataZ
* @brief Get Z-axis accelerometer value
* @return Z-axis accelerometer value, unit: mg
*/
float getAccelDataZ(void);

/**
* @fn getGyroDataX
* @brief Get X-axis gyroscope value
* @return X-axis gyroscope value, unit: dps
*/
float getGyroDataX(void);

/**
* @fn getGyroDataY
* @brief Get Y-axis gyroscope value
* @return Y-axis gyroscope value, unit: dps
*/
float getGyroDataY(void);

/**
* @fn getGyroDataZ
* @brief Get Z-axis gyroscope value
* @return Z-axis gyroscope value, unit: dps
*/
float getGyroDataZ(void);

/**
* @fn tapDetectionInit
* @brief Tap detection init
* @param accelMode Accelerometer operating mode
* @n      0 for operating in low-power mode
* @n      1 for operating in low-noise mode
*/
void tapDetectionInit(uint8_t accelMode);

/**
* @fn getTapInformation
* @brief Get tap information
*/
void getTapInformation();

/**
* @fn numberOfTap
* @brief Get the number of tap: single-tap or double tap
* @return The number of tap
* @retval TAP_SINGLE   Single-tap
* @retval TAP_DOUBLE   Double tap
*/
uint8_t numberOfTap();

/**
* @fn axisOfTap
* @brief Get the axis on which the tap occurred: X-axis, Y-axis, or Z-axis
* @return Tap axis
* @retval X_AXIS   X-axis
* @retval Y_AXIS   Y-axis
* @retval Z_AXIS   Z-axis
*/
uint8_t axisOfTap();

/**
* @fn wakeOnMotionInit
* @brief Wake on motion init
*/
void wakeOnMotionInit();

/**
* @fn setWOMTh
* @brief Set wake on motion interrupt threshold of axis accelerometer
* @param axis  x/y/z?
* @n       X_AXIS_WOM
* @n       Y_AXIS_WOM
* @n       Z_AXIS_WOM
* @n       ALL
* @param threshold  Range(0-255) [WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg]
*/
void setWOMTh(uint8_t axis,uint8_t threshold);

/**
* @fn setWOMInterrupt
* @brief Enable wake on motion interrupt
* @param axis  X-axis, Y-axis, or Z-axis
* @n       X_AXIS_WOM
* @n       Y_AXIS_WOM
* @n       Z_AXIS_WOM
*/
void setWOMInterrupt(uint8_t axis);

/**
* @fn enableSMDInterrupt
* @brief Set important motion detection mode and enable SMD interrupt
* @param mode  
* @n      0: disable SMD
* @n      2 : SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1 sec apart
* @n      3 : SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec apart
*/
void enableSMDInterrupt(uint8_t mode);

/**
* @fn readInterruptStatus
* @brief Read interrupt information and clear interrupt
* @param reg Interrupt information register
* @n      ICM42688_INT_STATUS2    Obtain interrupt information of SMD_INT, WOM_X_INT, WOM_Y_INT, WOM_Z_INT and clear them
* @n      ICM42688_INT_STATUS3    Obtain interrupt information of STEP_DET_INT, STEP_CNT_OVF_INT, TILT_DET_INT, WAKE_INT, TAP_DET_INT and clear them
* @return Interrupt information, return 0 when no interrupt.
*/
uint8_t readInterruptStatus(uint8_t reg);

/**
* @fn setODRAndFSR
* @brief Set ODR and Full-scale range of gyroscope or accelerometer.
* @param who  GYRO/ACCEL/ALL
* @n       GYRO: indicate only set gyroscope
* @n       ACCEL: indicate only set accelerometer
* @param ODR Output data rate
* @n       ODR_32KHZ         Support: Gyro/Accel(LN mode)
* @n       ODR_16KHZ         Support: Gyro/Accel(LN mode)
* @n       ODR_8KHZ          Support: Gyro/Accel(LN mode)
* @n       ODR_4KHZ          Support: Gyro/Accel(LN mode)
* @n       ODR_2KHZ          Support: Gyro/Accel(LN mode)
* @n       ODR_1KHZ          Support: Gyro/Accel(LN mode)
* @n       ODR_200HZ         Support: Gyro/Accel(LP or LN mode)
* @n       ODR_100HZ         Support: Gyro/Accel(LP or LN mode)
* @n       ODR_50HZ          Support: Gyro/Accel(LP or LN mode)
* @n       ODR_25KHZ         Support: Gyro/Accel(LP or LN mode)
* @n       ODR_12_5KHZ       Support: Gyro/Accel(LP or LN mode)
* @n       ODR_6_25KHZ       Support: Accel(LP mode)
* @n       ODR_3_125HZ       Support: Accel(LP mode)
* @n       ODR_1_5625HZ      Support: Accel(LP mode)
* @n       ODR_500HZ         Support: Accel(LP or LN mode)
* @param FSR Full-scale range
* @n       FSR_0      Gyro:±2000dps   /   Accel: ±16g
* @n       FSR_1      Gyro:±1000dps   /   Accel: ±8g
* @n       FSR_2      Gyro:±500dps    /   Accel: ±4g
* @n       FSR_3      Gyro:±250dps    /   Accel: ±2g
* @n       FSR_4      Gyro:±125dps    /   Accel: not optional
* @n       FSR_5      Gyro:±62.5dps   /   Accel: not optional
* @n       FSR_6      Gyro:±31.25dps  /   Accel: not optional
* @n       FSR_7      Gyro:±15.625dps /   Accel: not optional
* @return Set result
* @retval true   indicate the setting succeeds
* @retval flase  indicate selected parameter is wrong
*/
bool setODRAndFSR(uint8_t who,uint8_t ODR,uint8_t FSR);

/**
* @fn startFIFOMode
* @brief Enable FIFO
*/
void startFIFOMode();

/**
* @fn sotpFIFOMode
* @brief Disable FIFO
*/
void sotpFIFOMode();

/**
* @fn getFIFOData
* @brief Read FIFO data, read temperature, gyroscope and accelerometer data and save them for parse.
*/
void getFIFOData();

/**
* @fn setINTMode
* @brief Set interrupt mode
* @param INTPin  Interrupt pin 
* @n       1  Use INT1 interrupt pin
* @n       2  Use INT2 interrupt pin
* @param INTmode Set interrupt mode
* @n       1  Interrupt lock mode (polarity remains unchanged when interrupt triggered, and restore after clearing interrupt)
* @n       0  Pulse mode
* @param INTPolarity Level polarity output by interrupt
* @n       0  Interrupt pin polarity is LOW when producing interrupt
* @n       1  Interrupt pin polarity is HIGH when producing interrupt
* @param INTDriveCircuit  
* @n       0  Open drain
* @n       1  Push pull
*/
void setINTMode(uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit);

/**
* @fn startGyroMeasure
* @brief Start gyroscope
* @param mode Set gyroscope working mode
* @n       STANDBY_MODE_ONLY_GYRO 1  Set stanby mode, only support gyroscope
* @n       LN_MODE  3                Set low-noise mode
*/
void startGyroMeasure(uint8_t mode);

/**
* @fn startAccelMeasure
* @brief Start accelerometer
* @param mode Set accelerometer working mode
* @n       LP_MODE_ONLY_ACCEL  2     Set low-power mode, only support accelerometer
* @n       LN_MODE  3                Set low-noise mode
*/
void startAccelMeasure(uint8_t mode);

/**
* @fn startTempMeasure
* @brief Start thermometer
*/
void startTempMeasure();



/**
* @fn setFIFODataMode
* @brief Set FIFO data packet format
*/
void setFIFODataMode();

///**
//* @fn writeReg
//* @brief Write register function, design it as a virtual function, implemented by a derived class.
//* @param reg  Register address 8bits
//* @param pBuf Storage and buffer for data to be written
//* @param size Length of data to be written
//*/
//void writeReg(uint8_t reg, void* pBuf, size_t size) ;

///**
//* @fn readReg
//* @brief Read register function, design it as a virtual function, implemented by a derived class.
//* @param reg  Register address 8bits
//* @param pBuf Read data storage and buffer
//* @param size Read data length
//* @return return the read length, returning 0 means length reading failed
//*/
//uint8_t readReg(uint8_t reg, void* pBuf, size_t size) ;


/**
* @fn setGyroNotchFilterFHz
* @brief Set gyroscope notch filter frequency
* @param freq  1000HZ-3000HZ
* @param axis  
* @n       X_AXIS
* @n       Y_AXIS
* @n       Z_AXIS
* @n       ALL
*/
void setGyroNotchFilterFHz(double freq,uint8_t axis);

/**
* @fn setGyroNFbandwidth
* @brief Set gyroscope notch filter bandwidth
* @param freq  1000HZ-3000HZ
* @param axis  
* @n       X_AXIS
* @n       Y_AXIS
* @n       Z_AXIS
* @n       ALL
*/
void setGyroNFbandwidth(uint8_t bw);

/**
* @fn setGyroNotchFilter
* @brief Switch of gyroscope notch filter
* @param mode  
* @n       true  Enable
* @n       false Disable
*/
void setGyroNotchFilter(bool mode);

/**
* @fn setAAFBandwidth
* @brief Set AAF bandwidth of gyroscope or accelerometer
* @param who  
* @n       GYRO
* @n       ACCEL
* @n       ALL
* @param BWIndex Bandwidth index
* @n              uint16_t AAFHZ[63] = {42,84,126,170,213,258,303,348,394,441,488,536,585,634,684,734,785,837,890,943,
* @n                                    997,1051,1107,1163,1220,1277,1336,1395,1454,1515,1577,1639,1702,1766,1830,1896,
* @n                                    1962,2029,2097,2166,2235,2306,2377,2449,2522,2596,2671,2746,2823,2900,2978,3057,
* @n                                    3137,3217,3299,3381,3464,3548,3633,3718,3805,3892,3979};
*/
void setAAFBandwidth(uint8_t who,uint8_t BWIndex);

/**
* @fn setAAF
* @brief Set AAF switch of gyroscope or accelerometer
* @param who  
* @n       GYRO
* @n       ACCEL
* @n       ALL
* @param mode  
* @n       true  Enable
* @n       false Disable
*/
void setAAF(uint8_t who,bool mode);

/**
* @fn setUIFilter
* @brief Set UI filter of gyroscope or accelerometer
* @param who  GYRO\ACCEL\ALL
* @param filterOrder  (range:1-3) Selects order of UI filter
* @param UIFilterIndex (range:0-15) Bandwidth for GYRO\ACCEL LPF
* @return Set result
* @retval true   indicate the setting succeed
* @retval flase  indicate selected parameter is wrong
*/
bool setUIFilter(uint8_t who,uint8_t filterOrder ,uint8_t UIFilterIndex);



/**
* @fn writeReg
* @brief Write register values through I2C bus
* @param reg  Register address 8bits
* @param pBuf Storage and buffer for data to be written
* @param size Length of data to be written
* @return Return the read length, returning 0 means length reading failed
*/
void writeReg(uint8_t reg, void* pBuf, size_t size);

/**
* @fn readReg
* @brief Read register values through I2C bus
* @param reg  Register address 8bits
* @param pBuf Storage and buffer for data to be read
* @param size Length of data to be read
* @return Return the read length, returning 0 means length reading failed
*/
uint8_t readReg(uint8_t reg, void* pBuf, size_t size);



/**
* @fn startFIFOMode
* @brief Enable FIFO
*/
void startFIFOMode();

/**
* @fn sotpFIFOMode
* @brief Disable FIFO
*/
void sotpFIFOMode();

/**
* @fn getFIFOData
* @brief Read FIFO data, read temperature, gyroscope and accelerometer data and save them for parse.
*/
void getFIFOData();



/**
* @fn setINTMode
* @brief Set interrupt mode
* @param INTPin  Interrupt pin 
* @n       1  Use INT1 interrupt pin
* @n       2  Use INT2 interrupt pin
* @param INTmode Set interrupt mode
* @n       1  Interrupt lock mode (polarity remains unchanged when interrupt triggered, and restore after clearing interrupt)
* @n       0  Pulse mode
* @param INTPolarity Level polarity output by interrupt
* @n       0  Interrupt pin polarity is LOW when producing interrupt
* @n       1  Interrupt pin polarity is HIGH when producing interrupt
* @param INTDriveCircuit  
* @n       0  Open drain
* @n       1  Push pull
*/
void setINTMode(uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit);

/**
* @fn readInterruptStatus
* @brief Read interrupt information and clear interrupt
* @param reg Interrupt information register
* @n      ICM42688_INT_STATUS2    Obtain interrupt information of SMD_INT, WOM_X_INT, WOM_Y_INT, WOM_Z_INT and clear them
* @n      ICM42688_INT_STATUS3    Obtain interrupt information of STEP_DET_INT, STEP_CNT_OVF_INT, TILT_DET_INT, WAKE_INT, TAP_DET_INT and clear them
* @return Interrupt information, return 0 when no interrupt.
*/
uint8_t readInterruptStatus(uint8_t reg);

/**
* @fn tapDetectionInit
* @brief Tap detection init
* @param accelMode Accelerometer operating mode
* @n      0 for operating in low-power mode
* @n      1 for operating in low-noise mode
*/
void tapDetectionInit(uint8_t accelMode);

/**
* @fn getTapInformation
* @brief Get tap information
*/
void getTapInformation();

/**
* @fn numberOfTap
* @brief Get the number of tap: single-tap or double tap
* @return The number of tap
* @retval TAP_SINGLE   Single-tap
* @retval TAP_DOUBLE   Double tap
*/
uint8_t numberOfTap();

/**
* @fn axisOfTap
* @brief Get the axis on which the tap occurred: X-axis, Y-axis, or Z-axis
* @return Tap axis
* @retval X_AXIS   X-axis
* @retval Y_AXIS   Y-axis
* @retval Z_AXIS   Z-axis
*/
uint8_t axisOfTap();


/**
* @fn wakeOnMotionInit
* @brief Wake on motion init
*/
void wakeOnMotionInit();

/**
* @fn setWOMTh
* @brief Set wake on motion interrupt threshold of axis accelerometer
* @param axis  x/y/z?
* @n       X_AXIS_WOM
* @n       Y_AXIS_WOM
* @n       Z_AXIS_WOM
* @n       ALL
* @param threshold  Range(0-255) [WoM thresholds are expressed in fixed “mg” independent of the selected Range [0g : 1g]; Resolution 1g/256=~3.9mg]
*/
void setWOMTh(uint8_t axis,uint8_t threshold);

/**
* @fn setWOMInterrupt
* @brief Enable wake on motion interrupt
* @param axis  X-axis, Y-axis, or Z-axis
* @n       X_AXIS_WOM
* @n       Y_AXIS_WOM
* @n       Z_AXIS_WOM
*/
void setWOMInterrupt(uint8_t axis);
/**
* @fn enableSMDInterrupt
* @brief Set important motion detection mode and enable SMD interrupt
* @param mode  
* @n      0: disable SMD
* @n      2 : SMD short (1 sec wait) An SMD event is detected when two WOM are detected 1 sec apart
* @n      3 : SMD long (3 sec wait) An SMD event is detected when two WOM are detected 3 sec apart
*/
void enableSMDInterrupt(uint8_t mode);


void tiltDetectionInit();
void setTiltDetection();
void enableTILEInterrupt();

#endif
