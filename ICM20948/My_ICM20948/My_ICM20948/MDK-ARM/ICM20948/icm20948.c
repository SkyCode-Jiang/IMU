#include "icm20948.h"

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;
uint8_t currentBank;
uint8_t icm_buffer[20];
xyzFloat accOffsetVal;
xyzFloat accCorrFactor;
xyzFloat gyrOffsetVal;
uint8_t accRangeFactor;
uint8_t gyrRangeFactor;
uint8_t regVal;   // intermediate storage of register values
ICM20948_fifoType fifoType;

/************ Basic Settings ************/
void delay(uint16_t mesc)
{
	HAL_Delay(mesc);
}
	

uint8_t init(void){ 

			currentBank = 0;
    
    
    if(!reset_ICM20948()){
        return false;
    }

    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    accCorrFactor.x = 1.0;
    accCorrFactor.y = 1.0;
    accCorrFactor.z = 1.0;
    accRangeFactor = 1.0;
    gyrOffsetVal.x = 0.0;
    gyrOffsetVal.y = 0.0;
    gyrOffsetVal.z = 0.0;
    gyrRangeFactor = 1.0;
    fifoType = ICM20948_FIFO_ACC;
    
    icmsleep(false);
    writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR 
    
    return true;
}

void autoOffsets(void){
    xyzFloat accRawVal, gyrRawVal;
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    
    setGyrDLPF(ICM20948_DLPF_6); // lowest noise
    setGyrRange(ICM20948_GYRO_RANGE_250); // highest resolution
    setAccRange(ICM20948_ACC_RANGE_2G);
    setAccDLPF(ICM20948_DLPF_6);
    delay(100);
    
    for(int i=0; i<50; i++){
        readSensor();
        accRawVal = getAccRawValues();
        accOffsetVal.x += accRawVal.x;
        accOffsetVal.y += accRawVal.y;
        accOffsetVal.z += accRawVal.z;
        delay(10);
    }
    
    accOffsetVal.x /= 50;
    accOffsetVal.y /= 50;
    accOffsetVal.z /= 50;
    accOffsetVal.z -= 16384.0;
    
    for(int i=0; i<50; i++){
        readSensor();
        gyrRawVal = getGyrRawValues();
        gyrOffsetVal.x += gyrRawVal.x;
        gyrOffsetVal.y += gyrRawVal.y;
        gyrOffsetVal.z += gyrRawVal.z;
        delay(1);
    }
    
    gyrOffsetVal.x /= 50;
    gyrOffsetVal.y /= 50;
    gyrOffsetVal.z /= 50;
    
}

void setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    accOffsetVal.x = (xMax + xMin) * 0.5;
    accOffsetVal.y = (yMax + yMin) * 0.5;
    accOffsetVal.z = (zMax + zMin) * 0.5;
    accCorrFactor.x = (xMax + abs(xMin)) / 32768.0;
    accCorrFactor.y = (yMax + abs(yMin)) / 32768.0;
    accCorrFactor.z = (zMax + abs(zMin)) / 32768.0 ;    
}

void setGyrOffsets(float xOffset, float yOffset, float zOffset){
    gyrOffsetVal.x = xOffset;
    gyrOffsetVal.y = yOffset;
    gyrOffsetVal.z = zOffset;
}

uint8_t whoAmI(void){
    return readRegister8(0, ICM20948_WHO_AM_I);
}

uint8_t enableRes;
void enableAcc(uint8_t enAcc){
    regVal = readRegister8( 0, ICM20948_PWR_MGMT_2);
    if(enAcc){
        regVal &= ~ICM20948_ACC_EN;
    }
    else{
        regVal |= ICM20948_ACC_EN;
    }
    writeRegister8( 0, ICM20948_PWR_MGMT_2, regVal);
		
}

void setAccRange(ICM20948_accRange accRange){
    regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
    regVal &= ~(0x06);
    regVal |= (accRange<<1);
    writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
    accRangeFactor = 1<<accRange;
}

void setAccDLPF(ICM20948_dlpf dlpf){
    regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
    if(dlpf==ICM20948_DLPF_OFF){
        regVal &= 0xFE;
        writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
        return;
    }
    else{
        regVal |= 0x01;
        regVal &= 0xC7;
        regVal |= (dlpf<<3);
    }
    writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
}   

void setAccSampleRateDivider(uint16_t accSplRateDiv){
    writeRegister16(2, ICM20948_ACCEL_SMPLRT_DIV_1, accSplRateDiv);
}

void enableGyr(uint8_t enGyr){
    regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
    if(enGyr){
        regVal &= ~ICM20948_GYR_EN;
    }
    else{
        regVal |= ICM20948_GYR_EN;
    }
    writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

void setGyrRange(ICM20948_gyroRange gyroRange){
    regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);
    regVal &= ~(0x06);
    regVal |= (gyroRange<<1);
    writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
    gyrRangeFactor = (1<<gyroRange);
}

void setGyrDLPF(ICM20948_dlpf dlpf){
    regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);
    if(dlpf==ICM20948_DLPF_OFF){
        regVal &= 0xFE;
        writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
        return;
    }
    else{
        regVal |= 0x01;
        regVal &= 0xC7;
        regVal |= (dlpf<<3);
    }
    writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
}   

void setGyrSampleRateDivider(uint8_t gyrSplRateDiv){
    writeRegister8(2, ICM20948_GYRO_SMPLRT_DIV, gyrSplRateDiv);
}

void setTempDLPF(ICM20948_dlpf dlpf){
    writeRegister8(2, ICM20948_TEMP_CONFIG, dlpf);
}

void setI2CMstSampleRate(uint8_t rateExp){
    if(rateExp < 16){
        writeRegister8(3, ICM20948_I2C_MST_ODR_CFG, rateExp);
    }
}
    
/************* x,y,z results *************/
        
void readSensor(){
    readAllData(icm_buffer);
}

xyzFloat getAccRawValues(){
    xyzFloat accRawVal;
    accRawVal.x = (int16_t)(((icm_buffer[0]) << 8) | icm_buffer[1]) * 1.0;
    accRawVal.y = (int16_t)(((icm_buffer[2]) << 8) | (int16_t)icm_buffer[3]) * 1.0;
    accRawVal.z = (int16_t)(((icm_buffer[4]) << 8) | icm_buffer[5]) * 1.0;
    return accRawVal;
}

xyzFloat getCorrectedAccRawValues(){
    xyzFloat accRawVal = getAccRawValues();   
    accRawVal = correctAccRawValues(accRawVal);
    
    return accRawVal;
}

xyzFloat getGValues(){
    xyzFloat gVal, accRawVal;
    accRawVal = getCorrectedAccRawValues();
    
    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

xyzFloat getAccRawValuesFromFifo(){
    xyzFloat accRawVal = readICM20948xyzValFromFifo();
    return accRawVal;   
}

xyzFloat getCorrectedAccRawValuesFromFifo(){
    xyzFloat accRawVal = getAccRawValuesFromFifo();
    accRawVal = correctAccRawValues(accRawVal);
    
    return accRawVal;
}

xyzFloat getGValuesFromFifo(){
    xyzFloat gVal, accRawVal;
    accRawVal = getCorrectedAccRawValuesFromFifo();
    
    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

float getResultantG(xyzFloat gVal){
    float resultant = 0.0;
    resultant = sqrt((gVal.x * gVal.x) + (gVal.y *gVal.y) + (gVal.z * gVal.z));
    
    return resultant;
}

float getTemperature(){
    int16_t rawTemp = (int16_t)(((icm_buffer[12]) << 8) | icm_buffer[13]);
    float tmp = (rawTemp*1.0 - ICM20948_ROOM_TEMP_OFFSET)/ICM20948_T_SENSITIVITY + 21.0;
    return tmp;
}

xyzFloat getGyrRawValues(){
    xyzFloat gyrRawVal;
    
    gyrRawVal.x = (int16_t)(((icm_buffer[6]) << 8) | icm_buffer[7]) * 1.0;
    gyrRawVal.y = (int16_t)(((icm_buffer[8]) << 8) | icm_buffer[9]) * 1.0;
    gyrRawVal.z = (int16_t)(((icm_buffer[10]) << 8) | icm_buffer[11]) * 1.0;
    
    return gyrRawVal;
}

xyzFloat getCorrectedGyrRawValues(){
    xyzFloat gyrRawVal = getGyrRawValues(); 
    gyrRawVal = correctGyrRawValues(gyrRawVal);
    return gyrRawVal;
}

xyzFloat getGyrValues(){
    xyzFloat gyrVal = getCorrectedGyrRawValues();
    
    gyrVal.x = gyrVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrVal.z * gyrRangeFactor * 250.0 / 32768.0;
     
    return gyrVal;
}

xyzFloat getGyrValuesFromFifo(){
    xyzFloat gyrVal;
    xyzFloat gyrRawVal = readICM20948xyzValFromFifo();
    
    gyrRawVal = correctGyrRawValues(gyrRawVal);
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;
    
    return gyrVal;  
}

xyzFloat getMagValues(){
    int16_t x,y,z;
    xyzFloat mag;
    
    x = (int16_t)((icm_buffer[15]) << 8) | icm_buffer[14];
    y = (int16_t)((icm_buffer[17]) << 8) | icm_buffer[16];
    z = (int16_t)((icm_buffer[19]) << 8) | icm_buffer[18];
    
    mag.x = x * AK09916_MAG_LSB;
    mag.y = y * AK09916_MAG_LSB;
    mag.z = z * AK09916_MAG_LSB;
    
    return mag;
}


/********* Power, Sleep, Standby *********/ 

void enableCycle(ICM20948_cycle cycle){
    regVal = readRegister8(0, ICM20948_LP_CONFIG);
    regVal &= 0x0F;
    regVal |= cycle;
    
    writeRegister8(0, ICM20948_LP_CONFIG, regVal);
}

void enableLowPower(uint8_t enLP){    // vielleicht besser privat????
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    if(enLP){
        regVal |= ICM20948_LP_EN;
    }
    else{
        regVal &= ~ICM20948_LP_EN;
    }
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

void setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg){
    writeRegister8(2, ICM20948_GYRO_CONFIG_2, avg);
}

void setAccAverageInCycleMode(ICM20948_accAvgLowPower avg){
    writeRegister8(2, ICM20948_ACCEL_CONFIG_2, avg);
}

void icmsleep(uint8_t sleep){
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    if(sleep){
        regVal |= ICM20948_SLEEP;
    }
    else{
        regVal &= ~ICM20948_SLEEP;
    }
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}
        
/******** Angles and Orientation *********/ 
    
xyzFloat getAngles(){
    xyzFloat angleVal;
    xyzFloat gVal = getGValues();
    if(gVal.x > 1.0){
        gVal.x = 1.0;
    }
    else if(gVal.x < -1.0){
        gVal.x = -1.0;
    }
    angleVal.x = (asin(gVal.x)) * 57.296;
    
    if(gVal.y > 1.0){
        gVal.y = 1.0;
    }
    else if(gVal.y < -1.0){
        gVal.y = -1.0;
    }
    angleVal.y = (asin(gVal.y)) * 57.296;
    
    if(gVal.z > 1.0){
        gVal.z = 1.0;
    }
    else if(gVal.z < -1.0){
        gVal.z = -1.0;
    }
    angleVal.z = (asin(gVal.z)) * 57.296;
    
    return angleVal;
}

ICM20948_orientation getOrientation(){
    xyzFloat angleVal = getAngles();
    ICM20948_orientation orientation = ICM20948_FLAT;
    if(abs(angleVal.x) < 45){      // |x| < 45
        if(abs(angleVal.y) < 45){      // |y| < 45
            if(angleVal.z > 0){          //  z  > 0
                orientation = ICM20948_FLAT;
            }
            else{                        //  z  < 0
                orientation = ICM20948_FLAT_1;
            }
        }
        else{                         // |y| > 45 
            if(angleVal.y > 0){         //  y  > 0
                orientation = ICM20948_XY;
            }
            else{                       //  y  < 0
                orientation = ICM20948_XY_1;   
            }
        }
    }
    else{                           // |x| >= 45
        if(angleVal.x > 0){           //  x  >  0
            orientation = ICM20948_YX;       
        }
        else{                       //  x  <  0
            orientation = ICM20948_YX_1;
        }
    }
    return orientation;
}

char* getOrientationAsString(){
    ICM20948_orientation orientation = getOrientation();
	  char text[10]  = "123";
    char* orientationAsString = text;
		
    switch(orientation){
        case ICM20948_FLAT:      memcpy(orientationAsString,"z up",10);  break;
        case ICM20948_FLAT_1:    memcpy(orientationAsString,"z down",10); break;
        case ICM20948_XY:        memcpy(orientationAsString,"y up",10);  break;
        case ICM20948_XY_1:      memcpy(orientationAsString,"y down",10);; break;
        case ICM20948_YX:        memcpy(orientationAsString,"x up",10);  break;
        case ICM20948_YX_1:      memcpy(orientationAsString,"x down",10); break;
    }
    return orientationAsString;
}
    
float getPitch(){
    xyzFloat angleVal = getAngles();
    float pitch = (atan2(angleVal.x, sqrt(abs((angleVal.x*angleVal.y + angleVal.z*angleVal.z))))*180.0)/M_PI;
    return pitch;
}
    
float getRoll(){
    xyzFloat angleVal = getAngles();
    float roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
    return roll;
}


/************** Interrupts ***************/

void setIntPinPolarity(ICM20948_intPinPol pol){
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    if(pol){
        regVal |= ICM20948_INT1_ACTL;
    }
    else{
        regVal &= ~ICM20948_INT1_ACTL;
    }
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void enableIntLatch(uint8_t latch){
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    if(latch){
        regVal |= ICM20948_INT_1_LATCH_EN;
    }
    else{
        regVal &= ~ICM20948_INT_1_LATCH_EN;
    }
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void enableClearIntByAnyRead(uint8_t clearByAnyRead){
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    if(clearByAnyRead){
        regVal |= ICM20948_INT_ANYRD_2CLEAR;
    }
    else{
        regVal &= ~ICM20948_INT_ANYRD_2CLEAR;
    }
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void setFSyncIntPolarity(ICM20948_intPinPol pol){
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    if(pol){
        regVal |= ICM20948_ACTL_FSYNC;
    }
    else{
        regVal &= ~ICM20948_ACTL_FSYNC;
    }
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void enableInterrupt(ICM20948_intType intType){
    switch(intType){
        case ICM20948_FSYNC_INT:
            regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
            regVal |= ICM20948_FSYNC_INT_MODE_EN;
            writeRegister8(0, ICM20948_INT_PIN_CFG, regVal); // enable FSYNC as interrupt pin
            regVal = readRegister8(0, ICM20948_INT_ENABLE);
            regVal |= 0x80;
            writeRegister8(0, ICM20948_INT_ENABLE, regVal); // enable wake on FSYNC interrupt
            break;
        
        case ICM20948_WOM_INT:
            regVal = readRegister8(0, ICM20948_INT_ENABLE);
            regVal |= 0x08;
            writeRegister8(0, ICM20948_INT_ENABLE, regVal);
            regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
            regVal |= 0x02;
            writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
            break;
        
        case ICM20948_DMP_INT:
            regVal = readRegister8(0, ICM20948_INT_ENABLE);
            regVal |= 0x02;
            writeRegister8(0, ICM20948_INT_ENABLE, regVal);
            break;
        
        case ICM20948_DATA_READY_INT:
            writeRegister8(0, ICM20948_INT_ENABLE_1, 0x01);
            break;
        
        case ICM20948_FIFO_OVF_INT:
            writeRegister8(0, ICM20948_INT_ENABLE_2, 0x01);
            break;
        
        case ICM20948_FIFO_WM_INT:
            writeRegister8(0, ICM20948_INT_ENABLE_3, 0x01);
            break;
    }
}

void disableInterrupt(ICM20948_intType intType){
    switch(intType){
        case ICM20948_FSYNC_INT:
            regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
            regVal &= ~ICM20948_FSYNC_INT_MODE_EN; 
            writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
            regVal = readRegister8(0, ICM20948_INT_ENABLE);
            regVal &= ~(0x80);
            writeRegister8(0, ICM20948_INT_ENABLE, regVal);
            break;
        
        case ICM20948_WOM_INT:
            regVal = readRegister8(0, ICM20948_INT_ENABLE);
            regVal &= ~(0x08);
            writeRegister8(0, ICM20948_INT_ENABLE, regVal);
            regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
            regVal &= ~(0x02);
            writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
            break;
        
        case ICM20948_DMP_INT:
            regVal = readRegister8(0, ICM20948_INT_ENABLE);
            regVal &= ~(0x02);
            writeRegister8(0, ICM20948_INT_ENABLE, regVal);
            break;
        
        case ICM20948_DATA_READY_INT:
            writeRegister8(0, ICM20948_INT_ENABLE_1, 0x00);
            break;
        
        case ICM20948_FIFO_OVF_INT:
            writeRegister8(0, ICM20948_INT_ENABLE_2, 0x00);
            break;
        
        case ICM20948_FIFO_WM_INT:
            writeRegister8(0, ICM20948_INT_ENABLE_3, 0x00);
            break;
    }
}

uint8_t readAndClearInterrupts(){
    uint8_t intSource = 0;
    regVal = readRegister8(0, ICM20948_I2C_MST_STATUS);
    if(regVal & 0x80){
        intSource |= 0x01;
    }
    regVal = readRegister8(0, ICM20948_INT_STATUS);
    if(regVal & 0x08){
        intSource |= 0x02;
    } 
    if(regVal & 0x02){
        intSource |= 0x04;
    } 
    regVal = readRegister8(0, ICM20948_INT_STATUS_1);
    if(regVal & 0x01){
        intSource |= 0x08;
    } 
    regVal = readRegister8(0, ICM20948_INT_STATUS_2);
    if(regVal & 0x01){
        intSource |= 0x10;
    }
    regVal = readRegister8(0, ICM20948_INT_STATUS_3);
    if(regVal & 0x01){
        intSource |= 0x20;
    }
    return intSource;
}

uint8_t checkInterrupt(uint8_t source, ICM20948_intType type){
    source &= type;
    return source;
}
void setWakeOnMotionThreshold(uint8_t womThresh, ICM20948_womCompEn womCompEn){
    regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
    if(womCompEn){
        regVal |= 0x01;
    }
    else{
        regVal &= ~(0x01);
    }
    writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
    writeRegister8(2, ICM20948_ACCEL_WOM_THR, womThresh);   
}

/***************** FIFO ******************/

void enableFifo(uint8_t fifo){
    regVal = readRegister8(0, ICM20948_USER_CTRL);
    if(fifo){
        regVal |= ICM20948_FIFO_EN;
    }
    else{
        regVal &= ~ICM20948_FIFO_EN;
    }
    writeRegister8(0, ICM20948_USER_CTRL, regVal);
}

void setFifoMode(ICM20948_fifoMode mode){
    if(mode){
        regVal = 0x01;
    }
    else{
        regVal = 0x00;
    }
    writeRegister8(0, ICM20948_FIFO_MODE, regVal);
}

void startFifo(ICM20948_fifoType fifo){
    fifoType = fifo;
    writeRegister8(0, ICM20948_FIFO_EN_2, fifoType);
}

void stopFifo(){
    writeRegister8(0, ICM20948_FIFO_EN_2, 0);
}

void resetFifo(){
    writeRegister8(0, ICM20948_FIFO_RST, 0x01);
    writeRegister8(0, ICM20948_FIFO_RST, 0x00);
}

int16_t getFifoCount(){
    int16_t regVal16 = (int16_t) readRegister16(0, ICM20948_FIFO_COUNT);
    return regVal16;
}

int16_t getNumberOfFifoDataSets(){
    int16_t numberOfSets = getFifoCount();
        
    if((fifoType == ICM20948_FIFO_ACC) || (fifoType == ICM20948_FIFO_GYR)){
        numberOfSets /= 6;
    }
    else if(fifoType==ICM20948_FIFO_ACC_GYR){
        numberOfSets /= 12;
    }
    
    return numberOfSets;
}

void findFifoBegin(){
    int16_t count = getFifoCount();
    int16_t start = 0;
        
    if((fifoType == ICM20948_FIFO_ACC) || (fifoType == ICM20948_FIFO_GYR)){
        start = count%6;
        for(int i=0; i<start; i++){
            readRegister8(0, ICM20948_FIFO_R_W);
        }
    }
    else if(fifoType==ICM20948_FIFO_ACC_GYR){
        start = count%12;
        for(int i=0; i<start; i++){
            readRegister8(0, ICM20948_FIFO_R_W);
        }
    }
}


/************** Magnetometer **************/

uint8_t initMagnetometer(){
    enableI2CMaster();
    resetMag();
    reset_ICM20948();
    icmsleep(false);
    writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR 
    enableI2CMaster();
    
    
    if(!(whoAmIMag() == AK09916_WHO_AM_I)){
        return false;
    }
    
    setMagOpMode(AK09916_CONT_MODE_100HZ); 
   
    return true;
}

int16_t whoAmIMag(){
    return readAK09916Register16(AK09916_WIA_1);
}

void setMagOpMode(AK09916_opMode opMode){
    writeAK09916Register8(AK09916_CNTL_2, opMode);
    delay(10);
    if(opMode!=AK09916_PWR_DOWN){
        enableMagDataRead(AK09916_HXL, 0x08);
    }
}

void resetMag(){
    writeAK09916Register8(AK09916_CNTL_3, 0x01);
    delay(100);
}


/************************************************ 
     Private Functions
*************************************************/

void setClockToAutoSelect(){
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    regVal |= 0x01;
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
    delay(10);
}

xyzFloat correctAccRawValues(xyzFloat accRawVal){
    accRawVal.x = (accRawVal.x -(accOffsetVal.x / accRangeFactor)) / accCorrFactor.x;
    accRawVal.y = (accRawVal.y -(accOffsetVal.y / accRangeFactor)) / accCorrFactor.y;
    accRawVal.z = (accRawVal.z -(accOffsetVal.z / accRangeFactor)) / accCorrFactor.z;
    
    return accRawVal;
}

xyzFloat correctGyrRawValues(xyzFloat gyrRawVal){
    gyrRawVal.x -= (gyrOffsetVal.x / gyrRangeFactor);
    gyrRawVal.y -= (gyrOffsetVal.y / gyrRangeFactor);
    gyrRawVal.z -= (gyrOffsetVal.z / gyrRangeFactor);
    
    return gyrRawVal;
}

void switchBank(uint8_t newBank){
		uint8_t bank = 0;
    if(newBank != currentBank){
        currentBank = newBank;
//        _wire->beginTransmission(i2cAddress);
//        _wire->write(ICM20948_REG_BANK_SEL);
//        _wire->write(currentBank<<4);
//        _wire->endTransmission();
				bank = currentBank<<4;
				HAL_I2C_Mem_Write(&hi2c1,ICM20948_WRITE,ICM20948_REG_BANK_SEL,I2C_MEMADD_SIZE_8BIT,&bank,sizeof(bank),10);
    }
}



uint8_t writeRegister8(uint8_t bank, uint8_t reg, uint8_t val){
    switchBank(bank);
//    _wire->beginTransmission(i2cAddress);
//    _wire->write(reg);
//    _wire->write(val);
    
//    return _wire->endTransmission();
	
	HAL_I2C_Mem_Write(&hi2c1,ICM20948_WRITE,reg,I2C_MEMADD_SIZE_8BIT,&val,sizeof(val),10);
			return 0;
}

uint8_t writeRegister16(uint8_t bank, uint8_t reg, uint16_t val){
    switchBank(bank);
    uint8_t MSByte = ((val>>8) & 0xFF);
    uint8_t LSByte = val & 0xFF;
//    _wire->beginTransmission(i2cAddress);
//    _wire->write(reg);
//    _wire->write(MSByte);
//    _wire->write(LSByte);
//    
//    return _wire->endTransmission();  
//		HAL_I2C_Mem_Write(&hi2c1,i2cAddress,reg,I2C_MEMADD_SIZE_16BIT,&val,sizeof(val),10);
		HAL_I2C_Mem_Write(&hi2c1,ICM20948_WRITE,reg,I2C_MEMADD_SIZE_8BIT,&MSByte,sizeof(MSByte),10);
		HAL_I2C_Mem_Write(&hi2c1,ICM20948_WRITE,reg+1,I2C_MEMADD_SIZE_8BIT,&LSByte,sizeof(MSByte),10);
		return 0;
}

uint8_t readRegister8(uint8_t bank, uint8_t reg){
    switchBank(bank);
    uint8_t regValue = 0;
//    _wire->beginTransmission(i2cAddress);
//    _wire->write(reg);
//    _wire->endTransmission(false);
//    _wire->requestFrom(i2cAddress,1);
//    if(_wire->available()){
//        regValue = _wire->read();
//    }
		HAL_I2C_Mem_Read(&hi2c1,ICM20948_READ,reg,I2C_MEMADD_SIZE_8BIT,&regValue,sizeof(regValue),10);
    return regValue;
}

int16_t readRegister16(uint8_t bank, uint8_t reg){
    switchBank(bank);
    uint8_t MSByte = 0, LSByte = 0;
    int16_t reg16Val = 0;
//    _wire->beginTransmission(i2cAddress);
//    _wire->write(reg);
//    _wire->endTransmission(false);
//    _wire->requestFrom(i2cAddress,2);
//    if(_wire->available()){
//        MSByte = _wire->read();
//        LSByte = _wire->read();
//    }
//    reg16Val = (MSByte<<8) + LSByte;
			HAL_I2C_Mem_Read(&hi2c1,ICM20948_READ,reg,I2C_MEMADD_SIZE_8BIT,&MSByte,sizeof(MSByte),10);
			HAL_I2C_Mem_Read(&hi2c1,ICM20948_READ,reg+1,I2C_MEMADD_SIZE_8BIT,&LSByte,sizeof(MSByte),10);
			reg16Val = (MSByte<<8) + LSByte;
//		HAL_I2C_Mem_Read(&hi2c1,i2cAddress,reg,I2C_MEMADD_SIZE_16BIT,&reg16Val,sizeof(reg16Val),10);
    return reg16Val;
}
uint8_t nowReg = 0;
uint8_t getRes = 0;
void readAllData(uint8_t* data){    
     switchBank(0);
//    _wire->beginTransmission(i2cAddress);
//    _wire->write(ICM20948_ACCEL_OUT);
//    _wire->endTransmission(false);
//    _wire->requestFrom(i2cAddress,20);
//    if(_wire->available()){
//        for(int i=0; i<20; i++){
//            data[i] = _wire->read();
//        }
//    }
	uint8_t i = 0;	
	for(i = 0 ; i<20 ;i = i + 1)
  {
		nowReg = ICM20948_ACCEL_OUT + i;
		HAL_I2C_Mem_Read(&hi2c1,ICM20948_READ,nowReg,I2C_MEMADD_SIZE_8BIT,&getRes,1,10);
    data[i] = getRes;
  }
}

xyzFloat readICM20948xyzValFromFifo(){
    uint8_t MSByte = 0, LSByte = 0;
    xyzFloat xyzResult = {0.0, 0.0, 0.0};
    MSByte = readRegister8(0, ICM20948_FIFO_R_W);
    LSByte = readRegister8(0, ICM20948_FIFO_R_W);
    xyzResult.x = ((int16_t)((MSByte<<8) + LSByte)) * 1.0;
    MSByte = readRegister8(0, ICM20948_FIFO_R_W);
    LSByte = readRegister8(0, ICM20948_FIFO_R_W);
    xyzResult.y = ((int16_t)((MSByte<<8) + LSByte)) * 1.0;
    MSByte = readRegister8(0, ICM20948_FIFO_R_W);
    LSByte = readRegister8(0, ICM20948_FIFO_R_W);
    xyzResult.z = ((int16_t)((MSByte<<8) + LSByte)) * 1.0;
    return xyzResult; 
}

void writeAK09916Register8(uint8_t reg, uint8_t val){
    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS); // write AK09916
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be written to
    writeRegister8(3, ICM20948_I2C_SLV0_DO, val);
}


uint8_t readAK09916Register8(uint8_t reg){
    enableMagDataRead(reg, 0x01);
    enableMagDataRead(AK09916_HXL, 0x08);
    regVal = readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00);
    return regVal;
}

int16_t readAK09916Register16(uint8_t reg){
    int16_t regValue = 0;
    enableMagDataRead(reg, 0x02);
    regValue = readRegister16(0, ICM20948_EXT_SLV_SENS_DATA_00);
    enableMagDataRead(AK09916_HXL, 0x08);
    return regValue;
}

uint8_t reset_ICM20948(){
    uint8_t ack = writeRegister8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
    delay(10);  // wait for registers to reset
    return (ack == 0);
}

void enableI2CMaster(){
    writeRegister8(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN); //enable I2C master
    writeRegister8(3, ICM20948_I2C_MST_CTRL, 0x07); // set I2C clock to 345.60 kHz
    delay(10);
}

void enableMagDataRead(uint8_t reg, uint8_t bytes){
    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ); // read AK09916
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be read
    writeRegister8(3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
    delay(10);
}
  
