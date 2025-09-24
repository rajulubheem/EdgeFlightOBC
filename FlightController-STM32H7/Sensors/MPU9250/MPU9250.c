 /**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  MPU9250.C
 *        Author:
 *		   Date:  Jul 2, 2024
 *  Description:  <Write File DESCRIPTION here>     
 *  
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "MPU9250.h"

/**********************************************************************************************************************
*  LOCAL MACROS CONSTANT\FUNCTION
*********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL DATA 
 *********************************************************************************************************************/
   static xyzFloat accOffsetVal;
   static xyzFloat gyrOffsetVal;
   static uint8_t accRangeFactor;
   static uint8_t gyrRangeFactor;
   static MPU9250_fifo_type fifoType;
   static uint8_t const i2cAddress = 0x69<<1;
   static xyzFloat magCorrFactor;

/**********************************************************************************************************************
 *  GLOBAL DATA
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/
   static uint8_t init(uint8_t const expectedValue);
   static void correctAccRawValues(xyzFloat* rawValues);
   static void correctGyrRawValues(xyzFloat* rawValues);
   static void getAsaVals();
   static void reset_MPU9250();
   static void enableI2CMaster();
   static void writeMPU9250Register(uint8_t reg, uint8_t val);
   static uint8_t readMPU9250Register8(uint8_t reg);
   static int16_t readMPU9250Register16(uint8_t reg);
   static void readMPU9250Register3x16(uint8_t reg, uint8_t *buf);
   static xyzFloat readMPU9250xyzValFromFifo();

   static void enableMagDataRead(uint8_t reg, uint8_t bytes);
   static void resetMagnetometer();
   static void writeAK8963Register(uint8_t reg, uint8_t val);
   static uint8_t readAK8963Register8(uint8_t reg);
   static void readAK8963Data(uint8_t *buf);
   static void setMagnetometer16Bit();
   static uint8_t getStatus2Register();



/**********************************************************************************************************************
 *  LOCAL FUNCTIONS
 *********************************************************************************************************************/
   static uint8_t init(uint8_t const expectedValue)
   {
	   	reset_MPU9250();
	    HAL_Delay(10);
	    writeMPU9250Register(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);  // Bypass Enable
	    HAL_Delay(10);
		// printf("%x\t\n\n", MPU9250_whoAmI());
	    if(MPU9250_whoAmI() != expectedValue){
	        return false;
	    }

	    accOffsetVal.x = 0.0;
	    accOffsetVal.y = 0.0;
	    accOffsetVal.z = 0.0;
	    accRangeFactor = 1;
	    gyrOffsetVal.x = 0.0;
	    gyrOffsetVal.y = 0.0;
	    gyrOffsetVal.z = 0.0;
	    gyrRangeFactor = 1;
	    fifoType = MPU9250_FIFO_ACC;
	    MPU9250_sleep(false);
	    return true;
   }
   static void correctAccRawValues(xyzFloat* rawValues){
	     rawValues->x -= (accOffsetVal.x / accRangeFactor);
	     rawValues->y -= (accOffsetVal.y / accRangeFactor);
	     rawValues->z -= (accOffsetVal.z / accRangeFactor);
   }
   static void correctGyrRawValues(xyzFloat* rawValues)
   {
	    rawValues->x -= (gyrOffsetVal.x / gyrRangeFactor);
	    rawValues->y -= (gyrOffsetVal.y / gyrRangeFactor);
	    rawValues->z -= (gyrOffsetVal.z / gyrRangeFactor);
   }

   static void reset_MPU9250()
   {
	   writeMPU9250Register(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
	   HAL_Delay(10);  // wait for registers to reset
   }
   static void enableI2CMaster()
   {
	    uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
	    regVal |= REGISTER_VALUE_I2C_MST_EN;
	    writeMPU9250Register(REGISTER_USER_CTRL, regVal); //enable I2C master
	    writeMPU9250Register(REGISTER_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
	    HAL_Delay(10);
   }
   static void writeMPU9250Register(uint8_t reg, uint8_t val){
     HAL_I2C_Mem_Write(I2C_OBJ, i2cAddress, reg, 1, &val, 1, HAL_MAX_DELAY);
   }

   static uint8_t readMPU9250Register8(uint8_t reg){
       uint8_t regValue = 0;
   	HAL_I2C_Mem_Read(I2C_OBJ, i2cAddress, reg, 1, &regValue, 1, HAL_MAX_DELAY);
       return regValue;
   }

   static int16_t readMPU9250Register16(uint8_t reg){
       uint8_t Bytes[2]={0,0};
       int16_t regValue = 0;
       HAL_I2C_Mem_Read(I2C_OBJ, i2cAddress, reg, 1, Bytes, 2, HAL_MAX_DELAY);
       regValue = (Bytes[0]<<8) + Bytes[1];
       return regValue;
   }

   static void readMPU9250Register3x16(uint8_t reg, uint8_t *buf){
           HAL_I2C_Mem_Read(I2C_OBJ, i2cAddress, reg, 1, buf, 6, HAL_MAX_DELAY);
   }
   static xyzFloat readMPU9250xyzValFromFifo()
   {
	    uint8_t fifoTriple[6];
	     HAL_I2C_Mem_Read(I2C_OBJ, i2cAddress, REGISTER_FIFO_R_W, 1, fifoTriple, 6, HAL_MAX_DELAY);
	    xyzFloat xyzResult = {0.0, 0.0, 0.0};
	    xyzResult.x = (float)((int16_t)((fifoTriple[0]<<8) + fifoTriple[1]));
	    xyzResult.y = (float)((int16_t)((fifoTriple[2]<<8) + fifoTriple[3]));
	    xyzResult.z = (float)((int16_t)((fifoTriple[4]<<8) + fifoTriple[5]));
	    return xyzResult;
   }


   static void enableMagDataRead(uint8_t reg, uint8_t bytes){
	    writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS | REGISTER_VALUE_AK8963_READ); // read AK8963
	    writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be read
	    writeMPU9250Register(REGISTER_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
	    HAL_Delay(10);
   }
   static void resetMagnetometer(){
	      writeAK8963Register(REGISTER_AK8963_CNTL_2, 0x01);
	      HAL_Delay(100);
   }
   static void writeAK8963Register(uint8_t reg, uint8_t val){
	    writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS); // write AK8963
	    writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be written to
	    writeMPU9250Register(REGISTER_I2C_SLV0_DO, val);
   }
   static uint8_t readAK8963Register8(uint8_t reg){
	      enableMagDataRead(reg, 0x01);
	      uint8_t const regVal = readMPU9250Register8(REGISTER_EXT_SLV_SENS_DATA_00);
	      enableMagDataRead(REGISTER_AK8963_HXL, 0x08);

	      return regVal;
   }
   static void readAK8963Data(uint8_t *buf){
	          HAL_I2C_Mem_Read(I2C_OBJ, i2cAddress, REGISTER_EXT_SLV_SENS_DATA_00, 1, buf, 6, HAL_MAX_DELAY);
   }
   static void setMagnetometer16Bit(){
	   uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
	      regVal |= REGISTER_VALUE_AK8963_16_BIT;
	      writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
   }
   static uint8_t getStatus2Register(){
	   return readAK8963Register8(REGISTER_AK8963_STATUS_2);
   }
   static void getAsaVals()
   {
	       uint8_t rawCorr = 0;
	       rawCorr = readAK8963Register8(REGISTER_AK8963_ASAX);
	       magCorrFactor.x = (0.5 * (rawCorr-128)/128.0) + 1.0;
	       rawCorr = readAK8963Register8(REGISTER_AK8963_ASAY);
	       magCorrFactor.y = (0.5 * (rawCorr-128)/128.0) + 1.0;
	       rawCorr = readAK8963Register8(REGISTER_AK8963_ASAZ);
	       magCorrFactor.z = (0.5 * (rawCorr-128)/128.0) + 1.0;
   }
/**********************************************************************************************************************
 *  GLOBAL FUNCTIONS
 *********************************************************************************************************************/
   uint8_t MPU9250_init()
   {
		 
	   uint8_t status = init(WHO_AM_I_CODE);
		 if(status == 0)
			 return 0;
		 if (!MPU9250_initMagnetometer()) {
				printf("Magnetometer does not respond\t\n");
			  return 0;
			}
			printf("Position your MPU9250 flat and don't move it - calibrating...\t\n");
			HAL_Delay(1000);
			MPU9250_autoOffsets();
			printf("Done!\t\n");

			MPU9250_enableGyrDLPF();

			MPU9250_setGyrDLPF(MPU9250_DLPF_6);  

			MPU9250_setSampleRateDivider(5);

			MPU9250_setGyrRange(MPU9250_GYRO_RANGE_250);

			MPU9250_setAccRange(MPU9250_ACC_RANGE_2G);

			//  Enable/disable the digital low pass filter for the accelerometer 
			//  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
			 
			MPU9250_enableAccDLPF(true);

			MPU9250_setAccDLPF(MPU9250_DLPF_6);

			MPU9250_setMagOpMode(AK8963_CONT_MODE_100HZ);
			HAL_Delay(200);		 
		 
			return status;
   }
	 
   uint8_t MPU9250_whoAmI(){
	   return readMPU9250Register8(REGISTER_WHO_AM_I);
   }
   void MPU9250_autoOffsets()
   {
	    MPU9250_enableGyrDLPF();
	    MPU9250_setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
	    MPU9250_setGyrRange(MPU9250_GYRO_RANGE_250); // highest resolution
	    MPU9250_setAccRange(MPU9250_ACC_RANGE_2G);
	    MPU9250_enableAccDLPF(true);
	    MPU9250_setAccDLPF(MPU9250_DLPF_6);
	    HAL_Delay(100);

	    xyzFloat accelerationOffsetAccumulator={0.f, 0.f, 0.f};
	    xyzFloat gyroOffsetAccumulator={0.f, 0.f, 0.f};
	    xyzFloat tempAccRawValues={0.f, 0.f, 0.f};
	    xyzFloat tempGyrRawValues={0.f, 0.f, 0.f};
	    for(int i=0; i<50; i++){
	        // acceleration
	    	tempAccRawValues = MPU9250_getAccRawValues();
	    	accelerationOffsetAccumulator.x += tempAccRawValues.x;
	    	accelerationOffsetAccumulator.y += tempAccRawValues.y;
	    	accelerationOffsetAccumulator.z += tempAccRawValues.z;
	        // gyro
	    	tempGyrRawValues = MPU9250_getGyrRawValues();
	    	gyroOffsetAccumulator.x += tempGyrRawValues.x;
	    	gyroOffsetAccumulator.y += tempGyrRawValues.y;
	    	gyroOffsetAccumulator.z += tempGyrRawValues.z;
	        HAL_Delay(1);
	    }

	    // acceleration
	    accelerationOffsetAccumulator.x /= 50.f;
	    accelerationOffsetAccumulator.y /= 50.f;
	    accelerationOffsetAccumulator.z /= 50.f;

	    accelerationOffsetAccumulator.z -= 16384.0f;
	    accOffsetVal = accelerationOffsetAccumulator;
	    // gyro
	    gyrOffsetVal.x = gyroOffsetAccumulator.x / 50.f;
	    gyrOffsetVal.y = gyroOffsetAccumulator.y / 50.f;
	    gyrOffsetVal.z = gyroOffsetAccumulator.z / 50.f;
   }
   void MPU9250_setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
	    accOffsetVal.x = (xMax + xMin) * 0.5;
	    accOffsetVal.y = (yMax + yMin) * 0.5;
	    accOffsetVal.z = (zMax + zMin) * 0.5;
   }
   void MPU9250_setGyrOffsets(float xOffset, float yOffset, float zOffset){
	    gyrOffsetVal.x = xOffset;
	    gyrOffsetVal.y = yOffset;
	    gyrOffsetVal.z = zOffset;
   }
   xyzFloat MPU9250_getAccOffsets()
   {
	   return accOffsetVal;
   }
   xyzFloat MPU9250_getGyrOffsets()
   {
	   return gyrOffsetVal;
   }
   void MPU9250_setGyrDLPF(MPU9250_dlpf dlpf)
   {
	    uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);
	    regVal &= 0xF8;
	    regVal |= dlpf;
	    writeMPU9250Register(REGISTER_CONFIG, regVal);
   }
   void MPU9250_setSampleRateDivider(uint8_t splRateDiv)
   {
	   writeMPU9250Register(REGISTER_SMPLRT_DIV, splRateDiv);
   }
   void MPU9250_setGyrRange(MPU9250_gyroRange gyroRange){
	    uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
	    regVal &= 0xE7;
	    regVal |= (gyroRange<<3);
	    writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
	    gyrRangeFactor = (1<<gyroRange);
   }
   void MPU9250_enableGyrDLPF(){
	    uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
	    regVal &= 0xFC;
	    writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
   }
   void MPU9250_disableGyrDLPF(MPU9250_bw_wo_dlpf bw){
	   uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
	    regVal &= 0xFC;
	    regVal |= bw;
	    writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
   }
   void MPU9250_setAccRange(MPU9250_accRange accRange){
	   uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG);
	    regVal &= 0xE7;
	    regVal |= (accRange<<3);
	    writeMPU9250Register(REGISTER_ACCEL_CONFIG, regVal);
	    accRangeFactor = 1<<accRange;
   }
   void MPU9250_enableAccDLPF(uint8_t enable){
	   uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
	      if(enable){
	          regVal &= ~8;
	      }
	      else{
	          regVal |= 8;
	      }
	      writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
   }
   void MPU9250_setAccDLPF(MPU9250_dlpf dlpf){
	   uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
	    regVal &= 0xF8;
	    regVal |= dlpf;
	    writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
   }
   void MPU9250_setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr){
	   writeMPU9250Register(REGISTER_LP_ACCEL_ODR, lpaodr);
   }
   void MPU9250_MPU9250_enableAccAxes(MPU9250_xyzEn enable){
	   uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
	      regVal &= ~(0x38);
	      regVal |= (enable<<3);
	      writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
   }
   void MPU9250_enableGyrAxes(MPU9250_xyzEn enable){
	   uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
	     regVal &= ~(0x07);
	     regVal |= enable;
	     writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
   }


   /* x,y,z results */
   xyzFloat MPU9250_getAccRawValues(){
	     uint8_t rawData[6];
	     readMPU9250Register3x16(REGISTER_ACCEL_OUT, rawData);
	     int16_t const xRaw = (int16_t)((rawData[0] << 8) | rawData[1]);
	     int16_t const yRaw = (int16_t)((rawData[2] << 8) | rawData[3]);
	     int16_t const zRaw = (int16_t)((rawData[4] << 8) | rawData[5]);
	     xyzFloat ret={(float)(xRaw), (float)(yRaw), (float)(zRaw)};
	     return ret ;
   }
   xyzFloat MPU9250_getCorrectedAccRawValues(){
	   xyzFloat rawValue = MPU9250_getAccRawValues();
	    correctAccRawValues(&rawValue);
	    return rawValue;
   }
   xyzFloat MPU9250_getGValues()
   {
         xyzFloat ret ={0.f, 0.f, 0.f};
	     xyzFloat const acceleration = MPU9250_getCorrectedAccRawValues();
	     ret.x = acceleration.x * ((float)(accRangeFactor) / 16384.0f);
	     ret.y = acceleration.y * ((float)(accRangeFactor) / 16384.0f);
	     ret.z = acceleration.z * ((float)(accRangeFactor) / 16384.0f);
	     return ret;
   }
   xyzFloat MPU9250_getAccRawValuesFromFifo()
   {
	    xyzFloat accRawVal = readMPU9250xyzValFromFifo();
	    return accRawVal;
   }
   xyzFloat MPU9250_getCorrectedAccRawValuesFromFifo(){
	    xyzFloat accRawVal = MPU9250_getAccRawValuesFromFifo();
	    correctAccRawValues(&accRawVal);
	    return accRawVal;
   }
   xyzFloat MPU9250_getGValuesFromFifo()
   {
	   	xyzFloat ret ={0.f, 0.f, 0.f};
	    xyzFloat accRawVal = MPU9250_getCorrectedAccRawValuesFromFifo();
	    ret.x = accRawVal.x * ((float)(accRangeFactor) / 16384.0f);
	    ret.y = accRawVal.y * ((float)(accRangeFactor) / 16384.0f);
	    ret.z = accRawVal.z * ((float)(accRangeFactor) / 16384.0f);
	    return ret;
   }
   float MPU9250_getResultantG(xyzFloat gVal){
	   	 float resultant = 0.0;
	     resultant = sqrt((gVal.x*gVal.x) + (gVal.y*gVal.y) + (gVal.z*gVal.z));
	     return resultant;
   }
   float MPU9250_getTemperature(){
	    int16_t regVal16 = readMPU9250Register16(REGISTER_TEMP_OUT);
	    float tmp = (regVal16*1.0 - ROOM_TEMPERATURE_OFFSET)/TEMPERATURE_SENSITIVITY + 21.0;
	    return tmp;
   }
   xyzFloat MPU9250_getGyrRawValues()
   {
	    uint8_t rawData[6];
	    xyzFloat ret ={0.f, 0.f, 0.f};
	    readMPU9250Register3x16(REGISTER_GYRO_OUT, rawData);
	    int16_t const xRaw = (int16_t)((rawData[0] << 8) | rawData[1]);
	    int16_t const yRaw = (int16_t)((rawData[2] << 8) | rawData[3]);
	    int16_t const zRaw = (int16_t)((rawData[4] << 8) | rawData[5]);
	    ret.x = (float)(xRaw);
	    ret.y = (float)(yRaw);
	    ret.z = (float)(zRaw);
	    return ret;
   }
   xyzFloat MPU9250_getCorrectedGyrRawValues()
   {
	     xyzFloat gyrRawVal = MPU9250_getGyrRawValues();
	     correctGyrRawValues(&gyrRawVal);
	     return gyrRawVal;
   }
   xyzFloat MPU9250_getGyrValues(){
	     xyzFloat const gyroValues = MPU9250_getCorrectedGyrRawValues();
	     xyzFloat ret ={0.f, 0.f, 0.f};
	     ret.x = gyroValues.x * ((float)(gyrRangeFactor) * 250.f / 32768.0f);
	     ret.y = gyroValues.y * ((float)(gyrRangeFactor) * 250.f / 32768.0f);
	     ret.z = gyroValues.z * ((float)(gyrRangeFactor) * 250.f / 32768.0f);
	     return ret;
   }
   xyzFloat MPU9250_getGyrValuesFromFifo()
   {
	    xyzFloat gyroValues = readMPU9250xyzValFromFifo();
	    xyzFloat ret ={0.f, 0.f, 0.f};
	    correctGyrRawValues(&gyroValues);
	    ret.x = gyroValues.x * ((float)(gyrRangeFactor) * 250.f / 32768.0f);
	    ret.y = gyroValues.y * ((float)(gyrRangeFactor) * 250.f / 32768.0f);
	    ret.z = gyroValues.z * ((float)(gyrRangeFactor) * 250.f / 32768.0f);
	    return ret;
   }


   /* Angles and Orientation */

   xyzFloat MPU9250_getAngles()
   {
	    xyzFloat angleVal;
	    xyzFloat gVal = MPU9250_getGValues();
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
   MPU9250_orientation MPU9250_getOrientation()
   {
	    xyzFloat angleVal = MPU9250_getAngles();
	    MPU9250_orientation orientation = MPU9250_FLAT;
	    if(fabsf(angleVal.x) < 45){      // |x| < 45
	        if(fabsf(angleVal.y) < 45){      // |y| < 45
	            if(angleVal.z > 0){          //  z  > 0
	                orientation = MPU9250_FLAT;
	            }
	            else{                        //  z  < 0
	                orientation = MPU9250_FLAT_1;
	            }
	        }
	        else{                         // |y| > 45
	            if(angleVal.y > 0){         //  y  > 0
	                orientation = MPU9250_XY;
	            }
	            else{                       //  y  < 0
	                orientation = MPU9250_XY_1;
	            }
	        }
	    }
	    else{                           // |x| >= 45
	        if(angleVal.x > 0){           //  x  >  0
	            orientation = MPU9250_YX;
	        }
	        else{                       //  x  <  0
	            orientation = MPU9250_YX_1;
	        }
	    }
	    return orientation;
   }
   void MPU9250_getOrientationAsString(char* str)
   {
	    MPU9250_orientation orientation = MPU9250_getOrientation();
	    switch(orientation){
	        case MPU9250_FLAT:     sprintf(str, "z up");   break;
	        case MPU9250_FLAT_1:   sprintf(str, "z down"); break;
	        case MPU9250_XY:       sprintf(str, "y up");   break;
	        case MPU9250_XY_1:     sprintf(str,  "y down"); break;
	        case MPU9250_YX:       sprintf(str, "x up");   break;
	        case MPU9250_YX_1:     sprintf(str,  "x down"); break;
	    }
   }
   float MPU9250_getPitch()
   {
	    xyzFloat angleVal = MPU9250_getAngles();
	      float pitch = (atan2(-angleVal.x, sqrt(fabsf((angleVal.y*angleVal.y + angleVal.z*angleVal.z))))*180.0)/M_PI;
	      return pitch;
   }
   float MPU9250_getRoll()
   {
	    xyzFloat angleVal = MPU9250_getAngles();
	     float roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
	     return roll;
   }

   /* Power, Sleep, Standby */

   void MPU9250_sleep(uint8_t sleep){
	   uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
	     if(sleep){
	         regVal |= 0x40;
	     }
	     else{
	         regVal &= ~(0x40);
	     }
	     writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
   }
   void MPU9250_enableCycle(uint8_t cycle){
	    uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
	    if(cycle){
	        regVal |= 0x20;
	    }
	    else{
	        regVal &= ~(0x20);
	    }
	    writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
   }
   void MPU9250_enableGyrStandby(uint8_t gyroStandby){
	   uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
	    if(gyroStandby){
	        regVal |= 0x10;
	    }
	    else{
	        regVal &= ~(0x10);
	    }
	    writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
   }

   /* Interrupts */

   void MPU9250_setIntPinPolarity(MPU9250_intPinPol pol){
	   uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
	      if(pol){
	          regVal |= 0x80;
	      }
	      else{
	          regVal &= ~(0x80);
	      }
	      writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
   }
   void MPU9250_enableIntLatch(uint8_t latch){
	   uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
	    if(latch){
	        regVal |= 0x20;
	    }
	    else{
	        regVal &= ~(0x20);
	    }
	    writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
   }
   void MPU9250_enableClearIntByAnyRead(uint8_t clearByAnyRead){
	    uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
	    if(clearByAnyRead){
	        regVal |= 0x10;
	    }
	    else{
	        regVal &= ~(0x10);
	    }
	    writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
   }
   void MPU9250_enableInterrupt(MPU9250_intType intType){
	   uint8_t regVal = readMPU9250Register8(REGISTER_INT_ENABLE);
	    regVal |= intType;
	    writeMPU9250Register(REGISTER_INT_ENABLE, regVal);
   }
   void MPU9250_disableInterrupt(MPU9250_intType intType){
	   uint8_t regVal = readMPU9250Register8(REGISTER_INT_ENABLE);
	     regVal &= ~intType;
	     writeMPU9250Register(REGISTER_INT_ENABLE, regVal);
   }
   uint8_t MPU9250_checkInterrupt(uint8_t source, MPU9250_intType type){
	   source &= type;
	     return source;
   }
   uint8_t MPU9250_readAndClearInterrupts(){
	   uint8_t regVal = readMPU9250Register8(REGISTER_INT_STATUS);
	      return regVal;
   }
   void MPU9250_setWakeOnMotionThreshold(uint8_t womthresh){
	   writeMPU9250Register(REGISTER_WOM_THR, womthresh);
   }
   void MPU9250_enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn){
	   uint8_t regVal = 0;
	      if(womEn){
	          regVal |= 0x80;
	      }
	      if(womCompEn){
	          regVal |= 0x40;
	      }
	      writeMPU9250Register(REGISTER_MOT_DET_CTRL, regVal);
   }

   /* FIFO */

   void MPU9250_startFifo(MPU9250_fifo_type fifo){
	   fifoType = fifo;
	     writeMPU9250Register(REGISTER_FIFO_EN, fifoType);
   }
   void MPU9250_stopFifo(){
	   writeMPU9250Register(REGISTER_FIFO_EN, 0);
   }
   void MPU9250_enableFifo(uint8_t fifo){
	   uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
	      if(fifo){
	          regVal |= 0x40;
	      }
	      else{
	          regVal &= ~(0x40);
	      }
	      writeMPU9250Register(REGISTER_USER_CTRL, regVal);
   }
   void MPU9250_resetFifo(){
	   uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
	    regVal |= 0x04;
	    writeMPU9250Register(REGISTER_USER_CTRL, regVal);
   }
   int16_t MPU9250_getFifoCount(){
	   uint16_t regVal16 = (uint16_t) readMPU9250Register16(REGISTER_FIFO_COUNT);
	     return regVal16;
   }
   void MPU9250_setFifoMode(MPU9250_fifoMode mode){
	   uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);
	     if(mode){
	         regVal |= 0x40;
	     }
	     else{
	         regVal &= ~(0x40);
	     }
	     writeMPU9250Register(REGISTER_CONFIG, regVal);
   }
   int16_t MPU9250_getNumberOfFifoDataSets(){
	   int16_t numberOfSets = MPU9250_getFifoCount();

	     if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
	         numberOfSets /= 6;
	     }
	     else if(fifoType==MPU9250_FIFO_ACC_GYR){
	         numberOfSets /= 12;
	     }

	     return numberOfSets;
   }
   void MPU9250_findFifoBegin()
   {
	   int16_t count = MPU9250_getFifoCount();

	       if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
	           if(count > 510){
	               for(int i=0; i<2; i++){
	                   readMPU9250Register8(REGISTER_FIFO_R_W);
	               }
	           }
	       }
	       else if(fifoType==MPU9250_FIFO_ACC_GYR){
	           if(count > 504){
	               for(int i=0; i<8; i++){
	                   readMPU9250Register8(REGISTER_FIFO_R_W);
	               }
	           }
	       }
   }



   /* x,y,z results */

   xyzFloat MPU9250_getMagValues()
   {
	    xyzFloat magVal = {0.0, 0.0, 0.0};
	    uint8_t rawData[6];
	    readAK8963Data(rawData);
	    int16_t xRaw = (int16_t)((rawData[1] << 8) | rawData[0]);
	    int16_t yRaw = (int16_t)((rawData[3] << 8) | rawData[2]);
	    int16_t zRaw = (int16_t)((rawData[5] << 8) | rawData[4]);

	    float const scaleFactor = 4912.0 / 32760.0;

	    magVal.x = xRaw * scaleFactor * magCorrFactor.x;
	    magVal.y = yRaw * scaleFactor * magCorrFactor.y;
	    magVal.z = zRaw * scaleFactor * magCorrFactor.z;

	    return magVal;
   }

   /* Magnetometer */

   uint8_t MPU9250_initMagnetometer(){
	     enableI2CMaster();
	     resetMagnetometer();

	     if(!(MPU9250_whoAmIMag() == MAGNETOMETER_WHO_AM_I_CODE)){
	         return false;
	     }
	     MPU9250_setMagOpMode(AK8963_FUSE_ROM_ACC_MODE);
	     HAL_Delay(10);
	     getAsaVals();
	     HAL_Delay(10);
	     setMagnetometer16Bit();
	     HAL_Delay(10);
	     MPU9250_setMagOpMode(AK8963_CONT_MODE_100HZ);
	     HAL_Delay(10);

	     return true;
   }
   uint8_t MPU9250_whoAmIMag(){
	   return readAK8963Register8(REGISTER_AK8963_WIA);
   }
   void MPU9250_setMagOpMode(AK8963_opMode opMode){
	    uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
	    regVal &= 0xF0;
	    regVal |= opMode;
	    writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
	    HAL_Delay(10);
	    if(opMode!=AK8963_PWR_DOWN){
	        enableMagDataRead(REGISTER_AK8963_HXL, 0x08);
	    }
   }
   void MPU9250_startMagMeasurement(){
	    MPU9250_setMagOpMode(AK8963_TRIGGER_MODE);
	    HAL_Delay(200);
   }

void MPU9250_Read_All_Data(MPU9250_Def_t* obj)
{
	xyzFloat gValue = MPU9250_getGValues();
	xyzFloat gyr = MPU9250_getGyrValues();
	xyzFloat magValue = MPU9250_getMagValues();
	float temp = MPU9250_getTemperature();
	float resultantG = MPU9250_getResultantG(gValue);
	
	obj->accelX = gValue.x;
	obj->accelY = gValue.y;
	obj->accelZ = gValue.z;
	
	obj->gyroX = gyr.x;
	obj->gyroY = gyr.y;
	obj->gyroZ = gyr.z;
	
	obj->resultantG = resultantG;
	
	obj->magX = magValue.x;
	obj->magY = magValue.y;
	obj->magZ = magValue.z;
	
	obj->temperature = temp;
 
}
	 
void MPU9250_Print_All_Data(MPU9250_Def_t* obj)
{

  printf("Acceleration:\n");
	printf("X : %.4f  ", obj->accelX);
	printf("Y : %.4f  ",obj->accelY);
	printf("Z : %.4f\n\n",obj->accelZ);
	
	printf("Magnetometer:\n");
	printf("X : %.4f  ", obj->magX);
	printf("Y : %.4f  ",obj->magY);
	printf("Z : %.4f\n\n",obj->magZ);

	printf("Gyro:\n");
	printf("X : %.4f  ", obj->gyroX);
	printf("Y : %.4f  ",obj->gyroY);
	printf("Z : %.4f\n\n",obj->gyroZ);
	
	
	printf("Temperature : %.2f\n",obj->temperature);
	//printf("ResultantG : %.2f\n\n",obj->resultantG);
	printf("********************************************\t\n");		
	
}	
	 
/**********************************************************************************************************************
 *  END OF FILE: MPU9250.C
 *********************************************************************************************************************/
