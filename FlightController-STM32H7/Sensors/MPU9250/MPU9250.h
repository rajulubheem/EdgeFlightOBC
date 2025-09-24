 /**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  MPU9250.h
 *        Author:
 *		   Date:  Jul 2, 2024
 *  Description:  <Write File DESCRIPTION here>     
 *  
 *********************************************************************************************************************/
#ifndef MPU9250_WE_MPU9250_H_
#define MPU9250_WE_MPU9250_H_

/**********************************************************************************************************************
 * INCLUDES
 *********************************************************************************************************************/
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h> // for abs function

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/**********************************************************************************************************************
 *  GLOBAL CONSTANT MACROS
 *********************************************************************************************************************/
	#define REGISTER_SELF_TEST_X_GYRO     0x00
    #define REGISTER_SELF_TEST_Y_GYRO       0x01
    #define REGISTER_SELF_TEST_Z_GYRO       0x02
    #define REGISTER_SELF_TEST_X_ACCEL      0x0D
    #define REGISTER_SELF_TEST_Y_ACCEL      0x0E
    #define REGISTER_SELF_TEST_Z_ACCEL      0x0F
    #define REGISTER_XG_OFFSET_H            0x13
    #define REGISTER_XG_OFFSET_L            0x14
    #define REGISTER_YG_OFFSET_H            0x15
    #define REGISTER_YG_OFFSET_L            0x16
    #define REGISTER_ZG_OFFSET_H            0x17
    #define REGISTER_ZG_OFFSET_L            0x18
    #define REGISTER_SMPLRT_DIV             0x19
    #define REGISTER_CONFIG                 0x1A
    #define REGISTER_GYRO_CONFIG            0x1B
    #define REGISTER_ACCEL_CONFIG           0x1C
    #define REGISTER_ACCEL_CONFIG_2         0x1D
    #define REGISTER_LP_ACCEL_ODR           0x1E
    #define REGISTER_WOM_THR                0x1F
    #define REGISTER_FIFO_EN                0x23
    #define REGISTER_I2C_MST_CTRL           0x24
    #define REGISTER_I2C_SLV0_ADDR          0x25
    #define REGISTER_I2C_SLV0_REG           0x26
    #define REGISTER_I2C_SLV0_CTRL          0x27
    #define REGISTER_I2C_MST_STATUS         0x36
    #define REGISTER_INT_PIN_CFG            0x37
    #define REGISTER_INT_ENABLE             0x38
    #define REGISTER_INT_STATUS             0x3A
    #define REGISTER_ACCEL_OUT              0x3B // accel data registers begi
    #define REGISTER_TEMP_OUT               0x41
    #define REGISTER_GYRO_OUT               0x43 // gyro data registers begin
    #define REGISTER_EXT_SLV_SENS_DATA_00   0x49
    #define REGISTER_I2C_SLV0_DO            0x63
    #define REGISTER_I2C_MST_DELAY_CTRL     0x67
    #define REGISTER_SIGNAL_PATH_RESET      0x68
    #define REGISTER_MOT_DET_CTRL           0x69
    #define REGISTER_USER_CTRL              0x6A
    #define REGISTER_PWR_MGMT_1             0x6B
    #define REGISTER_PWR_MGMT_2             0x6C
    #define REGISTER_FIFO_COUNT             0x72 // 0x72 is COUNT_H
    #define REGISTER_FIFO_R_W               0x74
    #define REGISTER_WHO_AM_I               0x75
    #define REGISTER_XA_OFFSET_H            0x77
    #define REGISTER_XA_OFFSET_L            0x78
    #define REGISTER_YA_OFFSET_H            0x7A
    #define REGISTER_YA_OFFSET_L            0x7B
    #define REGISTER_ZA_OFFSET_H            0x7D
    #define REGISTER_ZA_OFFSET_L            0x7E

    /* Register Values */
    #define REGISTER_VALUE_RESET            0x80
    #define REGISTER_VALUE_BYPASS_EN        0x02
    #define REGISTER_VALUE_I2C_MST_EN       0x20
    #define REGISTER_VALUE_CLK_SEL_PLL      0x01

    /* Others */
    #define ROOM_TEMPERATURE_OFFSET           0.0f
    #define TEMPERATURE_SENSITIVITY           333.87f
    #define WHO_AM_I_CODE                     0x71


	#define I2C_TIMOUT_MS     HAL_MAX_DELAY
	#define I2C_OBJ 		  &hi2c4

	#define true 1
	#define false 0

	/*****************************Ak8963 Macros***************************/

	#define REGISTER_AK8963_WIA            0x00 // Who am I
    #define REGISTER_AK8963_INFO           0x01
    #define REGISTER_AK8963_STATUS_1       0x02
    #define REGISTER_AK8963_HXL            0x03
    #define REGISTER_AK8963_HYL            0x05
    #define REGISTER_AK8963_HZL            0x07
    #define REGISTER_AK8963_STATUS_2       0x09
    #define REGISTER_AK8963_CNTL_1         0x0A
    #define REGISTER_AK8963_CNTL_2         0x0B
    #define REGISTER_AK8963_ASTC           0x0C // Self Test
    #define REGISTER_AK8963_I2CDIS         0x0F
    #define REGISTER_AK8963_ASAX           0x10
    #define REGISTER_AK8963_ASAY           0x11
    #define REGISTER_AK8963_ASAZ           0x12

    /* Register Values */
    #define REGISTER_VALUE_AK8963_16_BIT   0x10
    #define REGISTER_VALUE_AK8963_OVF      0x08
    #define REGISTER_VALUE_AK8963_READ     0x80

    /* Others */
    #define MAGNETOMETER_I2C_ADDRESS       0x0C
    #define MAGNETOMETER_WHO_AM_I_CODE     0x48
/**********************************************************************************************************************
 *  GLOBAL FUNCTION MACROS
 *********************************************************************************************************************/


/**********************************************************************************************************************
 *  GLOBAL DATA TYPES AND STRUCTURES
 *********************************************************************************************************************/
typedef enum MPU9250_BW_WO_DLPF {
    MPU9250_BW_WO_DLPF_3600 = 0x02,
    MPU9250_BW_WO_DLPF_8800 = 0x01,
    MPU6500_BW_WO_DLPF_3600 = MPU9250_BW_WO_DLPF_3600,
    MPU6500_BW_WO_DLPF_8800 = MPU9250_BW_WO_DLPF_8800
} MPU9250_bw_wo_dlpf;

typedef enum MPU9250_DLPF {
    MPU9250_DLPF_0, MPU9250_DLPF_1, MPU9250_DLPF_2, MPU9250_DLPF_3, MPU9250_DLPF_4, MPU9250_DLPF_5,
    MPU9250_DLPF_6, MPU9250_DLPF_7,
    MPU6500_DLPF_0 = MPU9250_DLPF_0,
    MPU6500_DLPF_1 = MPU9250_DLPF_1,
    MPU6500_DLPF_2 = MPU9250_DLPF_2,
    MPU6500_DLPF_3 = MPU9250_DLPF_3,
    MPU6500_DLPF_4 = MPU9250_DLPF_4,
    MPU6500_DLPF_5 = MPU9250_DLPF_5,
    MPU6500_DLPF_6 = MPU9250_DLPF_6,
    MPU6500_DLPF_7 = MPU9250_DLPF_7
} MPU9250_dlpf;

typedef enum MPU9250_GYRO_RANGE {
    MPU9250_GYRO_RANGE_250, MPU9250_GYRO_RANGE_500, MPU9250_GYRO_RANGE_1000, MPU9250_GYRO_RANGE_2000,
    MPU6500_GYRO_RANGE_250  = MPU9250_GYRO_RANGE_250,
    MPU6500_GYRO_RANGE_500  = MPU9250_GYRO_RANGE_500,
    MPU6500_GYRO_RANGE_1000 = MPU9250_GYRO_RANGE_1000,
    MPU6500_GYRO_RANGE_2000 = MPU9250_GYRO_RANGE_2000
} MPU9250_gyroRange;

typedef enum MPU9250_ACC_RANGE {
    MPU9250_ACC_RANGE_2G, MPU9250_ACC_RANGE_4G, MPU9250_ACC_RANGE_8G, MPU9250_ACC_RANGE_16G,
    MPU6500_ACC_RANGE_2G  = MPU9250_ACC_RANGE_2G,
    MPU6500_ACC_RANGE_4G  = MPU9250_ACC_RANGE_4G,
    MPU6500_ACC_RANGE_8G  = MPU9250_ACC_RANGE_8G,
    MPU6500_ACC_RANGE_16G = MPU9250_ACC_RANGE_16G
} MPU9250_accRange;

typedef enum MPU9250_LOW_PWR_ACC_ODR {
    MPU9250_LP_ACC_ODR_0_24, MPU9250_LP_ACC_ODR_0_49, MPU9250_LP_ACC_ODR_0_98, MPU9250_LP_ACC_ODR_1_95,
    MPU9250_LP_ACC_ODR_3_91, MPU9250_LP_ACC_ODR_7_81, MPU9250_LP_ACC_ODR_15_63, MPU9250_LP_ACC_ODR_31_25,
    MPU9250_LP_ACC_ODR_62_5, MPU9250_LP_ACC_ODR_125, MPU9250_LP_ACC_ODR_250, MPU9250_LP_ACC_ODR_500,
    MPU6500_LP_ACC_ODR_0_24  = MPU9250_LP_ACC_ODR_0_24,
    MPU6500_LP_ACC_ODR_0_49  = MPU9250_LP_ACC_ODR_0_49,
    MPU6500_LP_ACC_ODR_0_98  = MPU9250_LP_ACC_ODR_0_98,
    MPU6500_LP_ACC_ODR_1_95  = MPU9250_LP_ACC_ODR_1_95,
    MPU6500_LP_ACC_ODR_3_91  = MPU9250_LP_ACC_ODR_3_91,
    MPU6500_LP_ACC_ODR_7_81  = MPU9250_LP_ACC_ODR_7_81,
    MPU6500_LP_ACC_ODR_15_63 = MPU9250_LP_ACC_ODR_15_63,
    MPU6500_LP_ACC_ODR_31_25 = MPU9250_LP_ACC_ODR_31_25,
    MPU6500_LP_ACC_ODR_62_5  = MPU9250_LP_ACC_ODR_62_5,
    MPU6500_LP_ACC_ODR_125   = MPU9250_LP_ACC_ODR_125,
    MPU6500_LP_ACC_ODR_250   = MPU9250_LP_ACC_ODR_250,
    MPU6500_LP_ACC_ODR_500   = MPU9250_LP_ACC_ODR_500
} MPU9250_lpAccODR;

typedef enum MPU9250_INT_PIN_POL {
    MPU9250_ACT_HIGH, MPU9250_ACT_LOW,
    MPU6500_ACT_HIGH = MPU9250_ACT_HIGH,
    MPU6500_ACT_LOW = MPU9250_ACT_LOW
} MPU9250_intPinPol;

typedef enum MPU9250_INT_TYPE {
    MPU9250_DATA_READY = 0x01,
    MPU9250_FIFO_OVF   = 0x10,
    MPU9250_WOM_INT    = 0x40,
    MPU6500_DATA_READY = MPU9250_DATA_READY,
    MPU6500_FIFO_OVF   = MPU9250_FIFO_OVF,
    MPU6500_WOM_INT    = MPU9250_WOM_INT
} MPU9250_intType;

typedef enum MPU9250_WOM_EN {
    MPU9250_WOM_DISABLE, MPU9250_WOM_ENABLE,
    MPU6500_WOM_DISABLE = MPU9250_WOM_DISABLE,
    MPU6500_WOM_ENABLE  = MPU9250_WOM_ENABLE
} MPU9250_womEn;

typedef enum MPU9250_WOM_COMP {
    MPU9250_WOM_COMP_DISABLE, MPU9250_WOM_COMP_ENABLE,
    MPU6500_WOM_COMP_DISABLE = MPU9250_WOM_COMP_DISABLE,
    MPU6500_WOM_COMP_ENABLE  = MPU9250_WOM_COMP_ENABLE
} MPU9250_womCompEn;

typedef enum MPU9250_XYZ_ENABLE {
    MPU9250_ENABLE_XYZ,  //all axes are enabled (default)
    MPU9250_ENABLE_XY0,  // x, y enabled, z disabled
    MPU9250_ENABLE_X0Z,
    MPU9250_ENABLE_X00,
    MPU9250_ENABLE_0YZ,
    MPU9250_ENABLE_0Y0,
    MPU9250_ENABLE_00Z,
    MPU9250_ENABLE_000,  // all axes disabled
    MPU6500_ENABLE_XYZ = MPU9250_ENABLE_XYZ,
    MPU6500_ENABLE_XY0 = MPU9250_ENABLE_XY0,
    MPU6500_ENABLE_X0Z = MPU9250_ENABLE_X0Z,
    MPU6500_ENABLE_X00 = MPU9250_ENABLE_X00,
    MPU6500_ENABLE_0YZ = MPU9250_ENABLE_0YZ,
    MPU6500_ENABLE_0Y0 = MPU9250_ENABLE_0Y0,
    MPU6500_ENABLE_00Z = MPU9250_ENABLE_00Z,
    MPU6500_ENABLE_000 = MPU9250_ENABLE_000
} MPU9250_xyzEn;

typedef enum MPU9250_ORIENTATION {
  MPU9250_FLAT, MPU9250_FLAT_1, MPU9250_XY, MPU9250_XY_1, MPU9250_YX, MPU9250_YX_1,
  MPU6500_FLAT   = MPU9250_FLAT,
  MPU6500_FLAT_1 = MPU9250_FLAT_1,
  MPU6500_XY     = MPU9250_XY,
  MPU6500_XY_1   = MPU9250_XY_1,
  MPU6500_YX     = MPU9250_YX,
  MPU6500_YX_1   = MPU9250_YX_1
} MPU9250_orientation;

typedef enum MPU9250_FIFO_MODE {
    MPU9250_CONTINUOUS, MPU9250_STOP_WHEN_FULL,
    MPU6500_CONTINUOUS     = MPU9250_CONTINUOUS,
    MPU6500_STOP_WHEN_FULL = MPU9250_STOP_WHEN_FULL
} MPU9250_fifoMode;

typedef enum MPU9250_FIFO_TYPE {
    MPU9250_FIFO_ACC        = 0x08,
    MPU9250_FIFO_GYR        = 0x70,
    MPU9250_FIFO_ACC_GYR    = 0x78,
    MPU6500_FIFO_ACC     = MPU9250_FIFO_ACC,
    MPU6500_FIFO_GYR     = MPU9250_FIFO_GYR,
    MPU6500_FIFO_ACC_GYR = MPU9250_FIFO_ACC_GYR
} MPU9250_fifo_type;

typedef enum AK8963_OP_MODE {
    AK8963_PWR_DOWN           = 0x00,
    AK8963_TRIGGER_MODE       = 0x01,
    AK8963_CONT_MODE_8HZ      = 0x02,
    AK8963_CONT_MODE_100HZ    = 0x06,
    AK8963_FUSE_ROM_ACC_MODE  = 0x0F
} AK8963_opMode;
typedef struct {
    float x;
    float y;
    float z;
}xyzFloat;

typedef struct{
	float accelX;
	float accelY;
	float accelZ;
	float gyroX;
	float gyroY;
	float gyroZ;
	float magX;
	float magY;
	float magZ;
	float temperature;
	float resultantG;
}MPU9250_Def_t;


/**********************************************************************************************************************
 *  GLOBAL DATA PROTOTYPES
 *********************************************************************************************************************/
extern I2C_HandleTypeDef hi2c4;

/**********************************************************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/
uint8_t MPU9250_init();
uint8_t MPU9250_whoAmI(void);
void MPU9250_autoOffsets();
void MPU9250_setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
void MPU9250_setGyrOffsets(float xOffset, float yOffset, float zOffset);
xyzFloat MPU9250_getAccOffsets();
xyzFloat MPU9250_getGyrOffsets();
void MPU9250_setGyrDLPF(MPU9250_dlpf dlpf);
void MPU9250_setSampleRateDivider(uint8_t splRateDiv);
void MPU9250_setGyrRange(MPU9250_gyroRange gyroRange);
void MPU9250_enableGyrDLPF();
void MPU9250_disableGyrDLPF(MPU9250_bw_wo_dlpf bw);
void MPU9250_setAccRange(MPU9250_accRange accRange);
void MPU9250_enableAccDLPF(uint8_t enable);
void MPU9250_setAccDLPF(MPU9250_dlpf dlpf);
void MPU9250_setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr);
void MPU9250_enableAccAxes(MPU9250_xyzEn enable);
void MPU9250_enableGyrAxes(MPU9250_xyzEn enable);


/* x,y,z results */

xyzFloat MPU9250_getAccRawValues();
xyzFloat MPU9250_getCorrectedAccRawValues();
xyzFloat MPU9250_getGValues();
xyzFloat MPU9250_getAccRawValuesFromFifo();
xyzFloat MPU9250_getCorrectedAccRawValuesFromFifo();
xyzFloat MPU9250_getGValuesFromFifo();
float MPU9250_getResultantG(xyzFloat gVal);
float MPU9250_getTemperature();
xyzFloat MPU9250_getGyrRawValues();
xyzFloat MPU9250_getCorrectedGyrRawValues();
xyzFloat MPU9250_getGyrValues();
xyzFloat MPU9250_getGyrValuesFromFifo();


/* Angles and Orientation */

xyzFloat MPU9250_getAngles();
MPU9250_orientation MPU9250_getOrientation();
void MPU9250_getOrientationAsString(char* str);
float MPU9250_getPitch();
float MPU9250_getRoll();

/* Power, Sleep, Standby */

void MPU9250_sleep(uint8_t sleep);
void MPU9250_enableCycle(uint8_t cycle);
void MPU9250_enableGyrStandby(uint8_t gyroStandby);

/* Interrupts */

void MPU9250_setIntPinPolarity(MPU9250_intPinPol pol);
void MPU9250_enableIntLatch(uint8_t latch);
void MPU9250_enableClearIntByAnyRead(uint8_t clearByAnyRead);
void MPU9250_enableInterrupt(MPU9250_intType intType);
void MPU9250_disableInterrupt(MPU9250_intType intType);
uint8_t MPU9250_checkInterrupt(uint8_t source, MPU9250_intType type);
uint8_t MPU9250_readAndClearInterrupts();
void MPU9250_setWakeOnMotionThreshold(uint8_t womthresh);
void MPU9250_enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn);

/* FIFO */

void MPU9250_startFifo(MPU9250_fifo_type fifo);
void MPU9250_stopFifo();
void MPU9250_enableFifo(uint8_t fifo);
void MPU9250_resetFifo();
int16_t MPU9250_getFifoCount();
void MPU9250_setFifoMode(MPU9250_fifoMode mode);
int16_t MPU9250_getNumberOfFifoDataSets();
void MPU9250_findFifoBegin();




/* x,y,z results */

xyzFloat MPU9250_getMagValues();

/* Magnetometer */

uint8_t MPU9250_initMagnetometer();
uint8_t MPU9250_whoAmIMag();
void MPU9250_setMagOpMode(AK8963_opMode opMode);
void MPU9250_startMagMeasurement();



void MPU9250_Read_All_Data(MPU9250_Def_t* obj);
void MPU9250_Print_All_Data(MPU9250_Def_t* obj);
#endif /* MPU9250_WE_MPU9250_H_ */

/**********************************************************************************************************************
 *  END OF FILE: MPU9250.h
 *********************************************************************************************************************/

