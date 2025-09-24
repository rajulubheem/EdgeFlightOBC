#ifndef GY_85_h
#define GY_85_h

#include "stm32h7xx_hal.h"

//----------addresses----------//
#define ADXL345 (0x53)         // Device address as specified in data sheet //ADXL345 accelerometer
#define DATAX0  (0x32)         //X-Axis Data 0
//#define DATAX1 0x33          //X-Axis Data 1
//#define DATAY0 0x34          //Y-Axis Data 0
//#define DATAY1 0x35          //Y-Axis Data 1
//#define DATAZ0 0x36          //Z-Axis Data 0
//#define DATAZ1 0x37          //Z-Axis Data 1
#define HMC5883 (0x1E)         //gyro
#define ITG3200 (0x68)         //compass





typedef struct{
	
	float accelX;
	float accelY;
	float accelZ;
	int16_t compassX;
	int16_t compassY;
	int16_t compassZ;
	float gyroX;
	float gyroY;
	float gyroZ;
	float gyroTemp;

}GY85_Def_t;
    

HAL_StatusTypeDef GY85_init(I2C_HandleTypeDef *hi2c);
int16_t* GY85_readFromCompass(I2C_HandleTypeDef *hi2c);
float* GY85_readFromAccelerometer(I2C_HandleTypeDef *hi2c);
float* GY85_readGyro(I2C_HandleTypeDef *hi2c);

void GY85_Read_All_Data(I2C_HandleTypeDef *hi2c, GY85_Def_t *obj);
void GY85_Print_All_Data(GY85_Def_t *obj);
//-----------------------------------

static void GY85_GyroCalibrate(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef GY85_SetGyro(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef GY85_SetCompass(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef GY85_SetAccelerometer(I2C_HandleTypeDef *hi2c);

//-----------------------------------

//callback functions
static float GY85_accelerometer_x( float* a ){ return *(   a ); }
static float GY85_accelerometer_y( float* a ){ return *( 1+a ); }
static float GY85_accelerometer_z( float* a ){ return *( 2+a ); }

//-----------------------------------

static int16_t GY85_compass_x( int16_t* a ){ return *(   a ); }
static int16_t GY85_compass_y( int16_t* a ){ return *( 1+a ); }
static int16_t GY85_compass_z( int16_t* a ){ return *( 2+a ); }

//-----------------------------------

static float GY85_gyro_x( float* a );
static float GY85_gyro_y( float* a );
static float GY85_gyro_z( float* a );
static float GY85_temp  ( float* a );
     

#endif
