#include "GY_85.h"


static float GY85_gyro_x( float* a )
{
	return *(   a ) / 14.375; 
}
static float GY85_gyro_y( float* a )
{ 
	return *( 1+a ) / 14.375; 
}
static float GY85_gyro_z( float* a )
{ 
	return *( 2+a ) / 14.375; 
}

static float GY85_temp  ( float* a )
{ 
	return  (35 + ( *( 3+a ) +13200 ) / 280.0); 
}



static HAL_StatusTypeDef GY85_SetAccelerometer(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];
    data[0] = 0x31;   // Register address
    data[1] = 0x01; // Value to write
    if(HAL_I2C_Master_Transmit(hi2c, ADXL345<<1, data, 2, HAL_MAX_DELAY) != HAL_OK)	
	{
		return HAL_ERROR;
	}
	
	data[0] = 0x2D;   // Register address
    data[1] = 0x08; // Value to write
	
    if(HAL_I2C_Master_Transmit(hi2c, ADXL345<<1, data, 2, HAL_MAX_DELAY) != HAL_OK)	
	{
		return HAL_ERROR;
	}
	return HAL_OK;	
}

float* GY85_readFromAccelerometer(I2C_HandleTypeDef *hi2c)
{
    static int16_t axis[3];
		static float res_axis[3];
    uint8_t buff[6];
    uint8_t data[1] = {DATAX0 };
	
	HAL_I2C_Master_Transmit(hi2c, ADXL345<<1, data, 1, HAL_MAX_DELAY);

	// Request 6 bytes from the ADXL345
    HAL_I2C_Master_Receive(hi2c, ADXL345<<1, buff, 6, HAL_MAX_DELAY);
        
    
    axis[0] = (int16_t)((buff[1]) << 8) | buff[0];
    axis[1] = (int16_t)((buff[3]) << 8) | buff[2];
    axis[2] = (int16_t)((buff[5]) << 8) | buff[4];
		
		res_axis[0] = axis[0] * 0.00006103515f * 9.81f;
 		res_axis[1] = axis[1] * 0.00006103515f * 9.81f;
		res_axis[2] = axis[2] * 0.00006103515f * 9.81f;
    
    return res_axis;
}
//----------------------------------------
static HAL_StatusTypeDef GY85_SetCompass(I2C_HandleTypeDef *hi2c)
{
    //Put the HMC5883 IC into the correct operating mode
	
	HAL_StatusTypeDef status;
	uint8_t data[2];
    data[0] = 0x02;   // Register address
    data[1] = 0x00; // Value to write
    if(HAL_I2C_Master_Transmit(hi2c, HMC5883<<1, data, 2, 10) != HAL_OK)	
	{
		return HAL_ERROR;
	}
	return HAL_OK;	
}

int16_t* GY85_readFromCompass(I2C_HandleTypeDef *hi2c)
{
    static int16_t axis[3];
	uint8_t buff[6]={0};
    uint8_t data[1];
    
	//Tell the HMC5883 where to begin reading data
	data[0] = 0x03;   // Register address
    HAL_I2C_Master_Transmit(hi2c, HMC5883<<1, data, 1, 10);	
	
	
    // Request 6 bytes from the ADXL345
    HAL_I2C_Master_Receive(hi2c, HMC5883<<1, buff, 6, 10);
	
	axis[0] = (int16_t)((buff[0]) << 8);         //X msb
	axis[0] |= buff[1];             //X lsb
	axis[2] = (int16_t)((buff[2])<<8);           //Z msb
	axis[2] |= buff[3];             //Z lsb
	axis[1] = (int16_t)((buff[4])<<8);           //Y msb
	axis[1] |= buff[5];             //Y lsb	
	
    return axis;
}

//----------------------------------------

static int16_t g_offx = 0;
static int16_t g_offy = 0;
static int16_t g_offz = 0;

static HAL_StatusTypeDef GY85_SetGyro(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];
    data[0] = 0x3E;   // Register address
    data[1] = 0x00; // Value to write

    if(HAL_I2C_Master_Transmit(hi2c, ITG3200<<1, data, 2, HAL_MAX_DELAY) != HAL_OK)	
	{
		return HAL_ERROR;
	}
	
	data[0] = 0x15;   // Register address
    data[1] = 0x07; // Value to write
	
    if(HAL_I2C_Master_Transmit(hi2c, ITG3200<<1, data, 2, HAL_MAX_DELAY) != HAL_OK)	
	{
		return HAL_ERROR;
	}
	
	data[0] = 0x16;   // Register address
    data[1] = 0x1E; // Value to write		// +/- 2000 dgrs/sec, 1KHz, 1E, 19
	
    if(HAL_I2C_Master_Transmit(hi2c, ITG3200<<1, data, 2, HAL_MAX_DELAY) != HAL_OK)	
	{
		return HAL_ERROR;
	}	
	
	data[0] = 0x17;   // Register address
    data[1] = 0x00; // Value to write
	
    if(HAL_I2C_Master_Transmit(hi2c, ITG3200<<1, data, 2, HAL_MAX_DELAY) != HAL_OK)	
	{
		return HAL_ERROR;
	}		
	
    HAL_Delay(10);
    
    GY85_GyroCalibrate(hi2c);
	
	return HAL_OK;	
}

static void GY85_GyroCalibrate(I2C_HandleTypeDef *hi2c)
{
    static int16_t tmpx = 0;
    static int16_t tmpy = 0;
    static int16_t tmpz = 0;
    
    g_offx = 0;
    g_offy = 0;
    g_offz = 0;
    
    for( uint8_t i = 0; i < 10; i ++ ) //take the mean from 10 gyro probes and divide it from the current probe
    {
        HAL_Delay(10);
        float* gp = GY85_readGyro(hi2c);
        tmpx += gp[0];
        tmpy += gp[1];
        tmpz += gp[2];
    }
    g_offx = tmpx/10;
    g_offy = tmpy/10;
    g_offz = tmpz/10;
}

float* GY85_readGyro(I2C_HandleTypeDef *hi2c)
{
	 static int16_t temp_axis[4]={0};
    static float axis[4]={0};
    
	
	uint8_t buff[8]={0};
    uint8_t data[1];
    
	//Tell the HMC5883 where to begin reading data
	data[0] = 0x1B;   // Register address
    HAL_I2C_Master_Transmit(hi2c, ITG3200<<1, data, 1, HAL_MAX_DELAY);	
	
	
    // Request 8 bytes from the ITG3200
    HAL_I2C_Master_Receive(hi2c, ITG3200<<1, buff, 8, HAL_MAX_DELAY);	

    temp_axis[0] = ((buff[2] << 8) | buff[3]) - g_offx;
    temp_axis[1] = ((buff[4] << 8) | buff[5]) - g_offy;
    temp_axis[2] = ((buff[6] << 8) | buff[7]) - g_offz;
    temp_axis[3] = ((buff[0] << 8) | buff[1]);       // temperature 
		
    axis[0] = ((float)(temp_axis[0]));
    axis[1] = ((float)(temp_axis[1])) ;
    axis[2] = ((float)(temp_axis[2]))  ;
    axis[3] =  temp_axis[3] ;      // temperature
    
    return axis;
}

HAL_StatusTypeDef GY85_init(I2C_HandleTypeDef *hi2c)
{
    if(GY85_SetAccelerometer(hi2c) != HAL_OK)
	{
		return HAL_ERROR;
	}
    HAL_Delay(50);
	if(GY85_SetCompass(hi2c) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_Delay(50);
    if(GY85_SetGyro(hi2c) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_Delay(50);
	return HAL_OK;
}


void GY85_Read_All_Data(I2C_HandleTypeDef *hi2c, GY85_Def_t *obj)
{
	float* accelerometerReadings = GY85_readFromAccelerometer(hi2c);
    obj->accelX = GY85_accelerometer_x(accelerometerReadings);
    obj->accelY = GY85_accelerometer_y(accelerometerReadings);
    obj->accelZ = GY85_accelerometer_z(accelerometerReadings);
		HAL_Delay(10);   
	
    int16_t* compassReadings = GY85_readFromCompass(hi2c);
    obj->compassX = GY85_compass_x(compassReadings)*0.73;
    obj->compassY = GY85_compass_y(compassReadings)*0.73;
    obj->compassZ = GY85_compass_z(compassReadings)*0.73;
		HAL_Delay(10); 
	
    float* gyroReadings = GY85_readGyro(hi2c);
    obj->gyroX = GY85_gyro_x(gyroReadings);
    obj->gyroY = GY85_gyro_y(gyroReadings);
    obj->gyroZ = GY85_gyro_z(gyroReadings);
    obj->gyroTemp = GY85_temp(gyroReadings);
		HAL_Delay(10);
	
}

void GY85_Print_All_Data(GY85_Def_t *obj)
{
  printf("Acceleration:\n");
	printf("X : %.4f  ", obj->accelX);
	printf("Y : %.4f  ",obj->accelY);
	printf("Z : %.4f\n\n",obj->accelZ);
	
	printf("Compass:\n");
	printf("X : %d  ", obj->compassX);
	printf("Y : %d  ",obj->compassY);
	printf("Z : %d\n\n",obj->compassZ);

	printf("Gyro:\n");
	printf("X : %1.3fg  ", obj->gyroX);
	printf("Y : %1.3fg  ",obj->gyroY);
	printf("Z : %1.3fg\n\n",obj->gyroZ);
	printf("Temperature : %.2f\n\n",obj->gyroTemp);
	printf("********************************************\t\n");		
	
}

