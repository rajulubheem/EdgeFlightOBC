#include "sfe_interface.h"


sfe_def_t sfeObj;

ISM330DHCX_Object_t imu_object;

// Mock functions for bus communication and delay
int32_t BSP_I2C_Init(void) { 
	
	MX_I2C4_Init();
	
	return 0;

}
int32_t BSP_I2C_DeInit(void) { /* Implement I2C deinit */ return 0; }

/*
int32_t BSP_I2C_ReadReg(uint16_t Addr, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
    // Step 1: Send the register address
    if(HAL_I2C_Master_Transmit(sfeObj.i2c_handler, Addr, &Reg, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Transmission error
        return -1;
    }

    // Step 2: Read the data from the device
    if(HAL_I2C_Master_Receive(sfeObj.i2c_handler, Addr, pData, Length, HAL_MAX_DELAY) != HAL_OK)
    {
        // Reception error
        return -1;
    }

    // Reception successful
    return 0;
}
*/

int32_t BSP_I2C_ReadReg(uint8_t Addr, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
 
    // Step 2: Read the data from the device
    if (HAL_I2C_Mem_Read(sfeObj.i2c_handler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, HAL_MAX_DELAY) != HAL_OK)
    {
        // Reception error
        return -1;
    }

    // Reception successful
    return 0;
}


/*
int32_t BSP_I2C_WriteReg(uint16_t Addr, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
    uint8_t data[Length + 1]; // Create a buffer to hold the register address and the data
    data[0] = Reg;            // The register address is a single byte

    // Copy the data to be written after the register address
    memcpy(&data[1], pData, Length);

    // Perform the I2C write operation
    if(HAL_I2C_Master_Transmit(sfeObj.i2c_handler, Addr, data, Length + 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Transmission error
        return -1;
    }
    
    // Transmission successful
    return 0;
}
*/


int32_t BSP_I2C_WriteReg(uint8_t Addr, uint8_t Reg, uint8_t *pData, uint16_t Length)
{
   // uint8_t data[Length + 2]; // Create a buffer to hold the register address and the data
    
   // data[0] = Reg ;        // Low byte of the register address
    
    // Copy the data to be written after the register address
    //memcpy(&data[2], pData, Length);

    // Perform the I2C write operation
    if(HAL_I2C_Mem_Write(sfeObj.i2c_handler, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, HAL_MAX_DELAY) != HAL_OK)
    {
        // Transmission error
        return -1;
    }
    
    // Transmission successful
    return 0;
}



int32_t BSP_GetTick(void)
{
    return HAL_GetTick(); // Returns the current tick value in milliseconds
}

void BSP_Delay(uint32_t ms)
{
    HAL_Delay(ms); // Calls HAL library function to introduce a delay in milliseconds
}

ISM330DHCX_IO_t io_ctx = {
    BSP_I2C_Init,
    BSP_I2C_DeInit,
    ISM330DHCX_I2C_BUS,
    ISM330DHCX_ID<<1, // I2C address of the ISM330DHCX
    BSP_I2C_WriteReg,
    BSP_I2C_ReadReg,
    BSP_GetTick,
    BSP_Delay
};


int8_t SFE_init(I2C_HandleTypeDef* i2c)
{
	sfeObj.i2c_handler = i2c;
	// Initialize the IMU sensor
	if (ISM330DHCX_RegisterBusIO(&imu_object, &io_ctx) != ISM330DHCX_OK) {
		printf("Error in ISM330DHCX_RegisterBusIO\r\n");
			return 0;
	}

	if (ISM330DHCX_Init(&imu_object) != ISM330DHCX_OK) {
		printf("Error in ISM330DHCX_Init\r\n");
			return 0;
	}

	// Enable the accelerometer
	if (ISM330DHCX_ACC_Enable(&imu_object) != ISM330DHCX_OK) {
		printf("Error in ISM330DHCX_ACC_Enable\r\n");
			return 0;
	}

	// Enable the gyroscope
	if (ISM330DHCX_GYRO_Enable(&imu_object) != ISM330DHCX_OK) {
		printf("Error in ISM330DHCX_GYRO_Enable\r\n");
			return 0;
	}
	mmc5983.hi2c = i2c;
	MMC5983MA_getOffset();
	//MMC5983MA_SWreset();
	//MMC5983MA_SET();
	MMC5983MA_init(i2c, MODR_100Hz, MBW_100Hz, MSET_100);
	
	return 1;
	
}

void SFE_Read_All_Data(sfe_io_t* obj)
{
	
    ISM330DHCX_Axes_t acceleration;
    ISM330DHCX_Axes_t angular_rate;

// Get accelaration data
    if (ISM330DHCX_ACC_GetAxes(&imu_object, &acceleration) == ISM330DHCX_OK) 
    {
        obj->accelX = acceleration.x/1000.0;
        obj->accelY = acceleration.y/1000.0;
        obj->accelZ = acceleration.z/1000.0;
    }

    // Get gyroscope data
    if (ISM330DHCX_GYRO_GetAxes(&imu_object, &angular_rate) == ISM330DHCX_OK) 
    {
        obj->gyroX = angular_rate.x/1000.0;
        obj->gyroY = angular_rate.y/1000.0;
        obj->gyroZ = angular_rate.z/1000.0;
    }
		// Get magnetometer data
		MMC5983MA_readData();
		
		obj->magX = mmc5983.magDataX;
		obj->magY = mmc5983.magDataY;
		obj->magZ = mmc5983.magDataZ;
		
}

void SFE_Print_All_Data(sfe_io_t *obj)
{
  printf("Acceleration [g]:\n");
	printf("X : %.4f  ", obj->accelX);
	printf("Y : %.4f  ",obj->accelY);
	printf("Z : %.4f\n\n",obj->accelZ);
	
	printf("Gyro [dps]:\n");
	printf("X : %.4f  ", obj->gyroX);
	printf("Y : %.4f  ",obj->gyroY);
	printf("Z : %.4f\n\n",obj->gyroZ);

	printf("Magnetometer [uT]:\n");
	printf("X : %.4f  ", obj->magX);
	printf("Y : %.4f  ",obj->magY);
	printf("Z : %.4f\n\n",obj->magZ);
	printf("********************************************\t\n");	
	
}



