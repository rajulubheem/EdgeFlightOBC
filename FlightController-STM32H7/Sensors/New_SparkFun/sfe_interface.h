#ifndef SFE_INTERFACE_H
#define SFE_INTERFACE_H

#include "ism330dhcx.h"
#include "main.h"
#include "i2c.h"
#include "MMC5983MA.h"

typedef struct{
	I2C_HandleTypeDef* i2c_handler;
}sfe_def_t;


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
	float Temp;
}sfe_io_t;

int8_t SFE_init(I2C_HandleTypeDef* i2c);
void SFE_Read_All_Data(sfe_io_t* obj);
void SFE_Print_All_Data(sfe_io_t *obj);
#endif