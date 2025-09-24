#ifndef SERIAL_ENCODER_H
#define SERIAL_ENCODER_H

#include "string.h"
#include <stdio.h>
#include "mpack.h"

#include "GY_85.h"
#include "HMC6343.h"
#include "MPU9250.h"
#include "sfe_interface.h"

typedef enum{
	GY85_ID = 0x10,
	SFE_ID,
	HMC6343_ID,
	MPU9250_ID,
	
}sensorId_Def_n;

#define ACCEL_ID 	  1
#define COMPASS_ID  2
#define GYRO_ID 		3
#define TEMP_ID     4

#define MAG_ID 5
#define TILT_ID 6

#define RESULTANTG_ID 7


extern GY85_Def_t GY85_Sensor_1;
extern GY85_Def_t GY85_Sensor_2;

extern HMC6343_Def_t HMC6343_Sensor;

extern MPU9250_Def_t MPU9250_Sensor;

extern sfe_io_t SFE_Sensor;

size_t Encode_GY85_Data(GY85_Def_t* obj,char *buffer, size_t buffer_size);
void Decode_GY85_Data(GY85_Def_t* obj, char *buffer, size_t buffer_size);
size_t Encode_HMC6343_Data(HMC6343_Def_t* obj, char *buffer, size_t buffer_size);
void Decode_HMC6343_Data(HMC6343_Def_t* obj, char *buffer, size_t buffer_size);
size_t Encode_MPU9250_Data(MPU9250_Def_t* obj, char *buffer, size_t buffer_size);
void Decode_MPU9250_Data(MPU9250_Def_t* obj, char *buffer, size_t buffer_size);
size_t Encode_SFE_Data(sfe_io_t* obj, char *buffer, size_t buffer_size);
void Decode_SFE_Data(sfe_io_t* obj, char *buffer, size_t buffer_size);

#endif