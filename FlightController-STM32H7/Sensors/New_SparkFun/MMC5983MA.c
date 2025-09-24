



#include "MMC5983MA.h"



#include "string.h"



#include <stdio.h>



#include <stdlib.h>



#include <stdbool.h>



//#include "arm_math.h"









MMC5983_t mmc5983;

/*****************************************Low-level communication functions*****************************************/

// Use IIC to read registers from MMC5983MA (a segment of data)

uint8_t IIC_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t * pdata) {

  uint8_t r_value = 0; // Return value

  uint8_t read_addr = dev_addr << 1 | 1; // Actual read address = device address + read command bit

  r_value = HAL_I2C_Mem_Read( mmc5983.hi2c, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0xFF); // Read data

  return r_value; // Return read result

}

// Use IIC to write registers to MMC5983MA (a segment of data)

uint8_t IIC_WriteBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t length, uint8_t * pdata) {

  uint8_t r_value = 0;

  uint8_t read_addr = dev_addr << 1 | 0;

  r_value = HAL_I2C_Mem_Write( mmc5983.hi2c, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, length, 0xFF);

  return r_value;

}

// Use IIC to read registers from MMC5983MA (one byte of data)

uint8_t IIC_ReadByte(uint8_t dev_addr, uint8_t reg_addr) {

  uint8_t rx_data = 0;

  uint8_t read_addr = dev_addr << 1 | 1;

  if (HAL_I2C_Mem_Read( mmc5983.hi2c, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, & rx_data, 1, 0xFF) != HAL_OK)

	return 0xFF;

  else

	return rx_data;

}

// Use IIC to write registers to MMC5983MA (one byte of data)

uint8_t IIC_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {

  uint8_t r_value = 0;

  uint8_t read_addr = dev_addr << 1 | 0;

  r_value = HAL_I2C_Mem_Write( mmc5983.hi2c, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, & data, 1, 0xFF);

  return r_value;

}

// Enable MMC5983MA by writing 0x08 to the MMC5983MA_CONTROL_0 register of MMC5983MA

void MMC5983MA_SET(void) {

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x08);

  HAL_Delay(1); // self clearing after 500 us

}



// Reset MMC5983MA by writing 0x10 to the MMC5983MA_CONTROL_0 register of MMC5983MA

void MMC5983MA_RESET(void) {

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x10);

  HAL_Delay(1); // self clearing after 500 us

}

// Power down MMC5983MA by writing 0 to the lowest four bits of the MMC5983MA_CONTROL_2 register, where the lowest four bits are MODR

void MMC5983MA_powerDown(void) {

  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp & ~(0x07)); // clear lowest four bits

  HAL_Delay(20); // make sure to finish the last measurement

	
}




// Power up MMC5983MA by writing MODR (non-zero) to the lowest four bits of the MMC5983MA_CONTROL_2 register

void MMC5983MA_powerUp(uint8_t MODR) {

  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2); // read register contents

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, temp | MODR);

}

// Get the ID number of MMC5983MA by reading the MMC5983MA_PRODUCT_ID register

uint8_t MMC5983MA_getChipID(void) {

	
  
	uint8_t c = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_PRODUCT_ID_REG);
  
	return c;
}




// Software reset MMC5983MA by writing 1 to the 7th bit of the MMC5983MA_CONTROL_1 register

void MMC5983MA_SWreset(void) {

  // reset device

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, 0x80); // Set bit 7 to 1 to reset MMC5983MA

  HAL_Delay(10); // Wait 10 ms for all registers to reset

}



// Initialize MMC5983MA

void MMC5983MA_init(I2C_HandleTypeDef *hi2c,uint8_t MODR, uint8_t MBW, uint8_t MSET) {

  mmc5983.hi2c = hi2c;

  //enable auto set/reset (bit 5 == 1)

  // this set/reset is a low current sensor offset measurement for normal use

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20);

	
  
	// set magnetometer bandwidth
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, MBW);

	
  
	// enable continuous measurement mode (bit 3 == 1), set sample rate
  
	// enable automatic Set/Reset (bit 7 == 1), set set/reset rate
  
	// this set/reset is a high-current "deGaussing" that should be used only to recover from
  
	// high magnetic field detuning of the magnetoresistive film
  
	// _i2c_bus->writeByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x80 | (MSET << 4) | 0x08 | MODR);
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x08 | MODR);
}




// Self-test

void MMC5983MA_selfTest(void) {

  uint8_t rawData[7] = {

    0

  }; // x/y/z mag register data stored here

  uint32_t data_set[3] = {

    0

  }, data_reset[3] = {

    0

  };

	
  
	// clear control registers
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x00);
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, 0x00);
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x00);

	
  
	MMC5983MA_SET(); // enable set current
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); // enable one-time mag measurement
  
	HAL_Delay(10); // wait for measurement to complete

	
  
	IIC_ReadBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 7, & rawData[0]); // Read the 6 raw data registers into data array
  
	data_set[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
  
	data_set[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
  
	data_set[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis



MMC5983MA_RESET(); // enable reset current

IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); // enable one-time mag measurement

HAL_Delay(10);



IIC_ReadBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 7, & rawData[0]); // Read the 6 raw data registers into data array

data_reset[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis

data_reset[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis

data_reset[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis



for (uint8_t ii = 0; ii < 3; ii++) // Get the difference between enabled and reset magnetometer measurements

{

	if (data_set[ii] > data_reset[ii]) {

		mmc5983.delta_data[ii] = data_set[ii] - data_reset[ii];

	} else {

		mmc5983.delta_data[ii] = data_reset[ii] - data_set[ii];

	}

}



}



// Get the offset of the magnetometer

void MMC5983MA_getOffset(void) {

  uint8_t rawData[7] = {

    0

  }; // x/y/z mag register data stored here

  uint32_t data_set[3] = {

    0

  }, data_reset[3] = {

    0

  };

	
  
	MMC5983MA_powerDown();

	
  
	MMC5983MA_SET(); // enable set current
  
	IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); // enable one-time mag measurement
  
	HAL_Delay(10);

	
  
	IIC_ReadBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 7, & rawData[0]); // Read the 6 raw data registers into data array
  
	data_set[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis
  
	data_set[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis
  
	data_set[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis



MMC5983MA_RESET(); // enable reset current

IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x01); // enable one-time mag measurement

HAL_Delay(10);



IIC_ReadBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 7, & rawData[0]); // Read the 6 raw data registers into data array

data_reset[0] = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // x-axis

data_reset[1] = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); // y-axis

data_reset[2] = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // z-axis



for (uint8_t ii = 0; ii < 3; ii++) {

	mmc5983.magOffset[ii] = (data_set[ii] + data_reset[ii]) >> 2;

}

}



uint8_t MMC5983MA_status(void) {

  // Read status register

  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_STATUS);

  return temp;

}



void MMC5983MA_clearInt(void) {

  // Clear data ready interrupts

  uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_STATUS);

  IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_STATUS, temp & 0x01);

}



// Read gyroscope data, this function is called in the EXTI external pin interrupt function, and the read data is stored in the MMC5983MA_t structure

void MMC5983MA_readData(void) {

  //MMC5983MA_clearInt(); // Clear interrupt

  uint8_t rawData[7]; // Store magnetometer data

  IIC_ReadBytes(MMC5983MA_ADDRESS, MMC5983MA_XOUT_0, 7, & rawData[0]); // Read the 7 register data of the magnetometer into the rawData array

  mmc5983.magX = (uint32_t)(rawData[0] << 10 | rawData[1] << 2 | (rawData[6] & 0xC0) >> 6); // Turn the 18 bits into a unsigned 32-bit value

  mmc5983.magY = (uint32_t)(rawData[2] << 10 | rawData[3] << 2 | (rawData[6] & 0x30) >> 4); //

	

	
	
	Turn the 18 bits into a unsigned 32-bit value
  
	mmc5983.magZ = (uint32_t)(rawData[4] << 10 | rawData[5] << 2 | (rawData[6] & 0x0C) >> 2); // Turn the 18 bits into a unsigned 32-bit value
  
	mmc5983.magDataX = (mmc5983.magX - mmc5983.magOffset[0]) * Mag_res;
  
	mmc5983.magDataY = (mmc5983.magY - mmc5983.magOffset[1]) * Mag_res;
  
	mmc5983.magDataZ = (mmc5983.magZ - mmc5983.magOffset[2]) * Mag_res;
	
	
	
	// mmc5983.magDataX = (mmc5983.magX - 131072.0) * Mag_res;

  //mmc5983.magDataY = (mmc5983.magY - 131072.0) * Mag_res;

  //mmc5983.magDataZ = (mmc5983.magZ - 131072.0) * Mag_res;

  //IIC_WriteByte(MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04); // Enable interrupt

}



// Read temperature function

uint8_t MMC5983MA_readTemperature(void) {

	
  
	uint8_t temp = IIC_ReadByte(MMC5983MA_ADDRESS, MMC5983MA_TOUT); // Read the raw temperature register
  
	return temp;
}


/*

void MMC5983MA_Check(void) {

LL_GPIO_ResetOutputPin(GPIOA, GPIO_PIN_8); // Enable magnetometer

LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_10); // Disable external interrupt

LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_10);

HAL_Delay(200); // Delay 200ms

mmc5983.ID = MMC5983MA_getChipID(); // Get chip ID

if (mmc5983.ID != MMC5983MA_ID) Error_Handler(); // If chip ID is not 0x30, report an error

else {

LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_8); // Turn off magnetometer

HAL_Delay(100); // Delay 100ms

MMC5983MA_selfTest(); // Self-test

MMC5983MA_getOffset(); // Get offset



MMC5983MA_SWreset(); // software reset MMC5983MA to default registers

MMC5983MA_clearInt(); // Clear interrupt

MMC5983MA_SET(); // "deGauss" magnetometer

MMC5983MA_init(MODR_100Hz, MBW_100Hz, MSET_100); // Initialize magnetometer, MODR_100Hz for 100Hz sampling rate, MBW_100Hz for 100Hz bandwidth, MSET_100 for set/reset once every 100 measurements

LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_10); // Enable external interrupt

LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10); // Enable external interrupt

}

}

*































































































