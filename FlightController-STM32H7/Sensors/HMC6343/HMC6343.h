/**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  HMC6343.h
 *        Author:
 *		   Date:  Jun 22, 2024
 *  Description:  <Write File DESCRIPTION here>     
 *  
 *********************************************************************************************************************/
#ifndef HMC6343_HMC6343_H_
#define HMC6343_HMC6343_H_

/**********************************************************************************************************************
 * INCLUDES
 *********************************************************************************************************************/
#include "stm32h7xx_hal.h"

/**********************************************************************************************************************
 *  GLOBAL CONSTANT MACROS
 *********************************************************************************************************************/
#define true 1
#define false 0

// HMC6343 I2C Address
#define HMC6343_I2C_ADDR 0x19 << 1


// HMC6343 Registers
#define SLAVE_ADDR 0x00
#define SW_VERSION 0x02
#define OP_MODE1 0x04
#define OP_MODE2 0x05
#define SN_LSB 0x06
#define SN_MSB 0x07
#define DATE_CODE_YY 0x08
#define DATE_CODE_WW 0x09
#define DEVIATION_LSB 0x0A
#define DEVIATION_MSB 0x0B
#define VARIATION_LSB 0x0C
#define VARIATION_MSB 0x0D
#define XOFFSET_LSB 0x0E
#define XOFFSET_MSB 0x0F
#define YOFFSET_LSB 0x10
#define YOFFSET_MSB 0x11
#define ZOFFSET_LSB 0x12
#define ZOFFSET_MSB 0x13
#define FILTER_LSB 0x14
#define FILTER_MSB 0x15

// HMC6343 Commands
#define POST_ACCEL 0x40
#define POST_MAG 0x45
#define POST_HEADING 0x50
#define POST_TILT 0x55
#define POST_OPMODE1 0x65
#define ENTER_CAL 0x71
#define ORIENT_LEVEL 0x72
#define ORIENT_SIDEWAYS 0x73
#define ORIENT_FLATFRONT 0x74
#define ENTER_RUN 0x75
#define ENTER_STANDBY 0x76
#define EXIT_CAL 0x7E
#define RESET 0x82
#define ENTER_SLEEP 0x83
#define EXIT_SLEEP 0x84
#define READ_EEPROM 0xE1
#define WRITE_EEPROM 0xF1

// HMC6343 Orientations
#define LEVEL 0     // X = forward, +Z = up (default)
#define SIDEWAYS 1  // X = forward, +Y = up
#define FLATFRONT 2 // Z = forward, -X = up


/**********************************************************************************************************************
 *  GLOBAL FUNCTION MACROS
 *********************************************************************************************************************/


/**********************************************************************************************************************
 *  GLOBAL DATA TYPES AND STRUCTURES
 *********************************************************************************************************************/
typedef struct{
	
	int16_t heading;
	int16_t pitch;
	int16_t roll;
	int16_t magX;
	int16_t magY;
	int16_t magZ;
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;
	int16_t temperature;

}HMC6343_Def_t;

/**********************************************************************************************************************
 *  GLOBAL DATA PROTOTYPES
 *********************************************************************************************************************/

extern I2C_HandleTypeDef hi2c4;
/**********************************************************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/

uint8_t HMC6343_init(HMC6343_Def_t* obj);

void HMC6343_readMag(HMC6343_Def_t* obj);
void HMC6343_readAccel(HMC6343_Def_t* obj);
void HMC6343_readHeading(HMC6343_Def_t* obj);
void HMC6343_readTilt(HMC6343_Def_t* obj);

void HMC6343_enterStandby();
void HMC6343_exitStandby();

void HMC6343_enterSleep();
void HMC6343_exitSleep();

void HMC6343_enterCalMode();
void HMC6343_exitCalMode();

void HMC6343_setOrientation(uint8_t orientation);

void HMC6343_reset();

uint8_t HMC6343_readOPMode1();

uint8_t HMC6343_readEEPROM(uint8_t reg);
void HMC6343_writeEEPROM(uint8_t reg, uint8_t data);
void HMC6343_autoSetOffset(HMC6343_Def_t* obj);
void HMC6343_Read_All_Data(HMC6343_Def_t* obj);
void HMC6343_Print_All_Data(HMC6343_Def_t* obj);







 
#endif /* HMC6343_HMC6343_H_ */

/**********************************************************************************************************************
 *  END OF FILE: HMC6343.h
 *********************************************************************************************************************/

