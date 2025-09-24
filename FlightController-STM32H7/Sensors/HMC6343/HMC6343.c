/**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  HMC6343.c
 *        Author:
 *		   Date:  Jun 22, 2024
 *  Description:  <Write File DESCRIPTION here>     
 *  
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "HMC6343.h"

/**********************************************************************************************************************
*  LOCAL MACROS CONSTANT\FUNCTION
*********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL DATA 
 *********************************************************************************************************************/
static uint8_t _addr;
static uint8_t rawData[6]={0};

static float headingOffset = 0;
static float pitchOffset = 0;
static float rollOffset = 0;
static float magXOffset = 0;
static float magYOffset = 0;
static float magZOffset = 0;
static float accelXOffset = 0;
static float accelYOffset = 0;
static float accelZOffset = 0;
/**********************************************************************************************************************
 *  GLOBAL DATA
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/
static void clearRawData();

static void sendCommand(uint8_t command);
static void readGeneric(uint8_t command, int16_t* first, int16_t* second, int16_t* third);



/**********************************************************************************************************************
 *  LOCAL FUNCTIONS
 *********************************************************************************************************************/
// Generic function which sends the HMC6343 a specified command
// It then collects 6 bytes of data via I2C and consolidates it into three integers
// whose addresses are passed to the function by the above read commands
static void readGeneric(uint8_t command, int16_t* first, int16_t* second, int16_t* third)
{
  sendCommand(command); // Send specified I2C command to HMC6343
  HAL_Delay(1); // HAL_Delay response time

  clearRawData(); // Clear object's rawData[] array before storing new values in the array

  // Wait for a 6 byte response via I2C and store them in rawData[] array
  HAL_I2C_Master_Receive(&hi2c4, 0x32, rawData, 6, 1000);
 HAL_Delay(10); // HAL_Delay response time
  // Convert 6 bytes received into 3 integers
  *first = (rawData[0] << 8); // MSB
  *first = *first | rawData[1];     // LSB
  *second = (rawData[2] << 8);
  *second |= rawData[3];
  *third = (rawData[4] << 8);
  *third |= rawData[5];
}

// Send specified I2C command to HMC6343
static void sendCommand(uint8_t command)
{
  HAL_I2C_Master_Transmit(&hi2c4, 0x32, &command, 1, 1000);
}

// Clears the sensor object's rawData[] array, used before taking new measurements
static void clearRawData()
{
  for (uint8_t i = 0; i < 6; i++)
  {
    rawData[i] = 0;
  }
}

/**********************************************************************************************************************
 *  GLOBAL FUNCTIONS
 *********************************************************************************************************************/


// Initialize - returns true if successful
// Starts I2C Communication
// Verifies sensor is there by checking its I2C Address in its EEPROM
uint8_t HMC6343_init(HMC6343_Def_t* obj)
{
  uint8_t ret = true;
  uint8_t data = 0x00;

  _addr = HMC6343_I2C_ADDR;

   obj->heading = obj->pitch = obj->roll = 0;
   obj->magX = obj->magY = obj->magZ = 0;
   obj->accelX = obj->accelY = obj->accelZ = 0;
   obj->temperature = 0;

  clearRawData();
  // Check for device by reading I2C address from EEPROM
  data = HMC6343_readEEPROM(SLAVE_ADDR);

  // Check if value read in EEPROM is the expected value for the HMC6343 I2C address
  if (!(data == 0x32))
  {
    ret = false; // Init failed, EEPROM did not read expected I2C address value
  }

  return ret;
}

// Send the HMC6343 a command to read the raw magnetometer values
// Store these values in the integers magX, magY, and magZ
void HMC6343_readMag(HMC6343_Def_t* obj)
{
  readGeneric(POST_MAG, &obj->magX, &obj->magY, &obj->magZ);
	obj->magX -= magXOffset;
	obj->magY -= magYOffset;
	obj->magZ -= magZOffset;		
	if(obj->magX<=7 && obj->magX>=-7)
	{
		obj->magX=0;
	}
	if(obj->magY<=7 && obj->magY>=-7)
	{
		obj->magY=0;
	}	
	if(obj->magZ<=7 && obj->magZ>=-7)
	{
		obj->magZ=0;
	}	
}

// Send the HMC6343 a command to read the raw accelerometer values
// Store these values in the integers accelX, accelY, and accelZ
void HMC6343_readAccel(HMC6343_Def_t* obj)
{
  readGeneric(0x40, &obj->accelX, &obj->accelY, &obj->accelZ);
	obj->accelX -= accelXOffset;
	obj->accelY -= accelYOffset;
	obj->accelZ -= accelZOffset;	
	
	if(obj->accelX<=8 && obj->accelX>=-8)
	{
		obj->accelX=0;
	}
	if(obj->accelY<=8 && obj->accelY>=-8)
	{
		obj->accelY=0;
	}	
	if(obj->accelZ<=8 && obj->accelZ>=-8)
	{
		obj->accelZ=0;
	}	
}

// Send the HMC6343 a command to read the raw calculated heading values
// Store these values in the integers heading, pitch, and roll
void HMC6343_readHeading(HMC6343_Def_t* obj)
{
  readGeneric(POST_HEADING, &obj->heading, &obj->pitch, &obj->roll);
	obj->heading -= headingOffset;
	obj->pitch -= pitchOffset;
	obj->roll -= rollOffset;
	
	if(obj->heading<=35 && obj->heading>=-35)
	{
		obj->heading=0;
	}
	if(obj->pitch<=8 && obj->pitch>=-8)
	{
		obj->pitch=0;
	}
	if(obj->roll<=8 && obj->roll>=-8)
	{
		obj->roll=0;
	}	
}

// Send the HMC6343 a command to read the raw calculated tilt values
// Store these values in the integers pitch, roll, and temperature
void HMC6343_readTilt(HMC6343_Def_t* obj)
{
  readGeneric(0x55, &obj->pitch, &obj->roll, &obj->temperature);
	obj->pitch -= pitchOffset;
	obj->roll -= rollOffset;	
	if(obj->pitch<=8 && obj->pitch>=-8)
	{
		obj->pitch=0;
	}
	if(obj->roll<=8 && obj->roll>=-8)
	{
		obj->roll=0;
	}		
}


// Send enter standby mode I2C command to HMC6343
void HMC6343_enterStandby()
{
  sendCommand(ENTER_STANDBY);
}

// Send exit standby (enter run) mode I2C command to HMC6343
void HMC6343_exitStandby()
{
  sendCommand(ENTER_RUN);
}

// Send enter sleep mode I2C command to HMC6343
void HMC6343_enterSleep()
{
  sendCommand(ENTER_SLEEP);
}

// Send exit sleep mode I2C command to HMC6343
void HMC6343_exitSleep()
{
  sendCommand(EXIT_SLEEP);
}

// Send enter calibration mode I2C command to HMC6343
void HMC6343_enterCalMode()
{
  sendCommand(ENTER_CAL);
}

// Send exit calibration mode I2C command to HMC6343
void HMC6343_exitCalMode()
{
  sendCommand(EXIT_CAL);
}

// Set the physical orientation of the HMC6343 IC to either LEVEL, SIDEWAYS, or FLATFRONT
// This allows the IC to calculate a proper heading, pitch, and roll in tenths of degrees
// LEVEL      X = forward, +Z = up (default)
// SIDEWAYS   X = forward, +Y = up
// FLATFRONT  Z = forward, -X = up
void HMC6343_setOrientation(uint8_t orientation)
{
  if (orientation == LEVEL)
  {
    sendCommand(ORIENT_LEVEL);
  }
  else if (orientation == SIDEWAYS)
  {
    sendCommand(ORIENT_SIDEWAYS);
  }
  else if (orientation == FLATFRONT)
  {
    sendCommand(ORIENT_FLATFRONT);
  }
}

// Send the I2C command to reset the processor on the HMC6343
void HMC6343_reset()
{
  sendCommand(RESET);
}

// Send the I2C command to read the OPMode1 status register of the HMC6343
// The register informs you of current calculation status, filter status, modes enabled and what orientation
// Refer to the HMC6343 datasheet for bit specifics
uint8_t HMC6343_readOPMode1()
{
  uint8_t opmode1 = 0x00;

  sendCommand(POST_OPMODE1);
  HAL_Delay(1);

  HAL_I2C_Master_Receive(&hi2c4, _addr, &opmode1, 1, 1000);

  return opmode1;
}

// Send a command to the HMC6343 to read a specified register of the EEPROM
uint8_t HMC6343_readEEPROM(uint8_t reg)
{
  uint8_t temp_arr[2]={READ_EEPROM,reg};
	uint8_t data =0;
  uint8_t data1 = READ_EEPROM;
	HAL_StatusTypeDef status=0;
	status =	HAL_I2C_Master_Transmit(&hi2c4, _addr, temp_arr, 2, HAL_MAX_DELAY);

  HAL_Delay(10);
	status =	HAL_I2C_Mem_Read(&hi2c4, _addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	
  return data;
}

// Send a command to the HMC6343 to write a specified register of the EEPROM
void HMC6343_writeEEPROM(uint8_t reg, uint8_t data)
{
	uint8_t temp_arr[3]={READ_EEPROM,reg,data};
    HAL_I2C_Master_Transmit(&hi2c4, _addr, temp_arr, 3, 1000);
}



void HMC6343_autoSetOffset(HMC6343_Def_t* obj)
{
	int32_t headingSum=0, pitchSum=0, rollSum=0;
	int32_t magXSum=0, magYSum=0, magZSum=0;
	int32_t accelXSum=0, accelYSum=0, accelZSum=0;
	int i=0;
	while(i<10){
		HMC6343_writeEEPROM(FILTER_LSB, 10); // 0x00-0x0F, default is 0x00
		HAL_Delay(10);
		HMC6343_writeEEPROM(OP_MODE1, 0x31); // Enable filter (default OPMODE1 + filter enable bit set);
		HAL_Delay(10);
		
		HMC6343_readMag(obj);	
		HAL_Delay(50);
		HMC6343_readAccel(obj);
		HAL_Delay(50);
		HMC6343_readHeading(obj);
		HAL_Delay(50);	
		headingSum+=obj->heading;
		pitchSum+=obj->pitch;
		rollSum+=obj->roll;
		magXSum+=obj->magX;
		magYSum+=obj->magY;
		magZSum+=obj->magZ;
		accelXSum+=obj->accelX;
		accelYSum+=obj->accelY;
		accelZSum+=obj->accelZ;
		 
		 i++;
		 HAL_Delay(250);
	}
	headingOffset = headingSum / 10.0;
	pitchOffset = pitchSum / 10.0;
	rollOffset = rollSum / 10.0;
	magXOffset = magXSum / 10.0;
	magYOffset = magYSum / 10.0;
	magZOffset = magZSum / 10.0;
	accelXOffset = accelXSum / 10.0;
	accelYOffset = accelYSum / 10.0;
	accelZOffset = accelZSum / 10.0;
}

void HMC6343_Read_All_Data(HMC6343_Def_t* obj)
{
	HMC6343_writeEEPROM(FILTER_LSB, 10); // 0x00-0x0F, default is 0x00
	HAL_Delay(10);
	HMC6343_writeEEPROM(OP_MODE1, 0x31); // Enable filter (default OPMODE1 + filter enable bit set);
	HAL_Delay(10);
	HMC6343_readMag(obj);
	HAL_Delay(50);
	HMC6343_readAccel(obj);
	HAL_Delay(50);
	HMC6343_readHeading(obj);
	HAL_Delay(50);
}

void HMC6343_Print_All_Data(HMC6343_Def_t* obj)
{
	printf("Acceleration in g (x,y,z):\n");
	printf("%.4f   ",obj->accelX/1024.0);
	printf("%.4f   ", obj->accelY/1024.0);
	printf("%.4f \n\n", obj->accelZ/1024.0);
	
	printf("\nRaw Magnetometer Data:\n");
  printf("X: %d   ",obj->magX);
  printf("Y: %d   ",obj->magY);
  printf("Z: %d\n\n",obj->magZ);
	
	printf("Tilt data in Raw and Degrees (x,y,z):\n");
	printf("Heading : %d    %.4f  \n", obj->heading, obj->heading/10.0);
	printf("Pitch :   %d    %.4f  \n", obj->pitch, obj->pitch/10.0);
	printf("Roll :    %d    %.4f  \n\n", obj->roll, obj->roll/10.0);
	printf("********************************************\t\n");	
}


/**********************************************************************************************************************
 *  END OF FILE: HMC6343.c
 *********************************************************************************************************************/
