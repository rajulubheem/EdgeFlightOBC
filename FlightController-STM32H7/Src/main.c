/**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  main.C
 *        Author:
 *		   Date:  Jul 8, 2024
 *  Description:  --     
 *  
 *********************************************************************************************************************/

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

#include "GY_85.h"
#include "HMC6343.h"
#include "MPU9250.h"
#include "sfe_interface.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "serial_encoder.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GY85_Def_t GY85_Sensor_1;
GY85_Def_t GY85_Sensor_2;

HMC6343_Def_t HMC6343_Sensor;

MPU9250_Def_t MPU9250_Sensor;
sfe_io_t SFE_Sensor;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
	uint8_t I2C_Scan(uint8_t address) {

  HAL_StatusTypeDef result;

  printf("Scanning I2C bus...\r\n");

  // The HAL_I2C_IsDeviceReady function checks if the device at 'address' is ready
  result = HAL_I2C_IsDeviceReady( & hi2c4, (uint16_t)(address << 1), 3, 10);
  if (result == HAL_OK) {
    printf("Device found at address 0x%02X\r\n", address);
    return 1;
  } else if (result == HAL_BUSY) {
    printf("I2C bus is busy\r\n");
  } else if (result == HAL_TIMEOUT) {
    printf("I2C timeout occurred\r\n");
  } else {
    // No response or other errors
  }

  // Add a delay of 10ms between each address check
  HAL_Delay(100);

  printf("........\r\n\n\n");
  return 0;
}
	
char transmit_buffer[100];
char receive_buffer[100];

void Clear_Buffers(void)
{
	//Clear the sending/receiving buffers
		for(int x=0; x<100; x++)
		{
			receive_buffer[x]=0;
			transmit_buffer[x]=0;
		}
}
	
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
	HAL_Delay(500);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_I2C4_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();

  /* USER CODE BEGIN 2 */

	if(FDCAN_Mode_Init(&hfdcan2) !=0){
		printf("FDCAN Configuration Failed \r\n\r\n");
		while(1);
	}
	if(FDCAN_Mode_Init(&hfdcan1) !=0){
		printf("FDCAN Configuration Failed \r\n\r\n");
		while(1);
	}

if (GY85_init(&hi2c4) != HAL_OK) {
    printf("GY_85 does not respond\t\n");
		while(1);
	} 

	if(!SFE_init(&hi2c4)){
		printf("SFE error ....\t\n");
		while(1);
	}
	
	HAL_Delay(50);
	
if (!HMC6343_init(&HMC6343_Sensor)) {
    printf("Error in HMC Init");
    while (1);
  };
	
//	printf("Applying the auto offset settings, please wait...");	
	//HMC6343_autoSetOffset(&HMC6343_Sensor);	
	HAL_Delay(50);

	
	int i=0;
/*while(i<128)
{
	I2C_Scan(i);
	//HAL_Delay(200);
	i++;
}*/

  if (!MPU9250_init()) {
    printf("MPU9250 does not respond\t\n");
		while(1);
  }

	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    
		GY85_Read_All_Data(&hi2c4, &GY85_Sensor_2);
		HMC6343_Read_All_Data(&HMC6343_Sensor);
		MPU9250_Read_All_Data(&MPU9250_Sensor);
		SFE_Read_All_Data(&SFE_Sensor);
		
		
		uint16_t mSize=0;
		
		mSize = Encode_HMC6343_Data(&HMC6343_Sensor, transmit_buffer,  sizeof(transmit_buffer));
		printf("Size = %d\r\n", mSize);
		
		if(FDCAN_Send_Msg(&hfdcan1, (uint8_t*)transmit_buffer, (uint32_t)mSize, HMC6343_ID) != 0){
			printf("Send Failed\r\n");
			while(1);
		}
		
		HAL_Delay(1);
		
		if(FDCAN_Receive_Msg(&hfdcan2,(uint8_t*)receive_buffer)!=0){
			printf("Receive Failed\r\n");
			while(1);
		}
		HAL_Delay(1);
		Clear_Buffers();
		
		mSize = Encode_MPU9250_Data(&MPU9250_Sensor, transmit_buffer,  sizeof(transmit_buffer));
		printf("Size = %d\r\n", mSize);
		
		if(FDCAN_Send_Msg(&hfdcan1, (uint8_t*)transmit_buffer, (uint32_t)mSize, MPU9250_ID) != 0){
			printf("Send Failed\r\n");
			while(1);
		}
		
		HAL_Delay(1);
		
		if(FDCAN_Receive_Msg(&hfdcan2,(uint8_t*)receive_buffer)!=0){
			printf("Receive Failed\r\n");
			while(1);
		}
		
		HAL_Delay(1);
		Clear_Buffers();
		
		mSize = Encode_SFE_Data(&SFE_Sensor, transmit_buffer,  sizeof(transmit_buffer));
		printf("Size = %d\r\n", mSize);
		
		if(FDCAN_Send_Msg(&hfdcan1, (uint8_t*)transmit_buffer, (uint32_t)mSize, SFE_ID) != 0){
			printf("Send Failed\r\n");
			while(1);
		}
		
		HAL_Delay(1);
		
		if(FDCAN_Receive_Msg(&hfdcan2,(uint8_t*)receive_buffer)!=0){
			printf("Receive Failed\r\n");
			while(1);
		}
		HAL_Delay(1);
		Clear_Buffers();
		
		
		mSize = Encode_GY85_Data(&GY85_Sensor_2, transmit_buffer,  sizeof(transmit_buffer));
		printf("Size = %d\r\n", mSize);
		
		if(FDCAN_Send_Msg(&hfdcan1, (uint8_t*)transmit_buffer, (uint32_t)mSize, GY85_ID) != 0){
			printf("Send Failed\r\n");
			while(1);
		}
		
		HAL_Delay(1);
		
		if(FDCAN_Receive_Msg(&hfdcan2,(uint8_t*)receive_buffer)!=0){
			printf("Receive Failed\r\n");
			while(1);
		}
		HAL_Delay(1);
		Clear_Buffers();
		
		//Printing the received GY_85 data via CAN
	  	//printf("///////////////Original///////////:\n");
		printf("///////////////GY_85 Sesnor///////////:\n");
		GY85_Print_All_Data(&GY85_Sensor_2);	
		printf("///////////////HMC6343 Sensor///////////:\n");
	  HMC6343_Print_All_Data(&HMC6343_Sensor);
		printf("///////////////MPU9250 Sensor///////////:\n");
		MPU9250_Print_All_Data(&MPU9250_Sensor);
		printf("///////////////SFE Sensor///////////:\n");
		SFE_Print_All_Data(&SFE_Sensor);
		//Clear the sending/receiving buffers
		for(int x=0; x<100; x++)
		{
			receive_buffer[x]=0;
			transmit_buffer[x]=0;
		}
		HAL_Delay(500);
 
	}	

 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Supply configuration update enable 
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
	  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}








#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
