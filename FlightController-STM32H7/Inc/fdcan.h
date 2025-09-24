/**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  fdcan.h
 *        Author:
 *		   Date:  Jul 8, 2024
 *  Description:  This is the fdcan header file    
 *  
 *********************************************************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fdcan_H
#define __fdcan_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "serial_encoder.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

	 
uint8_t FDCAN_Mode_Init(FDCAN_HandleTypeDef *hfdcan);
uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t* data, uint32_t len, uint8_t sensor_id);
uint32_t determine_CAN_DLC(uint32_t data_size);
uint8_t FDCAN_Receive_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);
	 
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ fdcan_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
