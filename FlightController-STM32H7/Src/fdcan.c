/**********************************************************************************************************************

 *  FILE DESCRIPTION
 *  -------------------------------------------------------------------------------------------------------------------
 *         File:  fdcan.C
 *        Author:
 *		   Date:  Jul 8, 2024
 *  Description:  This is the fdcan source file to init and interface with the CAN driver     
 *  
 *********************************************************************************************************************/


/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.msgRam.StandardFilterSA = 0;
  hfdcan1.msgRam.ExtendedFilterSA = 0;
  hfdcan1.msgRam.RxFIFO0SA = 0;
  hfdcan1.msgRam.RxFIFO1SA = 0;
  hfdcan1.msgRam.RxBufferSA = 0;
  hfdcan1.msgRam.TxEventFIFOSA = 0;
  hfdcan1.msgRam.TxBufferSA = 0;
  hfdcan1.msgRam.TxFIFOQSA = 0;
  hfdcan1.msgRam.TTMemorySA = 0;
  hfdcan1.msgRam.EndAddress = 0;
  hfdcan1.ErrorCode = 0;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 1;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan2.msgRam.StandardFilterSA = 0;
  hfdcan2.msgRam.ExtendedFilterSA = 0;
  hfdcan2.msgRam.RxFIFO0SA = 0;
  hfdcan2.msgRam.RxFIFO1SA = 0;
  hfdcan2.msgRam.RxBufferSA = 0;
  hfdcan2.msgRam.TxEventFIFOSA = 0;
  hfdcan2.msgRam.TxBufferSA = 0;
  hfdcan2.msgRam.TxFIFOQSA = 0;
  hfdcan2.msgRam.TTMemorySA = 0;
  hfdcan2.msgRam.EndAddress = 0;
  hfdcan2.ErrorCode = 0;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration    
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */
    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration    
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
  
    /**FDCAN1 GPIO Configuration    
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }
  
    /**FDCAN2 GPIO Configuration    
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/******************************************************************************
function:	Configuration filter
parameter:
    hfdcan  :   FDCAN_HandleTypeDef Pointer type
******************************************************************************/
uint8_t FDCAN_Mode_Init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef FDCAN1_RXFilter;
	
    FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;
    FDCAN1_RXFilter.FilterIndex=0;
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_RXFilter.FilterID1=0x0000;
    FDCAN1_RXFilter.FilterID2=0x0000;
    if(HAL_FDCAN_ConfigFilter(hfdcan,&FDCAN1_RXFilter)!=HAL_OK) return 1;
    HAL_FDCAN_Start(hfdcan);
    return 0;
}


/******************************************************************************
function:	Send a CAN FD frame of data
parameter:
    hfdcan  :   FDCAN_HandleTypeDef pointer type
    data    :   Data pointer to be sent
    len     :   Length of the data in bytes (up to 64 for CAN FD)
******************************************************************************/
uint8_t FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t* data, uint32_t len, uint8_t sensor_id)
{
	uint32_t mSize = determine_CAN_DLC(len);
    FDCAN_TxHeaderTypeDef FDCAN_Tx;

    FDCAN_Tx.Identifier = sensor_id;                    
    FDCAN_Tx.IdType = FDCAN_STANDARD_ID;            // Standard CAN identifier
    FDCAN_Tx.TxFrameType = FDCAN_DATA_FRAME;         // Data frame type
    FDCAN_Tx.DataLength = mSize;                      // Length of the data in bytes
    FDCAN_Tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // Error State Indicator active
    FDCAN_Tx.BitRateSwitch = FDCAN_BRS_OFF;          // Bit Rate Switch on for CAN FD
    FDCAN_Tx.FDFormat = FDCAN_FD_CAN;               // CAN FD format
    FDCAN_Tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // No Tx Event FIFO control
    FDCAN_Tx.MessageMarker = 0;                     // Message marker (not used in this example)

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN_Tx, data) != HAL_OK) {
        return 1; // Error adding message to Tx FIFO
    }
    
    return 0;   // Success
}


// Function to determine the appropriate CAN DLC based on data size
uint32_t determine_CAN_DLC(uint32_t data_size) {
    if (data_size <= 8) {
        return FDCAN_DLC_BYTES_8;
    } else if (data_size <= 12) {
        return FDCAN_DLC_BYTES_12;
    } else if (data_size <= 16) {
        return FDCAN_DLC_BYTES_16;
    } else if (data_size <= 20) {
        return FDCAN_DLC_BYTES_20;
    } else if (data_size <= 24) {
        return FDCAN_DLC_BYTES_24;
    } else if (data_size <= 32) {
        return FDCAN_DLC_BYTES_32;
    } else if (data_size <= 48) {
        return FDCAN_DLC_BYTES_48;
    } else if (data_size <= 64) {
        return FDCAN_DLC_BYTES_64;
    } else {
        // Handle error case where data size is larger than supported
        // or implement further logic as needed
        return FDCAN_DLC_BYTES_64; // Example fallback to maximum supported size
    }
}


/******************************************************************************
function:	Receive one frame of data
parameter:
		hfdcan  :   DCAN_HandleTypeDef Pointer type
		data		:		Store data pointer
******************************************************************************/
uint8_t FDCAN_Receive_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf) {
    // Receive the CAN message
		FDCAN_RxHeaderTypeDef FDCAN_Rx;
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_Rx, buf) != HAL_OK) {
        return 1; // Error receiving message
    }

    // Extract the sensor ID from the received CAN ID
    uint8_t sensorID = FDCAN_Rx.Identifier;
		char tempBuffer[100]={0};
		memcpy(tempBuffer, buf,  100); 
    // Handle decoding based on the identified sensor
    switch (sensorID) {
        case GY85_ID:
            Decode_GY85_Data(&GY85_Sensor_2, tempBuffer, FDCAN_Rx.DataLength); 
            break;
        case MPU9250_ID:
            // Decode data for Sensor 2 (call your sensor-specific decode function)
						Decode_MPU9250_Data(&MPU9250_Sensor, tempBuffer, FDCAN_Rx.DataLength); 
            break;
        case HMC6343_ID:
           Decode_HMC6343_Data(&HMC6343_Sensor, tempBuffer, FDCAN_Rx.DataLength);
            break;
        case SFE_ID:
            // Decode data for Sensor 4
						Decode_SFE_Data(&SFE_Sensor, tempBuffer, FDCAN_Rx.DataLength); 
            break;
        default:
            // Handle the case of an unknown sensor ID
            break;
    }

    return 0; // Success
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
