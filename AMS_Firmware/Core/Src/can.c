/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "gpio.h"

extern uint8_t AMS0_databytes[8];
extern uint8_t AMS1_databytes[8];
uint8_t DIC0_databytes[8];
uint8_t ts_ready = 0;
uint8_t test[8];
uint32_t current_data = 0;
uint16_t current = 0;



/* {StdId, ExtId, IDE, RTR, DLC}
 * uint32_t StdId;   Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF.

  uint32_t ExtId;    Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF.

  uint32_t IDE;      Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_identifier_type

  uint32_t RTR;      Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request

  uint32_t DLC;

 */
// Header from DBC
CAN_TxHeaderTypeDef AMS0_header = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8};
CAN_TxHeaderTypeDef AMS1_header = {0x201, 0, CAN_ID_STD, CAN_RTR_DATA, 8};

CAN_TxHeaderTypeDef test_header = {0x069, 0 , CAN_ID_STD, CAN_RTR_DATA, 8};


	// transmit CAN Message
void CAN_TX(CAN_HandleTypeDef hcan, CAN_TxHeaderTypeDef TxHeader, uint8_t* TxData)
{
	uint32_t TxMailbox;
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_TX(hcan, TxHeader, TxData);		//retry when failed
	}
}

void CAN_TX_IVT(CAN_HandleTypeDef hcan, CAN_TxHeaderTypeDef TxHeader, uint8_t* TxData)
{
	uint32_t TxMailbox2;
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox2) != HAL_OK)
	{
		CAN_TX_IVT(hcan, TxHeader, TxData);		//retry when failed
	}
}

	// receive CAN Message
void CAN_RX(CAN_HandleTypeDef hcan)
{

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];


	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{

	}

	if( RxHeader.StdId == 0x500)
	{
		/*
		if(RxData[0] > 0)
		{
			ts_ready = 1;
		}


		*/

		AMS0_databytes[6]|= (AIR_Logic(RxData[0], ts_ready, RxData[1]) << 3);
		//AMS0_databytes[6]|= (ts_ready << 3);
	}

	// hier kann man weitere Nachrichten zum Empfangen hinzufügen

}

void CAN_RX_IVT(CAN_HandleTypeDef hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[6];

	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{

	}
	current_data = 0;

	if(RxHeader.StdId == 0x521)
	{

		//hier muss Strom ausgewertet werden
/*
		for(uint8_t i = 0; i < 5; i++)
		{
			current_data |= (RxData[5-i] << (i*8));
		}
*/
		current_data = RxData[5] | (RxData[4] << (1*8)) | (RxData[3] << (2*8)) | (RxData[2] << (3*8));

		current = current_data/100;

		AMS0_databytes[2] = current;
		AMS0_databytes[3] = (current>>8);
	}


}

void CAN_50(uint8_t precharge_data[])		// CAN Messages transmitted with 50 Hz
{

	CAN_TX(hcan1, AMS0_header, precharge_data);
	test[0] = 1;
	CAN_TX_IVT(hcan2,test_header, test);

	//CAN_TX(hcan2, test_header, precharge_data);
}

void CAN_10(uint8_t bms_data[])		// CAN Messages transmitted with 10 Hz
{
	CAN_TX(hcan1, AMS1_header, bms_data);
	//CAN_TX_IVT(hcan2, test_header, bms_data);
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */


  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x500<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x7FF<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 8;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  CAN_FilterTypeDef canfilterconfig_ivt;

  canfilterconfig_ivt.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig_ivt.FilterBank = 14;  // which filter bank to use from the assigned ones
  canfilterconfig_ivt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig_ivt.FilterIdHigh = 0x521<<5;
  canfilterconfig_ivt.FilterIdLow = 0;
  canfilterconfig_ivt.FilterMaskIdHigh = 0x7FF<<5;
  canfilterconfig_ivt.FilterMaskIdLow = 0x0000;
  canfilterconfig_ivt.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig_ivt.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig_ivt.SlaveStartFilterBank = 14;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig_ivt);
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*
uint32_t CAN_Error( CAN_HandleTypeDef *hcan)
{
	can_error = HAL_CAN_GetError(hcan);
	return can_error;
}
*/
/* USER CODE END 1 */
