/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"
#include "bms.h"
#include "can.h"


/* USER CODE BEGIN 0 */
extern uint8_t precharge;
extern uint8_t ts_on;
extern uint8_t ts_start;
uint8_t error = 0;
uint8_t AIR_N_act = 0;
uint8_t AIR_N_int = 0; // SC END
uint8_t AIR_P_act = 0;
uint8_t AIR_P_int = 0; //AIR P Power
uint8_t AIR_OK = 0;
uint8_t ts_ready = 0;
uint8_t sc_closed = 0;


/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
void open_sdc()
{
	HAL_GPIO_WritePin(SC_OPEN_GPIO_Port, SC_OPEN_Pin, GPIO_PIN_RESET);

}

void close_sdc()
{
	HAL_GPIO_WritePin(SC_OPEN_GPIO_Port, SC_OPEN_Pin, GPIO_PIN_SET);

}

uint8_t read_sdc()
{
	// returns 1 if closed
	// returns 0 if open
	return HAL_GPIO_ReadPin(GPIOC, AIR_N_INT_Pin);
}



uint8_t check_AIRs() 		// returns 1 if all AIRs are in their intended state
{


	// 1 if high and 0 if low
	// high = switched

	AIR_N_int = HAL_GPIO_ReadPin(GPIOC, AIR_N_INT_Pin);
	AIR_N_act = HAL_GPIO_ReadPin(GPIOC, AIR_N_ACT_Pin);

	AIR_P_int = HAL_GPIO_ReadPin(GPIOB, AIR_P_INT_Pin);
	AIR_P_act = HAL_GPIO_ReadPin(GPIOB, AIR_P_ACT_Pin);

	if (AIR_N_int == AIR_N_act && AIR_P_int == AIR_P_act)
	{
		AIR_OK = 1;
	}
	else
	{
		AIR_OK = 0;
	}

	//if(ts_start > 0 && ts_on == 0) ts_start = 0;
/*
		 if (ts_on > 0)
		 {

			 HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_SET);
			 ts_ready = 1;
			 HAL_GPIO_WritePin(GPIOC, LED_YW_Pin, GPIO_PIN_RESET);
		 }

		 if (ts_ready > 0 && ts_start > 0)
		 {
			 HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_SET);
			 HAL_GPIO_WritePin(GPIOC, LED_RD_Pin, GPIO_PIN_RESET);
		 }
*/

	 return AIR_OK;
}
void get_ts_ready()
{


	if(read_sdc())//&& precharge)
	{

		if(ts_on)
		{

			 HAL_GPIO_WritePin(GPIOC, LED_YW_Pin, GPIO_PIN_RESET);

			 if(ts_start)
			 		{
				 sc_closed  = 1;
				 ts_ready = 1;
			 			 HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_SET);
			 			HAL_GPIO_WritePin(GPIOC, LED_RD_Pin, GPIO_PIN_RESET);
			 		}
		}
	}

	else
	{
		ts_ready = 0;
		ts_on = 0;
			if (sc_closed ==1)
			{
				ts_on = 0;
				ts_start = 0;
				HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);

				HAL_GPIO_WritePin(AIR_P_SW_GPIO_Port, AIR_P_SW_Pin, GPIO_PIN_RESET);
				sc_closed = 0;
			}
		 //HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);


	}

}
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOC, LED_GN_Pin|LED_YW_Pin|LED_RD_Pin|AIR_P_SW_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, LED_GN_Pin|LED_YW_Pin|LED_RD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_RESET);



  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, WDI_Pin|SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SC_OPEN_GPIO_Port, SC_OPEN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = LED_GN_Pin|LED_YW_Pin|LED_RD_Pin|AIR_P_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = IMD_STATE_Pin|IMD_SAFE_Pin|SC_OPENING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = WDI_Pin|SC_OPEN_Pin|SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SC_CLOSING_Pin|SC_STATE_Pin|AIR_N_ACT_Pin|AIR_N_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = TS_ACTIVATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TS_ACTIVATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = DANGER_V_Pin|AIR_P_ACT_Pin|AIR_P_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PCHRG_ACT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PCHRG_ACT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
