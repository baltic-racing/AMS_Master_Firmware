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

/* USER CODE BEGIN 0 */

#include "define.h"
#include "can.h"

extern uint8_t precharge;
extern uint8_t ts_on;
extern uint8_t ts_start;
extern uint8_t charging;
extern uint8_t switch_on;
extern uint8_t dc_current[8];

uint8_t error = 0;
uint8_t AIR_N_act = 0;
uint8_t AIR_N_int = 0; // SC END
uint8_t AIR_P_act = 0;
uint8_t AIR_P_int = 0; //AIR P Power
uint8_t AIR_OK = 0;
uint8_t ts_ready = 0;
uint8_t sc_state = 0;
uint32_t precharge_time = 0;
uint8_t precharge_check = 0;

uint8_t AIR_N_check = 0;

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/*
void open_sdc()
{
	HAL_GPIO_WritePin(SC_OPEN_GPIO_Port, SC_OPEN_Pin, GPIO_PIN_RESET);

}
*/

void close_sdc()
{
	HAL_GPIO_WritePin(SC_OPEN_GPIO_Port, SC_OPEN_Pin, GPIO_PIN_SET);

}

uint8_t read_sdc()
{
	// returns 1 if closed
	// returns 0 if open
	//return HAL_GPIO_ReadPin(GPIOC, AIR_N_INT_Pin);
	return (HAL_GPIO_ReadPin(GPIOC, AIR_N_INT_Pin) == GPIO_PIN_SET) ? 1 : 0;
}



uint8_t check_AIRs() 		// returns 1 if all AIRs are in their intended state
{
	// 1 if high and 0 if low
	// high = switched

	AIR_N_int = HAL_GPIO_ReadPin(GPIOC, AIR_N_INT_Pin);
	AIR_N_act = HAL_GPIO_ReadPin(GPIOC, AIR_N_ACT_Pin);

	AIR_P_int = HAL_GPIO_ReadPin(GPIOB, AIR_P_INT_Pin);
	AIR_P_act = HAL_GPIO_ReadPin(GPIOB, AIR_P_ACT_Pin);

	dc_current[3] = AIR_P_act;

	if (AIR_N_int == AIR_N_act && AIR_P_int == AIR_P_act)
	{
		AIR_OK = 1;
	}
	else
	{
		AIR_OK = 0;
	}

	 return AIR_OK;
}


void get_ts_ready()
{
	if(!switch_on)
			{
				precharge_time = HAL_GetTick();
			}

	if(ts_on == 1 && check_AIRs() == 1)
	{
		//HAL_GPIO_WritePin(GPIOC, LED_YW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_SET);

		if(HAL_GetTick() - precharge_time >= switch_time) //AIR switching time
		{

			if(read_sdc())
			{
				//delay checking precharge 1 sec for static measurements
				if(HAL_GetTick() - precharge_time >= precharge_check_time )//&& precharge)//&&!ts_ready
				{
					ts_ready = 1;
					HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC, LED_YW_Pin, GPIO_PIN_SET);
				}
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, LED_YW_Pin, GPIO_PIN_RESET);
				ts_ready = 0;
				ts_on = 0;
				switch_on = 0;
				if(!switch_on)
				{
					precharge_time = HAL_GetTick();
				}

				HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);
				precharge = 0;
			}
		}

	}


}

/*
void get_ts_ready()
{

	if(!(switch_on == 1))
	{
		precharge_time = HAL_GetTick();
	}

	if(ts_on == 1 && check_AIRs() == 1 && sc_state == 0)
	{
		HAL_GPIO_WritePin(GPIOC, LED_YW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_SET);

		//delay checking precharge 1 sec for static measurements
		if(HAL_GetTick() - precharge_time >= precharge_check_time)
		{
			precharge_check = 1;
			switch_on = 0;
		}

		if(precharge_check == 0 && ts_start == 1)
		{
			ts_start = 0;
		}


		if(HAL_GetTick() - precharge_time >= TSON_RESET_TIME)
		{
			AIR_N_check = 1;
		}

		if(AIR_N_check)
		{
			if(read_sdc() == 0)
			{
				ts_ready = 0;
				ts_on = 0;
				ts_start = 0;
				precharge_check = 0;
				//charging = 0;
				HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);
			}
			AIR_N_check = 0;
		}



		if(precharge_check == 1 && precharge == 1)//&& precharge)
		{
			if(read_sdc() == 0)
			{
				ts_ready = 0;
				ts_on = 0;
				ts_start = 0;
				precharge_check = 0;
				//charging = 0;
				HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);
			}
			else
			{
				if(charging == 1)
				{
					HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOC, LED_RD_Pin, GPIO_PIN_RESET);
				}

				if(ts_start == 1)
				{
					ts_ready = 1;
					HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOC, LED_RD_Pin, GPIO_PIN_RESET);
				}

			}
		}

	}
	else
	{
		ts_ready = 0;
		ts_on = 0;
		ts_start = 0;
		HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);
	}

}
*/
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
  HAL_GPIO_WritePin(GPIOC, LED_GN_Pin|LED_YW_Pin|LED_RD_Pin|SC_STATE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SC_OPEN_Pin|SPI3_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AIR_P_SW_GPIO_Port, AIR_P_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin */
  GPIO_InitStruct.Pin = LED_GN_Pin|LED_YW_Pin|LED_RD_Pin|SC_STATE_Pin
                          |AIR_P_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = WDI_Pin|SC_OPEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = IMD_SAFE_Pin|SC_OPENING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SC_CLOSING_Pin|AIR_N_ACT_Pin|AIR_N_INT_Pin;
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
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PCHRG_ACT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PCHRG_ACT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
