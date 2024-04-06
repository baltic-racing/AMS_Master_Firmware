/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bms.h"
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
CAN_RxHeaderTypeDef   RxHeader;
CAN_TxHeaderTypeDef   TxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;

int datacheck = 0;
uint8_t sc_error = 0;
uint8_t sc_closed =0;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_CAN_Start(&hcan1);





  	  TxHeader.DLC = 8;
  	  TxHeader.IDE = CAN_ID_STD;
  	  TxHeader.RTR = CAN_RTR_DATA;
  	  TxHeader.StdId = 0x203;




/*
   	uint8_t					RxData[8];
   	uint8_t					TxData[8];
   	uint32_t              	TxMailbox;
   	uint32_t				datacheck;
   	*/
   //	uint32_t now = 0, last_blink = 0, last_tx = 0, error =0, last_rx =0, datacheck =0;


   	TxData[0]= 0;
   	TxData[1]= 0xFF;
   	TxData[2]= 0xFF;
   	TxData[3]= 0xFF;
   	TxData[4]= 0xFF;
   	TxData[5]= 0xFF;
   	TxData[6]= 0xFF;
   	TxData[7]= 0xFF;

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
  	  Error_Handler();
    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	 // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  	  sc_closed =HAL_GPIO_ReadPin(GPIOC,AIR_N_INT_Pin);
	  	  if (sc_closed ==0 && TxData[0] == 1)
	  	  {
	  		  sc_error =1;
	  		 HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_RESET);
	  		 HAL_GPIO_WritePin(GPIOC,AIR_P_SW_Pin, GPIO_PIN_RESET);
	  		 TxData[0]= 0;
	  	  }
	  	 // HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

	  	 if (datacheck)
	  	{
	  	 HAL_GPIO_TogglePin(GPIOC, LED_RD_Pin);
	  	 datacheck =0;

			 if (sc_error == 0)
			 {
				 if (RxData[0]>= 1)
				 {

					 HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_SET);
					 TxData[0] = 1;

				 }

				 if (RxData[1]>= 1)
				 {
					 HAL_GPIO_WritePin(GPIOC,AIR_P_SW_Pin, GPIO_PIN_SET);
				 }
			}
	  	}
	  	 HAL_Delay(100);
	  //now = HAL_GetTick();

	 /* if(now-last_rx >= 20) {
		  if (HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		   {
			  error= HAL_CAN_GetError(&hcan1);
		    // Error_Handler();
		   }

		   if ((RxHeader.StdId == 0x203))
		   {
		 	  datacheck = 1;
		   }
		   last_rx= HAL_GetTick();
	  }
*/
	 // if (now - last_tx >= 100) {
	 //HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		 // uint32_t              	TxMailbox;
	// if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	// {
	//	error = HAL_CAN_GetError(&hcan1);
	   // Error_Handler ();

	// }



	 HAL_GPIO_TogglePin(GPIOA, WDI_Pin);
	 HAL_GPIO_TogglePin(GPIOC, LED_GN_Pin);
	//  }


  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  __disable_irq();
  while (1)
  {

  }

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
