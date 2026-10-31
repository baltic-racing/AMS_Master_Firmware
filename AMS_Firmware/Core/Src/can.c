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
#include "adc.h"
#include "bms.h"
#include "define.h"

#include <stdint.h>
#include <string.h>
#include <limits.h>

/* Messwerte werden bereits in bms.c angelegt. */
extern uint16_t cellVoltages[NUM_CELLS];
extern uint16_t temperature[NUM_CELLS];
extern int16_t ltcTemps_c10[NUM_STACK];

/* Dieses Nachrichtenformat setzt 144 Kanaele und 12 LTCs voraus. */
#if (NUM_CELLS != 144) || (NUM_STACK != 12)
#error "CAN-Detailformat an die Anzahl der Messwerte anpassen"
#endif

/*
 * CAN-IDs 0x300 bis 0x320:
 * Vor Verwendung gegen eure gesamte CAN-Belegung pruefen!
 */
#define DETAIL_ID_BASE          0x300U

#define DETAIL_PERIOD_MS        1000U
#define DETAIL_FRAME_GAP_MS        15U

#define DETAIL_DATA_BYTES        264U
#define DETAIL_FRAME_COUNT        33U

#define DETAIL_CELL_TEMP_OFFSET  144U
#define DETAIL_LTC_TEMP_OFFSET   252U

static uint8_t detail_data[DETAIL_DATA_BYTES];

static uint32_t detail_cycle_time = 0U;
static uint32_t detail_last_frame_time = 0U;

static uint8_t detail_frame = DETAIL_FRAME_COUNT;
static uint8_t detail_started = 0U;


/*
 * Spannung:
 * Eingang: 0,1 mV pro Schritt.
 *
 * Ausgang:
 *   0   = 2,00 V
 *   1   = 2,01 V
 *   ...
 *   254 = 4,54 V
 *   255 = ungueltig / ausserhalb des Bereichs
 */
static uint8_t Detail_EncodeVoltage(uint16_t raw)
{
    if (raw < 20000U || raw > 45400U)
    {
        return 255U;
    }

    /* Auf 10 mV runden, Offset von 2,00 V abziehen. */
    return (uint8_t)(
        ((uint32_t)raw - 20000U + 50U) / 100U
    );
}


/*
 * Zelltemperatur:
 * Eingang: bestehendes temperature[] in Milligrad Celsius.
 *
 * Ausgang:
 *   0  = -20 Grad C
 *   1  = -18 Grad C
 *   ...
 *   62 = 104 Grad C
 *   63 = ungueltig
 *
 * Achtung:
 * Das bestehende uint16_t-Array kann negative Temperaturen
 * und Temperaturen ueber ca. 65,5 Grad C nicht sauber speichern.
 * Dieser Adapter uebernimmt die vorhandenen nichtnegativen Werte.
 */
static uint8_t Detail_EncodeCellTemperature(uint16_t raw)
{
    if (raw == 0xFFFFU || raw == 0xFFFEU)
    {
        return 63U;
    }

    /* Auf 2 Grad C runden, dann Offset hinzufuegen. */
    return (uint8_t)(
        ((uint32_t)raw + 1000U) / 2000U + 10U
    );
}


/*
 * LTC-Temperatur:
 * Eingang: 0,1 Grad C pro Schritt.
 *
 * Ausgang:
 *   0   = -40 Grad C
 *   ...
 *   254 = 214 Grad C
 *   255 = ungueltig / ausserhalb des Bereichs
 */
static uint8_t Detail_EncodeLTCTemperature(int16_t raw)
{
    if (raw == INT16_MAX || raw == INT16_MIN)
    {
        return 255U;
    }

    int32_t shifted = (int32_t)raw + 400;

    if (shifted < 0 || shifted > 2540)
    {
        return 255U;
    }

    /* Auf ganze Grad C runden. */
    return (uint8_t)((shifted + 5) / 10);
}


/*
 * Einen festen Sendepuffer aus den aktuellen Messwerten aufbauen.
 *
 * Byte   0..143: Zellspannungen
 * Byte 144..251: gepackte Zelltemperaturen
 * Byte 252..263: LTC-Temperaturen
 */
static void Detail_BuildSnapshot(void)
{
    memset(detail_data, 0, sizeof(detail_data));

    /* 144 Zellspannungen mit je 8 Bit. */
    for (uint16_t i = 0U; i < NUM_CELLS; i++)
    {
        detail_data[i] =
            Detail_EncodeVoltage(cellVoltages[i]);
    }

    /* Vier Zelltemperaturen mit je 6 Bit in drei Bytes packen. */
    for (uint16_t i = 0U; i < NUM_CELLS; i += 4U)
    {
        uint32_t t0 =
            Detail_EncodeCellTemperature(temperature[i]);

        uint32_t t1 =
            Detail_EncodeCellTemperature(temperature[i + 1U]);

        uint32_t t2 =
            Detail_EncodeCellTemperature(temperature[i + 2U]);

        uint32_t t3 =
            Detail_EncodeCellTemperature(temperature[i + 3U]);

        uint32_t packed = t0
                        | (t1 << 6)
                        | (t2 << 12)
                        | (t3 << 18);

        uint16_t offset =
            DETAIL_CELL_TEMP_OFFSET + (i / 4U) * 3U;

        detail_data[offset] =
            (uint8_t)packed;

        detail_data[offset + 1U] =
            (uint8_t)(packed >> 8);

        detail_data[offset + 2U] =
            (uint8_t)(packed >> 16);
    }

    /* Zwoelf LTC-Temperaturen mit je 8 Bit. */
    for (uint8_t i = 0U; i < NUM_STACK; i++)
    {
        detail_data[DETAIL_LTC_TEMP_OFFSET + i] =
            Detail_EncodeLTCTemperature(ltcTemps_c10[i]);
    }
}


/*
 * Regelmaessig aus der Hauptschleife aufrufen.
 *
 * Ein neuer Datensatz wird alle 1000 ms vorbereitet.
 * Pro Aufruf wird hoechstens eine Nachricht eingereiht.
 * Bei belegten CAN-Mailboxen wird spaeter erneut versucht.
 */
void CAN_Detail_Task(void)
{
    uint32_t now = HAL_GetTick();

    /* Nach dem ersten Aufruf zunaechst eine Sekunde warten. */
    if (detail_started == 0U)
    {
        detail_started = 1U;
        detail_cycle_time = now;
        return;
    }

    if ((uint32_t)(now - detail_cycle_time)
        >= DETAIL_PERIOD_MS)
    {
        detail_cycle_time = now;

        /*
         * Ein eventuell unvollstaendiger alter Sendezyklus
         * wird durch aktuelle Daten ersetzt.
         */
        Detail_BuildSnapshot();
        detail_frame = 0U;

        detail_last_frame_time =
            now - DETAIL_FRAME_GAP_MS;
    }

    /* Alle 33 Nachrichten bereits eingereiht? */
    if (detail_frame >= DETAIL_FRAME_COUNT)
    {
        return;
    }

    /* Mindestabstand zwischen den Detailnachrichten. */
    if ((uint32_t)(now - detail_last_frame_time)
        < DETAIL_FRAME_GAP_MS)
    {
        return;
    }

    /* Eine Mailbox fuer andere Nachrichten freihalten. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 2U)
    {
        return;
    }

    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = DETAIL_ID_BASE + detail_frame;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    uint16_t offset = (uint16_t)detail_frame * 8U;

    if (HAL_CAN_AddTxMessage(
            &hcan1,
            &header,
            &detail_data[offset],
            &mailbox) == HAL_OK)
    {
        detail_frame++;
        detail_last_frame_time = now;
    }
}

extern uint8_t AMS0_databytes[8];
extern uint8_t AMS1_databytes[8];
extern uint8_t precharge;
extern  uint16_t adc_accu_volt;
extern uint16_t imdStatValue ;
uint8_t DIC0_databytes[8];
uint8_t dc_current[8];
uint32_t current_data = 0;
uint16_t current = 0;
uint8_t ts_on = 0;
uint8_t ts_start = 0;
uint8_t charging = 1;
uint8_t ams_status = 0;
uint8_t switch_on = 0;
uint32_t capacity_data = 0;

extern uint16_t ts_volt_can;


extern uint8_t ts_ready;
extern uint8_t IMD_ERROR_CAN;
extern uint8_t AMS_ERROR;

uint32_t ivt_error_time = 0;


#define STOP 0
#define RUN 1

#define IVT_MSG_RESULT_T 4
#define IVT_MSG_RESULT_W 5
#define IVT_MSG_RESULT_As 6
#define IVT_MSG_RESULT_Wh 7



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
CAN_TxHeaderTypeDef AMS2_header = {0x210, 0 , CAN_ID_STD, CAN_RTR_DATA, 8};

CAN_TxHeaderTypeDef AMS_DIAG_header = {
    .StdId = 0x204,
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef IMD_DIAG_header = {
    .StdId = 0x205,
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE
};

CAN_TxHeaderTypeDef IVT_MSG_COMMAND = {0x411, 0,CAN_ID_STD, CAN_RTR_DATA,8};



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

	if( RxHeader.StdId == 0x500)		// Buttons on DIC
	{
		if((RxData[0]& 1) == 1)				// close SC
		{
			ts_on = 1;
			switch_on = 1;
		}

		charging = (RxData[0]>>7);

		// verhindert das Drücken in falscher Reihenfolge
		//if(RxData[1] == 1 && ts_on == 1)		// AIRP
		//{
		//	ts_start = 1;
		//}
		//else
		//{
		//	ts_start = 0;
		//}

		//if(RxData[5] == 1)				// for use in Charger
		//{
			//charging = 1;
		//	ts_on = 1;
		//	switch_on = 1;
			//switch_stamp = HAL_GetTick();
			//HAL_GPIO_WritePin(TS_ACTIVATE_GPIO_Port, TS_ACTIVATE_Pin, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(GPIOC, AIR_P_SW_Pin, GPIO_PIN_SET);
		//}
	}
	/*
	if(ts_on){
		if(readSDC() = 0 && (HAL_GetTick()>= (switch_stamp + TSON_RESET_TIME))){

		ts_on = 0;
		switch_on = 0;
		charging = 0;
	}
	*/


	// hier kann man weitere Nachrichten zum Empfangen hinzufügen
}

void IVT_MODE(uint8_t mode)
{
	uint8_t data[8];

	data[0] = 0x34;
	data[1] = mode;
	data[2] = 0x01;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;

	CAN_TX_IVT(hcan2,IVT_MSG_COMMAND,data);
}

void IVT_ACTIVATE(uint8_t channel)
{
	uint8_t data [8];

	data[0] = 0x20 | channel;
	data[1] = 0x02;
	data[2] = 0x00;
	data[3] = 0x14;
	data[4] = 0x00;
	data[5] = 0x00;
	data[6] = 0x00;
	data[7] = 0x00;

	CAN_TX_IVT(hcan2,IVT_MSG_COMMAND,data);
}
void IVT_init()
{
	HAL_Delay(1000);
	IVT_MODE(STOP);

	HAL_Delay(100);
	IVT_ACTIVATE(IVT_MSG_RESULT_As);

	HAL_Delay(100);
	IVT_ACTIVATE(IVT_MSG_RESULT_W);

	HAL_Delay(100);
	IVT_MODE(RUN);


}

void can_put_data()
{
	AMS0_databytes[0] = ts_volt_can;
	AMS0_databytes[1] = (ts_volt_can >> 8);
	AMS0_databytes[2] = current;
	AMS0_databytes[3] = (current >> 8);
	AMS0_databytes[4] = imdStatValue;
	AMS0_databytes[5] = (imdStatValue>>8);
	AMS0_databytes[6] =  0  | (ts_ready << 3) | (precharge << 4) | (IMD_ERROR_CAN << 6) | (AMS_ERROR << 7);
	AMS0_databytes[7] = ams_status;
}

/*
 * Zusatzdaten fuer CAN-ID 0x204.
 * Anfangszustand:
 * Temperaturen = INT16_MIN (ungueltig)
 * Stacknummern = 0 (ungueltig)
 * Anzahl gueltiger Werte = 0
 */
static volatile uint8_t ltc_diag_data[7] = {
    0x00, 0x80,
    0x00, 0x80,
    0, 0, 0
};

void CAN_UpdateLTCDiagnostic(int16_t temp_min_c10,
                             int16_t temp_max_c10,
                             uint8_t stack_min,
                             uint8_t stack_max,
                             uint8_t valid_count)
{
    uint16_t min_bits = (uint16_t)temp_min_c10;
    uint16_t max_bits = (uint16_t)temp_max_c10;

    /*
     * Zusammengehoerigen Datensatz veroeffentlichen.
     * Der CAN-Interrupt darf nicht zwischen den Bytes lesen.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ltc_diag_data[0] = (uint8_t)(min_bits & 0xFFU);
    ltc_diag_data[1] = (uint8_t)(min_bits >> 8);
    ltc_diag_data[2] = (uint8_t)(max_bits & 0xFFU);
    ltc_diag_data[3] = (uint8_t)(max_bits >> 8);
    ltc_diag_data[4] = stack_min;
    ltc_diag_data[5] = stack_max;
    ltc_diag_data[6] = valid_count;

    __set_PRIMASK(primask);
}

void CAN_SendIMDCode(uint8_t code)
{
    uint8_t data[8] = {0};
    uint32_t mailbox;

    data[0] = code;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        (void)HAL_CAN_AddTxMessage(
            &hcan1,
            &IMD_DIAG_header,
            data,
            &mailbox
        );
    }
}

void CAN_RX_IVT(CAN_HandleTypeDef hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[6];

	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{

	}
	current_data = 0;

	if(RxHeader.StdId == 0x521)		// Current mA
	{
		current_data = RxData[5] | (RxData[4] << (1*8)) | (RxData[3] << (2*8)) | (RxData[2] << (3*8));

		if(RxData[2] >> 7 == 0)
		{
			//current = (current_data << 1 >> 1)/100;
			current = current_data/100;
		}
		else
		{
			current = ~current_data/100;
		}
		//current = current_data/100;

		ivt_error_time = HAL_GetTick();

		dc_current[4] = RxData[2];
		dc_current[5] = RxData[3];
		dc_current[6] = RxData[4];
		dc_current[7] = RxData[5];

	}
	if(RxHeader.StdId == 0x527)		// Capacity As
	{
		capacity_data = RxData[5] | (RxData[4] << (1*8)); //| (RxData[3] << (2*8)) | (RxData[2] << (3*8));

		AMS0_databytes[4] = RxData[4];
		AMS0_databytes[5] = RxData[5];
	}
}

void CAN_50(uint8_t precharge_data[])		// CAN Messages transmitted with 50 Hz
{

	CAN_TX(hcan1, AMS0_header, precharge_data);


	ams_status++;

	if(ams_status == 255)
	{
		ams_status = 0;
	}
}

void CAN_10(uint8_t bms_data[])		// CAN Messages transmitted with 10 Hz
{
	CAN_TX(hcan1, AMS1_header, bms_data);
	CAN_TX(hcan1, AMS2_header, dc_current);
	//send_usb();

	//get_ts_ready();
}

/*
void CAN_SendDiagnostic(uint8_t code)
{
    uint32_t mailbox;
    uint8_t data[8] = {0};

    data[0] = code;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        (void)HAL_CAN_AddTxMessage(
            &hcan1,
            &AMS_DIAG_header,
            data,
            &mailbox
        );
    }
}
*/
void CAN_SendDiagnostic(uint8_t code)
{
    uint32_t mailbox;
    uint8_t data[8];

    /* Byte 0: bestehender AMS-Fehlercode */
    data[0] = code;

    /* Byte 1 bis 7: zusammengehoerige LTC-Diagnosedaten */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint8_t i = 0; i < 7U; i++)
    {
        data[i + 1U] = ltc_diag_data[i];
    }

    __set_PRIMASK(primask);

    /* Bei belegten Mailboxen nicht blockieren. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
    {
        (void)HAL_CAN_AddTxMessage(
            &hcan1,
            &AMS_DIAG_header,
            data,
            &mailbox
        );
    }
}

void CAN_Recovery_Task()
{
	static uint32_t last_check = 0;
	if (HAL_GetTick() - last_check < 1000) return;
	last_check = HAL_GetTick();

	// CAN1 Recovery
	if (hcan1.Instance->ESR & CAN_ESR_BOFF)
	{
		HAL_CAN_Stop(&hcan1);
		HAL_CAN_Start(&hcan1);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}

	// CAN2 Recovery
	if (hcan2.Instance->ESR & CAN_ESR_BOFF)
	{
		HAL_CAN_Stop(&hcan2);
		HAL_CAN_Start(&hcan2);
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
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
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  //hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
 // hcan1.Init.AutoWakeUp = DISABLE;
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
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  //hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  //hcan2.Init.AutoWakeUp = DISABLE;
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

  CAN_FilterTypeDef canfilterconfig_ivt1;
  canfilterconfig_ivt1.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig_ivt1.FilterBank = 15;  // which filter bank to use from the assigned ones
    canfilterconfig_ivt1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig_ivt1.FilterIdHigh = 0x527<<5;
    canfilterconfig_ivt1.FilterIdLow = 0;
    canfilterconfig_ivt1.FilterMaskIdHigh = 0x7FF<<5;
    canfilterconfig_ivt1.FilterMaskIdLow = 0x0000;
    canfilterconfig_ivt1.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig_ivt1.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig_ivt1.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig_ivt1);


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
