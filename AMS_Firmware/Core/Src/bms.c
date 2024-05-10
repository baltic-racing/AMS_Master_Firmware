/*
 * bms.c
 *
 *  Created on: Feb 23, 2024
 *      Author: Admin
 */

#include "bms.h"
#include "LTC6811.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "can.h"
#include "adc.h"
#include "usbd_cdc_if.h"
#include "string.h"


#define NUM_STACK 1						 //total slaves
#define NUM_CELLS_STACK 12					 //Cells per stack Attention LTC6811 CH all modus
#define NUM_GPIO_STACK 6					 //GPIOs per slave
#define NUM_CELLS NUM_CELLS_STACK *NUM_STACK //Cells per accu container
#define NUM_GPIO NUM_GPIO_STACK *NUM_STACK   //GPIOs per slave


#define MAX_VOLTAGE 39000					// Wert in 0,1 mV
#define MIN_VOLTAGE 30000					// es gehen nur Vielfache von 16

#define MAX_TS_VOLTAGE 554
#define MIN_TS_VOLTAGE 343
/*
#define CYCLE_PERIOD 30 //bms cycle period in ms

//Timeouts in ms
#define VOLT_TIMEOUT (500 - CYCLE_PERIOD)
#define CUR_TIMEOUT (500 - CYCLE_PERIOD)
#define TEMP_TIMEOUT (1000 - CYCLE_PERIOD)
#define LTC_TIMEOUT (100 - CYCLE_PERIOD)
#define ISA_TIMEOUT (100 - CYCLE_PERIOD)

//Cell limits
#define CELL_TEMP_MIN_CHARGE 0			//in [°C]
#define CELL_TEMP_MIN_DISCHARGE -10		//in [°C]
#define CELL_TEMP_MAX_CHARGE 43			//in [°C]
#define CELL_TEMP_MAX_DISCHARGE 57		//in [°C]
#define CELL_VOLT_MIN 3100 * 10			//in 0.1[mV]
#define CELL_VOLT_MAX 42000 * 10		//in 0.1[mV]
#define ACCU_CUR_DIS_MAX 70 * 2 * 1000  //in [mA]
#define ACCU_CUR_CHG_MAX 139 * 2 * 1000 //in [mA]

*/


#define FAIL_TIMEOUT_ISA (1 << 0)
#define FAIL_TIMEOUT_LTC (1 << 1)
#define FAIL_OV (1 << 2)
#define FAIL_UV (1 << 3)
#define FAIL_OCD (1 << 4)
#define FAIL_OCC (1 << 5)
#define FAIL_OT (1 << 6)
#define FAIL_UT (1 << 7)

uint8_t failureState = 0;

uint8_t precharge = 0;

bool balancing = false;

uint16_t balanceMargin = 500; //in 0.1mV


uint16_t cellVoltages[NUM_CELLS] = {0};	//cell voltages in 0.1[mV]
int16_t cellTemperatures[NUM_CELLS] = {0}; //cell temperatures in 0.1[°C]
uint8_t cfg[NUM_STACK][6] = {{0}}; //0x38 disables the GPIO1..3 pulldown so GPIO1..3 can be used for measurement
uint16_t slaveGPIOs[NUM_GPIO] = {0};
uint16_t temperature[NUM_CELLS] = {0};

uint8_t usb_data[NUM_CELLS*2 + 1] = {0};
uint8_t usb_voltages[NUM_CELLS_STACK*NUM_STACK] = {0};
uint8_t usb_temperatures[NUM_CELLS_STACK*NUM_STACK] = {0};

uint8_t AMS0_databytes[8];
uint8_t AMS1_databytes[8];


uint16_t OV_flag[NUM_STACK];
uint16_t UV_flag[NUM_STACK];
uint8_t r_statb[NUM_STACK][6];

uint32_t can_cnt = 0; //can counter to adjust timings
uint64_t last10 =0;
uint64_t last100;
/* 1 ms interrupt
 * HLCK 96 MHz
 * APB1 48 MHz
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

CAN_interrupt();
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
   {
    CAN_RX(hcan1);
   }


void BMS_init()
{
	LTC6811_initialize();
}

void BMS()		// Battery Management System function for main loop.
{
	uint8_t pec = 0;
	static uint8_t selTemp = 0;
	//uint16_t VOV = MAX_VOLTAGE/16;					// Formeln aus Datenblatt S.65
	//uint16_t VUV = (MIN_VOLTAGE/16)-1;

	//precharge = 1 when complete and 0 when still charging
	precharge = ADC_TS_Voltage(MAX_TS_VOLTAGE, MIN_TS_VOLTAGE);

	for (uint8_t i = 0; i < NUM_STACK; i++)
	{
		//Balancing with flags
/*
		cfg[i][0] = 0x3C | ((selTemp << 6) & 0xC0);		//cfg : Databytes in config register of the LTC6811
		cfg[i][1] = 0x00 | VUV;
		cfg[i][2] = 0x00 | (VOV<<4) | (VUV>>4);
		cfg[i][3] = 0x00 | (VOV>>4);
		cfg[i][4] = 0x00 | OV_flag[i];
		cfg[i][5] = 0x00 | (OV_flag[i]>>8);

*/
		//Balancing without flags
		cfg[i][0] = 0x3C | ((selTemp << 6) & 0xC0);		//cfg : Databytes in config register of the LTC6811
		cfg[i][1] = 0x00;
		cfg[i][2] = 0x00;
		cfg[i][3] = 0x00;
		cfg[i][4] = 0x00;
		cfg[i][5] = 0x00;

		if(balancing)
		{
			if(selTemp < 3)
			{
				for(uint8_t j = 0; j < 8; j++)
				{
					if(cellVoltages[i * NUM_STACK + j] - MAX_VOLTAGE > balanceMargin)cfg[i][4] |= 1 << j;
				}
				for(uint8_t j = 0; j < 3; j++)
				{
					if(cellVoltages[i * NUM_STACK + j + 8] - MAX_VOLTAGE > balanceMargin)cfg[i][5] |= 1 << j;
				}
			}
		}

	}

	/*
	LTC6811_clrstat();
	HAL_Delay(3);
*/

	LTC6811_wrcfg(NUM_STACK, (uint8_t(*)[6])cfg);		// Write config
	HAL_Delay(3);

	//wakeup_idle();									// read config
	//LTC6811_rdcfg();
	//HAL_Delay(3);

	LTC6811_adcv();										// measure voltages
	HAL_Delay(3);

	pec += LTC6811_rdcv(0, NUM_STACK, (uint16_t(*)[12])cellVoltages);	//read voltages
	HAL_Delay(3);

	LTC6811_adax();										// measure 3 celltemp
	HAL_Delay(3);

	pec += LTC6811_rdaux(0, NUM_STACK, (uint16_t(*)[6])slaveGPIOs);	// read celltemp
	HAL_Delay(3);

	//CAN_interrupt();

	//pec += LTC6811_rdstatb(NUM_STACK, OV_flag, UV_flag, r_statb);
	//HAL_Delay(3);


	convertVoltage();

	convertTemperature(selTemp);


/*
	wakeup_idle();
	LTC6811_adstat();
	HAL_Delay(3);

	wakeup_idle();
	LTC6811_rdstat();
*/

	if (selTemp < 3)		// Variable for cycling the multiplexers for temp measurement.
	{
		selTemp++;

		//CAN_interrupt();
	}


	else
		selTemp = 0;


	send_usb();
}

void convertVoltage()		//convert and sort Voltages
{
	//double voltage[NUM_CELLS];


	for(uint8_t i = 0; i < NUM_CELLS; i++)
	{
		usb_voltages[i] = cellVoltages[i]/1000;
	}

	//uint16_t cell_max = 42890;
	//uint16_t cell_min = 37789;

	uint16_t cell_max = cellVoltages[0];
	uint16_t cell_min = cellVoltages[0];
	for(uint8_t k = 0; k < NUM_STACK; k++)
	{
		for(uint8_t i = 0; i < NUM_CELLS_STACK; i++)
		{
			if(cellVoltages[i + k * 12] > cell_max) cell_max = cellVoltages[i + k * 12];
			else if(cellVoltages[i + k * 12] < cell_min) cell_min = cellVoltages[i + k * 12];

			//voltage[i + k * 12] = (double)cellVoltages[i + k * 12]/10000;
			//printf(" Stack %d Cell %d = %.4f V \r\n", k, i, voltage[i + k * 12]);
		}
	}

	AMS1_databytes[0] = cell_min;
	AMS1_databytes[1] = (cell_min >> 8);
	AMS1_databytes[2] = cell_max;
	AMS1_databytes[3] = (cell_max >> 8);



}


uint16_t calculateTemperature(uint16_t voltageCode, uint16_t referenceCode)		//convert temp
{
	if(referenceCode - voltageCode != 0)
	{
		uint32_t convert_R = (voltageCode * 100000)/(referenceCode - voltageCode);
		return 1000.0 / ((1.0 / 298.15) - (log(10000.0 / convert_R) / 3435.0)) - 273150.0;
	}
	else
		return 0x00;


}

void CAN_interrupt()
{

	if (HAL_GetTick()>= last10 + 10)
	{

		AMS0_databytes[6] |= (precharge << 4);

		CAN_100(AMS0_databytes);
		last10 = HAL_GetTick();

	}
	if (HAL_GetTick()>= last100 + 100)
	{
		CAN_10(AMS1_databytes);

		HAL_GPIO_TogglePin(GPIOA, WDI_Pin);		// toggle watchdog
		HAL_GPIO_TogglePin(GPIOC, LED_GN_Pin);	// toggle LED
		last100 = HAL_GetTick();
	}
}


void convertTemperature(uint8_t selTemp)		// sort temp
{

	uint8_t indexOffset[12] = {9, 4, 11, 7, 6, 1, 0, 3, 10, 2, 5, 8};
	for(uint8_t k = 0; k < NUM_STACK; k++)
	{
			for(uint8_t j = 0; j < 3; j++)
			{
				temperature[k * NUM_CELLS_STACK + indexOffset[j + selTemp * 3]] = calculateTemperature(slaveGPIOs[j + k * 6], slaveGPIOs[5 + k * NUM_GPIO_STACK]);
			}
	}
	/*
			for(uint8_t k = 0; k < NUM_STACK; k++)
			{
				for(uint8_t i = 0; i < NUM_CELLS; i++)
				{
					//printf(" Stack %d Temperature %d = %d degC \r\n", k, i, temperature[k * NUM_STACK + i]);
				}
			}
			*/
		//USB STUFF

	if(selTemp == 3)
	{
		for(uint8_t i = 0; i < NUM_CELLS; i++)
		{
			usb_temperatures[i] = temperature[i]/1000;
		}

		//CAN stuff


		uint16_t temp_min = temperature[0];
			uint16_t temp_max = temperature[0];
		for(uint8_t k = 0; k < NUM_STACK; k++)
			{
				for(uint8_t i = 0; i < NUM_CELLS_STACK; i++)
				{
					if(temperature[i + k * 12] > temp_max) temp_max = temperature[i + k * 12];
					else if(temperature[i + k * 12] < temp_min) temp_min = temperature[i + k * 12];
				}

				AMS1_databytes[4] = temp_min;
				AMS1_databytes[5] = (temp_min >> 8);
				AMS1_databytes[6] = temp_max;
				AMS1_databytes[7] = (temp_max >> 8);
			}
	}
}

void send_usb()
{
	usb_data[NUM_CELLS * 2] = 0xff;
	for(uint8_t i = 0; i < NUM_CELLS; i++)
	{
		usb_data[i] = usb_voltages[i];
		usb_data[NUM_CELLS + i] = usb_temperatures[i];

	}

	CDC_Transmit_FS(usb_data, NUM_CELLS * 2 + 1);
}

