/*
 * bms.c
 *
 *  Created on: Feb 23, 2024
 *      Author: Admin
 */

#include "bms.h"
#include "LTC6811.h"
#include "stdio.h"
#include "math.h"
#include "can.h"


#define NUM_STACK 12						 //total slaves
#define NUM_CELLS_STACK 12					 //Cells per stack
#define NUM_GPIO_STACK 6					 //GPIOs per slave
#define NUM_CELLS NUM_CELLS_STACK *NUM_STACK //Cells per accu container
#define NUM_GPIO NUM_GPIO_STACK *NUM_STACK   //GPIOs per slave
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

bool balancing = false;
uint16_t balanceMargin = 500; //in 0.1mV


uint16_t cellVoltages[NUM_CELLS] = {0};	//cell voltages in 0.1[mV]
int16_t cellTemperatures[NUM_CELLS] = {0}; //cell temperatures in 0.1[°C]
uint8_t cfg[NUM_STACK][6] = {{0}}; //0x38 disables the GPIO1..3 pulldown so GPIO1..3 can be used for measurement
uint16_t slaveGPIOs[NUM_GPIO] = {0};

uint16_t cell_max = cellVoltages[0];
uint16_t cell_min = cellVoltages[0];
uint16_t temp_max = slaveGPIOs[0];
uint16_t temp_min = slaveGPIOs[0];

uint8_t AMS2_databytes[8];



uint8_t can_cnt = 0; //can counter to adjust timings

/* 1 ms interrupt
 * HLCK 96 MHz
 * APB1 48 MHz
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	switch(can_cnt)
	{
		case 10 + last10:
		{
			CAN_100();
			last10 =can_cnt;
			break; //100 Hz
		}
		case 100:
		{
			CAN_10();
			can_cnt = 0;
			break; //10 Hz
		}
	}

	can_cnt++;

}



void BMS_init()
{
	LTC6811_initialize();
}

void BMS()
{
	uint8_t pec = 0;
	static uint8_t selTemp = 0;

	for (uint8_t i = 0; i < NUM_STACK; i++)
	{
		cfg[i][0] = 0x3C | ((selTemp << 6) & 0xC0);
		cfg[i][1] = 0x00;
		cfg[i][2] = 0x00;
		cfg[i][3] = 0x00;
		cfg[i][4] = 0x00;
		cfg[i][5] = 0x00;
	}

	LTC6811_wrcfg(NUM_STACK, (uint8_t(*)[6])cfg);
	HAL_Delay(3);

	//wakeup_idle();
	//LTC6811_rdcfg();
	//HAL_Delay(3);

	LTC6811_adcv();
	HAL_Delay(3);

	pec += LTC6811_rdcv(0, NUM_STACK, (uint16_t(*)[12])cellVoltages);
	HAL_Delay(3);

	LTC6811_adax();
	HAL_Delay(3);

	pec += LTC6811_rdaux(0, NUM_STACK, (uint16_t(*)[6])slaveGPIOs);
	HAL_Delay(3);


	convertVoltage();

	convertTemperature(selTemp);

/*
	wakeup_idle();
	LTC6811_adstat();
	HAL_Delay(3);

	wakeup_idle();
	LTC6811_rdstat();
*/

	if (selTemp < 3)
		selTemp++;
	else
		selTemp = 0;

	HAL_Delay(10);

}

void convertVoltage()
{
	double voltage[NUM_CELLS];

	for(uint8_t k = 0; k < NUM_STACK; k++)
	{

		for(uint8_t i = 0; i < 12; i++)
		{
			if(cellVoltages[i + k * 12] > cell_max) cell_max = cellVoltages[i + k * 12];
			else if(cellVoltages[i + k * 12] < cell_min) cell_min = cellVoltages[i + k * 12];

			voltage[i + k * 12] = (double)cellVoltages[i + k * 12]/10000;
			printf(" Stack %d Cell %d = %.4f V \r\n", k, i, voltage[i + k * 12]);
		}
	}

}

void convertTemperature(uint8_t selTemp)
{
	static double calc_temp[NUM_STACK][12];
	//static double convert_R[3];

	for(uint8_t k = 0; k < NUM_STACK; k++)
	{
		for(uint8_t i = 0; i < 3; i++)
		{

			convert_R[i] = (slaveGPIOs[i + k * 6] * 100000)/(slaveGPIOs[5 + k * 6] - slaveGPIOs[i + k * 6]);
			calc_temp[k][i + selTemp * 3] = 1/((1/298.15)-(log(10000/convert_R[i])/3435)) - 273.15;
		}
	}





	uint8_t indexOffset[12] = {9, 4, 11, 7, 6, 1, 0, 3, 10, 2, 5, 8};
	for(uint8_t k = 0; k < NUM_STACK; k++)
	{

			for(uint8_t j = 0; j < 3; j++)
			{
				cell_Temp[k * NUM_CELLS_STACK + indexOffset[j + selTemp * 3]] = calculateTemperature(slaveGPIOs[j + k * 6], slaveGPIOs[5 + k * 6]);
			}
	}

	if(selTemp == 3)
	{
		//verarbeitung
	}

/*
	if(selTemp == 3)
	{
		for(uint8_t k = 0; k < NUM_STACK; k++)
		{
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 9, calc_temp[k][0]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 4, calc_temp[k][1]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 11, calc_temp[k][2]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 7, calc_temp[k][3]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 6, calc_temp[k][4]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 1, calc_temp[k][5]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 0, calc_temp[k][6]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 3, calc_temp[k][7]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 10, calc_temp[k][8]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 2, calc_temp[k][9]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 5, calc_temp[k][10]);
				printf(" Stack %d Temperature %d = %.4f degC \r\n", k, 8, calc_temp[k][11]);
		}
	}
	*/

}

uint16_t calculateTemperature(uint16_t voltageCode, uint16_t referenceCode)
{
	uint32_t convert_R = (voltageCode * 100000)/(referenceCode - voltageCode);
	return 1/((1/298.15)-(log(10000/convert_R)/3435)) - 273.15;

}


