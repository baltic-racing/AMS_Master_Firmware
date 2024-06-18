/*
 * define.h
 *
 *  Created on: Jun 11, 2024
 *      Author: shoot
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_

#define NUM_STACK 1						 //total slaves
#define NUM_CELLS_STACK 12					 //Cells per stack Attention LTC6811 CH all modus
#define NUM_GPIO_STACK 6					 //GPIOs per slave
#define NUM_CELLS NUM_CELLS_STACK *NUM_STACK //Cells per accu container
#define NUM_GPIO NUM_GPIO_STACK *NUM_STACK   //GPIOs per slave


#define MAX_VOLTAGE 42000					// Wert in 0,1 mV
#define MIN_VOLTAGE 26000					// es gehen nur Vielfache von 16

#define MAX_TS_VOLTAGE 554
#define MIN_TS_VOLTAGE 343

#define MAX_Temp 58611
#define MIN_Temp 1


#define BYTES_IN_REG 6
#define CMD_LEN 4 + (8 * NUM_STACK)

#define NUM_RX_BYT 8
#define CELL_IN_REG 3
#define AUX_IN_REG 3

#define error_max 5
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

#define FAIL_TIMEOUT_ISA (1 << 0)
#define FAIL_TIMEOUT_LTC (1 << 1)
#define FAIL_OV (1 << 2)
#define FAIL_UV (1 << 3)
#define FAIL_OCD (1 << 4)
#define FAIL_OCC (1 << 5)
#define FAIL_OT (1 << 6)
#define FAIL_UT (1 << 7)

*/

#endif /* INC_DEFINE_H_ */
