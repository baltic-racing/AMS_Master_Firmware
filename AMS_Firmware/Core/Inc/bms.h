/*
 * bms.h
 *
 *  Created on: Feb 23, 2024
 *      Author: Admin
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_


#include "main.h"
#include "stdint.h"
#include "stdbool.h"

/*
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_can.h"
*/

void BMS(void);
void BMS_init(void);
void convertVoltage(void);
void convertTemperature(uint8_t selTemp);
void CAN_interrupt(void);
void send_usb(void);
void checkIMD(void);


#endif /* INC_BMS_H_ */













