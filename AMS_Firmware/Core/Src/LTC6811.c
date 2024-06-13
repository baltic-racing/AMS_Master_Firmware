/*
 * LTC6811.c
 *
 *  Created on: Feb 8, 2024
 *      Author: Maximus
 */

#include <stdint.h>
#include "LTC6811.h"
#include <string.h>
#include <stdio.h>
#include <main.h>
#include <math.h>
#include "define.h"
/*@brief 6811 conversion command variables

*/
uint8_t ADCV[2]; //!< Cell Voltage conversion command.
uint8_t ADAX[2]; //!< GPIO conversion command.
uint8_t CVST[2]; //!< Cell Voltage selftest command
uint8_t AXST[2]; //!< GPIO selftest command

uint8_t wakeup = 0x00;

/*@brief Initializes all command variables
 */
void LTC6811_initialize()
{
  set_adc(MD_NORMAL, DCP_DISABLED, CELL_CH_ALL, AUX_CH_ALL, CHST_SC);
  //set_selftest(MD_NORMAL, ST_1);
}

void wakeup_idle()
{
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}


/*!******************************************************************************************************************
 \brief Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

@param[in] int MD The adc conversion mode
@param[in] int DCP Controls if Discharge is permitted during cell conversions
@param[in] int CH Determines which cells are measured during an ADC conversion command
@param[in] int CHG Determines which GPIO channels are measured during Auxiliary conversion command

 Command Code: \n
			|command	|  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
			|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
			|ADCV:	    |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
			|ADAX:	    |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 ******************************************************************************************************************/
void set_adc(uint8_t MD, uint8_t DCP, uint8_t CH, uint8_t CHG, uint8_t CHST)
{
  uint8_t md_bits;

  md_bits = (MD & 0x02) >> 1;
  ADCV[0] = md_bits | 0x02;
  md_bits = (MD & 0x01) << 7;
  ADCV[1] = md_bits | 0x60 | (DCP << 4) | CH;

  md_bits = (MD & 0x02) >> 1;
  ADAX[0] = md_bits | 0x04;
  md_bits = (MD & 0x01) << 7;
  ADAX[1] = md_bits | 0x60 | CHG;

  /*
  md_bits = (MD & 0x02) >> 1;
  ADSTAT[0] = md_bits | 0x04;
  md_bits = (MD & 0x01) << 7;
  ADSTAT[1] = md_bits | 0x68 | CHST;
  */
}

/*!*********************************************************************************************
 \brief Starts cell voltage conversion

  Starts ADC conversions of the LTC6811 Cpin inputs.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted	     |

***********************************************************************************************/
void LTC6811_adcv()
{
	uint8_t cmd[4];
	uint16_t temp_pec;
	//1
	cmd[0] = ADCV[0];
	cmd[1] = ADCV[1];
	//2
	temp_pec = pec15_calc(2, ADCV);
	cmd[2] = (uint8_t)(temp_pec >> 8);
	cmd[3] = (uint8_t)(temp_pec);
	//3
	wakeup_idle();
	//4
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	spi_write_array(4, cmd);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
}

/*!******************************************************************************************************
 \brief Start an GPIO Conversion

  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |

*********************************************************************************************************/
void LTC6811_adax()
{
	uint8_t cmd[4];
	uint16_t temp_pec;

	//1
	cmd[0] = ADAX[0];
	cmd[1] = ADAX[1];
	//2
	temp_pec = pec15_calc(2, ADAX);
	cmd[2] = (uint8_t)(temp_pec >> 8);
	cmd[3] = (uint8_t)(temp_pec);
	//3
	wakeup_idle();
	//4
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	spi_write_array(4, cmd);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
}


void LTC6811_wrcfg(uint8_t config [][6])

{
	uint16_t temp_pec;
	uint8_t current_ic;
	uint8_t WRCFG_index = 4;

	uint8_t WRCFG[CMD_LEN];

	for(current_ic = 0; current_ic < NUM_STACK; current_ic++)
	{
		for(uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
		{
			WRCFG[WRCFG_index] = config[current_ic][current_byte];
			WRCFG_index++;
		}

		temp_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic][0]);
		WRCFG[WRCFG_index] = (uint8_t)(temp_pec >> 8);
		WRCFG[WRCFG_index + 1] = (uint8_t)temp_pec;
		WRCFG_index += 2;
	}

	wakeup_idle();

	for(current_ic = 0; current_ic < NUM_STACK; current_ic++)
	{
	    WRCFG[0] = 0x80 + (current_ic << 3); //Setting address
	    WRCFG[1] = 0x01;
	    temp_pec = pec15_calc(2, WRCFG);
	    WRCFG[2] = (uint8_t)(temp_pec >> 8);
	    WRCFG[3] = (uint8_t)(temp_pec);
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
		spi_write_array(4, WRCFG);
		spi_write_array(8, &WRCFG[4 + (8 * current_ic)]);
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);


		HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
		HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);

	}
}


/*
void LTC6811_rdcfg()
{
	uint8_t RDCFG[8];
	uint8_t data[8];

	uint16_t temp_pec;

	RDCFG[0] = 0x80 + (addr << 3);
	RDCFG[1] = 0x02;
	temp_pec = pec15_calc(2, RDCFG);
	RDCFG[2] = (uint8_t)(temp_pec >> 8);
	RDCFG[3] = (uint8_t)(temp_pec);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	spi_write_read(RDCFG, 4, data, 8);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}
*/

uint8_t LTC6811_rdcv(uint8_t reg, uint16_t cell_codes[][12])
{
	uint8_t pec_error = 0; //pec Error wenn -1
	uint16_t received_pec;
	uint16_t data_pec;
	uint8_t data_counter = 0;

	uint8_t cell_data[NUM_RX_BYT * NUM_STACK];

	for(uint8_t cell_reg = 1; cell_reg < 5; cell_reg++)	//executes once for each of the LTC6804 cell voltage registers
	{
		data_counter = 0;
		LTC6811_rdcv_reg(cell_reg, cell_data);

		for(uint8_t current_ic = 0; current_ic < NUM_STACK; current_ic++)	// executes for every LTC6804 in the stack.
		{
			 // current_ic is used as an IC counter
			for(uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)	// This loop parses the read back data. Loops once for each cell voltages in the register
			{
				uint16_t parsed_cell = cell_data[data_counter] | (cell_data[data_counter + 1] << 8);
				cell_codes[current_ic][current_cell + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
				data_counter = data_counter + 2;
			}
			received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter + 1];
			data_pec = pec15_calc(BYTES_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
			if (received_pec != data_pec)
			{
			  pec_error = -1;
			}
			data_counter = data_counter + 2;
		}
	}
	return(pec_error);
}

void LTC6811_rdcv_reg(uint8_t reg, uint8_t *data)
{
	uint8_t RDCV[4];
	uint16_t temp_pec;

	switch(reg){
		case 1: RDCV[1] = 0x04; break;
		case 2: RDCV[1] = 0x06; break;
		case 3: RDCV[1] = 0x08; break;
		case 4: RDCV[1] = 0x0a; break;
	}

	wakeup_idle();

    //Register A-D = reg 1-4
    for(uint8_t current_ic = 0; current_ic < NUM_STACK; current_ic++)
    {
    	RDCV[0] = 0x80 + (current_ic << 3);
        temp_pec = pec15_calc(2, RDCV);
        RDCV[2] = (uint8_t)(temp_pec >> 8);
        RDCV[3] = (uint8_t)(temp_pec);
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
        spi_write_read(RDCV, 4, &data[current_ic * 8], 8);
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
    }
}

int8_t LTC6811_rdaux(uint8_t reg, uint16_t aux_codes[][6])
{
	uint8_t pec_error = 0; //pec Error wenn -1
	uint16_t received_pec;
	uint16_t data_pec;
	uint8_t data_counter = 0;

	uint8_t aux_data[NUM_RX_BYT * NUM_STACK];

	for(uint8_t aux_reg = 1; aux_reg < AUX_IN_REG; aux_reg++)	//executes once for each of the LTC6804 cell voltage registers
	{
		data_counter = 0;
		LTC6811_rdaux_reg(aux_reg, aux_data);

		for(uint8_t current_ic = 0; current_ic < NUM_STACK; current_ic++)	// executes for every LTC6804 in the stack.
		{
			 // current_ic is used as an IC counter
			for(uint8_t current_cell = 0; current_cell < AUX_IN_REG; current_cell++)	// This loop parses the read back data. Loops once for each cell voltages in the register
			{
				uint16_t parsed_cell = aux_data[data_counter] | (aux_data[data_counter + 1] << 8);
				aux_codes[current_ic][current_cell + ((aux_reg - 1) * AUX_IN_REG)] = parsed_cell;
				data_counter += 2;
			}
			received_pec = (aux_data[data_counter] << 8) + aux_data[data_counter + 1];
			data_pec = pec15_calc(BYTES_IN_REG, &aux_data[current_ic * NUM_RX_BYT]);
			if (received_pec != data_pec)
			{
			  pec_error = -1;
			}
			data_counter += 2;
		}
	}
	return(pec_error);
}

void LTC6811_rdaux_reg(uint8_t reg, uint8_t *data)
{
	uint8_t RDAUX[4];
	uint16_t temp_pec;

	switch(reg){
		case 1: RDAUX[1] = 0xC; break;
		case 2: RDAUX[1] = 0xE; break;
	}

	wakeup_idle();

    //Register A-D = reg 1-4
    for(uint8_t current_ic = 0; current_ic < NUM_STACK; current_ic++)
    {
    	RDAUX[0] = 0x80 + (current_ic << 3);
        temp_pec = pec15_calc(2, RDAUX);
        RDAUX[2] = (uint8_t)(temp_pec >> 8);
        RDAUX[3] = (uint8_t)(temp_pec);
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
        spi_write_read(RDAUX, 4, &data[current_ic * 8], 8);
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
    }
}


/*
int8_t LTC6811_rdstatb(uint8_t total_ic, uint16_t OV_flag[] ,uint16_t UV_flag[], uint8_t r_statb[][6])
{
  const uint8_t BYTES_IN_REG = 8;

  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t received_pec;
  rx_data = (uint8_t *)malloc((8 * total_ic) * sizeof(uint8_t));
  //1
  cmd[0] = 0x00;
  cmd[1] = 0x12;

  //2
  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  //3
  for (int current_ic = 0; current_ic < total_ic; current_ic++)
  {
    cmd[0] = 0x80 + (current_ic << 3); //Setting address
    data_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(data_pec >> 8);
    cmd[3] = (uint8_t)(data_pec);
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
    spi_write_read(cmd, 4, &rx_data[current_ic * 8], 8);
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
  }

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++) //executes for each LTC6804 in the stack
  {
	  uint8_t i =0;
	  OV_flag[current_ic] = 0;
	  /*
	  for (uint8_t byte = 2; byte < 5; byte++)
	  {
		  for (uint8_t bit = 0; bit < 8; bit+=2)
		  {
			  OV_flag[current_ic] |= (((rx_data[(current_ic * BYTES_IN_REG)+ byte])>>(1 + bit))&1)<<i;
			  UV_flag[current_ic] |= (((rx_data[(current_ic * BYTES_IN_REG)+ byte])>>(bit))&1)<<i;
			  i++;
		  }
	  }


    //4.a
    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_statb[current_ic][current_byte] = rx_data[current_byte + (current_ic * BYTES_IN_REG)];
    }

    for (uint8_t byte = 2; byte < 5; byte++)
   	  {
   		  for (uint8_t bit = 0; bit < 8; bit+=2)
   		  {
   			  OV_flag[current_ic] |= ((r_statb[current_ic][byte]>>(bit +1))&1)<<i;
   			  UV_flag[current_ic] |= (((r_statb[current_ic][byte])>>(bit))&1)<<i;
   			  i++;
   		  }
   	  }
    //4.b
    received_pec = (r_statb[current_ic][6] << 8) + r_statb[current_ic][7];
    data_pec = pec15_calc(6, &r_statb[current_ic][0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }
  free(rx_data);
  //5
  return (pec_error);
}
*/

void LTC6811_clrstat()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  //1
  cmd[0] = 0x07;
  cmd[1] = 0x13;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //3
  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  //4
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
  spi_write_array(4, cmd);
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

  HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
  HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
}


/*
void LTC6811_adstat()
{

  uint8_t cmd[4];
  uint16_t temp_pec;
  //uint8_t wakeup = 0xff;

  //1
  cmd[0] = ADSTAT[0];
  cmd[1] = ADSTAT[1];
  //2
  temp_pec = pec15_calc(2, ADSTAT);
  cmd[2] = (uint8_t)(temp_pec >> 8);
  cmd[3] = (uint8_t)(temp_pec);

  //4
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
	spi_write_array(4, cmd);
	HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
	HAL_SPI_Transmit(&hspi3, &wakeup, 1, 1);
}
*/

/*
void LTC6811_rdstat()
{
		uint8_t RDSTAT[4];
		uint8_t data[8];

		uint16_t temp_pec;

		RDSTAT[0] = 0x80 + (addr << 3);
		RDSTAT[1] = 0x10;
		temp_pec = pec15_calc(2, RDSTAT);
		RDSTAT[2] = (uint8_t)(temp_pec >> 8);
		RDSTAT[3] = (uint8_t)(temp_pec);
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
		spi_write_read(RDSTAT, 4, data, 8);
		HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

}
*/

//char *data uint8_t *data , uint8_t len
uint16_t pec15_calc(uint8_t len, uint8_t *data)
 {
 uint16_t remainder, address;

 remainder = 16;//PEC seed
 for (int i = 0; i < len; i++)
 {
 address = ((remainder >> 7) ^ data[i]) & 0xff;//calculate PEC table address
 remainder = (remainder << 8 ) ^ pec15Table[address];
 }
 return (remainder*2);//The CRC15 has a 0 in the LSB so the final value must be multiplied by 2
 }
