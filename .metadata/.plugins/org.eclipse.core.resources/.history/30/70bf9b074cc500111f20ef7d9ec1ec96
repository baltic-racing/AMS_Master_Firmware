/*
 * usb_control.c
 *
 *  Created on: Sep 20, 2026
 *      Author: finja
 */

#include <main.h>
#include "stdlib.h"
#include "stdbool.h"
#include "gpio.h"
#include "bms.h"
#include "LTC6811.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"
#include "usb_control.h"
#include "usbd_cdc.h"

#include <string.h>
#include <stdint.h>

extern volatile uint8_t BMS_state;
extern USBD_HandleTypeDef hUsbDeviceFS;

#define CMD_STX 0x02
#define CMD_MODE             0x00
#define DEVICE_SIGNATURE     0xA1

static volatile uint16_t rxWrite = 0;
static volatile uint16_t rxRead  = 0;

uint8_t debug_tx_buffer[64];
uint8_t debug_tx_len;

uint8_t debug_len = 0;
uint8_t debug_buffer[64];

static uint8_t packet[64];

#define USB_QUEUE_SIZE 8
#define USB_PACKET_SIZE 64

typedef struct
{
    uint8_t data[USB_PACKET_SIZE];
    uint8_t len;
} USB_Packet_t;

static USB_Packet_t usb_queue[USB_QUEUE_SIZE];

static volatile uint8_t usb_head = 0;
static volatile uint8_t usb_tail = 0;

static uint8_t usb_in_flight = 0;

/*
.______________.
| transmit_state | Description             |
|==========================================|
|      0x03      | slave_temp              |
|______________|
*/

void USB_control(const char *broadcaster, uint8_t *data_shit, uint8_t data_size_bytes)
{
    uint8_t type = 0x00;

	if (strcmp(broadcaster, "ID_TS_Voltage") == 0)
		type = 0x21;
	else if (strcmp(broadcaster, "ID_TS_Current") == 0)
		type = 0x22;
	else if (strcmp(broadcaster, "ID_TS_Currentdrawn") == 0)
		type = 0x23;
	else if (strcmp(broadcaster, "ID_TS_AMS_Status") == 0)
		type = 0x24;
	else if (strcmp(broadcaster, "ID_TS_Cell_Voltages_max") == 0)
		type = 0x25;
	else if (strcmp(broadcaster, "ID_TS_Cell_Voltages_min") == 0)
		type = 0x26;
	else if (strcmp(broadcaster, "ID_TS_Cell_Temprearure_max") == 0)
		type = 0x27;
	else if (strcmp(broadcaster, "ID_TS_Cell_Temprearure_min") == 0)
		type = 0x28;
	else if (strcmp(broadcaster, "ID_TS_Cellnumer_Voltages_max") == 0)
		type = 0x29;
	else if (strcmp(broadcaster, "ID_TS_Cellnumer_Voltages_min") == 0)
		type = 0x2A;
	else if (strcmp(broadcaster, "ID_TS_Cellnumer_Temprearure_max") == 0)
		type = 0x2B;
	else if (strcmp(broadcaster, "ID_TS_Cellnumer_Temprearure_min") == 0)
		type = 0x2C;
	else if (strcmp(broadcaster, "ID_LTC_Temperature") == 0)
		type = 0x2D;
	else
		return;

	USB_transmit(type, data_shit, data_size_bytes);
}

void USB_transmit(uint8_t type, const uint8_t *data_shit, uint8_t shit_count)
{
    //uint8_t payload_len = shit_count;	// Message_ID + Message_Value
    uint8_t payload_len = shit_count + 1;   // Message_ID + Message_Value

    // CMD_STX + CMD_Mode + Protocol_Length + Device_Signature + Message_ID (Payload) + Message_Value (Checksum)
    if ((5 + payload_len) > sizeof(packet))
        return;

    uint8_t idx = 0;
    uint8_t chk = 0;

    packet[idx++] = CMD_STX;				// CMD_STX
    packet[idx++] = CMD_MODE;				// CMD_Mode
    packet[idx++] = payload_len;			// Protocol_Length
    packet[idx++] = DEVICE_SIGNATURE;		// Device_Signature

    packet[idx++] = type;                   // Message_ID

    for (uint8_t i = 0; i < shit_count; i++)
    {
        packet[idx++] = data_shit[i];		// Message_Value (Payload)
    }

    for (uint8_t i = 0; i < idx; i++)
    {
        chk ^= packet[i];					// Checksum calculation
    }

    packet[idx++] = chk;					// Checksum

    //CDC_SendBlocking(packet, idx, 3);
    //CDC_SendNonBlocking(packet, idx);
    USB_QueuePacket(packet, idx);
}


uint8_t CDC_SendBlocking(uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0;
    }

    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS(buf, len) == USBD_BUSY)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0;
        }
    }

    return 1;
}
/*
uint8_t CDC_SendNonBlocking(uint8_t *buf, uint16_t len)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0;
    }

    if (CDC_Transmit_FS(buf, len) == USBD_OK)
    {
        return 1;
    }

    return 0;
}
*/

uint8_t USB_QueuePacket(const uint8_t *data, uint8_t len)
{
    if (len > USB_PACKET_SIZE)
    {
        return 0;
    }

    uint8_t next = (usb_head + 1) % USB_QUEUE_SIZE;

    // Queue voll
    if (next == usb_tail)
    {
        return 0;
    }

    memcpy(usb_queue[usb_head].data, data, len);
    usb_queue[usb_head].len = len;

    usb_head = next;

    return 1;
}

void USB_Task(void)
{
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (hcdc == NULL)
    {
        return;
    }

    /*
     * Ein vorheriges Paket läuft noch.
     */
    if (usb_in_flight)
    {
        if (hcdc->TxState != 0)
        {
            return;
        }

        /*
         * Übertragung abgeschlossen.
         * Erst jetzt Queue-Eintrag freigeben.
         */
        usb_tail = (usb_tail + 1) % USB_QUEUE_SIZE;
        usb_in_flight = 0;
    }

    // Queue leer
    if (usb_tail == usb_head)
    {
        return;
    }

    /*
     * Falls USB aus irgendeinem Grund noch beschäftigt ist:
     * NICHT warten.
     */
    if (hcdc->TxState != 0)
    {
        return;
    }

    if (CDC_Transmit_FS(
            usb_queue[usb_tail].data,
            usb_queue[usb_tail].len) == USBD_OK)
    {
        usb_in_flight = 1;
    }
}

