//
// Created by lanzer on 24-12-6.
//

#ifndef PROJ2_USB_H
#define PROJ2_USB_H

#include "sys.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usart.h"
#include "mpu6500.h"
#include "wit.h"

#define USB_BUFFER_SIZE  200
#define CDC_POLLING_INTERVAL    1

typedef struct
{
    uint8_t receive_buffer[USB_BUFFER_SIZE];
    uint8_t send_buffer[USB_BUFFER_SIZE];
    uint8_t state;
    uint8_t pointer;
}USB_TypeDef;

extern USB_TypeDef usb;

void USB_ProcessRxData(const uint8_t *buf, uint32_t len);
void USB_TransmitData(uint8_t *data, uint32_t len);
void USB_TransmitString(char *format, ...);
void USB_Service(void);

#endif //PROJ2_USB_H
