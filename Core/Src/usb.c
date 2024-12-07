/** @brief The user layer of usb code, containing receiving and sending data.
  * @author Lanzer
  * @date 2024-12-07
  * @version 1.0
  */
#include "usb.h"

USB_TypeDef usb = {0};
extern USBD_HandleTypeDef hUsbDeviceFS;

/** @brief The communication protocol of usb rx data processing.
  * @param buf the pointer of buffer
  * @param len the length of data
  * @retval void
  */
void USB_ProcessRxData(const uint8_t *buf, uint32_t len) {
    uint32_t i = 0;
    uint8_t byte = 0;
    for (i = 0; i < len; i++) {
        byte = buf[i];
        if (usb.state == UART_STATE_IDLE) {
            if (byte == '(') {
                usb.pointer = 0;
                usb.receive_buffer[usb.pointer++] = byte;
                usb.state++;
            }
        } else if (usb.state == 1) {
            usb.receive_buffer[usb.pointer++] = byte;
            if (byte == ')') {
                usb.receive_buffer[usb.pointer++] = '\0';
                usb.state = UART_STATE_BUSY;
            }
        }
    }
}

/** @brief Transmit data to usb
  * @param data the array of data
  * @param len the length of data
  * @retval void
  */
void USB_TransmitData(uint8_t *data, uint32_t len) {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, data, len);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
    HAL_Delay(CDC_POLLING_INTERVAL);
}

/** @brief Transmit formated string to usb
  * @param format a string format
  * @param other element in format
  * @retval void
  */
void USB_TransmitString(char *format, ...) {
    uint16_t i;
    va_list ap;
    va_start(ap, format);
    vsprintf((char *) (usb.send_buffer), format, ap);
    va_end(ap);
    i = strlen((const char *) (usb.send_buffer));
    USB_TransmitData(usb.send_buffer, i);
}

/** @brief Analysis and process usb message
  * @param void
  * @retval void
  */
void USB_Service(void) {
    if (usb.state == UART_STATE_BUSY) {
        USB_TransmitData(usb.receive_buffer, sizeof(usb.receive_buffer));
        if (strcmp((const char *) (usb.receive_buffer), "(MPU)") == 0) {
            USB_TransmitString("Roll: %8.3lf\tPitch: %8.3lf\tYaw: %8.3lf\r\n", mpu_imu.rol, mpu_imu.pit, mpu_imu.yaw);
        } else if (strcmp((const char *) (usb.receive_buffer), "(WIT)") == 0) {
            WIT_ReadData(&wit_imu);
            USB_TransmitString("Roll:%.3f\tPit:%.3f\tYaw:%.3f", wit_imu.angle[0], wit_imu.angle[1], wit_imu.angle[2]);
        }
        usb.state = UART_STATE_IDLE;
    }
}