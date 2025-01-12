#include "CAN_receive.h"

motor_measure_t motor_measure[4];
static CAN_TxHeaderTypeDef  can1_tx_message;
static uint8_t              can1_send_data[8];
static uint8_t              Rxflag = 0;
static void get_motor_measure(motor_measure_t *ptr, uint8_t *data)  \
{                                                                   \
    (ptr)->last_ecd = (ptr)->ecd;                                   \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
    (ptr)->temperate = (data)[6];
    if(ptr->ecd - ptr->last_ecd> 4096)
        ptr->round_cnt --;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt ++;
    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    Rxflag = 1;
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        Rxflag = 0;
        uint8_t i = rx_header.StdId - 0x201;
        get_motor_measure(&motor_measure[i],rx_data);
    }
}

void CAN1_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = CAN1_2006;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_send_data[0] = motor1 >> 8;
    can1_send_data[1] = motor1;
    can1_send_data[2] = motor2 >> 8;
    can1_send_data[3] = motor2;
    can1_send_data[4] = motor3 >> 8;
    can1_send_data[5] = motor3;
    can1_send_data[6] = motor4 >> 8;
    can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message,  can1_send_data, &send_mail_box);
}
