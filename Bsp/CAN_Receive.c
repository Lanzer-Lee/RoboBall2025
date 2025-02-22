#include "CAN_receive.h"


#define abs(x) ((x) > 0 ? (x) : -(x))


Motor_TypeDef motors[4];
static CAN_TxHeaderTypeDef can1_tx_message;
static uint8_t can1_send_data[8];
static uint8_t Rxflag = 0;

/**
 * @brief Receive CAN1 message and decode it to Motor_TypeDef struct.
 * @param ptr: pointer to Motor_TypeDef struct.
 * @param data: pointer to CAN1 message data.
 */
static void get_motor_measure(Motor_TypeDef *ptr, uint8_t *data) {
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t) (data[0] << 8 | data[1]);
    ptr->speed_rpm = (int16_t) (data[2] << 8 | data[3]);
    ptr->real_current = (int16_t) (data[4] << 8 | data[5]);
    ptr->temperate = data[6];
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;
    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;

    /*
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t) ((data)[0] << 8 | (data)[1]);
    (ptr)->speed_rpm = (uint16_t) ((data)[2] << 8 | (data)[3]);
    (ptr)->given_current = (uint16_t) ((data)[4] << 8 | (data)[5]);
    (ptr)->temperate = (data)[6];
    if (ptr->ecd - ptr->last_ecd > 4096)
        ptr->round_cnt--;
    else if (ptr->ecd - ptr->last_ecd < -4096)
        ptr->round_cnt++;
    ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd;
     */
}


/**
 * @brief CAN1 receive callback function.
 * @param hcan: pointer to CAN1 handle.
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    Rxflag = 1;
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        Rxflag = 0;
        uint8_t i = rx_header.StdId - 0x201;
        get_motor_measure(&motors[i], rx_data);
    }
}


/**
 * @brief Get motor offset. This function should be called after system and can init.
 * @param ptr: pointer to Motor_TypeDef struct.
 * @param data: pointer to CAN1 message data.
 **/
void get_motor_offset(Motor_TypeDef *ptr, uint8_t *data) {
    ptr->ecd = (uint16_t) (data[0] << 8 | data[1]);
    ptr->offset_ecd = ptr->ecd;
}


/**
 * @brief Get total angle of motor. When system init, ecd is 0, update ecd using this function.
 * @param ptr: pointer to Motor_TypeDef struct.
 **/
void get_total_angle(Motor_TypeDef *ptr) {
    int res1, res2, delta;
    if (ptr->ecd < ptr->last_ecd) {
        res1 = ptr->ecd + 8192 - ptr->last_ecd;     // corotation, delta up.
        res2 = ptr->ecd - ptr->last_ecd;            // reversal, delta down.
    }
    else {
        res1 = ptr->ecd - 8192 - ptr->last_ecd;     // reversal, delta down.
        res2 = ptr->ecd + 8192 - ptr->last_ecd;     // corotation, delta up.
    }
    if (abs(res1) < abs(res2)) {
        delta = res1;
    }
    else {
        delta = res2;
    }
    ptr->total_ecd += delta;
    ptr->last_ecd = ptr->ecd;
}

/**
 * @brief Set motor current.
 * @param motor1: motor 1 current.
 * @param motor2: motor 2 current.
 * @param motor3: motor 3 current.
 * @param motor4: motor 4 current.
 **/
void set_motor_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
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

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
}
