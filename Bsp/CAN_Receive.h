#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H
#include "main.h"
#include "can.h"


/**
  * @brief: Receive data from electrical regulation based on CAN1
  * @param Identifier: 0x200 + ID of the motor Electrical regulation
  * @param Data:
  *   Data[0]: Angle high byte
  *   Data[1]: Angle low byte
  *   Data[2]: Speed high byte
  *   Data[3]: Speed low byte
  *   Data[4]: Current high byte
  *   Data[5]: Current low byte
  *   Data[6]: Null
  *   Data[7]: Null
  **/


/**
  * @brief: Send data to electrical regulation based on CAN1
  * @param Identifier: 0x200 (0 <= ID <= 3), 0x1FF (4<= ID <= 7)
  * @param Data:
  *   Data[0]: Current high byte of ID 1
  *   Data[1]: Current low byte of ID 1
  *   Data[2]: Current high byte of ID 2
  *   Data[3]: Current low byte of ID 2
  *   Data[4]: Current high byte of ID 3
  *   Data[5]: Current low byte of ID 3
  *   Data[6]: Current high byte of ID 4
  *   Data[7]: Current low byte of ID 4
  **/


typedef enum {
    CAN1_2006      = 0x200,
    CAN1_2006_RID1 = 0x201,
    CAN1_2006_RID2 = 0x202,
    CAN1_2006_RID3 = 0x203,
    CAN1_2006_RID4 = 0x204,
}can_msg_id_e;


typedef struct
{
    int16_t speed_rpm;      // observed speed
    int16_t target_speed;   // target speed of PID
    float real_current;     // observed current
    int16_t given_current;  // current calculated by PID
    uint16_t ecd;           // abs angle range: [0, 8191]
    uint16_t last_ecd;      // abs angle range: [0, 8191]
    uint16_t offset_ecd;
    int32_t round_cnt;
    int32_t total_ecd;
    uint8_t temperate;
} Motor_TypeDef;


extern Motor_TypeDef motors[];


void set_motor_current(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void get_motor_offset(Motor_TypeDef *ptr, uint8_t *data);
void get_total_angle(Motor_TypeDef *ptr);


#endif
