

#ifndef _CAN_RECEIVE_H
#define _CAN_RECEIVE_H
#include "main.h"
#include "can.h"
typedef enum {
    CAN1_2006      = 0x200,
    CAN1_2006_RID1 = 0x201,
    CAN1_2006_RID2 = 0x202,
    CAN1_2006_RID3 = 0x203,
    CAN1_2006_RID4 = 0x204,
}can_msg_id_e;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
    int32_t round_cnt;
    int32_t total_ecd;
} motor_measure_t;


void CAN1_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern motor_measure_t motor_measure[4];

#endif