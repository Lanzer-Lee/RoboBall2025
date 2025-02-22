//
// Created by 王浩宇 on 2024/12/3.
//

#ifndef MOTOR_H
#define MOTOR_H


#include "pid.h"
#include "CAN_Receive.h"

extern pid_type_def PID_Speed_M2006;


void speed_control(int16_t speed_set1,int16_t speed_set2,int16_t speed_set3,int16_t speed_set4);

#endif //MOTOR_H
