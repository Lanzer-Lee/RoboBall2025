//
// Created by  on 2024/12/3.
//
#include "Motor.h"
#include "CAN_Receive.h"
#include "pid.h"


pid_type_def PID_Speed_M2006;


void speed_control(int16_t speed_set1,int16_t speed_set2,int16_t speed_set3,int16_t speed_set4)
{
    int16_t target_speed[4]={speed_set1,speed_set2,speed_set3,speed_set4};
    int16_t current_speed[4] = {motors[0].speed_rpm, motors[1].speed_rpm, motors[2].speed_rpm, motors[3].speed_rpm};

    float output_current[4] = {
            PID_calc(&PID_Speed_M2006, current_speed[0], target_speed[0]),
            PID_calc(&PID_Speed_M2006, current_speed[1], target_speed[1]),
            PID_calc(&PID_Speed_M2006, current_speed[2], target_speed[2]),
            PID_calc(&PID_Speed_M2006, current_speed[3], target_speed[3])
    };

    for(int i=0;i<4;i++)
        motors[i].given_current = (int16_t)output_current[i];
    set_motor_current(
            motors[0].given_current,
            motors[1].given_current,
            motors[2].given_current,
            motors[3].given_current
    );
}

