//
// Created by  on 2024/12/3.
//
#include "Motor.h"
#include "CAN_Receive.h"
#include "pid.h"
//M2006 电机控制
pid_type_def PID_Speed_M2006;
pid_type_def PID_Angle_M2006;

void speed_control(int16_t speed_set1,int16_t speed_set2,int16_t speed_set3,int16_t speed_set4)
{
    int16_t target_speed[4]={speed_set1,speed_set2,speed_set3,speed_set4};
    int16_t current_speed[4] = {motor_measure[0].speed_rpm,motor_measure[1].speed_rpm,motor_measure[2].speed_rpm,motor_measure[3].speed_rpm};

    float output_current[4] = {PID_calc(&PID_Speed_M2006, current_speed[0], target_speed[0]),PID_calc(&PID_Speed_M2006, current_speed[1], target_speed[1]),PID_calc(&PID_Speed_M2006, current_speed[2], target_speed[2]),PID_calc(&PID_Speed_M2006, current_speed[3], target_speed[3])};
    for(int i=0;i<4;i++)
        motor_measure[i].given_current = (int16_t)output_current[i];
    CAN1_cmd(motor_measure[0].given_current, motor_measure[1].given_current, motor_measure[2].given_current, motor_measure[3].given_current);
    HAL_Delay(5);
}

