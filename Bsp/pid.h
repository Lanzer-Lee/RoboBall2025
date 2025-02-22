//
// Created by 王浩宇 on 2024/12/4.
//

#ifndef _PID_H
#define _PID_H
typedef float fp32;
#include "main.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;   // PID_POSITION or PID_DELTA

    float Kp;       // PID parameters Kp
    float Ki;       // PID parameters Ki
    float Kd;       // PID parameters Kd

    float max_out;  //
    float max_iout; //

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //
    float error[3]; //

} pid_type_def;


extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

#endif //PROJ2_PID_H
