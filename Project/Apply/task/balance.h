#ifndef BALANCE_H
#define BALANCE_H

#include "stdint.h"

// 电机中立点
#define MOTOR_NEUTRAL 1500
// PWM 输出限幅范围 (假设 1000-2000)
#define PWM_MIN 1000
#define PWM_MAX 2000

// PID 控制器结构体
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float target;
} PID_Controller;

// 函数声明
void balance_init(float kp, float ki, float kd);
void balance_set_target(float target_yaw);
void balance_compute(float current_yaw, int* left_pwm, int* right_pwm);

#endif // BALANCE_H