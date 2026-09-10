#include "balance.h"

static PID_Controller pid;

void balance_init(float kp, float ki, float kd) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.target = 0.0f;  // 默认目标角度为 0
}

void balance_set_target(float target_yaw) {
    pid.target = target_yaw;
}

void balance_compute(float current_yaw, int* left_pwm, int* right_pwm) {
    // 计算误差
    float error = pid.target - current_yaw;

    // 积分项
    pid.integral += error;
    // 积分限幅，防止饱和
    if (pid.integral > 1000.0f) pid.integral = 1000.0f;
    if (pid.integral < -1000.0f) pid.integral = -1000.0f;

    // 微分项
    float derivative = error - pid.prev_error;

    // PID 输出
    float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

    // 更新前一误差
    pid.prev_error = error;

    // 限幅输出，防止飞车
    if (output > 500.0f) output = 500.0f;
    if (output < -500.0f) output = -500.0f;

    // 计算左右电机 PWM
    *left_pwm = MOTOR_NEUTRAL + (int)output;
    *right_pwm = MOTOR_NEUTRAL - (int)output;

    // 限幅 PWM 值
    if (*left_pwm > PWM_MAX) *left_pwm = PWM_MAX;
    if (*left_pwm < PWM_MIN) *left_pwm = PWM_MIN;
    if (*right_pwm > PWM_MAX) *right_pwm = PWM_MAX;
    if (*right_pwm < PWM_MIN) *right_pwm = PWM_MIN;
}