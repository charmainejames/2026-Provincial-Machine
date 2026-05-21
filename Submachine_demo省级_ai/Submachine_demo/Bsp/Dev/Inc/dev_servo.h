/****************************************************************************

* 文件名: dev_servo.h

* 内容简述：标准 PWM 舵机驱动（180° 量程，LY-QFS2020 / SG90 等通用类型）

* 抽象层级：Dev 层，复用 Bsp/Driver 的 Drv_PWM_HighLvTimeSet
* 使用方法：
*   1. 在 config.c 配一个 tagPWM_T 实例（50Hz / 7.5% duty 初值）
*   2. Drv_PWM_Init() 后调 Dev_Servo_Init() 关联
*   3. 调 Dev_Servo_SetAngle(-90 ~ +90) 设角度

* 文件历史：
*  1.0       2026-05-19                创建该文件

****************************************************************************/
#ifndef __DEV_SERVO_H_
#define __DEV_SERVO_H_

#include "drv_hal_conf.h"

/* 舵机量程：标准 180° 舵机
 * 0°    → 500μs
 * 90°   → 1500μs（中位）
 * 180°  → 2500μs
 *
 * 我们用 -90~+90° 对称坐标（task_scan 的约定，0°=正前方）：
 *   -90° → 500μs
 *    0°  → 1500μs
 *   +90° → 2500μs
 */
#define DEV_SERVO_RANGE_DEG     180     /* 量程（度） */
#define DEV_SERVO_PULSE_MIN_US  500     /* 最小脉宽 */
#define DEV_SERVO_PULSE_MID_US  1500    /* 中位脉宽 */
#define DEV_SERVO_PULSE_MAX_US  2500    /* 最大脉宽 */

typedef struct {
    tagPWM_T *ptPWM;        /* 指向 config.c 的 PWM 实例（如 &servo_front[0]） */
    int16_t   sCurAngle;    /* 当前角度记忆（-90~+90） */
} tagServo_T;


/**
 * @brief 关联舵机句柄到 PWM 实例，并回到中位
 * @param _t       舵机句柄
 * @param _ptPWM   已 Drv_PWM_Init() 过的 PWM 实例指针
 */
void Dev_Servo_Init(tagServo_T *_t, tagPWM_T *_ptPWM);


/**
 * @brief 设置舵机绝对角度
 * @param _t       舵机句柄
 * @param sAngle   目标角度，-90~+90 度（超出范围会自动限幅）
 */
void Dev_Servo_SetAngle(tagServo_T *_t, int16_t sAngle);


#endif /* __DEV_SERVO_H_ */
