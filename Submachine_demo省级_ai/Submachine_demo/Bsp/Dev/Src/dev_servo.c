/****************************************************************************

* 文件名: dev_servo.c

* 内容简述：标准 PWM 舵机驱动（详见 dev_servo.h）

* 文件历史：
*  1.0       2026-05-19                创建该文件

****************************************************************************/
#include "dev_servo.h"


void Dev_Servo_Init(tagServo_T *_t, tagPWM_T *_ptPWM)
{
    if (_t == NULL || _ptPWM == NULL) return;

    _t->ptPWM     = _ptPWM;
    _t->sCurAngle = 0;
    Dev_Servo_SetAngle(_t, 0);   /* 上电回中位 */
}


void Dev_Servo_SetAngle(tagServo_T *_t, int16_t sAngle)
{
    if (_t == NULL || _t->ptPWM == NULL) return;

    /* 限幅 */
    if (sAngle >  90) sAngle =  90;
    if (sAngle < -90) sAngle = -90;

    /* -90~+90 线性映射到 500~2500 us
     * pulse_us = 1500 + (sAngle / 90) * 1000
     *          = 1500 + sAngle * 1000 / 90
     * 用 2000/180 = 1000/90 等价表达，整数除法更准 */
    uint16_t pulse_us = (uint16_t)(DEV_SERVO_PULSE_MID_US
                        + ((int32_t)sAngle * 2000 / DEV_SERVO_RANGE_DEG));

    Drv_PWM_HighLvTimeSet(_t->ptPWM, pulse_us);
    _t->sCurAngle = sAngle;
}
