#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "drv_hal_conf.h"
#include "ocd_conf.h"
#include "algo_conf.h"
#include "dev_conf.h"

/* 用户句柄声明包含区 */

extern tagTIM_T tTimer2;

extern tagGPIO_T demoGPIO[];

extern tagUART_T Uart1;

/* DYP-L08 前向超声波（USART3，原 Uart3 替换） */
extern tagDYP_L08_T DYP_Forward;

extern tagPWM_T thruster[];

/* 前向扫描舵机 PWM 实例（TIM1_CH1 / PE9） */
extern tagPWM_T servo_front[];

//extern tagPWM_T paw[];

extern tagJY901_T JY901S;

extern tagPID_T PID;




extern tagIWDG_T demoIWDG;
#endif
