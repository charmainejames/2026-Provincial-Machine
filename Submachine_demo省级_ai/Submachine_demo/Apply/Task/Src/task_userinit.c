#include "task_conf.h"
#include "ocd_conf.h"
#include "config.h"

/* 外设初始化函数 */
void Task_UserInit(void)
{

	Drv_PWM_Init(thruster,6);				//初始化4个推进器PWM
//	Drv_PWM_Init(paw,1);					//初始化舵机爪子PWM	
	
//	Drv_Uart_DMAInit(&Uart3);			//初始化示例串口DMA
	Drv_Uart_DMAInit(&Uart1);			//初始化示例串口DMA	
	OCD_JY901_DMAInit(&JY901S);        //初始化JY901串口DMA		



	
	Drv_IWDG_Init(&demoIWDG);

}
