#include "task_conf.h"

#include "config.h"



////爪子打开函数
//void servo_open(void) 
//{
//   Drv_PWM_HighLvTimeSet(&paw[0],2500);	    //爪子打开
//   printf("paw open\n");
//}
////爪子闭合函数
//void servo_close(void) 
//{
//   Drv_PWM_HighLvTimeSet(&paw[0],500);	    //爪子闭合
//   printf("paw close\n");
//}


//水下机器人上升函数
void thruster_vertical_up(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[2],1750);	//推进器3向下
   Drv_PWM_HighLvTimeSet(&thruster[3],1750);	//推进器4向下
   printf("vertical up\n");
}

//水下机器人竖直停止函数
void thruster_vertical_stop(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[2],1500);	//推进器3停止
   Drv_PWM_HighLvTimeSet(&thruster[3],1500);	//推进器4停止
   printf("vertical stop\n");
}

//水下机器人下潜函数
void thruster_vertical_down(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[2],1250);	//推进器3向上
   Drv_PWM_HighLvTimeSet(&thruster[3],1250);	//推进器4向上
   printf("vertical down\n");
}
//水下机器人水平前进函数
void thruster_horizontal_forward(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[0],1750);	//推进器1前进
   Drv_PWM_HighLvTimeSet(&thruster[1],1750);	//推进器2前进
   printf("horizontal forward\n");
}
//水下机器人水平停止函数
void thruster_horizontal_stop(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[0],1500);	//推进器1停止
   Drv_PWM_HighLvTimeSet(&thruster[1],1500);	//推进器2停止
   printf("horizontal stop\n");
}
//水下机器人水平后退函数
void thruster_horizontal_backward(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[0],1250);	//推进器1后退
   Drv_PWM_HighLvTimeSet(&thruster[1],1250);	//推进器2后退
   printf("horizontal backward\n");
}
//水下机器人原地左转函数
void thruster_horizontal_left_turn(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[0],1250);	//推进器1后退
   Drv_PWM_HighLvTimeSet(&thruster[1],1750);	//推进器2前进
   printf("horizontal left turn\n");
}
//水下机器人原地右转函数
void thruster_horizontal_right_turn(void)
{
   Drv_PWM_HighLvTimeSet(&thruster[0],1750);	//推进器1前进
   Drv_PWM_HighLvTimeSet(&thruster[1],1250);	//推进器2后退
   printf("horizontal right turn\n");
}


