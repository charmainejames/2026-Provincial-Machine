#ifndef __TASK_THRUSTER_H_
#define	__TASK_THRUSTER_H_

void servo_open(void);              //爪子打开函数
void servo_close(void);             //爪子闭合函数
void thruster_vertical_up(void);    //水下机器人上升函数
void thruster_vertical_stop(void);      //水下机器人竖直停止函数
void thruster_vertical_down(void);          //水下机器人下潜函数
void thruster_horizontal_forward(void);         //水下机器人水平前进函数
void thruster_horizontal_stop(void);        //水下机器人水平停止函数
void thruster_horizontal_backward(void);    //水下机器人水平后退函数
void thruster_horizontal_left_turn(void);   //水下机器人原地左转函数
void thruster_horizontal_right_turn(void);  //水下机器人原地右转函数


#endif
