/****************************************************************************

* 文件名: usercode_wallfollow.h

* 内容简述：沿右湖岸自主绕行主循环（取代默认 UserLogic_Code 的另一版 main 逻辑）

* 切换方式：在 Keil "Options for Target → C/C++ → Define" 加宏 WALL_FOLLOW_MODE
*           main.c 里 #ifdef 切换调用 UserLogic_WallFollow() 还是 UserLogic_Code()

* 文件历史：
*  1.0       2026-05-19                创建该文件

****************************************************************************/
#ifndef __USERCODE_WALLFOLLOW_H_
#define __USERCODE_WALLFOLLOW_H_


/**
 * @brief 沿右岸自主主循环（不返回）
 * @note  只在编译宏 WALL_FOLLOW_MODE 定义时由 main.c 调用
 *        借用 usercode.c 的现有 PID（jy901_yaw_anti_forward_open）
 *        和共享变量（forward_yaw_target / global_base_yaw / forward_open_flag）
 */
void UserLogic_WallFollow(void);


#endif /* __USERCODE_WALLFOLLOW_H_ */
