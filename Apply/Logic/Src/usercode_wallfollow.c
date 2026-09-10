/****************************************************************************

* 文件名: usercode_wallfollow.c

* 内容简述：沿右湖岸自主绕行主循环

* 算法思路（参考 Water/App/Task/DYP_Scan_Task.c 大头菜原版）：
*   1) 扫描状态机已经在右侧 60/70/80/90° 4 个角度采到距离（g_scan_data）
*   2) 把 4 个 (角度, 距离) 换成机器人坐标系下的 4 个 XY 点
*   3) 最小二乘拟合岸线方向 shoreline_deg
*   4) 根据 shoreline_deg + 离岸距离（90° 方向）算航向修正 correction
*   5) 把 forward_yaw_target = global_base_yaw + correction 写入
*   6) 调 jy901_yaw_anti_forward_open() 让现成 PID 把推进器开起来

* 与 usercode.c 的关系：
*   不修改 usercode.c，只 extern 借用：
*     - global_base_yaw, global_base_locked
*     - forward_yaw_target, forward_yaw_get_flag, forward_open_flag
*     - jy901_yaw_anti_forward_open()

* 文件历史：
*  1.0       2026-05-19                创建该文件

****************************************************************************/
#include "usercode_wallfollow.h"
#include "task_conf.h"
#include "ocd_conf.h"
#include "algo_conf.h"
#include "config.h"
#include "task_scan.h"

#include <math.h>
#include <stdio.h>


/* —— 借自 usercode.c 的全局符号（不改 usercode.c，只 extern） —— */
extern float    global_base_yaw;
extern uint8_t  global_base_locked;
extern uint8_t  forward_open_flag;
extern uint8_t  forward_yaw_get_flag;
extern float    forward_yaw_target;
extern void     jy901_yaw_anti_forward_open(void);


/* —— 可调参数 —— */
#define WF_TARGET_NEAR_CM      30        /* 离岸太近：朝向左侧修正 */
#define WF_TARGET_FAR_CM       70        /* 离岸太远：朝向右侧修正 */
#define WF_SHORELINE_DEADBAND  10        /* |shoreline_deg| < 阈值时不修正 */
#define WF_NEAR_CORRECTION     (-10)     /* 太近时叠加的修正角（度）*/
#define WF_FAR_CORRECTION      (+10)     /* 太远时叠加的修正角（度）*/
#define WF_DECISION_PERIOD_MS  800       /* 决策周期 = 4 点扫一圈的时间 */


/* 调试用：暴露给 Keil Watch 的内部变量 */
static volatile double   s_last_shoreline_deg = 0.0;
static volatile uint16_t s_last_dist_cm       = 0;
static volatile int16_t  s_last_correction    = 0;


void UserLogic_WallFollow(void)
{
    printf("WallFollow mode started\r\n");

    /* —— 启动期：等 4 个角度都有有效数据，才锁定基准航向 —— */
    while (1) {
        uint8_t all_ok = 1;
        for (int i = 0; i < SCAN_NUM_POINTS; i++) {
            if (g_scan_data.distance_mm[i] == 0xFFFF) {
                all_ok = 0;
                break;
            }
        }
        Drv_IWDG_Feed(&demoIWDG);
        if (all_ok) break;
        HAL_Delay(50);
    }

    /* 锁定当前航向为基准 */
    global_base_yaw      = JY901S.stcAngle.ConYaw;
    global_base_locked   = 1;
    forward_yaw_target   = global_base_yaw;
    forward_yaw_get_flag = 1;     /* 跳过 usercode.c 里"从 step_offset 取目标"分支 */

    /* === 5 秒启动延时：把 AUV 放进水里再退后 ===
     * 这 5 秒内：扫描状态机已经在跑（舵机会转、距离会更新），
     *          但 forward_open_flag 还没置 1，推进器不动，AUV 不会冲。
     */
    printf("Wait 5s before thrusters engage...\r\n");
    for (int i = 0; i < 100; i++) {     /* 100 * 50ms = 5000ms */
        HAL_Delay(50);
        Drv_IWDG_Feed(&demoIWDG);
    }
    printf("Thrusters engaging now!\r\n");

    forward_open_flag    = 1;     /* 开启直行 PID */

    uint32_t last_decision_tick = 0;

    /* —— 主循环 —— */
    while (1) {
        uint32_t now = HAL_GetTick();

        /* 周期性决策 */
        if (now - last_decision_tick >= WF_DECISION_PERIOD_MS) {
            last_decision_tick = now;

            tagScanData_T snap;
            Task_Scan_GetSnapshot(&snap);

            /* 检查 4 个值全部有效，否则跳过本次决策（保留上一目标） */
            uint8_t valid = 1;
            for (int i = 0; i < SCAN_NUM_POINTS; i++) {
                if (snap.distance_mm[i] == 0xFFFF) { valid = 0; break; }
            }

            if (valid) {
                /* 4 点转 (x, y)，距离单位 mm → cm */
                tagPoint_T pts[SCAN_NUM_POINTS];
                for (int i = 0; i < SCAN_NUM_POINTS; i++) {
                    Algo_Geom_AngleDistToXY(
                        &pts[i],
                        (double)snap.angle_deg[i],
                        (double)snap.distance_mm[i] / 10.0
                    );
                }

                /* 最小二乘拟合岸线方向 */
                double shoreline_deg = Algo_Geom_LineAngleFromPoints(pts, SCAN_NUM_POINTS);

                /* 离岸距离取 90° 方向（最贴边的那个点） */
                uint16_t dist_cm = snap.distance_mm[SCAN_NUM_POINTS - 1] / 10;

                /* 算航向修正 */
                int16_t correction = 0;
                if (shoreline_deg >  WF_SHORELINE_DEADBAND ||
                    shoreline_deg < -WF_SHORELINE_DEADBAND) {
                    correction = (int16_t)shoreline_deg;
                }
                if (dist_cm > WF_TARGET_FAR_CM)  correction += WF_FAR_CORRECTION;
                if (dist_cm < WF_TARGET_NEAR_CM) correction += WF_NEAR_CORRECTION;

                /* 暴露调试值 */
                s_last_shoreline_deg = shoreline_deg;
                s_last_dist_cm       = dist_cm;
                s_last_correction    = correction;

                /* 把修正叠到目标航向 */
                forward_yaw_target   = global_base_yaw + (float)correction;
                forward_yaw_get_flag = 1;
            }
        }

        /* PID 每周期都跑（平滑控制） */
        if (forward_open_flag) {
            jy901_yaw_anti_forward_open();
        }

        Drv_IWDG_Feed(&demoIWDG);
    }
}
