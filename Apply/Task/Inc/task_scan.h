/****************************************************************************

* 文件名: task_scan.h

* 内容简述：前向超声波扇形扫描任务（对外接口）

* 队友使用方法：
*   1. #include "task_scan.h"
*   2. 读取 g_scan_data.distance_mm[i]   即可拿到对应角度的距离
*      其中 0xFFFF (DYP_INVALID_DISTANCE) 表示该角度暂无有效数据
*   3. 要拿一份"整圈快照"以避免读写竞争，调用 Task_Scan_GetSnapshot()

* 文件历史：
*  1.0       2026-05-17                创建该文件

****************************************************************************/
#ifndef __TASK_SCAN_H_
#define __TASK_SCAN_H_

#include "drv_hal_conf.h"

/* 扫描参数：只扫右侧 60°/70°/80°/90° 4 个关键角度（沿右岸 wall-follow 用）
 *   舵机在 60↔90 之间乒乓，800ms 走一圈，4 点最小二乘拟合岸线
 *   如要换回全扇形扫描：NUM=19、MIN=-90、STEP=10 */
#define SCAN_NUM_POINTS    4
#define SCAN_ANGLE_STEP    10
#define SCAN_ANGLE_MIN     60
#define SCAN_ANGLE_MAX     90

/* 扫描数据结构（中断写、主循环读） */
typedef struct {
    uint16_t distance_mm[SCAN_NUM_POINTS];   /* 距离图，0xFFFF 表示无效 */
    int16_t  angle_deg [SCAN_NUM_POINTS];    /* 对应角度（-90, -80, ..., +80, +90） */
    uint8_t  last_updated_index;             /* 最近一次更新的格子下标 */
    uint32_t last_update_tick;               /* 最近更新的时间戳 (HAL_GetTick) */
} tagScanData_T;

/* 舵机控制回调类型 */
/* 输入：角度（-90 ~ +90 度）；输出：用户在函数里自己调用 Drv_PWM_HighLvTimeSet */
typedef void (*ServoSetAngleFn)(int16_t angle_deg);

/* 全局扫描数据（队友只读用） */
extern tagScanData_T g_scan_data;


/**
 * @brief 初始化扫描任务
 * @param servo_set_angle 用户提供的舵机控制函数；传 NULL 则只测距、不动舵机
 * @note  调用前提：DYP_Forward 实例已在 config.c 定义；TIM2 已 Drv_Timer_Init
 *        本函数会调用 OCD_DYP_L08_DMAInit 启动 USART3 DMA 接收
 */
void Task_Scan_Init(ServoSetAngleFn servo_set_angle);

/**
 * @brief 扫描状态机心跳（在 TIM2_IRQHandler 里调用）
 * @note  每次推进一步状态：BOOT → STEP → WAIT → STEP → WAIT → ...
 *        100ms 节拍下，2 拍/角度，3.8s 扫一圈 ±90°
 */
void Task_Scan_TickFromISR(void);

/**
 * @brief 临界区拷贝整张扫描表（避免与中断读写竞争）
 * @param out 目标结构体指针
 */
void Task_Scan_GetSnapshot(tagScanData_T *out);


#endif /* __TASK_SCAN_H_ */
