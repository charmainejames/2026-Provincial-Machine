/****************************************************************************

* 文件名: task_scan.c

* 内容简述：前向超声波扇形扫描状态机（TIM2 100ms 中断驱动，无阻塞）

* 状态机：
*   BOOT (700ms 等传感器上电) → STEP → WAIT → STEP → WAIT → ...
*
*   每 100ms 心跳：
*     STEP: 通过用户回调移动舵机 → 触发超声波测距
*     WAIT: 等 DMA 收齐 4 字节（约 25ms）→ 解析 → 写入 g_scan_data
*
*   全扫一圈：19 格 × 2 拍 × 100ms = 3.8 秒/帧

* 与队友交互：
*   - 队友只读 g_scan_data.distance_mm[]
*   - 舵机控制通过 ServoSetAngleFn 回调，调用方自己实现

* 文件历史：
*  1.0       2026-05-17                创建该文件

****************************************************************************/
#include "task_scan.h"
#include "ocd_dyp_l08.h"
#include "config.h"   /* 拿 DYP_Forward 全局实例 */


/* ========== 全局共享：扫描结果（外部只读） ========== */
tagScanData_T g_scan_data;


/* ========== 内部状态 ========== */

/* 状态机状态 */
typedef enum {
    ST_BOOT = 0,    /* 上电等待 */
    ST_STEP,        /* 舵机走位 + 触发测距 */
    ST_WAIT,        /* 等响应 + 解析 */
} ScanState_E;

static ServoSetAngleFn s_servo_set    = NULL;   /* 用户注册的舵机回调（可为 NULL） */
static ScanState_E     s_state        = ST_BOOT;
static uint8_t         s_boot_ticks   = 0;       /* BOOT 状态累计的 100ms 拍数 */
static int8_t          s_idx          = 0;       /* 当前扫描格 0..18 */
static int8_t          s_dir          = 1;       /* 乒乓方向：+1 右扫 / -1 左扫 */

/* BOOT 等待拍数（100ms × 7 = 700ms，规格书 "上电工作时间 <600ms"） */
#define BOOT_WAIT_TICKS    7


/**
 * @brief 初始化扫描任务
 * @param servo_set_angle 用户的舵机控制函数，NULL 表示不动舵机
 */
void Task_Scan_Init(ServoSetAngleFn servo_set_angle)
{
    /* 记录用户回调 */
    s_servo_set = servo_set_angle;

    /* 初始化扫描数据表：每格的角度固定、距离全部置为无效 */
    for (int i = 0; i < SCAN_NUM_POINTS; i++) {
        g_scan_data.angle_deg[i]   = (int16_t)(SCAN_ANGLE_MIN + i * SCAN_ANGLE_STEP);
        g_scan_data.distance_mm[i] = DYP_INVALID_DISTANCE;
    }
    g_scan_data.last_updated_index = 0;
    g_scan_data.last_update_tick   = 0;

    /* 启动时舵机回中位（如果用户注册了回调） */
    if (s_servo_set) {
        s_servo_set(0);
    }

    /* 启动超声波 DMA 接收 */
    OCD_DYP_L08_DMAInit(&DYP_Forward);

    /* 状态机初值 */
    s_state      = ST_BOOT;
    s_boot_ticks = 0;
    s_idx        = 0;
    s_dir        = 1;
}


/**
 * @brief 状态机心跳，每 100ms 调用一次（在 TIM2_IRQHandler 里）
 * @note  绝不可阻塞：每个 case 只有几条指令，确保中断快速返回
 */
void Task_Scan_TickFromISR(void)
{
    switch (s_state) {

    case ST_BOOT:
        /* 等传感器上电完成（≥ 600ms） */
        if (++s_boot_ticks >= BOOT_WAIT_TICKS) {
            s_idx   = 0;
            s_dir   = 1;
            s_state = ST_STEP;
        }
        break;

    case ST_STEP:
        /* ① 通过用户回调移动舵机（NULL 时跳过） */
        if (s_servo_set) {
            s_servo_set(g_scan_data.angle_deg[s_idx]);
        }
        /* ② 触发测距（内置 33ms 守门，太快会自动拒绝） */
        OCD_DYP_L08_Trigger(&DYP_Forward);
        /* ③ 下一拍解析数据 */
        s_state = ST_WAIT;
        break;

    case ST_WAIT:
        /* 这一拍 DMA 已经把 4 字节收齐（约 25ms 完成），现在解析 */
        OCD_DYP_L08_DataProcess(&DYP_Forward);

        /* 写入共享数据表（不管成功失败，distance_mm 已经被 DataProcess 设好了） */
        g_scan_data.distance_mm[s_idx]    = DYP_Forward.usDistance_mm;
        g_scan_data.last_updated_index    = (uint8_t)s_idx;
        g_scan_data.last_update_tick      = HAL_GetTick();

        /* 推进到下一格：乒乓扫描 ±90° */
        s_idx += s_dir;
        if (s_idx >= SCAN_NUM_POINTS - 1) {
            s_idx = SCAN_NUM_POINTS - 1;
            s_dir = -1;
        } else if (s_idx <= 0) {
            s_idx = 0;
            s_dir = 1;
        }

        s_state = ST_STEP;
        break;

    default:
        /* 异常状态恢复 */
        s_state = ST_BOOT;
        s_boot_ticks = 0;
        break;
    }
}


/**
 * @brief 临界区拷贝整张扫描表，避免与中断读写竞争
 */
void Task_Scan_GetSnapshot(tagScanData_T *out)
{
    if (out == NULL) return;

    __disable_irq();
    *out = g_scan_data;
    __enable_irq();
}
