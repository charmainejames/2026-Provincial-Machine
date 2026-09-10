/****************************************************************************

* 文件名: ocd_dyp_l08.h

* 内容简述：DYP-L08 系列水下超声波传感器驱动头文件（UART 受控模式）

* 协议要点（实测确认 2026-05-17）：
*   - L081MTW 不支持 Modbus，只支持"低脉冲触发 → 4 字节回包"模式
*   - 触发：发任意单字节（默认 0x00），UART 起始位+数据位形成低电平段
*   - 回包：4 字节 [0xFF] [DataH] [DataL] [SUM]
*   - 距离 = DataH × 256 + DataL  (mm)
*   - SUM = (0xFF + DataH + DataL) & 0xFF
*   - 特殊码 0xFF 0xFF 0xFD 0xFB 表示"无有效回波/出水"
*   - 触发间隔严格 > 33ms，否则传感器会卡死，必须断电重启

* 文件历史：
*  1.0       2026-05-17                创建该文件

****************************************************************************/
#ifndef __OCD_DYP_L08_H_
#define __OCD_DYP_L08_H_

#include "drv_hal_conf.h"

/* 协议常量 */
#define DYP_TRIGGER_BYTE       0x00     /* 触发字节，0x00 时整个 UART 帧都是低电平最有效 */
#define DYP_RX_FRAME_LEN       4        /* 回包固定 4 字节 */
#define DYP_RX_DMA_BUF_SIZE    16       /* DMA 接收缓冲（SGA UART 用 IDLE 模式，留点余量） */
#define DYP_FRAME_HEAD         0xFF     /* 帧头 */

/* 软件守门：触发间隔最小值（规格书 33ms，留余量 35ms） */
#define DYP_MIN_TRIGGER_GAP_MS 35

/* 距离无效标记（用于上层数据） */
#define DYP_INVALID_DISTANCE   0xFFFF

/* DataProcess 返回码 */
#define DYP_OK             0    /* 解析成功，usDistance_mm 已更新 */
#define DYP_ERR_NOT_READY  1    /* DMA 数据未就绪 */
#define DYP_ERR_LEN        2    /* 回包长度不对（不是 4 字节） */
#define DYP_ERR_HEAD       3    /* 帧头不是 0xFF */
#define DYP_ERR_SUM        4    /* SUM 校验失败 */
#define DYP_ERR_NO_ECHO    5    /* 无回波特殊码 FF FF FD FB（已标记为 INVALID） */

/* Trigger 返回码 */
#define DYP_TRIG_OK              0
#define DYP_TRIG_TOO_FAST        1    /* 距上次触发不到 33ms，拒绝（防卡死） */


/* 超声波实例结构体 */
typedef struct {
    tagUART_T  tUART;                            /* SGA UART 句柄（含 DMA、GPIO、中断配置） */
    uint8_t    ucTriggerByte;                    /* 触发字节，默认 0x00 */
    uint16_t   usDistance_mm;                    /* 解析后的距离，0xFFFF 表示无效 */
    uint32_t   ulLastTriggerTick;                /* 上次触发时间戳（HAL_GetTick），用于 33ms 守门 */
} tagDYP_L08_T;


/* ============== 函数声明 ============== */

/**
 * @brief 初始化 DYP-L08 传感器（含 UART + DMA 接收）
 * @param _t-传感器句柄指针
 */
void OCD_DYP_L08_DMAInit(tagDYP_L08_T *_t);

/**
 * @brief 触发一次测距（非阻塞）
 * @param _t-传感器句柄指针
 * @retval DYP_TRIG_OK(0)        成功发送
 *         DYP_TRIG_TOO_FAST(1)  距上次触发未满 33ms，拒绝（防卡死保护）
 * @note  务必让两次触发间隔 > 33ms，否则传感器锁死需要断电
 */
uint8_t OCD_DYP_L08_Trigger(tagDYP_L08_T *_t);

/**
 * @brief 解析 DMA 收到的 4 字节回包，更新 usDistance_mm
 * @param _t-传感器句柄指针
 * @retval DYP_OK(0)           有效距离
 *         DYP_ERR_NOT_READY(1) 数据未就绪
 *         DYP_ERR_LEN(2)      长度错
 *         DYP_ERR_HEAD(3)     帧头错
 *         DYP_ERR_SUM(4)      SUM 校验错
 *         DYP_ERR_NO_ECHO(5)  无回波（FF FF FD FB），已标记 INVALID
 * @note  错误情况下 usDistance_mm 一律被置为 DYP_INVALID_DISTANCE(0xFFFF)
 */
uint8_t OCD_DYP_L08_DataProcess(tagDYP_L08_T *_t);


#endif /* __OCD_DYP_L08_H_ */
