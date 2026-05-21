/****************************************************************************

* 文件名: ocd_dyp_l08.c

* 内容简述：DYP-L08 系列水下超声波传感器驱动（UART 受控模式）

* 协议、卡死保护、特殊错误码处理见 ocd_dyp_l08.h 文件头注释

* 文件历史：
*  1.0       2026-05-17                创建该文件

****************************************************************************/
#include "ocd_dyp_l08.h"

#include <string.h>


/**
 * @brief 参数匹配（设置默认值）
 * @param _t-传感器句柄指针
 */
static void S_DYP_L08_ParamMatch(tagDYP_L08_T *_t)
{
    /* 默认触发字节 0x00 */
    if (_t->ucTriggerByte == 0x00) {
        _t->ucTriggerByte = DYP_TRIGGER_BYTE;
    }

    /* DMA 接收缓冲大小（如果用户没设，SGA 默认 100；我们设小一点节省内存） */
    DEFAULT(_t->tUART.tRxInfo.usDMARxMAXSize, DYP_RX_DMA_BUF_SIZE);

    /* 波特率默认 115200（DYP 默认值） */
    DEFAULT(_t->tUART.tUARTHandle.Init.BaudRate, 115200);

    /* 串口号默认 USART3（本工程的硬件分配） */
    DEFAULT(_t->tUART.tUARTHandle.Instance, USART3);

    /* 使能 DMA 收发 */
    _t->tUART.tUartDMA.bRxEnable = true;
    _t->tUART.tUartDMA.bTxEnable = true;
}


/**
 * @brief 初始化 DYP-L08 传感器
 * @param _t-传感器句柄指针
 * @note  调用 SGA 的 Drv_Uart_DMAInit，会自动配置 GPIO/DMA/中断
 */
void OCD_DYP_L08_DMAInit(tagDYP_L08_T *_t)
{
    S_DYP_L08_ParamMatch(_t);

    /* 初始化 UART + DMA（含 IDLE 中断、DMA 接收循环） */
    Drv_Uart_DMAInit(&_t->tUART);

    /* 状态字段清零 */
    _t->usDistance_mm     = DYP_INVALID_DISTANCE;
    _t->ulLastTriggerTick = 0;
}


/**
 * @brief 触发一次测距
 * @param _t-传感器句柄指针
 * @retval DYP_TRIG_OK / DYP_TRIG_TOO_FAST
 * @note  内置 33ms 软件守门，防止过快连续触发导致传感器锁死
 */
uint8_t OCD_DYP_L08_Trigger(tagDYP_L08_T *_t)
{
    /* 33ms 守门检查 */
    uint32_t now = HAL_GetTick();
    if (_t->ulLastTriggerTick != 0 &&
        (now - _t->ulLastTriggerTick) < DYP_MIN_TRIGGER_GAP_MS)
    {
        return DYP_TRIG_TOO_FAST;
    }

    /* 发送 1 字节触发（HAL_UART_Transmit 阻塞，但 115200 下 1 字节仅 ~87us） */
    uint8_t b = _t->ucTriggerByte;
    Drv_Uart_Transmit(&_t->tUART, &b, 1);

    _t->ulLastTriggerTick = now;
    return DYP_TRIG_OK;
}


/**
 * @brief 解析 DMA 收到的回包
 * @param _t-传感器句柄指针
 * @retval DYP_OK / DYP_ERR_* 见 .h
 * @note  错误时 usDistance_mm = 0xFFFF
 */
uint8_t OCD_DYP_L08_DataProcess(tagDYP_L08_T *_t)
{
    /* 检查 DMA 接收完成标志（由 Drv_Uart_DMA_RxHandler 在 IDLE 中断里置位） */
    if (_t->tUART.tRxInfo.ucDMARxCplt == 0) {
        return DYP_ERR_NOT_READY;
    }

    /* 取出本次收到的字节数和缓冲指针 */
    uint16_t len  = _t->tUART.tRxInfo.usDMARxLength;
    uint8_t  *buf = _t->tUART.tRxInfo.ucpDMARxCache;

    /* 清标志位（消费掉这次接收） */
    _t->tUART.tRxInfo.ucDMARxCplt = 0;

    /* 期望恰好 4 字节 */
    if (len != DYP_RX_FRAME_LEN) {
        _t->usDistance_mm = DYP_INVALID_DISTANCE;
        return DYP_ERR_LEN;
    }

    /* 帧头校验 */
    if (buf[0] != DYP_FRAME_HEAD) {
        _t->usDistance_mm = DYP_INVALID_DISTANCE;
        return DYP_ERR_HEAD;
    }

    /* 无回波特殊码：FF FF FD FB（实测在空气中或目标超距时常见） */
    if (buf[1] == 0xFF && buf[2] == 0xFD && buf[3] == 0xFB) {
        _t->usDistance_mm = DYP_INVALID_DISTANCE;
        return DYP_ERR_NO_ECHO;
    }

    /* SUM 校验：(0xFF + DataH + DataL) & 0xFF */
    uint8_t sum = (uint8_t)(buf[0] + buf[1] + buf[2]);
    if (sum != buf[3]) {
        _t->usDistance_mm = DYP_INVALID_DISTANCE;
        return DYP_ERR_SUM;
    }

    /* 成功：组装距离 = DataH × 256 + DataL */
    _t->usDistance_mm = ((uint16_t)buf[1] << 8) | (uint16_t)buf[2];
    return DYP_OK;
}
