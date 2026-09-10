#include "task_conf.h"
#include "ocd_conf.h"
#include "dev_conf.h"
#include "config.h"
#include "task_scan.h"

/* ============================================================================
 * 前向扫描舵机：
 *   - 硬件：LY-QFS2020（180° 标准舵机），信号线 PE9，独立 5V 电源
 *   - PWM：TIM1_CH1 全重映射 → PE9（servo_front[0] 在 config.c）
 *   - 回调：task_scan 状态机每次到 ST_STEP 调 my_servo_set_angle()，
 *           参数是 -90~+90 度（task_scan.h 的对称坐标）；当前扫描范围
 *           只用 60~90 这一段（沿右岸 wall-follow），但 Dev_Servo 接口
 *           兼容全 ±90 范围，将来切扇形扫描不用改。
 * ========================================================================== */

static tagServo_T s_servo_front;

/* task_scan 回调：把扫描状态机算出的角度直接交给 Dev_Servo */
static void my_servo_set_angle(int16_t deg)
{
	Dev_Servo_SetAngle(&s_servo_front, deg);
}


/* 外设初始化函数 */
void Task_UserInit(void)
{
	Drv_PWM_Init(thruster, 6);				//初始化6个推进器PWM
	Drv_PWM_Init(servo_front, 1);			//初始化前向扫描舵机PWM（TIM1_CH1 / PE9）
	Dev_Servo_Init(&s_servo_front, &servo_front[0]);	//关联舵机句柄，上电回中位

#ifdef SERVO_INSTALL_MODE
	/* === 装舵盘模式：把舵机锁在 90°，不启动扫描，不开门狗 ===
	 * 用法：Keil → Options for Target → C/C++ → Define 加 SERVO_INSTALL_MODE
	 *       烧录 → 舵机停在 90° → 装好舵盘 → 删掉宏 → 重新烧录恢复正常
	 */
	Dev_Servo_SetAngle(&s_servo_front, 90);
	while (1) {
		HAL_Delay(1000);
	}
#endif

	Drv_Uart_DMAInit(&Uart1);				//初始化树莓派通信串口
	OCD_JY901_DMAInit(&JY901S);				//初始化JY901串口DMA

	/* === 前向超声波扫描启动 === */
	Drv_Timer_Init(&tTimer2);				//配置 TIM2 寄存器和 NVIC
	Drv_Timer_Enable(&tTimer2);				//真正启动 TIM2，开始 100ms 嘀嗒
	Task_Scan_Init(my_servo_set_angle);		//启动前向扫描 + 舵机回调

	Drv_IWDG_Init(&demoIWDG);
}
