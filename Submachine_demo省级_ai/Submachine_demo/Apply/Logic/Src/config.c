#include "config.h"

/*
	推进器引脚分配说明：
	PB6  水平左推进器
	PB7  水平右推进器
	PB8  垂直左前推进器
	PB9  垂直右前推进器
	PB1  垂直左后推进器
	PB0  垂直右后推进器
*/

tagGPIO_T demoGPIO[] =
{
	[0] =
	{ 
		.tGPIOInit.Pin 		= GPIO_PIN_13,				/* GPIO引脚 */
		.tGPIOInit.Mode 	= GPIO_MODE_OUTPUT_PP,		/* 推挽输出模式 */
		.tGPIOInit.Pull 	= GPIO_NOPULL,				/* 无上下拉 */
		.tGPIOInit.Speed 	= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
		.tGPIOPort 			= GPIOD,					/* GPIO端口 */
	},
	[1] =
	{ 
		.tGPIOInit.Pin 		= GPIO_PIN_14,				/* GPIO引脚 */
		.tGPIOInit.Mode 	= GPIO_MODE_OUTPUT_PP,		/* 推挽输出模式 */
		.tGPIOInit.Pull 	= GPIO_NOPULL,				/* 无上下拉 */
		.tGPIOInit.Speed 	= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
		.tGPIOPort 			= GPIOD,					/* GPIO端口 */
	},
	[2] =
	{ 
		.tGPIOInit.Pin 		= GPIO_PIN_15,				/* GPIO引脚 */
		.tGPIOInit.Mode 	= GPIO_MODE_OUTPUT_PP,		/* 推挽输出模式 */
		.tGPIOInit.Pull 	= GPIO_NOPULL,				/* 无上下拉 */
		.tGPIOInit.Speed 	= GPIO_SPEED_FREQ_HIGH,		/* GPIO速度 */	
		.tGPIOPort 			= GPIOD,					/* GPIO端口 */
	},
};

/* Timer2 定时0.1s中断 */
tagTIM_T tTimer2 = 
{
	.tTimerHandle.Instance				= TIM2,						/* 通用定时器2 */
	.tTimerHandle.Init.Prescaler		= 7200-1,					/* 预分频系数 */
	.tTimerHandle.Init.CounterMode		= TIM_COUNTERMODE_UP,		/* 向上计数模式 */
	.tTimerHandle.Init.Period			= 1000-1,					/* 0.1S产生一次中断 */
	.tTimerHandle.Init.ClockDivision	= TIM_CLOCKDIVISION_DIV1,	/* 时钟分频系数 */
	.tTimerHandle.Init.RepetitionCounter = 0,						/* 重复计数器，高级定时器用 */
	.tTimerHandle.Init.AutoReloadPreload = TIM_AUTOMATICOUTPUT_ENABLE, /* 自动重装载 */

	.ucPriority = 0,
	.ucSubPriority = 2,
};

/* 蓝牙通信串口1 */
tagUART_T Uart1 = 
{
	//串口基础配置
	.tUARTHandle.Instance 				= USART1,					/* 串口1 */
	.tUARTHandle.Init.BaudRate   		= 9600,						/* 波特率 */
	.tUARTHandle.Init.WordLength 		= UART_WORDLENGTH_8B,		/* 8位数据位 */
	.tUARTHandle.Init.StopBits   		= UART_STOPBITS_1,			/* 1位停止位 */
	.tUARTHandle.Init.Parity     		= UART_PARITY_NONE,			/* 无校验 */
	.tUARTHandle.Init.HwFlowCtl  		= UART_HWCONTROL_NONE,		/* 无硬件流控 */
	.tUARTHandle.Init.Mode       		= UART_MODE_TX_RX,			/* 收发模式 */
	.tUARTHandle.Init.OverSampling 		= UART_OVERSAMPLING_16,		/* 16倍过采样 */

#if defined (STM32L4_SGA_ENABLE)
	.tUARTHandle.Init.OneBitSampling 	= UART_ONE_BIT_SAMPLE_DISABLE,
	.tUARTHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT,
#endif
	
	.ucPriority							= 1,						/* 中断优先级 */
	.ucSubPriority						= 3,						/* 中断子优先级 */
	
	//接收DMA配置
	.tUartDMA.bRxEnable					= true,						/* 使能接收DMA */
	.tUartDMA.tDMARx.Instance			= DMA1_Channel5,
	.tUartDMA.tDMARx.Init.Direction		= DMA_PERIPH_TO_MEMORY,
	.tUartDMA.tDMARx.Init.PeriphInc		= DMA_PINC_DISABLE,
	.tUartDMA.tDMARx.Init.MemInc		= DMA_MINC_ENABLE,
	.tUartDMA.tDMARx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
	.tUartDMA.tDMARx.Init.MemDataAlignment	  = DMA_MDATAALIGN_BYTE,
	.tUartDMA.tDMARx.Init.Mode			= DMA_CIRCULAR,
	.tUartDMA.tDMARx.Init.Priority		= DMA_PRIORITY_LOW,

	.tRxInfo.usDMARxMAXSize             	= 100,              		/* 接收缓冲区大小 */

	.tUartDMA.ucDMARxPriority				= 1,
	.tUartDMA.ucDMARxSubPriority			= 1,
	
	//发送DMA配置
	.tUartDMA.bTxEnable					= true,						/* 使能发送DMA */
	.tUartDMA.tDMATx.Instance			= DMA1_Channel4,
	.tUartDMA.tDMATx.Init.Direction		= DMA_MEMORY_TO_PERIPH,
	.tUartDMA.tDMATx.Init.PeriphInc		= DMA_PINC_DISABLE,
	.tUartDMA.tDMATx.Init.MemInc		= DMA_MINC_ENABLE,
	.tUartDMA.tDMATx.Init.PeriphDataAlignment	= DMA_PDATAALIGN_BYTE,
	.tUartDMA.tDMATx.Init.MemDataAlignment		= DMA_MDATAALIGN_BYTE,
	.tUartDMA.tDMATx.Init.Mode			= DMA_NORMAL,
	.tUartDMA.tDMATx.Init.Priority		= DMA_PRIORITY_LOW,

	.tTxInfo.usDMATxMAXSize				= 50,						/* 发送缓冲区大小 */
	
	.tUartDMA.ucDMATxPriority				= 1,
	.tUartDMA.ucDMATxSubPriority			= 1,

	//串口GPIO配置 TX
	.tGPIO[0].tGPIOInit.Pin 			= GPIO_PIN_9,
	.tGPIO[0].tGPIOInit.Mode 			= GPIO_MODE_AF_PP,
	.tGPIO[0].tGPIOInit.Pull 			= GPIO_NOPULL,
	.tGPIO[0].tGPIOInit.Speed 			= GPIO_SPEED_FREQ_HIGH,	
	.tGPIO[0].tGPIOPort 				= GPIOA,
	.tGPIO[0].ucAFMode					= NO_REMAP,
	
	//串口GPIO配置 RX
	.tGPIO[1].tGPIOInit.Pin 			= GPIO_PIN_10,
	.tGPIO[1].tGPIOInit.Mode 			= GPIO_MODE_INPUT,
	.tGPIO[1].tGPIOInit.Pull 			= GPIO_NOPULL,
	.tGPIO[1].tGPIOInit.Speed 			= GPIO_SPEED_FREQ_HIGH,	
	.tGPIO[1].tGPIOPort 				= GPIOA,
	.tGPIO[1].ucAFMode					= NO_REMAP,
};

/* DYP-L08 前向超声波传感器（USART3, PB10/PB11, 115200, DMA1_Ch3 RX / DMA1_Ch2 TX） */
tagDYP_L08_T DYP_Forward =
{
	/* === 触发字节 === */
	.ucTriggerByte = 0x00,

	/* === UART 基础配置 === */
	.tUART.tUARTHandle.Instance 			= USART3,					/* 串口 3 */
	.tUART.tUARTHandle.Init.BaudRate   		= 115200,					/* DYP 默认 115200 */
	.tUART.tUARTHandle.Init.WordLength 		= UART_WORDLENGTH_8B,
	.tUART.tUARTHandle.Init.StopBits   		= UART_STOPBITS_1,
	.tUART.tUARTHandle.Init.Parity     		= UART_PARITY_NONE,
	.tUART.tUARTHandle.Init.HwFlowCtl  		= UART_HWCONTROL_NONE,
	.tUART.tUARTHandle.Init.Mode       		= UART_MODE_TX_RX,
	.tUART.tUARTHandle.Init.OverSampling 	= UART_OVERSAMPLING_16,

#if defined (STM32L4_SGA_ENABLE)
	.tUART.tUARTHandle.Init.OneBitSampling 	= UART_ONE_BIT_SAMPLE_DISABLE,
	.tUART.tUARTHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT,
#endif

	.tUART.ucPriority						= 3,						/* UART 中断优先级（比 JY901 的 1 低） */
	.tUART.ucSubPriority					= 0,

	/* === 接收 DMA 配置（USART3_RX 必须用 DMA1_Channel3） === */
	.tUART.tUartDMA.bRxEnable				= true,
	.tUART.tUartDMA.tDMARx.Instance			= DMA1_Channel3,			/* USART3_RX 硬连通道 */
	.tUART.tUartDMA.tDMARx.Init.Direction	= DMA_PERIPH_TO_MEMORY,
	.tUART.tUartDMA.tDMARx.Init.PeriphInc	= DMA_PINC_DISABLE,
	.tUART.tUartDMA.tDMARx.Init.MemInc		= DMA_MINC_ENABLE,
	.tUART.tUartDMA.tDMARx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
	.tUART.tUartDMA.tDMARx.Init.MemDataAlignment	= DMA_MDATAALIGN_BYTE,
	.tUART.tUartDMA.tDMARx.Init.Mode		= DMA_NORMAL,				/* 每次 4 字节收完重启 */
	.tUART.tUartDMA.tDMARx.Init.Priority	= DMA_PRIORITY_LOW,

	.tUART.tRxInfo.usDMARxMAXSize			= 16,						/* 回包 4 字节，留点余量 */

	.tUART.tUartDMA.ucDMARxPriority			= 3,
	.tUART.tUartDMA.ucDMARxSubPriority		= 0,

	/* === 发送 DMA 配置（USART3_TX 必须用 DMA1_Channel2） === */
	.tUART.tUartDMA.bTxEnable				= true,
	.tUART.tUartDMA.tDMATx.Instance			= DMA1_Channel2,			/* USART3_TX 硬连通道 */
	.tUART.tUartDMA.tDMATx.Init.Direction	= DMA_MEMORY_TO_PERIPH,
	.tUART.tUartDMA.tDMATx.Init.PeriphInc	= DMA_PINC_DISABLE,
	.tUART.tUartDMA.tDMATx.Init.MemInc		= DMA_MINC_ENABLE,
	.tUART.tUartDMA.tDMATx.Init.PeriphDataAlignment	= DMA_PDATAALIGN_BYTE,
	.tUART.tUartDMA.tDMATx.Init.MemDataAlignment	= DMA_MDATAALIGN_BYTE,
	.tUART.tUartDMA.tDMATx.Init.Mode		= DMA_NORMAL,
	.tUART.tUartDMA.tDMATx.Init.Priority	= DMA_PRIORITY_LOW,

	.tUART.tTxInfo.usDMATxMAXSize			= 8,						/* 触发只发 1 字节，余量足够 */

	.tUART.tUartDMA.ucDMATxPriority			= 3,
	.tUART.tUartDMA.ucDMATxSubPriority		= 0,

	/* === GPIO TX (PB10) === */
	.tUART.tGPIO[0].tGPIOInit.Pin 			= GPIO_PIN_10,				/* PB10 */
	.tUART.tGPIO[0].tGPIOInit.Mode 			= GPIO_MODE_AF_PP,
	.tUART.tGPIO[0].tGPIOInit.Pull 			= GPIO_NOPULL,
	.tUART.tGPIO[0].tGPIOInit.Speed 		= GPIO_SPEED_FREQ_HIGH,
	.tUART.tGPIO[0].tGPIOPort 				= GPIOB,
	.tUART.tGPIO[0].ucAFMode				= NO_REMAP,

	/* === GPIO RX (PB11, FT 引脚可承受 5V) === */
	.tUART.tGPIO[1].tGPIOInit.Pin 			= GPIO_PIN_11,				/* PB11 */
	.tUART.tGPIO[1].tGPIOInit.Mode 			= GPIO_MODE_INPUT,
	.tUART.tGPIO[1].tGPIOInit.Pull 			= GPIO_NOPULL,
	.tUART.tGPIO[1].tGPIOInit.Speed 		= GPIO_SPEED_FREQ_HIGH,
	.tUART.tGPIO[1].tGPIOPort 				= GPIOB,
	.tUART.tGPIO[1].ucAFMode				= NO_REMAP,
};

/* 推进器PWM配置数组 */
tagPWM_T thruster[] =
{
	[0] =
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比(%) */
		.ulFreq					= 50,				/* 频率(Hz) */
		.ucChannel				= TIM_CHANNEL_1,	/* 通道1 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_6,		/* PB6 */
		.tGPIO.tGPIOPort		= GPIOB,
		.tGPIO.ucAFMode			= NO_REMAP,			/* 无重映射 */
	},
	[1] =
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比(%) */
		.ulFreq					= 50,				/* 频率(Hz) */
		.ucChannel				= TIM_CHANNEL_2,	/* 通道2 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_7,		/* PB7 */
		.tGPIO.tGPIOPort		= GPIOB,
		.tGPIO.ucAFMode			= NO_REMAP,
	},	
	[2] =
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比(%) */
		.ulFreq					= 50,				/* 频率(Hz) */
		.ucChannel				= TIM_CHANNEL_3,	/* 通道3 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_8,		/* PB8 */
		.tGPIO.tGPIOPort		= GPIOB,
		.tGPIO.ucAFMode			= NO_REMAP,
	},
	[3]=
	{
		.tPWMHandle.Instance	= TIM4,         	/* 定时器4 */
		.fDuty					= 7.5,				/* 初始占空比(%) */
		.ulFreq					= 50,				/* 频率(Hz) */
		.ucChannel				= TIM_CHANNEL_4,	/* 通道4 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_9,		/* PB9 */
		.tGPIO.tGPIOPort		= GPIOB,
		.tGPIO.ucAFMode			= NO_REMAP,
	},

	[4]=
	{
		.tPWMHandle.Instance	= TIM3,          	/* 定时器3 */
		.fDuty				    = 7.5,                /* 初始占空比(%) */
		.ulFreq				    = 50,                /* 频率(Hz) */
		.ucChannel			    = TIM_CHANNEL_4,     /* 通道4 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_1,        /* PB1 */
		.tGPIO.tGPIOPort		= GPIOB,
		.tGPIO.ucAFMode			= NO_REMAP,
	},
	[5]=
	{
		.tPWMHandle.Instance	= TIM3,         	    /* 定时器3 */
		.fDuty					= 7.5,				    /* 初始占空比(%) */
		.ulFreq					= 50,				    /* 频率(Hz) */
		.ucChannel				= TIM_CHANNEL_3,	    /* 通道3 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_0,		    /* PB0 */
		.tGPIO.tGPIOPort		= GPIOB,
		.tGPIO.ucAFMode			= NO_REMAP,
	},
};

/* 前向扫描舵机 PWM 配置：TIM1_CH1 → PE9（TIM1 全重映射）
 * 50Hz 标准舵机周期，初始 7.5% duty = 1500us = 中位 */
tagPWM_T servo_front[] =
{
	[0] =
	{
		.tPWMHandle.Instance	= TIM1,         	    /* 定时器1 */
		.fDuty					= 7.5,				    /* 初始占空比 1500us / 20000us */
		.ulFreq					= 50,				    /* 50Hz 标准舵机 */
		.ucChannel				= TIM_CHANNEL_1,	    /* 通道1 */
		.tGPIO.tGPIOInit.Pin	= GPIO_PIN_9,		    /* PE9 */
		.tGPIO.tGPIOPort		= GPIOE,
		.tGPIO.ucAFMode			= FULL_REMAP,		    /* TIM1 全重映射 → CH1 出在 PE9 */
	},
};

/* 姿态传感器 JY901S 配置 */
tagJY901_T JY901S = 
{
	.tConfig.ucBaud 	= JY901_RXBAUD_9600,		/* 波特率9600 */
	.tConfig.ucRate		= JY901_RX_200HZ,			/* 输出频率200Hz */
	.tConfig.usType		= JY901_OUTPUT_ACCEL | JY901_OUTPUT_GYRO | JY901_OUTPUT_ANGLE | JY901_OUTPUT_MAG | JY901_OUTPUT_GPS,
    .tConfig.ucOrient   = JY901_ORIENT_HORIZONTAL,
    .tConfig.ucAxis     = JY901_AXIS_6,

	.tUART.tUARTHandle.Instance 				= USART2,			/* 串口2 */
	.tUART.tUARTHandle.Init.BaudRate   			= 9600,				/* 波特率 */
	.tUART.tUARTHandle.Init.WordLength 			= UART_WORDLENGTH_8B,
	.tUART.tUARTHandle.Init.StopBits   			= UART_STOPBITS_1,
	.tUART.tUARTHandle.Init.Parity     			= UART_PARITY_NONE,
	.tUART.tUARTHandle.Init.HwFlowCtl  			= UART_HWCONTROL_NONE,
	.tUART.tUARTHandle.Init.Mode       			= UART_MODE_TX_RX,
	.tUART.tUARTHandle.Init.OverSampling 		= UART_OVERSAMPLING_16,

	.tUART.tRxInfo.usDMARxMAXSize             	= 300,              /* 接收缓冲区大小 */

#if defined (STM32L4_SGA_ENABLE)
	.tUARTHandle.Init.OneBitSampling 	= UART_ONE_BIT_SAMPLE_DISABLE,
	.tUARTHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT,
#endif
	
	.tUART.ucPriority							= 1,
	.tUART.ucSubPriority						= 3,
	
    .tUART.tUartDMA.bRxEnable					    = true,
	.tUART.tUartDMA.tDMARx.Instance					= DMA1_Channel6,
	.tUART.tUartDMA.tDMARx.Init.Direction			= DMA_PERIPH_TO_MEMORY,
	.tUART.tUartDMA.tDMARx.Init.PeriphInc			= DMA_PINC_DISABLE,
	.tUART.tUartDMA.tDMARx.Init.MemInc				= DMA_MINC_ENABLE,
	.tUART.tUartDMA.tDMARx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
	.tUART.tUartDMA.tDMARx.Init.MemDataAlignment	= DMA_MDATAALIGN_BYTE,
	.tUART.tUartDMA.tDMARx.Init.Mode				= DMA_CIRCULAR,
	.tUART.tUartDMA.tDMARx.Init.Priority			= DMA_PRIORITY_LOW,

	.tUART.tUartDMA.ucDMARxPriority				= 1,
	.tUART.tUartDMA.ucDMARxSubPriority			= 1,
	
	//TX PA2
	.tUART.tGPIO[0].tGPIOInit.Pin 			= GPIO_PIN_2,
	.tUART.tGPIO[0].tGPIOInit.Mode 			= GPIO_MODE_AF_PP,
	.tUART.tGPIO[0].tGPIOInit.Pull 			= GPIO_NOPULL,
	.tUART.tGPIO[0].tGPIOInit.Speed 		= GPIO_SPEED_FREQ_HIGH,	
	.tUART.tGPIO[0].tGPIOPort 				= GPIOA,
	.tUART.tGPIO[0].ucAFMode				= NO_REMAP,
	
	//RX PA3
	.tUART.tGPIO[1].tGPIOInit.Pin 			= GPIO_PIN_3,
	.tUART.tGPIO[1].tGPIOInit.Mode 			= GPIO_MODE_INPUT,
	.tUART.tGPIO[1].tGPIOInit.Pull 			= GPIO_NOPULL,
	.tUART.tGPIO[1].tGPIOInit.Speed 		= GPIO_SPEED_FREQ_HIGH,	
	.tUART.tGPIO[1].tGPIOPort 				= GPIOA,
	.tUART.tGPIO[1].ucAFMode				= NO_REMAP,
};





/* 独立看门狗配置 */
tagIWDG_T demoIWDG =
{
	.usResetTime = 1000,			/* 1秒喂狗一次 */
};

