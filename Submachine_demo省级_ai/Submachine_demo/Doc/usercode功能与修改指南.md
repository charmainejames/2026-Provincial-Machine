# usercode.c 功能与修改指南

> 文件位置：[Apply/Logic/Src/usercode.c](../Apply/Logic/Src/usercode.c) · 头文件 [Apply/Logic/Inc/usercode.h](../Apply/Logic/Inc/usercode.h)
> 配套：[config.c](../Apply/Logic/Src/config.c)（PID 系数 / 句柄）、[task_thruster.c](../Apply/Task/Src/task_thruster.c)（动作函数）、[task_irq.c](../Apply/Task/Src/task_irq.c)（JY901 中断接收）

---

## 0. 一句话总览

`usercode.c` 是水下机器人的**业务大脑**，做三件事：

1. **听** —— 串口 1 DMA 收树莓派字符串指令；
2. **想** —— 维护一组标志位 + 偏航角 PID，决定推进器该出多少力；
3. **动** —— 把 PWM 脉宽写到 6 个推进器。

所有底层封装（PWM/UART/JY901/PID）都已在 [task_userinit.c](../Apply/Task/Src/task_userinit.c) 里初始化好，本文件只调 SGA 库 API，不接触寄存器。

---

## 1. 主循环 `UserLogic_Code()`

```
┌────────────────────────────────────────────────────────────┐
│ printf("Underwater robot starts")                          │
│                                                            │
│ while (1) {                                                │
│   memset(RaspberryPi_data, 0, 150);            // 清缓冲   │
│   num = Drv_Uart_Receive_DMA(&Uart1, ...);     // 1. 听    │
│   if (num > 0)  thruster_start_open();         // 2. 解析  │
│                                                            │
│   if (rotate_180_open_flag)  jy901_yaw_anti_rotate_open(); │
│   if (forward_open_flag)     jy901_yaw_anti_forward_open();│
│                                                            │
│   Drv_IWDG_Feed(&demoIWDG);                    // 喂狗     │
│ }                                                          │
└────────────────────────────────────────────────────────────┘
```

**关键设计**：业务函数全部用 **"标志位 + 主循环轮询"** 触发，不在中断里做控制。这样：

- JY901 数据更新（USART2 中断里调 `OCD_JY901_DataConversion`）和 PID 计算（主循环里读 `JY901S.stcAngle.ConYaw`）解耦；
- 主循环节奏稳定（约等于一帧 JY901 周期，5 ms @ 200 Hz）；
- 看门狗在循环末尾喂，任何业务函数死循环都会触发 1 s 复位。

---

## 2. 业务模式与标志位

| 标志 | 含义 | 谁置位 | 谁清零 |
|---|---|---|---|
| `forward_open_flag` | 直行稳定开启 | `JSB 7 Press` | `jy901_yaw_anti_rotation_cancel()` |
| `rotate_180_open_flag` | 180° 自转开启 | `JSB 5 Press` | `jy901_yaw_anti_rotation_cancel()` 或 自转到位后 |
| `forward_yaw_get_flag` | 本次直行已锁定目标角 | `jy901_yaw_anti_forward_open` 内部 | 每次 `JSB 7 Press` 时 |
| `right_rotate_180_get_flag` | 本次自转已锁定目标角和方向 | `jy901_yaw_anti_rotate_open` 内部 | `cancel` 时 |
| `forward_count` | 已收到的直走指令次数 | `JSB 7 Press` 自增 | 永不清零（电源复位才清） |

**两个开启标志互斥：** 收到 `JSB 5/7` 时都会先调 `jy901_yaw_anti_rotation_cancel()`，把对方关掉。

---

## 3. 树莓派指令协议

主板和树莓派约定 ASCII 字符串，格式 `JSB <按键号> <事件>`，事件有 `Press` / `Release`。

| 字符串 | 触发动作 | 实现 |
|---|---|---|
| `JSB 3 Press` | 垂直上升 | `thruster_vertical_up()` → thruster[2/3]=1750 |
| `JSB 3 Release` | 垂直停止 | `thruster_vertical_stop()` → thruster[2/3]=1500 |
| `JSB 0 Press` | 垂直下潜 | `thruster_vertical_down()` → thruster[2/3]=1250 |
| `JSB 7 Press` | 直行（带 PID 偏航稳定） | 关其他模式 → `forward_count++` → 解锁目标角 → 置 `forward_open_flag=1` |
| `JSB 7 Release` | 停止直行 | `jy901_yaw_anti_rotation_cancel()` |
| `JSB 5 Press` | 右转 180° | 关其他模式 → 置 `rotate_180_open_flag=1` |
| `JSB 6 Press` | 左转（无 PID） | 关其他模式 → `thruster_horizontal_left_turn()` |
| `JSB 11 Press` | **测试**：6 个推进器全 1750 | 直接调 `Drv_PWM_HighLvTimeSet` |

> 缓冲区每次循环 `memset` 清零，所以**树莓派必须每次发送都带完整字符串**，不能只发增量字节。
> 帧匹配靠 `strcmp`，所以**数据末尾必须是 `\0`**。SGA 的 DMA 接收会自动在尾部追写 0。

---

## 4. 直行稳定 `jy901_yaw_anti_forward_open()`

### 4.1 控制思路

**位置式 PID**：以收到第一次 `JSB 7 Press` 时的偏航角 `oneoneone` 为永久基准。后续每次按下：

| forward_count | 目标角 |
|---|---|
| 1 | `oneoneone` |
| 2 | `oneoneone − 180°` |
| 3 | `oneoneone` |
| 4 | `oneoneone − 180°` |
| ... | 奇数次 = `oneoneone`，偶数次 = `oneoneone − 180°` |

⇒ 实现"出发 → 折返 → 折返 → ..." 的水下来回巡线。

### 4.2 控制特性

代码顶部一组 `const` 常量是所有可调旋钮：

```c
const float DEAD_ZONE      = 0.8f;    // 误差小于此值不动作
const float INT_SEPARATION = 12.0f;   // 仅误差小于此才积分
const float LARGE_ERROR    = 25.0f;   // 大偏差阈值
const float INT_MAX        = 250.0f;  // 积分项限幅
const int   OUT_MAX_SMALL  = 120;     // 小偏差时输出限幅
const int   OUT_MAX_LARGE  = 220;     // 大偏差时输出限幅（救火）
const int   MIN_EFFECTIVE  = 30;      // ESC 死区补偿
const int   SLEW_RATE      = 18;      // 输出每周期最大变化量
```

### 4.3 流程图

```
读 JY901S.stcAngle.ConYaw
        │
        ▼
error = -AngleDifference(target, current)   ← 最短路径，避开 ±180° 跳变
        │
        ├─ |err| < DEAD_ZONE  ─→ 积分缓衰减 + 输出向 0 平滑 + 退出
        │
        ├─ |err| > LARGE_ERROR (首进) ─→ 清积分 + 重置微分参考
        │
        ├─ |err| < INT_SEPARATION ─→ 积分累积，否则积分 *= 0.85
        │
        ▼
output = Kp*err + Ki*integral + Kd*derivative
        │
        ├─ 动态限幅 (OUT_MAX_SMALL / OUT_MAX_LARGE)
        ├─ 速率限幅 (|delta| ≤ SLEW_RATE)
        └─ ESC 死区补偿 (|out| < MIN_EFFECTIVE 时拉到 ±MIN_EFFECTIVE)
        │
        ▼
forward_adjust(output, -output)
        ├─ thruster[0] = 1700 + output   ← 左推进器
        └─ thruster[1] = 1700 − output   ← 右推进器
```

> 误差正负的物理含义：`error > 0` ⇒ 当前角相对目标偏顺时针 ⇒ 需要"向左"修正 ⇒ 左推进器加力，右推进器减力。

---

## 5. 180° 自转 `jy901_yaw_anti_rotate_open()`

### 5.1 控制思路

**三段式速度规划 + 方向锁存**：

```
   目标角 (current + 180°)
            │
   ┌────────┼────────┐  距离目标的剩余角度
   │        │        │
  远段     中段     近端
  (>20°)   (5-20°)   (<5°)
   │        │        │
   PID      固定      固定
   主导     SLOW_OFF  CREEP_OFF
            (90)      (60)
```

为什么需要三段？纯 PID 在接近 180° 时容易因为最短路径方向切换而**"在终点附近来回抖动"**。
解决：**进入旋转的瞬间锁定 `rotate_direction`（+1 或 −1），中段/近端固定按这个方向出力**。

### 5.2 关键参数

```c
const float TARGET_TOLERANCE = 1.0f;   // 到位容限
const float SLOW_ANGLE   = 20.0f;      // 减速阈值
const float CREEP_ANGLE  = 5.0f;       // 蠕行阈值
const int   MIN_OFFSET   = 55;         // ESC 死区
const int   MAX_OFFSET   = 260;        // 最大旋转推力
const int   SLOW_OFFSET  = 90;         // 减速段固定推力
const int   CREEP_OFFSET = 60;         // 蠕行段固定推力
const int   SLEW_RATE    = 25;
const uint8_t STABLE_COUNT = 3;        // 连续 3 帧到位才算成功
```

### 5.3 退出条件

`|err| ≤ TARGET_TOLERANCE` 连续 `STABLE_COUNT` 帧 → 调 `jy901_yaw_anti_rotation_cancel()` 自动退出，
推进器复位 1500，同时 `rotate_180_open_flag = 0`。

---

## 6. 输出层（PWM 写入）

| 函数 | 基准 PWM | 限幅 | 用于 |
|---|---|---|---|
| `forward_adjust(L, R)` | 1700 | [1400, 2000] | 直行（左右推进器同向，加 ± offset 实现纠偏） |
| `rotate_180_adjust(L, R)` | 1500 | [1300, 1700] | 自转（左右反向，含 40us 电机死区补偿） |

> 推进器编号见 [config.c](../Apply/Logic/Src/config.c) 的 `thruster[]` 数组：
> `[0] PB6 水平左` · `[1] PB7 水平右` · `[2] PB8 垂直左前` · `[3] PB9 垂直右前` · `[4] PB1 垂直左后` · `[5] PB0 垂直右后`

---

## 7. 全局变量地图

| 变量 | 类型 | 用途 | 修改频率 |
|---|---|---|---|
| `RaspberryPi_data[150]` | `uint8_t[]` | 串口 1 DMA 接收缓冲 | 每循环 memset |
| `tx_buf[150]` | `uint8_t[]` | 预留发送缓冲 | 当前未使用 |
| `num` | `uint8_t` | 本次 DMA 收到的字节数 | 每循环 |
| `forward_count` | `uint8_t` | 已收到的直走指令次数 | `JSB 7 Press` 时 +1 |
| `oneoneone` | `float` | 第一次直走的基准偏航角 | 仅 `forward_count==1` 时写一次 |
| `forward_yaw_target` | `float` | 当前直行目标角 | 每次 `JSB 7 Press` 重算 |
| `turn_180_target` | `float` | 当前自转目标角 | 进入自转时算一次 |
| `yaw_current` | `float` | 当前偏航角缓存（调试用） | 每次 PID 周期 |
| `yaw_output` / `left_offset` / `right_offset` | `int` | PID 输出缓存（调试用） | 每次 PID 周期 |
| `rotate_180_ready_count` | `uint8_t` | 自转到位稳定计数 | 每帧 |
| 上面所有标志位 | `uint8_t` | 见 §2 | 状态切换时 |

> ⚠ 当前所有变量都没加 SGA 规范要求的 `g_/uc/us/ul/f` 前缀。这是历史遗留，**新加的全局变量请按规范命名**：例如 `g_ucForwardCount`、`g_fOneonenoe`、`g_fForwardYawTarget`。

---

## 8. 修改方法（菜谱）

### 8.1 加一条新指令（最常见）

例：让 `JSB 4 Press` 触发"垂直推进器全反向"。

1. 在 [task_thruster.c](../Apply/Task/Src/task_thruster.c) 新增动作函数：
   ```c
   void thruster_vertical_reverse(void)
   {
       Drv_PWM_HighLvTimeSet(&thruster[2], 1250);
       Drv_PWM_HighLvTimeSet(&thruster[3], 1250);
   }
   ```
2. 在 [task_thruster.h](../Apply/Task/Inc/task_thruster.h) 加声明 `void thruster_vertical_reverse(void);`
3. 在本文件 `thruster_start_open()` 链尾加一个 `else if`：
   ```c
   else if (strcmp((char*)RaspberryPi_data, "JSB 4 Press") == 0)
   {
       thruster_vertical_reverse();
   }
   ```

### 8.2 调 PID 系数（不动逻辑）

直接改 [config.c:321-336](../Apply/Logic/Src/config.c) 的 `PID` / `PID_init`：

```c
tagPID_T PID =
{
    .fKp = 2.5f,    // ← 改这里：增大 = 响应更快但易振荡
    .fKi = 0.15f,   // ← 改这里：增大 = 稳态误差更小但易超调
    .fKd = 0.5f     // ← 改这里：增大 = 阻尼更强但对噪声敏感
};
```

> 注意 `PID` 和 `PID_init` 的 Ki 值**当前不一致**（0.15 vs 0.05），usercode 实际生效的是 `PID.fKp/fKi/fKd`（被 `PID_Update` 一刷就成 init 的值）。建议两边保持一致，或只改一处。

### 8.3 调直行控制特性（不动 PID）

例：把死区放宽到 1.5°，加大救火时的限幅。
改 [usercode.c](../Apply/Logic/Src/usercode.c) `jy901_yaw_anti_forward_open()` 顶部的 `const`：

```c
const float DEAD_ZONE     = 1.5f;  // 0.8 → 1.5
const int   OUT_MAX_LARGE = 280;   // 220 → 280
```

### 8.4 调直行基准推力

改 [usercode.c](../Apply/Logic/Src/usercode.c) `forward_adjust()`：

```c
int right_pwm = 1700 + right_offset;  // ← 改这个 1700
int left_pwm  = 1700 + left_offset;   // ← 改这个 1700
```

> 同步注意限幅：`(right_pwm < 1400) ? 1400 : (right_pwm > 2000 ? 2000 : ...)`。
> 基准上调时上限 2000 也可适度上调（注意 ESC 上限通常 2000）。

### 8.5 修改往返模式（例如改成 4 个方位巡逻）

`jy901_yaw_anti_forward_open()` 里 `if (forward_count == ...)` 链：

```c
if (forward_count == 1)
    forward_yaw_target = NormalizeAngle(oneoneone +   0.0f);   // 北
else if (forward_count == 2)
    forward_yaw_target = NormalizeAngle(oneoneone +  90.0f);   // 东
else if (forward_count == 3)
    forward_yaw_target = NormalizeAngle(oneoneone + 180.0f);   // 南
else if (forward_count == 4)
    forward_yaw_target = NormalizeAngle(oneoneone + 270.0f);   // 西
else
    forward_yaw_target = NormalizeAngle(oneoneone + (forward_count % 4) * 90.0f);
```

每个分支记得保留 `integral=0; last_error=0; last_output=0; large_err_latch=0; PID_Clear(&PID);`。

### 8.6 调 180° 自转特性

改 [usercode.c](../Apply/Logic/Src/usercode.c) `jy901_yaw_anti_rotate_open()` 顶部 `const`：

| 想要的效果 | 改谁 |
|---|---|
| 转到位的精度提高 | `TARGET_TOLERANCE` 1.0 → 0.5（同时 `STABLE_COUNT` 适当增大） |
| 中段提前减速 | `SLOW_ANGLE` 20.0 → 30.0 |
| 蠕行段更稳但更慢 | `CREEP_OFFSET` 60 → 50；`CREEP_ANGLE` 5.0 → 8.0 |
| 自转更快 | `MAX_OFFSET` 260 → 320；`SLOW_OFFSET` 90 → 110 |

### 8.7 改成"用左转代替右转"

把 `jy901_yaw_anti_rotate_open()` 第一段：

```c
turn_180_target = NormalizeAngle(current_yaw + 180.0f);   // 右转 180°
```

改成：

```c
turn_180_target = NormalizeAngle(current_yaw - 180.0f);   // 左转 180°
```

注意 `current_yaw + 180.0f` 和 `current_yaw - 180.0f` 在数学上是同一个角，
所以方向其实由后面的 `rotate_direction` 决定。要真正切换方向，应该**强制 `rotate_direction = -1.0f`**：

```c
float init_diff = AngleDifference(turn_180_target, current_yaw);
rotate_direction = -1.0f;   // 强制左转，不再依据 init_diff
```

### 8.8 关闭 PID 直行，回到"裸 PWM"模式

最简单：把 `JSB 7 Press` 分支改成直接调动作函数：

```c
else if (strcmp((char*)RaspberryPi_data, "JSB 7 Press") == 0)
{
    jy901_yaw_anti_rotation_cancel();
    thruster_horizontal_forward();   // 不用 PID，左右各 1750
}
```

### 8.9 主循环加新业务

在 `UserLogic_Code()` while 里看门狗之前加：

```c
if (your_flag == 1)
{
    your_business_function();
}
```

业务函数做成"标志位驱动"，不要在里面 `while` 循环，否则会饿死看门狗。

---

## 9. 调试技巧

### 9.1 看 JY901 实时姿态
在 USART2 中断里取消注释 [task_irq.c](../Apply/Task/Src/task_irq.c) 这几行：

```c
OCD_JY901_Printf(&JY901S);
printf("\r\n");
```

> 注意：在中断里 printf 可能阻塞，调试用就好，正式跑要关掉。

### 9.2 看 PID 内部量
当前 `yaw_output` / `left_offset` / `right_offset` / `yaw_current` 都是全局，可以在主循环末尾加：

```c
printf("yaw=%.2f tgt=%.2f out=%d L=%d R=%d\r\n",
       yaw_current, forward_yaw_target, yaw_output, left_offset, right_offset);
```

### 9.3 看主循环节奏
临时给一个 LED 翻转：

```c
Drv_GPIO_Toggle(&demoGPIO[1]);   // 绿灯
```

灯频率 ≈ 主循环频率 / 2，正常应该 100 Hz 左右（5 ms 一帧 JY901）。

### 9.4 复位卡死
把 `Drv_IWDG_Feed(&demoIWDG);` 临时注释掉，可让 1 s 内死循环触发硬复位，配合串口看上电日志判断是哪段死的。

---

## 10. 常见坑与禁区

| 现象 | 原因 / 解决 |
|---|---|
| 直行时角度跳到 ±180° 附近就乱转 | 不要用 `target - current` 直接做误差，必须用 `AngleDifference()`（已修） |
| `JSB 7 Press` 收不到反应 | 检查 [task_irq.c](../Apply/Task/Src/task_irq.c) USART1 中断里有没有 `Drv_Uart_DMA_RxHandler(&Uart1)` |
| JY901 数据不更新（一直是 0） | 检查 USART2 中断里 `OCD_JY901_DataProcess + DataConversion` 有没有调；检查 [config.c](../Apply/Logic/Src/config.c) 的 JY901 波特率 / 输出速率 |
| 推进器抖动严重 | `SLEW_RATE` 太大；或 `MIN_EFFECTIVE` 太小没盖住 ESC 死区 |
| 转 180° 在终点附近来回抖 | 没启用 `rotate_direction` 锁存；或 `CREEP_OFFSET` 比死区还小 |
| 改了 `forward_count==1` 分支但 `oneoneone` 没刷 | `oneoneone` 只在第一次写。要重置必须断电或加复位指令把 `forward_count = 0` |
| 改 PID 常量没生效 | `usercode.c` 里那组 `const` 是函数局部，必须重新编译固件。`config.c` 的 `PID.fKp/fKi/fKd` 是 RAM 全局，调试时可以改 RAM 值热更新 |
| 主循环越来越慢 | 看是不是在某个分支里 `printf` 调太多；DMA 串口和裸 printf 共用 USART1，密集调用会阻塞 |

---

## 11. 文件依赖速览

```
usercode.c
├── usercode.h          → 函数声明
├── drv_hal_conf.h      → 拉入所有 Drv_* API
├── task_conf.h         → 拉入 task_* (含 task_thruster 的动作函数)
├── ocd_conf.h          → 拉入 OCD_*（JY901）
├── dev_conf.h          → 拉入 Dev_*
├── algo_conf.h         → 拉入 Algo_* + PID_*
└── config.h            → 拉入 thruster[] / Uart1 / JY901S / PID / demoIWDG 等句柄
```

要新加业务用到的外设/算法，直接 `#include` 对应 `*_conf.h`，对外句柄在 [config.c](../Apply/Logic/Src/config.c) 创建、[config.h](../Apply/Logic/Inc/config.h) 加 `extern`，[task_userinit.c](../Apply/Task/Src/task_userinit.c) 加 `Drv_xxx_Init`。

---

## 12. 参考

- [工程接手指南.md](工程接手指南.md) —— 工程整体架构
- [SGA库代码规范V1.1.md](SGA库代码规范V1.1.md) —— 命名/注释规范
- [句柄资源示例.txt](句柄资源示例.txt) —— 加新外设时的模板
