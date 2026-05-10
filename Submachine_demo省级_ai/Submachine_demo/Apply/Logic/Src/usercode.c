#include "usercode.h"
#include "drv_hal_conf.h"
#include "task_conf.h"
#include "ocd_conf.h"
#include "dev_conf.h"
#include "config.h"

uint8_t num;
uint8_t RaspberryPi_data[150];  // 树莓派接收数据缓冲区

// ====================== 角度获取标志位 ======================
uint8_t forward_yaw_get_flag = 0;        // 直行时目标偏航角获取标志
uint8_t right_rotate_180_get_flag = 0;   // 180度旋转目标角获取标志

float yaw_current = 0.0f;    // 当前偏航角
int yaw_output = 0;          // PID计算输出值
int left_offset = 0;         // 左侧推进器补偿量
int right_offset = 0;        // 右侧推进器补偿量
uint8_t rotate_180_ready_count = 0;  // 180度旋转到位稳定计数

uint8_t tx_buf[150];         // 发送缓冲区

// ====================== 运动开启标志 ======================
uint8_t forward_open_flag = 0;        // 直行开启标志
uint8_t rotate_180_open_flag = 0;     // 180度旋转开启标志

// ====================== 目标角度 ======================
float forward_yaw_target = 0.0f;      // 直行目标偏航角
float turn_180_target = 0.0f;         // 180度右转目标角度

// ====================== 直接在这里定义 Kp Ki Kd，不依赖任何库 ======================
float Kp = 12.0f;  // 你自己可以调
float Ki = 0.3f;
float Kd = 0.8f;

/* 用户主逻辑函数 */
void UserLogic_Code(void)
{
    printf("Underwater robot starts\r\n");  // 水下机器人启动

    while(1)
    {
        // 清空接收缓冲区
        memset(RaspberryPi_data, 0, sizeof(RaspberryPi_data));

        // 1. 串口DMA接收树莓派指令
        num = Drv_Uart_Receive_DMA(&Uart1, RaspberryPi_data);
        if (num > 0)
        {
            thruster_start_open();  // 解析树莓派指令
        }

        // 2. 执行180度旋转
        if(rotate_180_open_flag == 1)
        {
            jy901_yaw_anti_rotate_open();
        }

        // 3. 执行直线行驶（带偏航角稳向）
        if(forward_open_flag == 1)
        {
            jy901_yaw_anti_forward_open();
        }

        // 喂狗（看门狗）
        Drv_IWDG_Feed(&demoIWDG);
    }
}

// 树莓派指令解析函数
void thruster_start_open(void)
{
/*-----------------------------------垂直方向控制-----------------------------------*/
    // 垂直向上
    if(strcmp((char *)RaspberryPi_data,"JSB 3 Press") == 0)
    {
        thruster_vertical_up();
    }
    // 垂直停止
    else if(strcmp((char *)RaspberryPi_data,"JSB 3 Release") == 0)
    {
        thruster_vertical_stop();
    }
    // 垂直向下
    else if(strcmp((char *)RaspberryPi_data,"JSB 0 Press") == 0)
    {
        thruster_vertical_down();
    }

/*-----------------------------------水平方向控制-----------------------------------*/
    // 水平直走（带稳向）
    else if(strcmp((char *)RaspberryPi_data,"JSB 7 Press") == 0)
    {
        jy901_yaw_anti_rotation_cancel();  // 先关闭其他稳向功能
        forward_open_flag = 1;             // 开启直行标志
    }
    // 右转180度掉头
    else if(strcmp((char *)RaspberryPi_data,"JSB 5 Press") == 0)
    {
        jy901_yaw_anti_rotation_cancel();  // 先关闭其他稳向功能
        rotate_180_open_flag = 1;          // 开启180度旋转标志
    }

    // 停止直行
    else if(strcmp((char *)RaspberryPi_data,"JSB 7 Release") == 0)
    {
        jy901_yaw_anti_rotation_cancel();
    }

    // 左转
    else if(strcmp((char *)RaspberryPi_data,"JSB 6 Press") == 0)
    {
        jy901_yaw_anti_rotation_cancel();
        thruster_horizontal_left_turn();
    }
    // 全推进器向下测试
    else if(strcmp((char *)RaspberryPi_data,"JSB 11 Press") == 0)
    {
        Drv_PWM_HighLvTimeSet(&thruster[0],1750);
        Drv_PWM_HighLvTimeSet(&thruster[1],1750);
        Drv_PWM_HighLvTimeSet(&thruster[2],1750);
        Drv_PWM_HighLvTimeSet(&thruster[3],1750);
        Drv_PWM_HighLvTimeSet(&thruster[4],1750);
        Drv_PWM_HighLvTimeSet(&thruster[5],1750);
    }
}

// ====================== 角度归一化 & 最短路径差值 ======================
// 归一化到 [-180, 180]
static float NormalizeAngle(float angle)
{
    while(angle > 180.0f)  angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

// 目标 - 当前（最短路径）
static float AngleDifference(float target, float current)
{
    float diff = target - current;
    while(diff > 180.0f)  diff -= 360.0f;
    while(diff < -180.0f) diff += 360.0f;
    return diff;
}

// 偏航角稳向直行函数（位置式PID + 环绕安全 + 积分分离 + 斜率限幅）
void jy901_yaw_anti_forward_open(void)
{
    // —— 可调参数 ——
    const float DEAD_ZONE       = 0.8f;    // 死区（度）
    const float INT_SEPARATION  = 12.0f;   // 积分分离阈值（度）
    const float LARGE_ERROR     = 25.0f;   // 大偏差阈值（度）
    const float INT_MAX         = 250.0f;  // 积分限幅
    const int   OUT_MAX_SMALL   = 120;     // 小偏差时输出上限
    const int   OUT_MAX_LARGE   = 220;     // 大偏差时输出上限（留出恢复能力）
    const int   MIN_EFFECTIVE   = 30;      // ESC 死区补偿
    const int   SLEW_RATE       = 18;      // 每周期最大变化量，防止抖动

    static float integral     = 0.0f;
    static float last_error   = 0.0f;
    static int   last_output  = 0;
    static uint8_t large_err_latch = 0;    // 大偏差锁存，用于退出时重置

    // 首次进入：锁定目标偏航角
    if(forward_yaw_get_flag == 0)
    {
        forward_yaw_target = JY901S.stcAngle.ConYaw;
        forward_yaw_get_flag = 1;
        integral = 0.0f;
        last_error = 0.0f;
        last_output = 0;
        large_err_latch = 0;
    }

    yaw_current = JY901S.stcAngle.ConYaw;

    // 【关键】用最短路径差值计算误差，彻底解决 ±180° 跳变
    // error 正 → 当前相对目标偏"顺时针"（与原代码符号一致: current - target）
    float error    = -AngleDifference(forward_yaw_target, yaw_current);
    float abs_err  = fabs(error);

    // —— 死区：小扰动下不动作，但维持状态连续性 ——
    if(abs_err < DEAD_ZONE)
    {
        integral *= 0.95f;     // 缓慢泄放积分，避免稳态残余
        last_error = error;
        // 平滑收敛到 0，避免突变
        if(last_output > SLEW_RATE)       last_output -= SLEW_RATE;
        else if(last_output < -SLEW_RATE) last_output += SLEW_RATE;
        else                              last_output  = 0;
        left_offset  = last_output;
        right_offset = -last_output;
        forward_adjust(left_offset, right_offset);
        return;
    }

    // —— 大偏差事件：首次进入时清除历史，避免旧状态污染 ——
    if(abs_err > LARGE_ERROR)
    {
        if(!large_err_latch)
        {
            integral = 0.0f;           // 抗积分饱和
            last_error = error;        // 重置微分参考，防止微分项爆冲
            large_err_latch = 1;
        }
    }
    else
    {
        large_err_latch = 0;
    }

    // —— 积分分离：仅在误差较小时积分，防止 windup ——
    if(abs_err < INT_SEPARATION)
    {
        integral += error;
        if(integral >  INT_MAX) integral =  INT_MAX;
        if(integral < -INT_MAX) integral = -INT_MAX;
    }
    else
    {
        integral *= 0.85f;  // 大误差时缓慢衰减积分
    }

    // —— 位置式 PID（使用本地定义的 Kp/Ki/Kd）——
    float derivative = error - last_error;
    last_error = error;

    float p_term = Kp * error;
    float i_term = Ki * integral;
    float d_term = Kd * derivative;

    int output = (int)(p_term + i_term + d_term);

    // —— 动态输出限幅：大偏差给更大修正权限，但仍受控 ——
    int out_max = (abs_err > LARGE_ERROR) ? OUT_MAX_LARGE : OUT_MAX_SMALL;
    if(output >  out_max) output =  out_max;
    if(output < -out_max) output = -out_max;

    // —— 斜率限幅：防止输出突变造成抖动/振荡 ——
    int delta = output - last_output;
    if(delta >  SLEW_RATE) output = last_output + SLEW_RATE;
    if(delta < -SLEW_RATE) output = last_output - SLEW_RATE;
    last_output = output;

    // —— ESC 死区补偿 ——
    if(output > 0 && output <  MIN_EFFECTIVE) output =  MIN_EFFECTIVE;
    if(output < 0 && output > -MIN_EFFECTIVE) output = -MIN_EFFECTIVE;

    yaw_output   = output;
    left_offset  = output;
    right_offset = -output;
    forward_adjust(left_offset, right_offset);
}

// 180度旋转主函数（位置式PID + 环绕安全 + 三段式速度规划）
void jy901_yaw_anti_rotate_open(void)
{
    // —— 可调参数 ——
    const float TARGET_TOLERANCE = 1.0f;   // 到位容限（度）
    const float SLOW_ANGLE       = 20.0f;  // 临近目标减速区（度）
    const float CREEP_ANGLE      = 5.0f;   // 精定位爬行区（度）
    const int   MIN_OFFSET       = 55;     // ESC 死区补偿
    const int   MAX_OFFSET       = 260;    // 转弯最大推力
    const int   SLOW_OFFSET      = 90;     // 减速段输出
    const int   CREEP_OFFSET     = 60;     // 爬行段输出
    const int   SLEW_RATE        = 25;     // 斜率限幅
    const float INT_SEPARATION   = 20.0f;  // 积分分离阈值
    const float INT_MAX          = 200.0f;
    const uint8_t STABLE_COUNT   = 3;

    static float integral    = 0.0f;
    static float last_error  = 0.0f;
    static int   last_output = 0;
    static float rotate_direction = 0.0f;  // 初始旋转方向（锁定，防止近端震荡切向）

    // 【第一步】初始化：锁定目标角和初始旋转方向
    if(right_rotate_180_get_flag == 0)
    {
        float current_yaw = JY901S.stcAngle.ConYaw;
        turn_180_target = NormalizeAngle(current_yaw + 180.0f);
        // 锁定初始旋转方向（右转为负，左转为正），避免到达附近时 180° 跳边切换方向
        float init_diff = AngleDifference(turn_180_target, current_yaw);
        rotate_direction = (init_diff >= 0.0f) ? 1.0f : -1.0f;

        right_rotate_180_get_flag = 1;
        rotate_180_ready_count = 0;
        integral = 0.0f;
        last_error = 0.0f;
        last_output = 0;
        forward_open_flag = 0;
    }

    yaw_current = JY901S.stcAngle.ConYaw;

    // 环绕安全误差: error 正 → 当前相对目标偏"顺时针"，需要向左修正
    float angle_diff = AngleDifference(turn_180_target, yaw_current);
    float error      = -angle_diff;
    float abs_err    = fabs(angle_diff);

    // —— 到达目标：逐步停止 ——
    if(abs_err <= TARGET_TOLERANCE)
    {
        rotate_180_ready_count++;
        rotate_180_adjust(0, 0);
        last_output = 0;
        integral = 0.0f;
        if(rotate_180_ready_count >= STABLE_COUNT)
        {
            jy901_yaw_anti_rotation_cancel();
        }
        return;
    }
    rotate_180_ready_count = 0;

    // —— 积分分离 ——
    if(abs_err < INT_SEPARATION)
    {
        integral += error;
        if(integral >  INT_MAX) integral =  INT_MAX;
        if(integral < -INT_MAX) integral = -INT_MAX;
    }
    else
    {
        integral = 0.0f;
    }

    // —— 位置式 PID ——
    float derivative = error - last_error;
    last_error = error;

    int output = (int)(Kp * error + Ki * integral + Kd * derivative);

    // —— 三段式速度规划：远程全力 / 中程减速 / 近程爬行 ——
    int target_output;
    if(abs_err > SLOW_ANGLE)
    {
        // 远程：PID 主导，限幅到 MAX
        if(output >  MAX_OFFSET) output =  MAX_OFFSET;
        if(output < -MAX_OFFSET) output = -MAX_OFFSET;
        target_output = output;
    }
    else if(abs_err > CREEP_ANGLE)
    {
        // 中程：固定减速量，方向由锁存方向决定（防近端方向抖动）
        target_output = (int)(rotate_direction * SLOW_OFFSET);
    }
    else
    {
        // 近程：爬行定位，小步慢调
        target_output = (int)(rotate_direction * CREEP_OFFSET);
    }

    // —— 斜率限幅 ——
    int delta = target_output - last_output;
    if(delta >  SLEW_RATE) target_output = last_output + SLEW_RATE;
    if(delta < -SLEW_RATE) target_output = last_output - SLEW_RATE;
    last_output = target_output;

    // —— ESC 死区补偿 ——
    if(target_output > 0 && target_output <  MIN_OFFSET) target_output =  MIN_OFFSET;
    if(target_output < 0 && target_output > -MIN_OFFSET) target_output = -MIN_OFFSET;

    yaw_output   = target_output;
    left_offset  = target_output;
    right_offset = -target_output;
    rotate_180_adjust(left_offset, right_offset);
}

// 关闭所有稳向功能，停止电机
void jy901_yaw_anti_rotation_cancel(void)
{
    forward_yaw_get_flag = 0;
    right_rotate_180_get_flag = 0;

    // 左右推进器停止
    Drv_PWM_HighLvTimeSet(&thruster[0], 1500);
    Drv_PWM_HighLvTimeSet(&thruster[1], 1500);

    left_offset = 0;
    right_offset = 0;

    forward_open_flag = 0;
    rotate_180_open_flag = 0;
}

// 直行稳向调整（基础1650推力 + PID纠偏）
void forward_adjust(int left_offset,int right_offset)
{
    int right_pwm = 1700 + right_offset;
    int left_pwm  = 1700 + left_offset;

    // PWM限幅保护
    right_pwm = (right_pwm < 1400) ? 1400 : (right_pwm > 2000 ? 2000 : right_pwm);
    left_pwm  = (left_pwm  < 1400) ? 1400 : (left_pwm  > 2000 ? 2000 : left_pwm);

    Drv_PWM_HighLvTimeSet(&thruster[1], right_pwm);
    Drv_PWM_HighLvTimeSet(&thruster[0], left_pwm);
}

// 180度旋转调整（自旋模式，基础1500）
void rotate_180_adjust(int left_offset,int right_offset)
{
    // 偏移量为0 → 直接停止
    if(left_offset == 0 && right_offset == 0)
    {
        Drv_PWM_HighLvTimeSet(&thruster[0], 1500);
        Drv_PWM_HighLvTimeSet(&thruster[1], 1500);
        return;
    }

    const int pwm_deadzone = 40;  // 电机死区

    // 左电机死区处理
    if(left_offset > 0 && left_offset < pwm_deadzone)
        left_offset = pwm_deadzone;
    else if(left_offset < 0 && left_offset > -pwm_deadzone)
        left_offset = -pwm_deadzone;

    // 右电机死区处理
    if(right_offset > 0 && right_offset < pwm_deadzone)
        right_offset = pwm_deadzone;
    else if(right_offset < 0 && right_offset > -pwm_deadzone)
        right_offset = -pwm_deadzone;

    int right_pwm = 1500 + right_offset;
    int left_pwm  = 1500 + left_offset;

    // PWM限幅，提高旋转最大力度
    right_pwm = (right_pwm < 1300) ? 1300 : (right_pwm > 1700 ? 1700 : right_pwm);
    left_pwm  = (left_pwm  < 1300) ? 1300 : (left_pwm  > 1700 ? 1700 : left_pwm);

    Drv_PWM_HighLvTimeSet(&thruster[0], left_pwm);
    Drv_PWM_HighLvTimeSet(&thruster[1], right_pwm);
}
