/****************************************************************************

* 文件名: algo_geometry.h

* 内容简述：几何工具——角度+距离 → 平面点，最小二乘求一组点的拟合直线角度

* 用途：沿岸/绕物等任务里，把若干"方向+距离"的雷达样本换算成机器人本体坐标系
*       下的 (x, y) 点，再用最小二乘拟合直线斜率，得到岸线/物体边界相对机器人
*       朝向的偏角。

* 来源参考：Water/App/Task/Position_Calculate.c（大头菜原版）

* 文件历史：
*  1.0       2026-05-19                创建该文件

****************************************************************************/
#ifndef __ALGO_GEOMETRY_H_
#define __ALGO_GEOMETRY_H_

#include <stdint.h>


/* 平面点（机器人本体坐标系，单位 cm）
 *   坐标约定：机器人正前方为 +X 轴，正右方为 +Y 轴
 *   角度约定：0° = 正前方，+90° = 正右方，-90° = 正左方 */
typedef struct {
    int16_t x;
    int16_t y;
} tagPoint_T;


/**
 * @brief 把(角度, 距离)换算成 (x, y) 平面点
 * @param p          输出点
 * @param angle_deg  方向角（度），0°=正前 / +90°=正右 / -90°=正左
 * @param dist       距离（任意单位，输出 x/y 同单位）
 */
void Algo_Geom_AngleDistToXY(tagPoint_T *p, double angle_deg, double dist);


/**
 * @brief 用最小二乘法求一组点的拟合直线与 X 轴的夹角
 * @param points     点数组
 * @param n          点的个数（至少 2）
 * @retval 拟合直线与机器人前方（X 轴）的夹角，单位度，范围 [-90°, +90°]
 *         返回值正/负 = 直线相对正前方向右/左倾斜
 *         所有点 x 相同（垂直线）时返回 0.0
 */
double Algo_Geom_LineAngleFromPoints(const tagPoint_T *points, int n);


#endif /* __ALGO_GEOMETRY_H_ */
