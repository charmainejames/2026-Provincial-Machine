/****************************************************************************

* 文件名: algo_geometry.c

* 内容简述：详见 algo_geometry.h

* 文件历史：
*  1.0       2026-05-19                创建该文件

****************************************************************************/
#include "algo_geometry.h"

#include <math.h>
#include <stddef.h>


#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif


void Algo_Geom_AngleDistToXY(tagPoint_T *p, double angle_deg, double dist)
{
    if (p == NULL) return;

    double rad = angle_deg * M_PI / 180.0;
    p->x = (int16_t)(dist * cos(rad));
    p->y = (int16_t)(dist * sin(rad));
}


/* 内部工具：最小二乘求斜率 y = k*x + b 的 k */
static double s_slope_least_squares(const tagPoint_T *points, int n)
{
    if (n < 2) return 0.0;

    double sum_x  = 0.0, sum_y  = 0.0;
    double sum_xy = 0.0, sum_x2 = 0.0;

    for (int i = 0; i < n; i++) {
        sum_x  += points[i].x;
        sum_y  += points[i].y;
        sum_xy += (double)points[i].x * points[i].y;
        sum_x2 += (double)points[i].x * points[i].x;
    }

    double denom = (double)n * sum_x2 - sum_x * sum_x;
    if (fabs(denom) < 1e-6) return 0.0;     /* 所有点 x 几乎相同，垂直线 */

    return ((double)n * sum_xy - sum_x * sum_y) / denom;
}


double Algo_Geom_LineAngleFromPoints(const tagPoint_T *points, int n)
{
    double k       = s_slope_least_squares(points, n);
    double rad     = atan(k);
    double deg     = rad * 180.0 / M_PI;
    return deg;
}
