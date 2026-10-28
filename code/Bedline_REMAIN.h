/*
 * Bedline_REMAIN.h
 *
 *  寻找边缘基线
 *
 *  边缘基线：从边缘基点出发，逐行向上跟踪同一条黑白边，把跟踪到的边缘点做最小二乘拟合，
 *            得到直线 col = k * row + b，这条直线就是该侧边线的基线。
 *            基线可用于判断赛道走向、补线、计算中线。
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_BEDLINE_REMAIN_H_
#define CODE_BEDLINE_REMAIN_H_
#include "zf_common_headfile.h"
#include "Badpoint_REMAIN.h"                            // 需要边缘基点

/* ================================ 可调参数 ================================ */
#define BEDLINE_SEARCH_GAP      (6)                     // 向上跟踪时每行允许的列偏移范围（±列）
#define BEDLINE_MAX_POINTS      (60)                    // 单侧最多记录的边缘点数
#define BEDLINE_MAX_LOST        (6)                     // 连续丢线多少行后停止跟踪
#define BEDLINE_MIN_POINTS      (4)                     // 拟合基线所需的最少点数

/* ================================ 数据结构 ================================ */
typedef struct
{
    uint8 row;                                          // 行（图像坐标，0 在最上方）
    uint8 col;                                          // 列
} bedline_pt_struct;

typedef struct
{
    uint8 valid;                                        // 1-基线拟合成功
    uint8 num;                                          // 参与拟合的点数
    bedline_pt_struct pt[BEDLINE_MAX_POINTS];           // 跟踪到的边缘点
    uint8 row_start, col_start;                         // 基线起点（就是边缘基点）
    uint8 row_end,   col_end;                           // 基线终点（最远跟踪行）
    float k;                                            // 斜率 d(col)/d(row)
    float b;                                            // 截距 col = k * row + b
} bedline_struct;

extern bedline_struct bedline_left;                     // 左侧边缘基线
extern bedline_struct bedline_right;                    // 右侧边缘基线

/* ================================ 接口声明 ================================ */
void    Bedline_Init            (void);                                         // 清空基线信息
uint8   Find_Bedline            (const uint8 *image, uint16 width, uint16 height);  // 寻找左右边缘基线
void    Bedline_Show_On_TFT180  (void);                                         // 在屏幕上画出基线

#endif /* CODE_BEDLINE_REMAIN_H_ */
