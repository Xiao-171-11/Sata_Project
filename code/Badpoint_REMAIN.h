/*
 * Badpoint_REMAIN.h
 *
 *  寻找边缘基点
 *
 *  边缘基点：在图像最下方的一行（BADPOINT_ROW）上，从图像中心列向左右两侧扫描，
 *            遇到的第一个黑白跳变点就是左 / 右边缘基点，它是后续边线跟踪（基线拟合）的起点。
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_BADPOINT_REMAIN_H_
#define CODE_BADPOINT_REMAIN_H_
#include "zf_common_headfile.h"

/* ================================ 可调参数 ================================ */
#define BADPOINT_ROW            (MT9V03X_H - 4)         // 基点所在行（图像底部往上第 4 行，最贴近车头）
#define BADPOINT_ROW_RETRY      (5)                     // 该行整行同色时，继续往上尝试的最大行数

/* ================================ 数据结构 ================================ */
typedef struct
{
    uint8 valid;                                        // 1-找到基点
    uint8 row;                                          // 基点行
    uint8 col;                                          // 基点列
    uint8 color_in;                                     // 靠图像中心一侧的颜色（0-黑  255-白）
    uint8 color_out;                                    // 靠外侧的颜色（0-黑  255-白）
} badpoint_struct;

extern badpoint_struct badpoint_left;                   // 左侧边缘基点
extern badpoint_struct badpoint_right;                  // 右侧边缘基点

/* ================================ 接口声明 ================================ */
void    Badpoint_Init           (void);                                         // 清空基点信息
uint8   Find_Badpoint           (const uint8 *image, uint16 width, uint16 height);  // 寻找左右边缘基点
void    Badpoint_Show_On_TFT180 (void);                                         // 在屏幕上标出基点

#endif /* CODE_BADPOINT_REMAIN_H_ */
