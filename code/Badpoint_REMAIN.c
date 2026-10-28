/*
 * Badpoint_REMAIN.c
 *
 *  寻找边缘基点
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */
#include "Badpoint_REMAIN.h"

badpoint_struct badpoint_left;                          // 左侧边缘基点
badpoint_struct badpoint_right;                         // 右侧边缘基点

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像坐标 -> TFT180 显示坐标（与 tft180_show_gray_image 的缩放一致）
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint16 badpoint_to_dis_x(uint8 col)
{
    return ((uint16)((uint32)col * CAMERA_DIS_W / MT9V03X_W));
}

static uint16 badpoint_to_dis_y(uint8 row)
{
    return ((uint16)((uint32)row * CAMERA_DIS_H / MT9V03X_H));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空基点信息
// 参数说明     void
// 返回参数     void
// 使用示例     Badpoint_Init();
// 备注信息     初始化时调用一次
//-------------------------------------------------------------------------------------------------------------------
void Badpoint_Init(void)
{
    memset(&badpoint_left,  0, sizeof(badpoint_left));
    memset(&badpoint_right, 0, sizeof(badpoint_right));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在某一行上从起始列向一个方向找第一个黑白跳变
// 参数说明     row_ptr         该行首地址
// 参数说明     start_col       起始列（图像中心）
// 参数说明     step            -1-向左  +1-向右
// 参数说明     width           图像宽度
// 参数说明     out_col         输出：跳变后的第一个像素列
// 参数说明     out_color_in    输出：靠中心一侧的颜色
// 参数说明     out_color_out   输出：靠外侧的颜色
// 返回参数     uint8           1-找到  0-整行同色
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 badpoint_scan_row(const uint8 *row_ptr, uint8 start_col, int8 step, uint16 width,
                               uint8 *out_col, uint8 *out_color_in, uint8 *out_color_out)
{
    uint8 color_in = row_ptr[start_col];
    int   col      = (int)start_col + (int)step;

    while((col > 0) && (col < (int)width - 1))
    {
        if(row_ptr[col] != color_in)                    // 出现黑白跳变
        {
            *out_col       = (uint8)col;                // 跳变后的第一个像素
            *out_color_in  = color_in;                  // 靠中心一侧的颜色
            *out_color_out = row_ptr[col];              // 靠外侧的颜色
            return (1);
        }
        col += (int)step;
    }
    return (0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找左右边缘基点
// 参数说明     image           二值化图像数组指针（base_image[0]）
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     uint8           0-左右基点都找到  1-至少有一侧没找到
// 使用示例     Find_Badpoint(base_image[0], MT9V03X_W, MT9V03X_H);
// 备注信息     1. 基点行取 BADPOINT_ROW；该行整行同色（飞坡/盲区）时最多再往上找 BADPOINT_ROW_RETRY 行
//              2. 左右两侧各自记录第一次找到的位置，互不覆盖
//              3. 找到的 color_in / color_out 会作为基线跟踪的特征（区分同一条黑白边）
//-------------------------------------------------------------------------------------------------------------------
uint8 Find_Badpoint(const uint8 *image, uint16 width, uint16 height)
{
    uint8 center = (uint8)(width / 2);                  // 图像中心列
    uint8 row    = BADPOINT_ROW;

    if(row >= (uint8)height)                            // 参数保护
    {
        row = (uint8)(height - 2);
    }

    badpoint_left.valid  = 0;
    badpoint_right.valid = 0;

    for(uint8 retry = 0; retry < BADPOINT_ROW_RETRY; retry ++)
    {
        const uint8 *row_ptr = image + (uint32)row * width;

        if(!badpoint_left.valid)                        // 左侧基点
        {
            uint8 col = 0, c_in = 0, c_out = 0;
            if(badpoint_scan_row(row_ptr, center, -1, width, &col, &c_in, &c_out))
            {
                badpoint_left.valid     = 1;
                badpoint_left.row       = row;
                badpoint_left.col       = col;
                badpoint_left.color_in  = c_in;
                badpoint_left.color_out = c_out;
            }
        }

        if(!badpoint_right.valid)                       // 右侧基点
        {
            uint8 col = 0, c_in = 0, c_out = 0;
            if(badpoint_scan_row(row_ptr, center, +1, width, &col, &c_in, &c_out))
            {
                badpoint_right.valid     = 1;
                badpoint_right.row       = row;
                badpoint_right.col       = col;
                badpoint_right.color_in  = c_in;
                badpoint_right.color_out = c_out;
            }
        }

        if(badpoint_left.valid && badpoint_right.valid)
        {
            break;
        }
        if(0 == row)
        {
            break;
        }
        row --;
    }

    return ((badpoint_left.valid && badpoint_right.valid) ? (0) : (1));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在屏幕上画一个 3x3 的十字标记
// 参数说明     pt              基点
// 参数说明     color           RGB565 颜色
// 返回参数     void
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void badpoint_draw_cross(const badpoint_struct *pt, uint16 color)
{
    if(!pt->valid)
    {
        return;
    }

    int x = (int)badpoint_to_dis_x(pt->col);
    int y = (int)badpoint_to_dis_y(pt->row);

    for(int dy = -1; dy <= 1; dy ++)
    {
        for(int dx = -1; dx <= 1; dx ++)
        {
            int px = x + dx;
            int py = y + dy;
            if((px >= 0) && (px < (int)tft180_width_max) && (py >= 0) && (py < (int)tft180_height_max))
            {
                tft180_draw_point((uint16)px, (uint16)py, color);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在屏幕二值图像上标出左右边缘基点
// 参数说明     void
// 返回参数     void
// 使用示例     Badpoint_Show_On_TFT180();
// 备注信息     左基点红色、右基点蓝色；画在 tft180_show_gray_image 之后才会覆盖在图像上
//-------------------------------------------------------------------------------------------------------------------
void Badpoint_Show_On_TFT180(void)
{
#if (1 == CAMERA_DIS_IMAGE)
    badpoint_draw_cross(&badpoint_left,  RGB565_RED);   // 十字画在图像上，图像关了就没有意义
    badpoint_draw_cross(&badpoint_right, RGB565_BLUE);
#endif

    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(0, CAMERA_INFO_Y2, "BP");                 // 基点列号
    tft180_show_int(20, CAMERA_INFO_Y2, (int32)badpoint_left.col,  3);
    tft180_show_int(68, CAMERA_INFO_Y2, (int32)badpoint_right.col, 3);
}
