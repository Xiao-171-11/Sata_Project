/*
 * Bedline_REMAIN.c
 *
 *  寻找边缘基线
 *
 *  1) 以边缘基点为起点（Badpoint_REMAIN 找到的 badpoint_left / badpoint_right）
 *  2) 逐行向上跟踪：每行只在上一行列号的 ±BEDLINE_SEARCH_GAP 内搜索“同类型跳变”
 *     （同类型 = 中心侧颜色 color_in -> 外侧颜色 color_out，和基点处完全一致）
 *  3) 连续丢线超过 BEDLINE_MAX_LOST 行则停止，防止跟踪跑到别的边上去
 *  4) 对跟踪到的点做最小二乘拟合，得到基线 col = k * row + b
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */
#include "Bedline_REMAIN.h"

bedline_struct bedline_left;                            // 左侧边缘基线
bedline_struct bedline_right;                           // 右侧边缘基线

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像坐标 -> TFT180 显示坐标（与 tft180_show_gray_image 的缩放一致）
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint16 bedline_to_dis_x(uint8 col)
{
    return ((uint16)((uint32)col * CAMERA_DIS_W / MT9V03X_W));
}

static uint16 bedline_to_dis_y(uint8 row)
{
    return ((uint16)((uint32)row * CAMERA_DIS_H / MT9V03X_H));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空基线信息
// 参数说明     void
// 返回参数     void
// 使用示例     Bedline_Init();
// 备注信息     初始化时调用一次
//-------------------------------------------------------------------------------------------------------------------
void Bedline_Init(void)
{
    memset(&bedline_left,  0, sizeof(bedline_left));
    memset(&bedline_right, 0, sizeof(bedline_right));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某一行第 col 列是否为“与基点同类型”的黑白跳变
// 参数说明     row_ptr         该行首地址
// 参数说明     col             待判断的列
// 参数说明     side            -1-左边界  +1-右边界
// 参数说明     base            该侧边缘基点（提供 color_in / color_out）
// 返回参数     uint8           1-是  0-否
// 备注信息     左边界：靠中心一侧的颜色在 col，靠外侧的颜色在 col-1
//              右边界：靠中心一侧的颜色在 col，靠外侧的颜色在 col+1
//-------------------------------------------------------------------------------------------------------------------
static uint8 bedline_is_transition(const uint8 *row_ptr, int col, int8 side, const badpoint_struct *base)
{
    if(side < 0)
    {
        return (((row_ptr[col] == base->color_in) && (row_ptr[col - 1] == base->color_out)) ? (1) : (0));
    }
    return (((row_ptr[col] == base->color_in) && (row_ptr[col + 1] == base->color_out)) ? (1) : (0));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     最小二乘拟合基线 col = k * row + b
// 参数说明     line            基线结构体（pt[] / num 已填好）
// 返回参数     void
// 备注信息     点数不足 BEDLINE_MIN_POINTS 或所有点在同一行时判为失败（valid = 0）
//-------------------------------------------------------------------------------------------------------------------
static void bedline_fit(bedline_struct *line)
{
    line->valid = 0;
    line->k     = 0.0f;
    line->b     = 0.0f;

    if(line->num < BEDLINE_MIN_POINTS)
    {
        return;
    }

    float sum_r = 0, sum_c = 0, sum_rr = 0, sum_rc = 0;
    for(uint8 i = 0; i < line->num; i ++)
    {
        float r = (float)line->pt[i].row;
        float c = (float)line->pt[i].col;
        sum_r  += r;
        sum_c  += c;
        sum_rr += r * r;
        sum_rc += r * c;
    }

    float n   = (float)line->num;
    float den = (n * sum_rr) - (sum_r * sum_r);         // n*Σr2 - (Σr)2
    float ad  = (den < 0) ? (-den) : (den);
    if(ad < 0.0001f)                                    // 所有点在同一行，拟合退化
    {
        return;
    }

    line->k = ((n * sum_rc) - (sum_r * sum_c)) / den;   // 斜率 d(col)/d(row)
    line->b = (sum_c - (line->k * sum_r)) / n;          // 截距

    line->row_start = line->pt[0].row;
    line->col_start = line->pt[0].col;
    line->row_end   = line->pt[line->num - 1].row;

    float c_end = (line->k * (float)line->row_end) + line->b;
    if(c_end < 0.0f)
    {
        c_end = 0.0f;
    }
    if(c_end > (float)(MT9V03X_W - 1))
    {
        c_end = (float)(MT9V03X_W - 1);
    }
    line->col_end = (uint8)c_end;

    line->valid = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从基点出发向上跟踪一侧边缘，并拟合出基线
// 参数说明     image           二值化图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     base            该侧边缘基点
// 参数说明     side            -1-左侧  +1-右侧
// 参数说明     line            输出：该侧基线
// 返回参数     uint8           0-基线有效  1-无效
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8 bedline_track(const uint8 *image, uint16 width,
                           const badpoint_struct *base, int8 side, bedline_struct *line)
{
    line->num   = 0;
    line->valid = 0;

    if(!base->valid)
    {
        return (1);
    }

    line->pt[0].row = base->row;                        // 起点就是基点
    line->pt[0].col = base->col;
    line->num       = 1;

    int   col  = (int)base->col;
    uint8 lost = 0;

    for(int row = (int)base->row - 1; (row >= 0) && (line->num < BEDLINE_MAX_POINTS); row --)
    {
        const uint8 *row_ptr = image + (uint32)row * width;
        int          found   = -1;

        for(int d = 0; d <= BEDLINE_SEARCH_GAP; d ++)
        {
            int c_out_side = col + ((int)side * d);     // 先往赛道外侧偏
            int c_in_side  = col - ((int)side * d);     // 再往赛道内侧偏

            if((c_out_side > 0) && (c_out_side < (int)width - 1) &&
               bedline_is_transition(row_ptr, c_out_side, side, base))
            {
                found = c_out_side;
                break;
            }
            if((d > 0) && (c_in_side > 0) && (c_in_side < (int)width - 1) &&
               bedline_is_transition(row_ptr, c_in_side, side, base))
            {
                found = c_in_side;
                break;
            }
        }

        if(found < 0)                                   // 本行没找到，允许连续丢几行
        {
            lost ++;
            if(lost > BEDLINE_MAX_LOST)
            {
                break;
            }
            continue;
        }

        lost = 0;
        col  = found;
        line->pt[line->num].row = (uint8)row;
        line->pt[line->num].col = (uint8)col;
        line->num ++;
    }

    bedline_fit(line);

    return ((line->valid) ? (0) : (1));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找左右边缘基线
// 参数说明     image           二值化图像数组指针（base_image[0]）
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     uint8           0-左右基线都有效  1-至少有一侧无效
// 使用示例     Find_Bedline(base_image[0], MT9V03X_W, MT9V03X_H);
// 备注信息     必须先调用 Find_Badpoint()，否则两侧基点无效、基线也无效
//-------------------------------------------------------------------------------------------------------------------
uint8 Find_Bedline(const uint8 *image, uint16 width, uint16 height)
{
    uint8 ret = 0;
    (void)height;                                       // 行范围由基点决定，这里用不到

    ret |= bedline_track(image, width, &badpoint_left,  -1, &bedline_left);
    ret |= bedline_track(image, width, &badpoint_right, +1, &bedline_right);

    return (ret);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把拟合出的基线画到屏幕上
// 参数说明     line            基线
// 参数说明     color           RGB565 颜色
// 返回参数     void
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void bedline_draw_line(const bedline_struct *line, uint16 color)
{
    if(!line->valid)
    {
        return;
    }

    for(int row = (int)line->row_start; row >= (int)line->row_end; row --)
    {
        float c = (line->k * (float)row) + line->b;
        if(c < 0.0f)
        {
            c = 0.0f;
        }
        if(c > (float)(MT9V03X_W - 1))
        {
            c = (float)(MT9V03X_W - 1);
        }

        uint16 x = bedline_to_dis_x((uint8)c);
        uint16 y = bedline_to_dis_y((uint8)row);
        if((x < tft180_width_max) && (y < tft180_height_max))
        {
            tft180_draw_point(x, y, color);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在屏幕二值图像上画出左右边缘基线，并显示斜率
// 参数说明     void
// 返回参数     void
// 使用示例     Bedline_Show_On_TFT180();
// 备注信息     左基线红色、右基线蓝色；必须排在 tft180_show_gray_image 之后才会覆盖在图像上
//-------------------------------------------------------------------------------------------------------------------
void Bedline_Show_On_TFT180(void)
{
#if (1 == CAMERA_DIS_IMAGE)
    bedline_draw_line(&bedline_left,  RGB565_RED);      // 基线画在图像上，图像关了就没有意义
    bedline_draw_line(&bedline_right, RGB565_BLUE);
#endif

    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_show_string(0, CAMERA_INFO_Y3, "K");                  // 基线斜率
    tft180_show_float(14, CAMERA_INFO_Y3, (double)bedline_left.k,  2, 2);
    tft180_show_float(68, CAMERA_INFO_Y3, (double)bedline_right.k, 2, 2);
}
