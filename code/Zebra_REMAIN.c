/*!
 * @file    Zebra_REMAIN.c
 * @brief   斑马线起跑/停车识别
 * @note    1. 用摄像头原始灰度图 + camera_threshold，不额外算阈值
 *          2. 只扫画面下部 ZEBRA_ROW_TOP ~ ZEBRA_ROW_BOT 行：发车时斑马线就在车头前
 *          3. 数"窄黑条"：宽度落在 [ZEBRA_BAR_MIN_PX, ZEBRA_BAR_MAX_PX] 的暗游程
 *          4. 一行里黑条够多、且这一行白像素够多，才算一行有效；有效行够多 -> 本帧命中
 *          5. 状态机负责"起点出发"和"终点停车"，出发后必须等斑马线离开视野才允许再触发
 *
 * Created on: 2026年10月23日
 * Author: Freed
 */

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量
//-------------------------------------------------------------------------------------------------------------------
zebra_info_struct zebra_info;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Zebra_Init();
//-------------------------------------------------------------------------------------------------------------------
void Zebra_Init(void)
{
    zebra_info.state     = 0;
    zebra_info.hit       = 0;
    zebra_info.votes     = 0;
    zebra_info.hit_cnt   = 0;
    zebra_info.clear_cnt = 0;
    zebra_info.run_cnt   = 0;
    zebra_info.armed     = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检测本帧有没有斑马线
// 参数说明     void
// 返回参数     1-有
// 使用示例     内部调用
// 备注信息     判据：同一行里 >= ZEBRA_MIN_BARS 根窄黑条，且白像素占比 >= ZEBRA_WHITE_MIN
//-------------------------------------------------------------------------------------------------------------------
static uint8 zebra_detect(void)
{
    uint8 votes = 0;
    uint8 thr = camera_threshold;

    for(int v = ZEBRA_ROW_TOP; v <= ZEBRA_ROW_BOT; v ++)
    {
        uint8  *row = mt9v03x_image[v];
        uint8   bars  = 0;
        uint8   white = 0;
        uint8   inbar = 0;
        int     start = 0;

        for(int u = 0; u < MT9V03X_W; u ++)
        {
            uint8 dark = (row[u] < thr) ? (1) : (0);
            if(0 == dark)
            {
                white ++;
            }
            if((1 == dark) && (0 == inbar))
            {
                start = u;
                inbar = 1;
            }
            else if((0 == dark) && (1 == inbar))
            {
                int len = u - start;
                inbar = 0;
                if((len >= ZEBRA_BAR_MIN_PX) && (len <= ZEBRA_BAR_MAX_PX))
                {
                    bars ++;
                }
            }
            else
            {
                /* 什么也不做：游程还没结束 */
            }
        }

        if((bars >= ZEBRA_MIN_BARS) && (((float)white / (float)MT9V03X_W) >= ZEBRA_WHITE_MIN))
        {
            votes ++;
        }
    }

    zebra_info.votes = votes;
    return ((votes >= ZEBRA_MIN_ROWS) ? (1) : (0));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每帧扫描 + 状态机
// 参数说明     void
// 返回参数     void
// 使用示例     在主循环里 Camera 处理之后调
// 备注信息     状态 0->1：看到斑马线就出发；1->2：斑马线离开视野过、且跑够时间后再看到就停车
//-------------------------------------------------------------------------------------------------------------------
void Zebra_Scan(void)
{
    uint8 hit = zebra_detect();

    zebra_info.hit = hit;

    if(0 != hit)
    {
        if(zebra_info.hit_cnt < 250)
        {
            zebra_info.hit_cnt ++;
        }
        zebra_info.clear_cnt = 0;
    }
    else
    {
        if(zebra_info.clear_cnt < 250)
        {
            zebra_info.clear_cnt ++;
        }
        zebra_info.hit_cnt = 0;
    }

    switch(zebra_info.state)
    {
        case 0:                                         // 等待出发
        {
            if(zebra_info.hit_cnt >= ZEBRA_HIT_FRAMES)
            {
                zebra_info.state     = 1;               // 走！
                zebra_info.run_cnt   = 0;
                zebra_info.clear_cnt = 0;
                zebra_info.armed     = 0;
            }
            break;
        }
        case 1:                                         // 行驶中
        {
            if(zebra_info.run_cnt < 250)
            {
                zebra_info.run_cnt ++;
            }
            // 起点必须先"跑过去"：斑马线离开视野够久 + 跑够距离，才武装终点判据
            if((zebra_info.clear_cnt >= ZEBRA_CLEAR_FRAMES) &&
               (zebra_info.run_cnt   >= ZEBRA_RUN_MIN_FRAMES))
            {
                zebra_info.armed = 1;
            }
            // 武装之后再看到斑马线 -> 到终点
            if((1 == zebra_info.armed) && (zebra_info.hit_cnt >= ZEBRA_HIT_FRAMES))
            {
                zebra_info.state = 2;                   // 到终点，停车
            }
            break;
        }
        default:                                        // 已到终点：锁定
        {
            zebra_info.state = 2;
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕显示
// 参数说明     void
// 返回参数     void
// 使用示例     Zebra_Show_On_TFT180();
// 备注信息     Y1 行 x=112 显示一个字母：W 等待 / R 行驶 / F 到终点
//-------------------------------------------------------------------------------------------------------------------
void Zebra_Show_On_TFT180(void)
{
#if (1 == ZEBRA_DIS_EN)
    tft180_set_font(TFT180_6X8_FONT);
    if(0 == zebra_info.state)
    {
        tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
        tft180_show_string(112, CAMERA_INFO_Y1, "W");
    }
    else if(1 == zebra_info.state)
    {
        tft180_set_color(RGB565_GREEN, RGB565_BLACK);
        tft180_show_string(112, CAMERA_INFO_Y1, "R");
    }
    else
    {
        tft180_set_color(RGB565_RED, RGB565_BLACK);
        tft180_show_string(112, CAMERA_INFO_Y1, "F");
    }
    tft180_set_color(0xFFFF, 0x0000);
    tft180_set_font(TFT180_8X16_FONT);
#endif
}
