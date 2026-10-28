/*
 * Encoder_REMAIN.c
 *
 *  编码器测速
 *
 *  用法：初始化一次，然后在固定周期（5ms）里调用 Encoder_Get_Speed()，
 *        得到的就是"每 5ms 的脉冲数"，可以直接当作速度量送进速度环 PID。
 *        换控制周期时不需要改这里，只要保证调用周期一致即可。
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */
#include "Encoder_REMAIN.h"

static float speed_l = 0.0f;                            // 左轮滤波后的速度
static float speed_r = 0.0f;                            // 右轮滤波后的速度

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Encoder_Init();
// 备注信息     使用方向模式（AB 相里一相计数、一相给方向），兼容逐飞带方向输出的编码器
//-------------------------------------------------------------------------------------------------------------------
void Encoder_Init(void)
{
    encoder_dir_init(ENCODER_L_INDEX, ENCODER_L_CH1, ENCODER_L_CH2);
    encoder_dir_init(ENCODER_R_INDEX, ENCODER_R_CH1, ENCODER_R_CH2);
    encoder_clear_count(ENCODER_L_INDEX);
    encoder_clear_count(ENCODER_R_INDEX);
    speed_l = 0.0f;
    speed_r = 0.0f;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取并清零编码器计数
// 参数说明     is_right        0-左轮  1-右轮
// 返回参数     int16           本周期内的脉冲数（带方向）
// 使用示例     int16 cnt = Encoder_Get_Count(0);
// 备注信息     读一次清一次，所以必须放在固定周期里调用
//-------------------------------------------------------------------------------------------------------------------
int16 Encoder_Get_Count(uint8 is_right)
{
    int16 count = 0;
    if(is_right)
    {
        count = encoder_get_count(ENCODER_R_INDEX);
        encoder_clear_count(ENCODER_R_INDEX);
#if (1 == ENCODER_R_REVERSE)
        count = -count;
#endif
    }
    else
    {
        count = encoder_get_count(ENCODER_L_INDEX);
        encoder_clear_count(ENCODER_L_INDEX);
#if (1 == ENCODER_L_REVERSE)
        count = -count;
#endif
    }
    return (count);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取速度（脉冲 / 控制周期），带一阶低通
// 参数说明     is_right        0-左轮  1-右轮
// 返回参数     float           滤波后的速度
// 使用示例     float v = Encoder_Get_Speed(1);
// 备注信息     ENCODER_SPEED_FILTER = 0 时不滤波
//-------------------------------------------------------------------------------------------------------------------
float Encoder_Get_Speed(uint8 is_right)
{
    float count = (float)Encoder_Get_Count(is_right);

    if(is_right)
    {
        speed_r = (speed_r * ENCODER_SPEED_FILTER) + (count * (1.0f - ENCODER_SPEED_FILTER));
        return (speed_r);
    }

    speed_l = (speed_l * ENCODER_SPEED_FILTER) + (count * (1.0f - ENCODER_SPEED_FILTER));
    return (speed_l);
}
