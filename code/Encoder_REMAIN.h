/*
 * Encoder_REMAIN.h
 *
 *  编码器测速：每 5ms 控制周期读一次脉冲数，作为速度环的反馈量
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_ENCODER_REMAIN_H_
#define CODE_ENCODER_REMAIN_H_
#include "zf_common_headfile.h"

/* ================================ 接线（按实际接线改这几行） ================================ */
/* 实测接线：
 *   左后轮编码器：信号 A -> P20_3，信号 B -> P20_0，G -> GND，3V3 -> 3.3V
 *   右后轮编码器：信号 A -> P10_3，信号 B -> P10_1，G -> GND，3V3 -> 3.3V
 *
 * 为什么是这两个定时器：TC264 的 GPT1 只有下面这几组引脚能硬件计数，
 *   P20_3 / P20_0  ->  T6INA / T6EUDA  ->  TIM6_ENCODER
 *   P10_3 / P10_1  ->  T5INB / T5EUDB  ->  TIM5_ENCODER
 * 换成别的引脚就没有硬件计数单元了（只能软件中断数脉冲，200Hz 下会漏）。
 *
 * 注意：逐飞的 encoder_quad_init()（4 倍频正交）硬性限制 encoder_n <= TIM4_ENCODER，
 *       而这里用的是 T5/T6，所以只能用方向模式 encoder_dir_init()：
 *       A 相上升沿计数、B 相电平给方向，1 倍频。
 *       想要 4 倍频提高低速分辨率，必须把编码器挪到 T2/T3/T4 的引脚上。 */
#define ENCODER_L_INDEX         TIM6_ENCODER                    // 左后轮用 GPT1 的 T6
#define ENCODER_L_CH1           TIM6_ENCODER_CH1_P20_3         // 左后轮 A 相（计数，接 P20_3）
#define ENCODER_L_CH2           TIM6_ENCODER_CH2_P20_0         // 左后轮 B 相（方向，接 P20_0）

#define ENCODER_R_INDEX         TIM5_ENCODER                    // 右后轮用 GPT1 的 T5
#define ENCODER_R_CH1           TIM5_ENCODER_CH1_P10_3         // 右后轮 A 相（计数，接 P10_3）
#define ENCODER_R_CH2           TIM5_ENCODER_CH2_P10_1         // 右后轮 B 相（方向，接 P10_1）

/* 方向反了就把对应的宏改成 1（现象：正转速度读出来是负数） */
#define ENCODER_L_REVERSE       (1)                     // 实测：车轮向前滚时读数是负，取反
#define ENCODER_R_REVERSE       (1)                     // 同上

/* 速度一阶低通系数：0-不用滤波  越小越平滑但越滞后 */
#define ENCODER_SPEED_FILTER    (0.70f)

/* ================================ 接口 ================================ */
void    Encoder_Init        (void);                             // 初始化两路编码器
int16   Encoder_Get_Count   (uint8 is_right);                   // 读并清零，返回本周期的脉冲数
float   Encoder_Get_Speed   (uint8 is_right);                   // 读并清零，返回滤波后的速度（脉冲/周期）

#endif /* CODE_ENCODER_REMAIN_H_ */
