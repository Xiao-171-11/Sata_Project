/*
 * Motor_REMAIN.h
 *
 *  左右电机：PWM + 方向脚，正数正转、负数反转
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */

#ifndef CODE_MOTOR_REMAIN_H_
#define CODE_MOTOR_REMAIN_H_
#include "zf_common_headfile.h"

/* 注意：Motor_L / Motor_R 指的是**后左轮 / 后右轮**（驱动轮），
 *       前轮由舵机转向，两个后轮之间的速度差就是后轮差速。 */
/* ================================ 接线 ================================ */
#define Motor_R_pwm         ATOM0_CH6_P02_6             // 右电机 PWM
#define Motor_R_turn        P02_7                       // 右电机方向
#define Motor_L_pwm         ATOM0_CH4_P02_4             // 左电机 PWM
#define Motor_L_turn        P02_5                       // 左电机方向

/* ================================ 参数 ================================ */
#define MOTOR_PWM_FREQ      (5000)                      // 电机 PWM 频率 5kHz
#define MOTOR_DUTY_MAX      (8000)                      // 占空比上限（PWM_DUTY_MAX=10000，留余量）

/* 方向反了就把对应的宏改成 1（现象：给正占空比车轮倒转） */
#define MOTOR_DIR_REVERSE_L (1)                         // 实测：左后轮方向与右后轮相反，这里取反
#define MOTOR_DIR_REVERSE_R (0)

/* ================================ 接口 ================================ */
void Motor_Init     (void);                             // 初始化 PWM 与方向脚
void Motor_Set_Duty (int32 duty_l, int32 duty_r);       // 有符号占空比，负数=反转
void Motor_Stop     (void);                             // 两轮停转

#endif /* CODE_MOTOR_REMAIN_H_ */
