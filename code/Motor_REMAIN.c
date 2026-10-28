/*
 * Motor_REMAIN.c
 *
 *  电机驱动
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */
#include "Motor_REMAIN.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单个电机的方向 + 占空比输出
// 参数说明     pwm_ch          该电机的 PWM 通道
// 参数说明     dir_pin         该电机的方向引脚
// 参数说明     duty            有符号占空比
// 参数说明     reverse         1-该电机方向需要取反
// 返回参数     void
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void motor_output(pwm_channel_enum pwm_ch, gpio_pin_enum dir_pin, int32 duty, uint8 reverse)
{
    uint8 dir_level = 0;

    if(duty < 0)                                        // 负数 = 反转
    {
        duty      = -duty;
        dir_level = 1;
    }

    if(reverse)                                         // 接线反了的时候取反
    {
        dir_level = (uint8)(1 - dir_level);
    }

    if(duty > MOTOR_DUTY_MAX)                           // 限幅，保护电机和驱动
    {
        duty = MOTOR_DUTY_MAX;
    }

    gpio_set_level(dir_pin, dir_level);
    pwm_set_duty(pwm_ch, (uint32)duty);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Motor_Init();
// 备注信息     上电两轮停转
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    gpio_init(Motor_R_turn, GPO, 0, GPO_PUSH_PULL);
    gpio_init(Motor_L_turn, GPO, 0, GPO_PUSH_PULL);

    pwm_init(Motor_R_pwm, MOTOR_PWM_FREQ, 0);
    pwm_init(Motor_L_pwm, MOTOR_PWM_FREQ, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置左右轮占空比
// 参数说明     duty_l          左轮，正数正转
// 参数说明     duty_r          右轮，正数正转
// 返回参数     void
// 使用示例     Motor_Set_Duty(3000, 3000);
// 备注信息     内部已做 0~MOTOR_DUTY_MAX 限幅
//-------------------------------------------------------------------------------------------------------------------
void Motor_Set_Duty(int32 duty_l, int32 duty_r)
{
    motor_output(Motor_L_pwm, Motor_L_turn, duty_l, MOTOR_DIR_REVERSE_L);
    motor_output(Motor_R_pwm, Motor_R_turn, duty_r, MOTOR_DIR_REVERSE_R);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     两轮停转
// 参数说明     void
// 返回参数     void
// 使用示例     Motor_Stop();
// 备注信息     丢线、未发车、初始化阶段都用它
//-------------------------------------------------------------------------------------------------------------------
void Motor_Stop(void)
{
    Motor_Set_Duty(0, 0);
}
