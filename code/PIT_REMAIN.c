/*
 * PIT_REMAIN.c
 *
 *  控制节拍：CCU60_CH0 产生 5ms 周期中断
 *  中断服务函数在 user/isr.c 的 cc60_pit_ch0_isr 里，里面调用 Control_Tick()
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */
#include "PIT_REMAIN.h"
#include "Control_REMAIN.h"                             // CONTROL_PERIOD_MS

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动控制节拍
// 参数说明     void
// 返回参数     void
// 使用示例     PIT_Init();
// 备注信息     必须在电机、舵机、编码器都初始化完成之后再调用，
//              否则中断一开就会去操作还没初始化的外设
//-------------------------------------------------------------------------------------------------------------------
void PIT_Init(void)
{
    pit_ms_init(c_00, CONTROL_PERIOD_MS);               // 5ms 一次，中断里跑 Control_Tick()
}
