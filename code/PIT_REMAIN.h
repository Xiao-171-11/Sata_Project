/*
 * PIT_REMAIN.h
 *
 *  周期中断（控制节拍）
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */

#ifndef CODE_PIT_REMAIN_H_
#define CODE_PIT_REMAIN_H_
#include "zf_common_headfile.h"

#define c_00 CCU60_CH0
#define c_01 CCU60_CH1
#define c_10 CCU61_CH0
#define c_11 CCU61_CH1

void PIT_Init(void);                                    // 启动控制节拍（5ms）

#endif /* CODE_PIT_REMAIN_H_ */
