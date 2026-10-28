/*
 * SATA_MAIN.h
 *
 *  主程序入口
 *
 *  Created on: 2026年10月20日
 *      Author: Freed
 */

#ifndef CODE_SATA_MAIN_H_
#define CODE_SATA_MAIN_H_
#include "zf_common_headfile.h"

/*============================= 代码版本 =============================*/
/*  V4.6  2026-10-26
 *      - 加权中线判定法（已彻底移除逆透视 + 纯追踪）
 *      - 十字路口：五特征打分 + 直角对连线补基线 + 路口内锁修正量并逐拍衰减
 *      - 丢线三层兜底：raw 行重算 / 5 帧前历史中线 / 软收敛回中
 *      - 舵机按加权中线信息动态缩放：形状(bend) + 置信度(conf) 两层增益，只在直道生效
 *      - 小偏差区线性化：0~1.5° 给 0~4 码、1.5~3.0° 给 4~8 码，消除"0° 或 4°"的开关式打角
 *      - 直道打角 ×0.9025（累计 -9.75%），弯道与路口不缩放
 *      - 电机输出 BASE 99.25 / DUTY 794（V4.5 之后两次 +5%）
 *      - 屏幕只显示处理后的图像 112x71，蓝线只画下 4/5（上 1/5 不扫描）
 *  还原：把 D:\V4.6_project\Seekfree_TC264_Opensource_Library\code 覆盖回来
 */
#define CODE_VERSION_STR        "V4.6"
#define CODE_VERSION_NO         (406)
#define CODE_VERSION_DATE       "2026-10-26"

void SATA_MAIN(void);                                   // 主程序：外设初始化 + 主循环（不返回）

#endif /* CODE_SATA_MAIN_H_ */
