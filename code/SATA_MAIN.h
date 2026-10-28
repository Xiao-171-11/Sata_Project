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
/*  V4.7  2026-10-28
 *      - 电机输出整体 +5%（与历史每次提速同一套改法，7 个参数同比例缩放）：
 *          CONTROL_BASE_SPEED   99.25   -> 104.2125   （脉冲/周期）
 *          CONTROL_SPEED_MAX    148.875 -> 156.31875  （= BASE x 1.5）
 *          CONTROL_SPEED_MIN    29.775  -> 31.26375   （= BASE x 0.3）
 *          CONTROL_DUTY_BASE    794     -> 834        （前馈占空比，增益 834/104.2125 ≈ 8.0 不变）
 *          CONTROL_PID_OUT_MAX  661.5   -> 694.575    （速度环修正限幅同比例）
 *          CONTROL_OVERSPEED_SPEED 225.0 -> 236.25    （超速看门狗门限）
 *          CONTROL_ENC_FAULT_DUTY  1200 -> 1260       （堵转/掉线检测门限，巡航占空比约 1088）
 *      - 前馈增益与 PID 参数都没动，实际效果就是"目标速度 +5%、前馈占空比 +5%"
 *  还原：把上面 7 个宏改回 V4.6 的数值即可
 */
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
#define CODE_VERSION_STR        "V4.7"
#define CODE_VERSION_NO         (407)
#define CODE_VERSION_DATE       "2026-10-28"

void SATA_MAIN(void);                                   // 主程序：外设初始化 + 主循环（不返回）

#endif /* CODE_SATA_MAIN_H_ */
