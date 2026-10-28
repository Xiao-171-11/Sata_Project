/*
 * Servo_REMAIN.h
 *
 *  舵机：偏差 -> 打角（分段增益 + 微分阻尼 + 死区）
 *
 *  为什么分段：
 *    直道上偏差只有几像素，如果增益统一给很大，车就会在直道上来回摆（转向过度）；
 *    弯道里偏差几十像素，又需要足够的打角量，不然打不动方向、直接冲出赛道。
 *    所以小偏差用小增益（柔），大偏差用大增益（有力），中间一个拐点 SERO_KNEE 衔接。
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */

#ifndef CODE_SERVO_REMAIN_H_
#define CODE_SERVO_REMAIN_H_
#include "zf_common_headfile.h"

#define Servo_MID               (460)                   // 舵机中值（50Hz，实测）
#define SERVO_TRIM              (0)                     // 中值微调：直道上车往哪边偏，就往反方向调几个数
#define SERVO_PWM_PIN           ATOM0_CH1_P33_9         // 舵机 PWM 引脚
#define SERVO_PWM_FREQ          (50)                    // 舵机固定 50Hz

/* ======================== 机械行程（实测，不许越过） ========================
 * 占空比可用区间 395 ~ 534，中值 460：
 *   向下最多 460 - 395 = 65 个码，向上最多 534 - 460 = 74 个码。
 * 所有限幅都从这两个数推出来，任何通路（偏差 PD / 纯追踪 / 出弯保持）都不会越界；
 * 一旦顶到死点就是堵转发烫，所以边界只写在这里一次。 */
#define SERVO_DUTY_MIN          (395)                   // 实测机械下限
#define SERVO_DUTY_MAX          (534)                   // 实测机械上限

/* ======================== 打角极性（单开关） ========================
 * 约定：out（偏移量）> 0 表示"向右打"，由下面这个符号翻译成占空比。
 *   SERVO_DIR = -1 ：out 正 -> 占空比减小（占空比越小越向右）——仓库原有方向，实车已验证
 *   SERVO_DIR = +1 ：out 正 -> 占空比增大（占空比越小越向左）——若实测是"号小往左"就改这里
 * 偏差 PD 与纯追踪两条通路共用这一个符号，改一次两边一起反向，不会互相打架。
 * 标定方法：上电后把 Servo_Set_Duty(400) / Servo_Set_Duty(530) 各打一次，
 *           看前轮往哪边偏，再决定这个符号（不要凭记忆改）。 */
#define SERVO_DIR               (-1)

#if (SERVO_DIR < 0)
#define SERVO_DUTY_RANGE_L      ((Servo_MID + SERVO_TRIM) - SERVO_DUTY_MIN)   // out 正（向右）可用 65 个码（占空比减小方向）
#define SERVO_DUTY_RANGE_R      (SERVO_DUTY_MAX - (Servo_MID + SERVO_TRIM))   // out 负（向左）可用 74 个码（占空比增大方向）
#else
#define SERVO_DUTY_RANGE_L      (SERVO_DUTY_MAX - (Servo_MID + SERVO_TRIM))   // 74 个码
#define SERVO_DUTY_RANGE_R      ((Servo_MID + SERVO_TRIM) - SERVO_DUTY_MIN)   // 65 个码
#endif
#define SERVO_DUTY_RANGE        (SERVO_DUTY_RANGE_L)    // 兼容旧名字（取较大一侧）
#define SERVO_DUTY_FROM_OUT(off)    ((int32)((float)(Servo_MID + SERVO_TRIM) + ((float)(SERVO_DIR) * (float)(off))))
#define SERVO_DEAD_ZONE         (3.0f)
#define SERVO_KNEE              (8.0f)
#define SERVO_KP_NEAR           (1.2f)
#define SERVO_KP_FAR            (4.0f)
#define SERVO_ERR_FILTER        (0.65f)                 // 偏差低通：0.65*本次 + 0.35*上次，压掉直道上的逐帧抖动
#define SERVO_HOLD_MIN_PEAK     (30.0f)                 // 出弯保持的最小峰值：没真进过弯就不许锁
#define SERVO_KD                (2.0f)                  // 微分阻尼：抑制来回摆，0 = 关掉
/* ---- exit hold ---------------- */
#define SERVO_EXIT_HOLD_EN      (1)                     // 1-open exit hold
#define SERVO_HOLD_TICKS        (40)                    // hold time: 40 x 5ms = 0.2 s
#define SERVO_HOLD_DECAY        (1)                     // 1-keep then return to center gradually over the hold window / 0-hold constant
#define SERVO_ARM_ERROR         (30.0f)
#define SERVO_ARM_MIN_TICKS     (10)                    // need 10 ticks(50ms) to arm
#define SERVO_RELEASE_ERROR     (15.0f)                 // |error| below this = just left corner

/* 偏差通路的 error 取反开关。
 * 注意：打角极性已经统一到上面的 SERVO_DIR 一处（两条通路共用），
 * 这里再开 1 只会把偏差通路单独翻过去，纯追踪那条不会跟着翻，两条通路就打架了。
 * 除非你确定只想翻偏差通路，否则保持 0。 */
#define SERVO_REVERSE           (0)

/* ---- 连续打角（纯追踪用）----
 * 纯追踪每帧算出来的是"应该打多少度"，是连续量，不需要分段增益，但必须限制变化率：
 * 舵机是机械件，一帧之内从 +69 跳到 -70 会听到"咔"一声并且大电流，长期这么干会烧齿轮。 */
#define SERVO_SLEW_MAX          (14.0f)                 // 每个 5ms 控制周期最多变化多少占空比（69/14 ≈ 5 拍打满）

/* ======================== 分档打角（实车标定值） ========================
 * 实车要求：舵机不要连续微调，直接给固定的几个档位，靠"档位 + 停留时间"转向。
 *   直道微调：452（右微调）/ 468（左微调）  —— 只 ±8 码，直道上绝不甩头
 *   直道大偏 / 缓弯：440（右小）/ 474（左小）
 *   弯道转向：418（右大）/ 500（左大）
 *   急弯（转不过来）：400（右超大）/ 530（左超大）
 *   中位/路口：460（绝不转弯）
 * 核心思想：直道用"微调档"（±4° 转角）慢慢把车摆正，弯道才允许"大档/超大档"（±21°/±30°）。
 *   这样同一套代码里直道的姿态修正是软的，弯道的转向是硬的，两者不会互相将就。
 * 方向：占空比小 = 前轮向右，占空比大 = 前轮向左（实车实测，中值 460）。
 *   这条与仓库原注释「偏大向左转，偏小向右转」一致；实车现象也证明如此：
 *   老 PD 通路在弯道上一直是对的，而老 PD 是 error>0（赛道偏右）-> 占空比变小 -> 车往右走。
 *
 * 滞环（hysteresis）：进档用 IN 阈值，退档用 OUT 阈值，OUT < IN。
 *   没有滞环的话，需求在阈值附近抖动时会 460/440/460/440 反复打，舵机会"哒哒"响并且车头摆。
 * 停留（dwell）：每次换档后至少保持 SERVO_STEP_DWELL_TICKS 个控制周期（5ms/拍）才允许再换，
 *   进一步压掉抖动；路口强制回中不受停留限制（安全优先）。 */
#define SERVO_STEP_EN           (1)                     // 1 = 启用分档打角（0 = 回到连续打角）
#define SERVO_STEP_DIR          (1)                     // +1: 需求为正（要向右）-> 用 R 档（占空比变小）；-1: 反过来
#define SERVO_DUTY_MID          (Servo_MID + SERVO_TRIM) // 中位（直线/路口）
#define SERVO_DUTY_R_TINY       (452)                   // 直道微调 向右（占空比 -8  ≈ 4°）
#define SERVO_DUTY_L_TINY       (468)                   // 直道微调 向左（占空比 +8）
#define SERVO_DUTY_R_SMALL      (440)                   // 直道大偏/缓弯 向右（小，占空比 -20）
#define SERVO_DUTY_L_SMALL      (474)                   // 直道 向左（小，占空比 +14）
#define SERVO_DUTY_R_BIG        (418)                   // 弯道 向右（大，占空比 -42）
#define SERVO_DUTY_L_BIG        (500)                   // 弯道 向左（大，占空比 +40）
#define SERVO_DUTY_R_HUGE       (400)                   // 急弯 向右（超大，占空比 -60）
#define SERVO_DUTY_L_HUGE       (530)                   // 急弯 向左（超大，占空比 +70）
#define SERVO_STEP_TINY_IN      (1.50f)                 // |需求| ≥ 1.50° 进微调档（直道专用，软的）
#define SERVO_STEP_TINY_OUT     (1.00f)                 // |需求| < 1.00° 才回中位（原 0.80，加大滞环防来回切）
#define SERVO_STEP_TINY_LINEAR  (1)                     // 已由 SERVO_STEP_LINEAR_EN 取代（保留兼容，不再被引用）
#define SERVO_STEP_LINEAR_EN    (1)                     // 1-小偏差区线性化：0~1.5° 给 0~4 码、1.5~3.0° 给 4~8 码（把"0°或4°"的开关式打角变成斜坡）
#define SERVO_TINY_DUTY_MIN     (4)                     // 微调档起点（需求 = TINY_IN 处）的码数，4 码 ≈ 2°
#define SERVO_TINY_DUTY_MAX     (8)                     // 微调档终点（需求 = SMALL_IN 处）= 原来的 ±8 码 ≈ 4°
#define SERVO_STEP_SMALL_IN     (3.00f)                 // |需求| ≥ 3.00° 进小档
#define SERVO_STEP_SMALL_OUT    (2.20f)                 // |需求| < 2.20° 才退到微调档
#define SERVO_STEP_BIG_IN       (5.50f)                 // |需求| ≥ 5.50° 进大档（真弯道才给）
#define SERVO_STEP_BIG_OUT      (4.00f)                 // |需求| < 4.00° 才退到小档
#define SERVO_STEP_HUGE_IN      (10.00f)                // |需求| ≥ 10.0° 进超大档（急弯打不动就用它）
#define SERVO_STEP_HUGE_OUT     (7.50f)                 // |需求| < 7.50° 才退到大档
#define SERVO_STEP_DWELL_TICKS  (1)                     // 每档至少保持 1 拍 = 5ms（急弯要快要跟得上）

/* 长弯出弯延时回中（需求：连续转弯超过 1 秒，出弯时先保持当时的打角 0.4 秒再回中位）
   作用：大弯/发夹弯出弯瞬间如果立刻回中，车身会"甩"一下（残余侧偏 + 惯性），
         保持一会儿出弯角度能让车先摆正再回中。 */
#define SERVO_LONG_EXIT_EN      (1)                     // 1 = 启用长弯延时回中；0 = 出弯立刻回中（老行为）
#define SERVO_LONG_TURN_TICKS   (200)                   // 连续打角 >= 200 拍 = 1.00s 才算"长弯"
#define SERVO_LONG_EXIT_TICKS   (80)                    // 长弯出弯后再保持 80 拍 = 0.40s 才回中位

void Servo_Init     (void);                             // 初始化舵机 PWM
void Servo_Set_Duty (int32 duty);                       // 直接输出占空比（内部限幅）
void Servo_Set_Turn (float error);                      // 按中线偏差打角
void Servo_Set_Offset (float offset);                   // 直接按占空比偏移打角（正=向左），带斜率限制
void Servo_Set_Step   (float demand_deg, uint8 force_center); // 分档打角（正=向右）；force_center!=0 时强制回中位
int32 Servo_Get_Duty  (void);                           // 读当前实际输出的占空比（屏幕显示/调试）
void Servo_Clear_Hold (void);                           // 立刻取消出弯保持（路口用）

#endif /* CODE_SERVO_REMAIN_H_ */
