/*
 * Servo_REMAIN.c
 *
 *  舵机控制：分段增益 + 微分阻尼 + 死区
 *
 *  打角曲线（out 是相对中值的占空比偏移，正数=向右（占空比减小）、负数=向左（占空比增大））：
 *      |error| <= SERVO_DEAD_ZONE :  out = 0
 *      |error| <= SERVO_KNEE      :  out = ± KP_NEAR * |error|
 *      |error| >  SERVO_KNEE      :  out = ± (KP_NEAR*KNEE + KP_FAR*(|error|-KNEE))
 *      最后叠加微分项 KD*(error - last_error) 做阻尼
 *
 *  段与段之间在拐点处连续，不会跳变。
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */
#include "Servo_REMAIN.h"

static float servo_last_error = 0.0f;                   // 上一次的偏差（微分项用）
static float  servo_hold_out = 0.0f;
static uint16 servo_hold_cnt = 0;
static uint16 servo_big_cnt  = 0;
static uint8  servo_armed    = 0;
static float  servo_peak_abs = 0.0f;
static float  servo_peak_out = 0.0f;
static float  servo_err_f    = 0.0f;                   // 低通后的偏差
static uint8  servo_deep     = 0;                      // 本弯峰值够大（真进过弯）
static float  servo_out_prev = 0.0f;                   // 上一拍实际输出的偏移（纯追踪斜率限制用）
static uint8  servo_out_valid = 0;                     // 上一拍输出是否有效
static uint8  servo_mode      = 0;                     // 0=偏差打角 1=纯追踪打角（用于无扰切换）
static int32  servo_step_duty  = (int32)SERVO_DUTY_MID; // 分档打角：当前档位的占空比
static int8   servo_step_level = 0;                     // 分档打角：当前档位 -2/-1/0/+1/+2
static uint8  servo_step_hold  = 0;                     // 分档打角：本档已保持的拍数
static int32  servo_duty_now   = (int32)SERVO_DUTY_MID;  // 当前实际写出的占空比（调试用）
static int32  servo_move_duty   = (int32)SERVO_DUTY_MID; // 最近一次"在弯里"输出的档位占空比
static uint16 servo_turn_ticks  = 0;                     // 本次连续打角已持续的拍数（1 拍 = 5ms）
static uint16 servo_release_ticks = 0;                   // 长弯出弯后剩余保持拍数（>0 表示正在延时回中）
static int32  servo_release_duty  = (int32)SERVO_DUTY_MID; // 出弯保持时锁定的占空比

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Servo_Init();
// 备注信息     50Hz 舵机，上电先回中值，避免上电瞬间乱打角
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void)
{
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, Servo_MID + SERVO_TRIM);
    servo_last_error = 0.0f;
    servo_err_f      = 0.0f;
    servo_out_prev   = 0.0f;
    servo_out_valid  = 0;
    servo_mode       = 0;
    servo_step_duty  = (int32)SERVO_DUTY_MID;
    servo_step_level = 0;
    servo_step_hold  = 0;
    servo_move_duty  = (int32)SERVO_DUTY_MID;
    servo_turn_ticks = 0;
    servo_release_ticks = 0;
    servo_release_duty  = (int32)SERVO_DUTY_MID;
    Servo_Set_Duty((int32)SERVO_DUTY_MID);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接输出舵机占空比（带限幅）
// 参数说明     duty            占空比，实测允许范围 SERVO_DUTY_MIN ~ SERVO_DUTY_MAX（395 ~ 534）
// 返回参数     void
// 使用示例     Servo_Set_Duty(Servo_MID);
// 备注信息     限幅是为了保护舵机，不要让它长期顶到机械死点
//-------------------------------------------------------------------------------------------------------------------
void Servo_Set_Duty(int32 duty)
{
    /* 硬限幅：不管指令从哪来（偏差 PD、纯追踪、标定、以后新加的函数），都在这里被卡死，
     * 保证占空比永远落在实测行程 395 ~ 534 之内，不会把舵机推到机械死点上堵转发烫。 */
    if(duty > (int32)SERVO_DUTY_MAX)
    {
        duty = (int32)SERVO_DUTY_MAX;
    }
    if(duty < (int32)SERVO_DUTY_MIN)
    {
        duty = (int32)SERVO_DUTY_MIN;
    }
    pwm_set_duty(SERVO_PWM_PIN, (uint32)duty);
    servo_duty_now = duty;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读当前实际输出的占空比（只读，调试/屏幕显示用）
// 参数说明     void
// 返回参数     int32           395 ~ 534
// 使用示例     tft180_show_int(40, y3, Servo_Get_Duty(), 3);
//-------------------------------------------------------------------------------------------------------------------
int32 Servo_Get_Duty(void)
{
    return servo_duty_now;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按中线偏差打角（分段增益 + 微分 + 死区）
// 参数说明     error           中线偏差（像素），正数=赛道偏右
// 返回参数     void
// 使用示例     Servo_Set_Turn(midline_info.error);
// 备注信息     1. 直道小偏差只给小打角，弯道大偏差给大打角，避免“直道摆、弯道打不动”
//              2. KD 用相邻两帧偏差的差做阻尼，能明显压住来回摆
//-------------------------------------------------------------------------------------------------------------------
void Servo_Clear_Hold(void)
{
    servo_hold_cnt = 0;
    servo_armed    = 0;
    servo_big_cnt  = 0;
    servo_deep     = 0;
}

void Servo_Set_Turn(float error)
{
    float abs_error = 0.0f;
    float p_out     = 0.0f;
    float out       = 0.0f;

#if (1 == SERVO_REVERSE)
    error = -error;
#endif

    /* 从纯追踪切回偏差打角：偏差和上次的偏差可能差了几十，"微分项"会瞬间蹦出去一大截，
     * 表现就是切换那一拍舵机猛甩一下。这里把历史量对齐成当前值，微分项归零，切换无扰。 */
    if(1 == servo_mode)
    {
        servo_err_f      = error;
        servo_last_error = error;
        servo_mode       = 0;
    }

    /* 偏差低通：直道上边线逐帧抖动 1~3 像素，不过滤就会被高增益放大成来回打角 */
    error = (SERVO_ERR_FILTER * error) + ((1.0f - SERVO_ERR_FILTER) * servo_err_f);
    servo_err_f = error;

    abs_error = (error > 0.0f) ? (error) : (-error);

    if(abs_error <= SERVO_DEAD_ZONE)
    {
        p_out = 0.0f;
    }
    else if(abs_error <= SERVO_KNEE)
    {
        p_out = SERVO_KP_NEAR * error;
    }
    else
    {
        float sign = (error > 0.0f) ? (1.0f) : (-1.0f);
        p_out = sign * ((SERVO_KP_NEAR * SERVO_KNEE) + (SERVO_KP_FAR * (abs_error - SERVO_KNEE)));
    }

    out = p_out + (SERVO_KD * (error - servo_last_error));
    servo_last_error = error;

#if (1 == SERVO_EXIT_HOLD_EN)
    if(abs_error >= SERVO_ARM_ERROR)
    {
        if(0 == servo_big_cnt)                          /* new run of large error: restart peak */
        {
            servo_peak_abs = abs_error;
            servo_peak_out = p_out;
        }
        if(servo_big_cnt < 60000)
        {
            servo_big_cnt ++;
        }
        if(servo_big_cnt >= SERVO_ARM_MIN_TICKS)
        {
            servo_armed = 1;
        }
        if(abs_error > servo_peak_abs)
        {
            servo_peak_abs = abs_error;
            servo_peak_out = p_out;
        }
        if(servo_peak_abs >= SERVO_HOLD_MIN_PEAK)
        {
            servo_deep = 1;
        }
        servo_hold_cnt = 0;                             /* back into a corner: drop the hold */
    }
    else
    {
        servo_big_cnt = 0;
        if(1 == servo_armed)
        {
            if(abs_error > SERVO_RELEASE_ERROR)         /* still in the corner: track the peak */
            {
                if(abs_error > servo_peak_abs)
                {
                    servo_peak_abs = abs_error;
                    servo_peak_out = p_out;
                }
            }
            else if((0 == servo_hold_cnt) && (1 == servo_deep))   /* just left a real corner: latch the peak */
            {
                servo_hold_out = servo_peak_out;
                if(servo_hold_out >  (float)SERVO_DUTY_RANGE_L)
                {
                    servo_hold_out =  (float)SERVO_DUTY_RANGE_L;
                }
                if(servo_hold_out < -(float)SERVO_DUTY_RANGE_R)
                {
                    servo_hold_out = -(float)SERVO_DUTY_RANGE_R;
                }
                servo_hold_cnt = SERVO_HOLD_TICKS;
                servo_peak_abs = 0.0f;
                servo_deep     = 0;
            }
        }
    }

    if(servo_hold_cnt > 0)
    {
        /* 误差已经反过来了：车被往另一边推，继续锁着旧打角只会推它出去，立即释放 */
        if(((error * servo_hold_out) < 0.0f) && (abs_error > SERVO_RELEASE_ERROR))
        {
            servo_hold_cnt = 0;
            servo_armed    = 0;
            servo_big_cnt  = 0;
            servo_deep     = 0;
        }
    }

    if(servo_hold_cnt > 0)
    {
        out = servo_hold_out;
#if (1 == SERVO_HOLD_DECAY)
        out = servo_hold_out * ((float)servo_hold_cnt / (float)SERVO_HOLD_TICKS);
#endif
        servo_hold_cnt --;
        if(0 == servo_hold_cnt)
        {
            servo_armed = 0;
        }
    }
#endif

    /* 记录前先夹到行程内：偏差 PD 的远端增益是不封顶的（|error|=60 时 out 能到 200 多），
     * 不夹的话 servo_out_prev 会记成一个物理上不存在的偏移，纯追踪接管时要花十几拍才爬回来。 */
    if(out > (float)SERVO_DUTY_RANGE_L)
    {
        out = (float)SERVO_DUTY_RANGE_L;
    }
    if(out < -(float)SERVO_DUTY_RANGE_R)
    {
        out = -(float)SERVO_DUTY_RANGE_R;
    }

    Servo_Set_Duty(SERVO_DUTY_FROM_OUT(out));                   /* 极性由 SERVO_DIR 决定 */

    servo_out_prev  = out;                              /* 记录本次真实输出，供纯追踪斜率限制衔接 */
    servo_out_valid = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接按占空比偏移打角（纯追踪用，正=向右 / 占空比减小）
// 参数说明     offset          想要的偏移量，范围 -SERVO_DUTY_RANGE_R ~ +SERVO_DUTY_RANGE_L
// 返回参数     void
// 使用示例     Servo_Set_Offset(pursuit_info.steer_out);
// 备注信息     1. 和 Servo_Set_Turn 走同一条输出通路，只是不做分段增益和微分，因为纯追踪算出来的
//                 已经是"最终该打多少"，再叠一层增益就把物理量又变成了魔法数。
//              2. 只做两件事：非对称限幅（保护舵机）和斜率限制（保护齿轮、抑制电流冲击）。
//              3. 和 Servo_Set_Turn 之间可以任意切换，切换那一拍不会跳。
//-------------------------------------------------------------------------------------------------------------------
void Servo_Set_Offset(float offset)
{
    float d = 0.0f;

    if(offset > (float)SERVO_DUTY_RANGE_L)
    {
        offset = (float)SERVO_DUTY_RANGE_L;
    }
    if(offset < -(float)SERVO_DUTY_RANGE_R)
    {
        offset = -(float)SERVO_DUTY_RANGE_R;
    }

    if(0 == servo_out_valid)
    {
        servo_out_prev  = offset;                       /* 第一拍不限制斜率，否则上电会慢慢爬过去 */
        servo_out_valid = 1;
    }

    d = offset - servo_out_prev;
    if(d > SERVO_SLEW_MAX)
    {
        offset = servo_out_prev + SERVO_SLEW_MAX;
    }
    if(d < (-SERVO_SLEW_MAX))
    {
        offset = servo_out_prev - SERVO_SLEW_MAX;
    }

    servo_out_prev = offset;
    servo_mode     = 1;
    servo_hold_cnt = 0;                                 /* 纯追踪接管时不要残留出弯保持 */
    servo_armed    = 0;
    servo_turn_ticks    = 0;                            /* 走连续通路时清掉分档的长弯延时状态 */
    servo_release_ticks = 0;

    Servo_Set_Duty(SERVO_DUTY_FROM_OUT(offset));                /* 极性由 SERVO_DIR 决定 */
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     分档打角（中位 460 / 微调 452·478 / 小 440·474 / 大 418·500 / 超大 400·530）
// 参数说明     demand_deg      需要的打角量（度），正数=向右，负数=向左
//              force_center    非 0 = 强制回中位（十字路口专用，绝不转弯）
// 返回参数     void
// 使用示例     Servo_Set_Step(control_ps.delta_deg, control_ps.cross);
// 备注信息     1. 和 Servo_Set_Turn / Servo_Set_Offset 走同一条输出通路，三者可以任意切换；
//              2. 进出档用不同阈值（滞环），并且每档至少停留 SERVO_STEP_DWELL_TICKS 拍，压掉抖动；
//              3. 路口（force_center）不受停留限制，立刻回中位——这是"十字路口一定直行"的最后一道保险。
//              4. 长弯（连续打角 >= 1.0s）出弯时不立刻回中，先把出弯角度保持 0.4s（SERVO_LONG_EXIT_* 可调/可关）。
//-------------------------------------------------------------------------------------------------------------------
void Servo_Set_Step(float demand_deg, uint8 force_center)
{
    int8  want  = 0;
    int8  abs_cur = 0;
    float a = 0.0f;
    float keep = 0.0f;
    int32 duty = (int32)SERVO_DUTY_MID;

#if (0 == SERVO_STEP_EN)
    /* 关掉分档时退化成连续打角：需求(度) -> 偏移量，符号一致（正=向右）。
       32.0f 必须与 PURSUIT_DELTA_MAX_DEG 保持一致（满打角对应满偏移） */
    Servo_Set_Offset(demand_deg * (float)SERVO_DUTY_RANGE_L / 32.0f);
    return;
#else
    if(servo_step_hold < 250)
    {
        servo_step_hold ++;
    }

    if(0 != force_center)                               /* 路口：立刻回中位，且清掉一切旧状态 */
    {
        servo_step_level = 0;
        servo_step_duty  = (int32)SERVO_DUTY_MID;
        servo_step_hold  = 0;
        servo_out_prev   = 0.0f;
        servo_out_valid  = 1;
        servo_mode       = 1;
        servo_hold_cnt   = 0;
        servo_armed      = 0;
        servo_turn_ticks = 0;                           /* 路口强直行：把长弯延时回中也一并清掉 */
        servo_release_ticks = 0;
        Servo_Set_Duty((int32)SERVO_DUTY_MID);
        return;
    }

    a = (demand_deg >= 0.0f) ? demand_deg : (-demand_deg);

    /* 1. 按"进档阈值"算想要的档位（带方向号）：
          0 中位 / 1 微调(直道, ±8) / 2 小(±20) / 3 大(±42) / 4 超大(急弯, ±60~70) */
    if(a >= SERVO_STEP_HUGE_IN)
    {
        want = (demand_deg >= 0.0f) ? 4 : -4;
    }
    else if(a >= SERVO_STEP_BIG_IN)
    {
        want = (demand_deg >= 0.0f) ? 3 : -3;
    }
    else if(a >= SERVO_STEP_SMALL_IN)
    {
        want = (demand_deg >= 0.0f) ? 2 : -2;
    }
    else if(a >= SERVO_STEP_TINY_IN)
    {
        want = (demand_deg >= 0.0f) ? 1 : -1;
    }
    else
    {
        want = 0;
    }

    /* 2. 滞环：当前档位比想要的更"大"时，必须掉到退档阈值以下才允许降 */
    abs_cur = (servo_step_level >= 0) ? servo_step_level : (int8)(-servo_step_level);
    if((abs_cur > 0) && ((want >= 0 ? want : -want) < abs_cur))
    {
        if(4 == abs_cur)
        {
            keep = SERVO_STEP_HUGE_OUT;
        }
        else if(3 == abs_cur)
        {
            keep = SERVO_STEP_BIG_OUT;
        }
        else if(2 == abs_cur)
        {
            keep = SERVO_STEP_SMALL_OUT;
        }
        else
        {
            keep = SERVO_STEP_TINY_OUT;
        }
        if(a >= keep)
        {
            want = servo_step_level;                    /* 保持原档 */
        }
    }

    /* 3. 停留时间不足就不许换档（路口已被上面提前拦掉，不受这里限制） */
    if(want != servo_step_level)
    {
        if(servo_step_hold < SERVO_STEP_DWELL_TICKS)
        {
            want = servo_step_level;
        }
        else
        {
            servo_step_hold = 0;
        }
    }

    servo_step_level = want;

    /* 4. 档位 -> 占空比：实车实测"占空比小 = 向右"，所以 demand>0（向右）用 R 档（数值更小） */
    switch(want)
    {
        case 4:  duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_R_HUGE  : (int32)SERVO_DUTY_L_HUGE;  break;
        case 3:  duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_R_BIG   : (int32)SERVO_DUTY_L_BIG;   break;
        case 2:  duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_R_SMALL : (int32)SERVO_DUTY_L_SMALL; break;
        case 1:  duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_R_TINY  : (int32)SERVO_DUTY_L_TINY;  break;
        case -1: duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_L_TINY  : (int32)SERVO_DUTY_R_TINY;  break;
        case -2: duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_L_SMALL : (int32)SERVO_DUTY_R_SMALL; break;
        case -3: duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_L_BIG   : (int32)SERVO_DUTY_R_BIG;   break;
        case -4: duty = (SERVO_STEP_DIR > 0) ? (int32)SERVO_DUTY_L_HUGE  : (int32)SERVO_DUTY_R_HUGE;  break;
        default: duty = (int32)SERVO_DUTY_MID;                                                        break;
    }

#if (1 == SERVO_STEP_LINEAR_EN)
    /* 4.2 小偏差区线性化（治"直道调姿过度"）
     *  旧行为：|需求| 一过 1.50° 就给满 ±8 码（≈4°）。直道上死区(4.5 px)×0.9025×0.45 刚好落在 1.8°，
     *          也就是舵机在"0°"和"4°"之间二选一 —— 这就是左右摆的根：每一拍都在满打/回正之间跳。
     *  新行为：0~1.50° 从 0 码线性长到 4 码(≈2°)，1.50°~3.00° 再线性长到 8 码(≈4°)，
     *          3° 以上仍用原分档码数（真弯要的就是果断的大角度，不能软）。
     *          改的只是"台阶变成斜坡"，滞环（各档的 IN/OUT 阈值）原样不动，不会自己抖。 */
    if((1 == want) || (-1 == want))
    {
        float span_s = (float)SERVO_STEP_SMALL_IN - (float)SERVO_STEP_TINY_IN;   /* 1.50 -> 3.00 这一段 */
        float span_t = (float)SERVO_STEP_TINY_IN;                                /* 0 -> 1.50 这一段 */
        float dmid   = (float)SERVO_TINY_DUTY_MIN;                               /* 4 码：TINY_IN 处的输出 */
        float dmax   = (float)SERVO_TINY_DUTY_MAX;                               /* 8 码：SMALL_IN 处的输出 */
        float off;

        if(a >= (float)SERVO_STEP_TINY_IN)
        {
            off = dmid + (((a - (float)SERVO_STEP_TINY_IN) * (dmax - dmid)) / ((span_s > 0.001f) ? span_s : 1.0f));
        }
        else
        {
            off = (dmid * a) / ((span_t > 0.001f) ? span_t : 1.0f);
        }

        if(off < 0.0f)
        {
            off = 0.0f;
        }
        if(off > dmax)
        {
            off = dmax;
        }
        off = off + 0.5f;                                   /* 四舍五入成整数码 */

        if(SERVO_STEP_DIR > 0)
        {
            duty = (want > 0) ? ((int32)SERVO_DUTY_MID - (int32)off) : ((int32)SERVO_DUTY_MID + (int32)off);
        }
        else
        {
            duty = (want > 0) ? ((int32)SERVO_DUTY_MID + (int32)off) : ((int32)SERVO_DUTY_MID - (int32)off);
        }
    }
#endif

    /* 4.5 长弯出弯延时回中（需求：转弯时间超过 1 秒，出弯时先保持 0.4 秒再回中位）
           1) 还在弯里（want != 0）：累计转弯拍数，并记住当前弯道角度；
           2) 刚出弯、且这次弯已经持续 >= SERVO_LONG_TURN_TICKS：锁定出弯那一刻的角度，延时 SERVO_LONG_EXIT_TICKS 拍；
           3) 延时期间需求又变大（进新弯）：立刻取消保持，照常打角。 */
#if (1 == SERVO_LONG_EXIT_EN)
    if((want >= 2) || (want <= -2))                      /* 真的在弯里（小档及以上才算转弯，微调档不算） */
    {
        servo_move_duty = duty;                         /* 记住"弯里用的角度"，出弯时就用它延时 */
        if(servo_turn_ticks < 60000u)
        {
            servo_turn_ticks ++;
        }
        servo_release_ticks = 0;                        /* 还在弯里，取消未完成的延时 */
    }
    else if(servo_release_ticks > 0u)
    {
        servo_release_ticks --;
        duty = servo_release_duty;                      /* 保持出弯角度，这一拍还不回中位 */
    }
    else if((servo_turn_ticks >= SERVO_LONG_TURN_TICKS) && (servo_move_duty != (int32)SERVO_DUTY_MID))
    {
        servo_release_duty  = servo_move_duty;
        servo_release_ticks = SERVO_LONG_EXIT_TICKS;
        servo_turn_ticks    = 0;
        duty = servo_release_duty;                      /* 出弯这一拍就开始保持，不立刻回中 */
    }
    else
    {
        servo_turn_ticks = 0;                           /* 短弯（< 1s）或直道微调：直接回中位 */
        if(0 == want)
        {
            servo_move_duty = (int32)SERVO_DUTY_MID;    /* 已经回中了，清掉锁存角度，避免下次误保持 */
        }
    }
#endif

    servo_step_duty = duty;
    servo_mode      = 1;

    /* 5. 同步"连续通路"的历史量：这样从分档切回 Servo_Set_Turn / Servo_Set_Offset 时不会跳 */
    servo_out_prev  = (float)SERVO_DIR * ((float)duty - (float)(Servo_MID + SERVO_TRIM));
    servo_out_valid = 1;

    Servo_Set_Duty(duty);
#endif
}

