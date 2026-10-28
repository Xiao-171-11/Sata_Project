/*
 * Pursuit_REMAIN.c
 *
 *  纯追踪 + 逆透视巡线
 *
 *  处理流程（全部在主循环里跑，帧率 50fps）：
 *      1. IPM_Process() 已经把 base_image 重采样成俯视图 ipm_image[56][96]
 *      2. pursuit_extract_track()：从最近行往上，逐行向左右找边线，得到每行中线
 *      3. 十字状态机：用"左右同时丢"的行数区分路口和弯道，路口保持原航向
 *      4. 纯追踪：在车前方 Ld 处取前瞻点，算曲率 -> 前轮转角 -> 舵机偏移
 *      5. 发布快照（双缓冲），5ms 控制中断里 Pursuit_Get() 取用
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */

#include "Pursuit_REMAIN.h"

#define PURSUIT_CENTER_COL  (((float)(IPM_W - 1)) * 0.5f)   /* 96 列 -> 47.5 */
#define PURSUIT_RAD2DEG     (57.2957795f)

/* ---- 每行中线与可信度（只在本文件内用） ---- */
static float trk_center[IPM_H];
static uint8 trk_ok[IPM_H];
static uint8 trk_lost_l[IPM_H];
static uint8 trk_lost_r[IPM_H];
static uint8 trk_valid;
static uint8 trk_open;
static uint8 trk_one;
static uint8 trk_wide;                                      /* 路面变宽的行数（十字/岔口的正面特征） */
static uint8 trk_band;                                      /* 中断带的长度（丢线/变宽的连续行数） */
static uint8 trk_above;                                     /* 中断带"上方"又看清的行数 —— 这是路口和弯道的分水岭 */
static uint8 trk_mark;                                      /* 路面内黑色短横条标记的行数（十字最硬特征） */
static uint8 trk_white;                                     /* 整行全白（一个暗点都没有）的行数 */
static float trk_hw;                                        /* 本帧学习到的半赛道宽（格点） */

/* ---- 十字路口状态机 ---- */
static uint8 cross_state;                                   /* 0=不在路口 1=正在过路口 */
static uint8 cross_hits;                                    /* 连续多少帧可疑 */
static uint8 cross_ticks;                                   /* 路口内剩余帧数 */
static uint8 cross_clean;                                   /* 连续多少帧干净 */
static uint8 cross_reject;                                  /* 出路口后的禁止判定帧数 */
static float cross_kappa;                                   /* 进路口那一刻的曲率 */

/* ---- 视野丢失保持 ---- */
static uint8 pursuit_hold;                                  /* 已经沿用旧快照多少帧 */

/* ---- 双缓冲发布 ---- */
static pursuit_struct ps_buf[2];
static volatile uint8 ps_pub;

pursuit_struct pursuit_info;
int16 pursuit_center_row[IPM_H];

/* ============================================================================
 *  从 ipred 沿 dir 方向找赛道边界（本赛道的边界是"白路面 -> 暗地板"的跳变，
 *  不是一条细黑线；所以不能用"找黑点"的思路，必须找"够厚的暗区"）
 *
 *    dir = -1 向左、+1 向右；*thick 返回该暗区的厚度（格点）
 *    返回边界所在的列号（暗区内侧第一个格子），找不到返回 -1
 *
 *  两条硬规矩（都是踩过坑才写下来的）：
 *    1. 路面里那些细的暗条纹（斑马线、横臂、地面反光缝）厚度 <= PURSUIT_LINE_W_MAX，
 *       一律跳过继续往外找；把它们当边界会让赛道"凭空变窄"。
 *    2. 只有厚度 > PURSUIT_LINE_W_MAX 的暗区才算地板。真地板一直延伸到画面外，
 *       所以命中的一定是"内侧边缘"，不会出现游程中点那种把边界算到画面中间的错。
 * ==========================================================================*/
static float pursuit_find_floor(uint8 row, int16 ipred, int8 dir, uint8 *thick)
{
    int16 x;
    int16 run;
    int16 step;

    *thick = 0;

    if(ipred < 0)
    {
        ipred = 0;
    }
    if(ipred > (IPM_W - 1))
    {
        ipred = (IPM_W - 1);
    }

    step = (int16)dir;
    x    = ipred + step;
    run  = 0;

    while((x >= 0) && (x < IPM_W) && (((x - ipred) * step) <= PURSUIT_SEARCH_COLS))
    {
        if(ipm_image[row][x] < 128)
        {
            run ++;
            if(run > PURSUIT_LINE_W_MAX)                    /* 够厚：这是地板，不是路面上的暗条纹 */
            {
                *thick = (uint8)((run > 250) ? 250 : run);

                return ((float)(x - (step * (run - 1))));   /* 暗区内侧第一个格子 = 边界 */
            }
        }
        else
        {
            run = 0;                                        /* 又回到路面：刚才是暗条纹，跳过 */
        }
        x += step;
    }

    return (-1.0f);
}

/* ============================================================================
 *  逐行提取赛道中线（四种情况严格分开，绝不混算）
 *    左右都在、宽度正常、无横跳 -> 中点，可信（trk_ok=1，唯一能给前瞻点用的行）
 *    左右都在但路面变宽       -> 支路开口，中线沿航向外推，计入 wide_rows
 *    只有一侧                 -> 用学习到的半赛道宽推算，且必须和上一行的预测值吻合
 *    两侧都没有 / 太窄        -> 沿航向外推，计入 open_rows
 *  实车教训：早期版本把"都在但太宽"的行落到 else 分支里，既不计 wide 也不写中线，
 *  trk_center[] 残留上一帧的值，前瞻点被拖进支路，车直接冲出赛道。
 * ==========================================================================*/
/* ============================================================================
 *  数一行路面里"短暗条"的段数
 *
 *    十字路口在这个赛道上不是靠"边线断了"来认的 —— 白路面上会横着贴两排
 *    黑色短横条（斑马线样式），横条只有 1~2 格厚，pursuit_find_floor() 会
 *    把它们当"路面内的暗条纹"跳过（所以边界照样找得到）。
 *    一行里能数出 >= PURSUIT_MARK_MIN_RUNS 段短暗条，就是路口标记。
 *
 *    弯道永远数不出这个特征（弯道边线是一整片连续的地板，不算短暗条）。
 *    所以这条判据是把"路口"和"弯道"分开最硬的一条，优先级高于其它所有判据。
 * ==========================================================================*/
static uint8 pursuit_count_marks(uint8 row, int16 lc, int16 rc)
{
    int16 x;
    int16 run;
    uint8 cnt;

    cnt = 0;
    run = 0;

    if(rc <= (lc + 1))
    {
        return (0);
    }

    for(x = (int16)(lc + 1); x < rc; x ++)
    {
        if(ipm_image[row][x] < 128)
        {
            run ++;
        }
        else
        {
            if((run > 0) && (run <= PURSUIT_LINE_W_MAX))
            {
                cnt ++;
            }
            run = 0;
        }
    }
    if((run > 0) && (run <= PURSUIT_LINE_W_MAX))
    {
        cnt ++;
    }

    return (cnt);
}

static void pursuit_extract_track(void)
{
    float pc;
    float slope;
    float hw;
    float pred;
    float lp;
    float rp;
    float w;
    float c2;
    int16 ip;
    uint8 tl;
    uint8 tr;
    uint8 lost_l;
    uint8 lost_r;
    uint8 r;

    pc    = PURSUIT_CENTER_COL;
    slope = 0.0f;
    hw    = (PURSUIT_TRACK_W_MIN * 1.6f);

    trk_valid = 0;
    trk_open  = 0;
    trk_one   = 0;
    trk_wide  = 0;
    trk_mark  = 0;
    trk_white = 0;

    for(r = 0; r < IPM_H; r ++)
    {
        pred = pc + slope;
        if(pred < 0.0f)
        {
            pred = 0.0f;
        }
        if(pred > ((float)(IPM_W - 1)))
        {
            pred = (float)(IPM_W - 1);
        }
        ip   = (int16)(pred + 0.5f);

        tl   = 0;
        tr   = 0;
        lp   = pursuit_find_floor(r, ip, -1, &tl);
        rp   = pursuit_find_floor(r, ip,  1, &tr);

        if((lp < 0.0f) || (tl <= PURSUIT_LINE_W_MAX))       /* 没找到"够厚的暗区" -> 这一侧算丢 */
        {
            lp = -1.0f;
        }
        if((rp < 0.0f) || (tr <= PURSUIT_LINE_W_MAX))
        {
            rp = -1.0f;
        }

        /* 数这一行路面里有几段短暗条：十字路口那两排黑色横条就是这个特征。
           注意横条太粗会被 pursuit_find_floor() 当成地板找成边界，
           只有"细到不构成边界"的那些才会在这里被数到 —— 正好就是标记。 */
        if((lp >= 0.0f) && (rp >= 0.0f) &&
           ((rp - lp) >= (float)PURSUIT_TRACK_W_MIN) && ((rp - lp) <= (float)PURSUIT_TRACK_W_MAX))
        {
            if(pursuit_count_marks(r, (int16)(lp + 0.5f), (int16)(rp + 0.5f)) >= PURSUIT_MARK_MIN_RUNS)
            {
                trk_mark ++;
            }
        }

        lost_l    = 1;
        lost_r    = 1;
        trk_ok[r] = 0;

        if((lp >= 0.0f) && (rp >= 0.0f))
        {
            w = rp - lp;
            /* 第 0 行是车头正下方那一行：只要求宽度落在物理可能范围内，
               不做"和学习半宽差多少"的校验 —— 否则赛道比初值窄一点（比如 200mm），
               半宽永远学不出来，整帧都判不出可信行，纯追踪等于没开。 */
            if((w >= (float)PURSUIT_TRACK_W_MIN) &&
               (w <= (float)PURSUIT_TRACK_W_MAX) &&
               ((0 == r) || (fabsf(w - (2.0f * hw)) <= (PURSUIT_WIDTH_TOL * 2.0f * hw))))
            {
                c2 = (lp + rp) * 0.5f;
                if(0 == r)
                {
                    /* 基准行：直接采信，半宽和预测中线都按它对齐 */
                    trk_center[r] = c2;
                    trk_ok[r]     = 1;
                    lost_l        = 0;
                    lost_r        = 0;
                    trk_valid ++;

                    hw = (w * 0.5f);
                    pc = c2;
                }
                else if(fabsf(c2 - pred) > PURSUIT_JUMP_MAX)
                {
                    /* 两侧都找到了，但中线相对上一行突跳 8 格以上：
                       典型情况是"一边是自己赛道的边线、另一边已经跑到支路远端"，
                       这种行宁可外推，也绝对不能跟着它拐。 */
                    trk_center[r] = pred;
                    trk_open ++;
                }
                else
                {
                    trk_center[r] = c2;
                    trk_ok[r]     = 1;
                    lost_l        = 0;
                    lost_r        = 0;
                    trk_valid ++;

                    hw = (0.85f * hw) + (0.15f * (w * 0.5f));   /* 缓慢学习赛道半宽 */

                    slope = (0.5f * slope) + (0.5f * (c2 - pc));
                    if(slope > PURSUIT_SLOPE_MAX)               /* 限制斜率：不让一行坏点把后面 40 行全带偏 */
                    {
                        slope = PURSUIT_SLOPE_MAX;
                    }
                    if(slope < (-PURSUIT_SLOPE_MAX))
                    {
                        slope = (-PURSUIT_SLOPE_MAX);
                    }
                    pc = c2;
                }
            }
            else if(w > ((2.0f * hw) * PURSUIT_WIDE_RATIO))
            {
                /* 两边都在，但路面一下子宽了快一半以上：横向支路开口。
                   这一行的中线直接沿当前航向外推，并且计入 wide_rows（十字的正特征）。 */
                trk_center[r] = pred;
                trk_wide ++;
            }
            else
            {
                trk_center[r] = pred;                       /* 太窄：黑块、杂点、横臂边缘 */
                trk_open ++;
            }
        }
        else if(lp >= 0.0f)                                 /* 只看到左边：用半宽推算右边 */
        {
            c2 = lp + hw;
            if(fabsf(c2 - pred) <= PURSUIT_SINGLE_TOL)
            {
                trk_center[r] = c2;
                lost_l        = 0;
                trk_one ++;
            }
            else
            {
                trk_center[r] = pred;
                trk_open ++;
            }
        }
        else if(rp >= 0.0f)                                 /* 只看到右边：用半宽推算左边 */
        {
            c2 = rp - hw;
            if(fabsf(c2 - pred) <= PURSUIT_SINGLE_TOL)
            {
                trk_center[r] = c2;
                lost_r        = 0;
                trk_one ++;
            }
            else
            {
                trk_center[r] = pred;
                trk_open ++;
            }
        }
        else                                                /* 两侧都没有 */
        {
            trk_center[r] = pred;
            trk_open ++;
        }

        trk_lost_l[r] = lost_l;
        trk_lost_r[r] = lost_r;
        pc            = trk_center[r];
    }

    trk_hw = hw;

    /* ---- 统计"整行全白"的行数 ----
       白路面铺在深色地板上：正常赛道每一行，两侧栏外一定有深色地板，
       所以"96 格一个暗点都没有"只可能出现在横向大路面（十字/岔口）里。
       弯道永远不会有全白行（弯外是地板），因此这条判据对弯道零误伤。
       出界格点在俯视图里被填成白色，所以含出界点的行必须先排除。 */
    {
        uint8 rr;
        uint8 cc;
        uint8 all_white;
        uint8 countable;
        const uint8 *const *lut;

        trk_white = 0;

        for(rr = 0; rr < IPM_H; rr ++)
        {
            lut = IPM_Lut_Row(rr);
            if(NULL == lut)
            {
                continue;
            }

            all_white = 1;
            countable = 1;

            for(cc = 0; cc < IPM_W; cc ++)
            {
                if(!IPM_Lut_Valid(lut[cc]))
                {
                    countable = 0;                          /* 这一行有出界点，不能用来判全白 */
                    break;
                }
                if(ipm_image[rr][cc] < 128)
                {
                    all_white = 0;
                    break;
                }
            }

            if((1 == countable) && (1 == all_white))
            {
                trk_white ++;
            }
        }
    }

    /* ---- 统计"中断带"：第一段连续不可信行有多长、它上方又看清了多少行 ----
       十字/岔口：路面在中间"消失一段又回来"，带上方还能看清 -> above 大
       弯道出画：外侧边线一旦跑出画面，上方再也看不清 -> above = 0
       这是把"路口"和"弯道"分开的唯一可靠依据，比宽度阈值稳得多。 */
    {
        uint8 near_cnt = 0;
        uint8 in_band  = 0;

        trk_band  = 0;
        trk_above = 0;

        for(r = 0; r < IPM_H; r++)
        {
            if(trk_ok[r])
            {
                if(0 == in_band)
                {
                    near_cnt ++;
                }
                else
                {
                    trk_above ++;
                }
            }
            else
            {
                if(0 == in_band)
                {
                    if(near_cnt >= (PURSUIT_CROSS_MIN_VALID / 2))  /* 近处先要看清几行，才算"中断带"开始 */
                    {
                        in_band = 1;
                        trk_band = 1;
                    }
                }
                else if(0 == trk_above)
                {
                    trk_band ++;                                   /* 带还没结束（上方还没出现可信行） */
                }
            }
        }
    }
}

/* ============================================================================
 *  十字路口状态机
 *    路口特征：路面在中间"消失一段又回来"（band 长、band 上方又能看清）
 *              或者两侧边线同时被横向支路打开（wide 行数多）
 *    弯道特征：只有外侧边线跑出画面，上方永远看不清（above = 0）
 *    进出都用"连续帧数"去抖，出路口后再禁判一段，防止连续路口抖动。
 * ==========================================================================*/
static void pursuit_cross_update(float kappa_prev)
{
    uint8 suspect;

    suspect = 0;

    /* 判据一（最硬）：路面在中间消失一段，又在上方回来了 —— 十字/岔口。
       弯道不会这样：外侧边线一旦出画，上方永远是盲区。 */
    if((trk_band >= PURSUIT_CROSS_MIN_BAND) &&
       (trk_above >= PURSUIT_CROSS_MIN_ABOVE))
    {
        suspect = 1;
    }
    if(trk_wide >= PURSUIT_CROSS_MIN_WIDE)                  /* 判据二：路面整体变宽 -> 横向支路开口 */
    {
        suspect = 1;
    }
    if((trk_open >= PURSUIT_CROSS_MIN_OPEN) &&
       (trk_open > ((trk_one * 3) / 2)) &&
       (trk_above >= 2))                                   /* 判据三：两条边线同时消失、且上方路面又回来了 */
    {
        suspect = 1;
    }
    if((trk_wide + trk_open) > ((IPM_H * 3) / 5))           /* 大半个画面都没了 -> 摄像头问题 */
    {
        suspect = 0;
    }
    if((trk_valid < PURSUIT_CROSS_MIN_VALID) &&             /* 看得太少，且不是"消失又回来"的路口 */
       (trk_above < PURSUIT_CROSS_MIN_ABOVE))
    {
        suspect = 0;
    }
    if(fabsf(kappa_prev) > PURSUIT_CROSS_MAX_KAPPA)         /* 进路口前正在大转弯 -> 是弯道 */
    {
        suspect = 0;
    }
    /* 判据零（最硬，放在所有否决之后，带否决权）：白路面上有黑色短横条标记。
       实车视频里这个赛道的十字路口就是两排这样的横条，弯道永远没有，
       所以它一旦成立就认定路口，不管上面那些否决条件怎么说。 */
    if(trk_mark >= PURSUIT_MARK_MIN_ROWS)
    {
        suspect = 1;
    }

    /* 判据零之二（同样放在所有否决之后）：出现"整行全白"。
       横向大路面（十字/岔口）才会让一整行 96 格一个暗点都没有；
       弯道外侧永远有深色地板，所以这条不会把弯道误判成路口。
       它专门解决"路口太宽、wide/open 被大半个画面否决掉"的情形。 */
    if(trk_white >= PURSUIT_FULL_WHITE_MIN)
    {
        suspect = 1;
    }

    if(cross_reject > 0)
    {
        cross_reject --;
        suspect = 0;
    }

    if(1 == cross_state)
    {
        if(cross_ticks > 0)
        {
            cross_ticks --;
        }
        if(0 != suspect)
        {
            cross_clean = 0;
            cross_ticks = PURSUIT_CROSS_TICKS;              /* 还在路口里：续期。
                                                               否则车慢的时候 32 帧用完了、
                                                               人还没开过路口就退出去，又变成跟着支路拐。 */
        }
        else if(cross_clean < 200)
        {
            cross_clean ++;
        }

        if((0 == cross_ticks) || (cross_clean >= PURSUIT_CROSS_CLEAN))
        {
            cross_state  = 0;
            cross_ticks  = 0;
            cross_clean  = 0;
            cross_reject = PURSUIT_CROSS_REJECT;
        }
    }
    else
    {
        if(0 != suspect)
        {
            if(cross_hits < 200)
            {
                cross_hits ++;
            }
        }
        else
        {
            cross_hits = 0;
        }

        if(cross_hits >= 2)
        {
            cross_state = 1;
            cross_ticks = PURSUIT_CROSS_TICKS;
            cross_clean = 0;
            cross_hits  = 0;
            cross_kappa = kappa_prev;                       /* 记住进路口前车头的走向 */
        }
    }
}

/* ============================================================================
 *  主处理：边界提取 -> 十字判定 -> 纯追踪 -> 发布快照
 * ==========================================================================*/
void Pursuit_Process(void)
{
    pursuit_struct snap;
    float kappa_prev;
    float kappa;
    float kappa_use;
    float speed_ref;
    float norm;
    float ld;
    float cc;
    float xt;
    float yt;
    float delta;
    float steer;
    float ratio;
    float slow;
    float near_sum;
    float near_off;
    float xt_max;
    int16 r0;
    int16 ld_row;
    uint8 off_track;
    uint8 r;
    uint8 near_cnt;

    kappa_prev = pursuit_info.kappa;

    memset(&snap, 0, sizeof(snap));
    snap.valid = 0;
    snap.slow  = 1.0f;

    if(0 == ipm_info.ok)                                    /* 标定表没建好，直接交给老算法 */
    {
        ps_buf[ps_pub ^ 1] = snap;
        ps_pub = (uint8)(ps_pub ^ 1);
        pursuit_info = snap;
        return;
    }

    pursuit_extract_track();
    pursuit_cross_update(kappa_prev);

    for(r = 0; r < IPM_H; r ++)
    {
        pursuit_center_row[r] = (int16)trk_center[r];
    }

    /* ---------------- 纯追踪 ---------------- */
    speed_ref = control_base_speed_ref;
    norm      = speed_ref / CONTROL_SPEED_MAX;
    if(norm < PURSUIT_SPEED_NORM_MIN)
    {
        norm = PURSUIT_SPEED_NORM_MIN;
    }
    if(norm > 1.0f)
    {
        norm = 1.0f;
    }

    ld = PURSUIT_LD_MIN_MM + ((PURSUIT_LD_MAX_MM - PURSUIT_LD_MIN_MM) * norm);
    ld_row = (int16)(((ld - IPM_Y_NEAR_MM) / ipm_info.dy_mm) + 0.5f);
    if(ld_row < PURSUIT_TARGET_ROW_MIN)
    {
        ld_row = PURSUIT_TARGET_ROW_MIN;
    }
    if(ld_row > (IPM_H - 2))
    {
        ld_row = (IPM_H - 2);
    }

    /* 前瞻点的铁律：只能用"真正可信"的行（左右边线都在、宽度正常、没有横向突跳）。
       理想前瞻行要是落在路口/支路开口里，就往下找最近的可信行。
       实车冲出路口的直接原因就是拿外推行当前瞻点：那几行的中线是沿航向外推出来的，
       横向偏移一大，纯追踪就会算出一个大曲率，车顺着就拐进支路了。 */
    r0 = -1;
    for(r = (uint8)ld_row; r >= (uint8)PURSUIT_TARGET_ROW_MIN; r --)
    {
        if((1 == trk_ok[r]) && (1 == trk_ok[r - 1]) && (1 == trk_ok[r + 1]))
        {
            r0 = (int16)r;
            break;
        }
    }

    /* 行数太少、或者整幅俯视图里找不到可信行：
       先沿用上一份快照，最多 PURSUIT_HOLD_FRAMES 帧；超了就这一帧不出结论，
       控制层看到 valid=0 会自动回退到老的偏差 PD（不会失控，只是变回旧手感）。
       —— 但路口是例外：路口里本来就没有远期可信行，这时必须继续由纯追踪接管，
          否则一回退到老 PD，它就会跟着被支路拉走的中线往左右拐。 */
    if(1 == cross_state)
    {
        if(r0 < 0)
        {
            r0 = (int16)PURSUIT_CROSS_NEAR_ROWS;             /* 路口内 r0 只用于显示/组包 */
        }
        pursuit_hold = 0;
    }
    else if((trk_valid < PURSUIT_CROSS_MIN_VALID) || (r0 < 0))
    {
        if(pursuit_hold < PURSUIT_HOLD_FRAMES)
        {
            pursuit_hold ++;
            snap               = ps_buf[ps_pub];
            snap.valid_rows    = trk_valid;
            snap.open_rows     = trk_open;
            snap.one_side_rows = trk_one;
            snap.wide_rows     = trk_wide;
            snap.band_rows     = trk_band;
            snap.above_rows    = trk_above;
            snap.mark_rows     = trk_mark;
            snap.white_rows    = trk_white;
            ps_buf[ps_pub ^ 1] = snap;
            ps_pub             = (uint8)(ps_pub ^ 1);
            pursuit_info       = snap;
            return;
        }

        ps_buf[ps_pub ^ 1] = snap;
        ps_pub             = (uint8)(ps_pub ^ 1);
        pursuit_info       = snap;
        return;
    }
    pursuit_hold = 0;

    /* ---- 选目标点 ----
       正常情况：用"真正可信"的前瞻行（上面已经筛过）。
       路口情况：**只用近场车道轴**（车头前 170~310mm 那几行），前瞻距离固定 400mm。
       路口这条路的硬性要求是"只能直行"，绝不允许往左右支路拐；
       近场永远是脚下的真路面，所以就算横向路把远区整片挡住，目标点也跑不进支路。 */
    if(1 == cross_state)
    {
        near_cnt = 0;
        near_sum = 0.0f;
        for(r = 1; r <= PURSUIT_CROSS_NEAR_ROWS; r ++)
        {
            if(1 == trk_ok[r])
            {
                near_sum += trk_center[r];
                near_cnt ++;
            }
        }
        if(near_cnt >= 2)
        {
            cc = near_sum / ((float)near_cnt);
        }
        else
        {
            cc = trk_center[0];
        }

        yt = PURSUIT_CROSS_LOOKAHEAD_MM;
    }
    else
    {
        cc = (trk_center[r0 - 1] + trk_center[r0] + trk_center[r0 + 1]) / 3.0f;
        yt = IPM_Y_NEAR_MM + (((float)r0) * ipm_info.dy_mm);
    }

    xt = (cc - PURSUIT_CENTER_COL) * ipm_info.dx_mm;

    /* 横向偏移硬上限：前瞻点横向最多到俯视图边缘的 92%。
       路口处就算有一侧边界被支路拉走，也不会算出离谱的偏移量；
       真正的"别拐进支路"靠上面的可信行筛选 + 斜率限幅 + 路口内转角限幅。 */
    xt_max = PURSUIT_XT_CLAMP_MM;
    if(xt_max < 20.0f)
    {
        xt_max = 20.0f;
    }
    if(xt > xt_max)
    {
        xt = xt_max;
    }
    if(xt < (-xt_max))
    {
        xt = (-xt_max);
    }

    kappa = (2.0f * xt) / ((xt * xt) + (yt * yt));

    delta = atanf(kappa * PURSUIT_WHEELBASE_MM) * PURSUIT_RAD2DEG;
    if(delta > PURSUIT_DELTA_MAX_DEG)
    {
        delta = PURSUIT_DELTA_MAX_DEG;
    }
    if(delta < (-PURSUIT_DELTA_MAX_DEG))
    {
        delta = (-PURSUIT_DELTA_MAX_DEG);
    }

    if(1 == cross_state)                                    /* 路口内硬限幅：绝不许横着拐进支路 */
    {
        if(delta > PURSUIT_CROSS_DELTA_MAX)
        {
            delta = PURSUIT_CROSS_DELTA_MAX;
        }
        if(delta < (-PURSUIT_CROSS_DELTA_MAX))
        {
            delta = (-PURSUIT_CROSS_DELTA_MAX);
        }
    }

    kappa_use = tanf(delta / PURSUIT_RAD2DEG) / PURSUIT_WHEELBASE_MM;

    /* delta 正 = 右转；steer 正 = 向右（占空比减小，与 Servo_Set_Offset 的约定一致）。
       早期版本这里取了负号，实车表现为"左弯往右打"，这是转向反了的根因，已修正。 */
    steer = ((delta / PURSUIT_DELTA_MAX_DEG)) * ((float)SERVO_DUTY_RANGE_L);
    if(steer > ((float)SERVO_DUTY_RANGE_L))
    {
        steer = (float)SERVO_DUTY_RANGE_L;
    }
    if(steer < (-(float)SERVO_DUTY_RANGE_R))
    {
        steer = -(float)SERVO_DUTY_RANGE_R;
    }

    ratio = kappa_use * PURSUIT_REAR_TRACK_MM * 0.5f;
    if(ratio > CONTROL_DIFF_K_MAX)
    {
        ratio = CONTROL_DIFF_K_MAX;
    }
    if(ratio < (-CONTROL_DIFF_K_MAX))
    {
        ratio = (-CONTROL_DIFF_K_MAX);
    }

    slow = 1.0f - (PURSUIT_CURV_SLOW_K * fabsf(delta));
    if(slow < PURSUIT_CURV_SLOW_MIN)
    {
        slow = PURSUIT_CURV_SLOW_MIN;
    }
    if(slow > 1.0f)
    {
        slow = 1.0f;
    }

    /* ---------------- 冲出赛道保护 ---------------- */
    near_sum = 0.0f;
    for(r = 0; r < 6; r ++)
    {
        near_sum += trk_center[r];
    }
    near_off  = ((near_sum / 6.0f) - PURSUIT_CENTER_COL) * ipm_info.dx_mm;
    off_track = 0;
    if((fabsf(near_off) > PURSUIT_OFFTRACK_MM) && (fabsf(near_off) < (3.0f * PURSUIT_OFFTRACK_MM)))
    {
        off_track = 1;
        steer     = (-near_off) * PURSUIT_RECOVER_GAIN;     /* 直接朝赛道中心打 */
        if(steer > ((float)SERVO_DUTY_RANGE_L))
        {
            steer = (float)SERVO_DUTY_RANGE_L;
        }
        if(steer < (-(float)SERVO_DUTY_RANGE_R))
        {
            steer = -(float)SERVO_DUTY_RANGE_R;
        }
    }

    /* ---------------- 组包发布 ---------------- */
    /* 路口里即使远期看不清也要继续由纯追踪接管（否则回退老 PD 会被支路带走） */
    snap.valid         = ((trk_valid >= PURSUIT_CROSS_MIN_VALID) || (1 == cross_state)) ? 1 : 0;
    snap.cross         = cross_state;
    snap.off_track     = off_track;
    snap.valid_rows    = trk_valid;
    snap.open_rows     = trk_open;
    snap.one_side_rows = trk_one;
    snap.wide_rows     = trk_wide;
    snap.band_rows     = trk_band;
    snap.above_rows    = trk_above;
    snap.mark_rows     = trk_mark;
    snap.white_rows    = trk_white;
    snap.target_row    = r0;
    snap.target_col    = (int16)cc;
    snap.kappa         = kappa;
    snap.delta_deg     = delta;
    snap.steer_out     = steer;
    snap.slow          = slow;
    snap.ratio         = ratio;
    snap.lookahead_mm  = ld;
    snap.target_x_mm   = xt;
    snap.target_y_mm   = yt;
    snap.near_off_mm   = near_off;
    snap.half_width_mm = trk_hw * ipm_info.dx_mm;

    ps_buf[ps_pub ^ 1] = snap;
    ps_pub             = (uint8)(ps_pub ^ 1);

    pursuit_info = snap;
}

/* ============================================================================
 *  控制中断里取快照（双缓冲：读的时候读一个完整的旧包，不会读到写了一半的）
 * ==========================================================================*/
uint8 Pursuit_Get(pursuit_struct *out)
{
    uint8 idx;

    if(0 == out)
    {
        return (0);
    }

    idx  = ps_pub;
    *out = ps_buf[idx];

    return (out->valid);
}

void Pursuit_Init(void)
{
    uint8 r;

    memset(trk_center, 0, sizeof(trk_center));
    memset(trk_ok, 0, sizeof(trk_ok));
    memset(trk_lost_l, 0, sizeof(trk_lost_l));
    memset(trk_lost_r, 0, sizeof(trk_lost_r));
    memset(pursuit_center_row, 0, sizeof(pursuit_center_row));
    memset(&pursuit_info, 0, sizeof(pursuit_info));
    memset(ps_buf, 0, sizeof(ps_buf));

    for(r = 0; r < IPM_H; r ++)
    {
        trk_center[r]       = PURSUIT_CENTER_COL;
        pursuit_center_row[r] = (int16)PURSUIT_CENTER_COL;
    }

    trk_valid    = 0;
    trk_open     = 0;
    trk_one      = 0;
    trk_wide     = 0;
    trk_band     = 0;
    trk_above    = 0;
    trk_mark     = 0;
    trk_white    = 0;
    trk_hw       = 0.0f;
    cross_state  = 0;
    cross_hits   = 0;
    cross_ticks  = 0;
    cross_clean  = 0;
    cross_reject = 0;
    cross_kappa  = 0.0f;
    pursuit_hold = 0;
    ps_pub       = 0;
}

/* ============================================================================
 *  屏幕显示：TFT180 默认竖屏 128x160、字体 8x16（一个字符 8x16 像素）
 *    Y1~Y6（y = 2/18/34/50/68/84）已被相机/基点/基线/中线/控制五块占满，
 *    所以这里补第 7、8、9 行（y = 100 / 116 / 132）：
 *      第 7 行：P 看得清的行数 | c 路口标志 | d 前轮转角 | o 冲出赛道标志
 *      第 8 行：b 中断带长度 | A 带上方看清行数 | W 变宽行数 | M 黑色横条行数
 *      第 9 行：F 整行全白行数 | D 舵机当前档位占空比（460/440/474/418/500）
 *    关键：zf 库的 show_int 字符串长度是 num+1，show_float 是 num+pointnum+2，
 *          不足补空格（定长），所以 x 必须按这个长度留格子，最后一格一旦到 x=128
 *          就会在 tft180.c:460 触发 zf_assert(x < tft180_width_max) 死机。
 * ==========================================================================*/
void Pursuit_Show_On_TFT180(void)
{
#if ((1 == PURSUIT_DIS_EN) && (0 == IPM_DIS_EN))
    uint16 y1 = 0;
    uint16 y2 = 0;
    uint16 y3 = 0;

    y1 = (uint16)(CAMERA_INFO_TOP + 100);
    y2 = (uint16)(y1 + 16);
    if((y1 + 16) > tft180_height_max)
    {
        return;                                         /* 屏幕放不下（比如开了 CAMERA_DIS_IMAGE 整体下移）就不画 */
    }

    tft180_show_string(0,   y1, "P");
    tft180_show_int   (8,   y1, (int32)pursuit_info.valid_rows, 2);     /* 8  .. 31 */
    tft180_show_string(32,  y1, "c");
    tft180_show_int   (40,  y1, (int32)pursuit_info.cross, 1);          /* 40 .. 55 */
    tft180_show_string(56,  y1, "d");
    tft180_show_float (64,  y1, (double)pursuit_info.delta_deg, 2, 1);  /* 64 .. 103 */
    tft180_show_string(104, y1, "o");
    tft180_show_int   (112, y1, (int32)pursuit_info.off_track, 1);      /* 112 .. 127 */

    if((y2 + 16) > tft180_height_max)
    {
        return;
    }
    tft180_show_string(0,  y2, "b");
    tft180_show_int   (8,  y2, (int32)pursuit_info.band_rows, 2);               /* 8  .. 31 */
    tft180_show_string(32, y2, "A");
    tft180_show_int   (40, y2, (int32)pursuit_info.above_rows, 2);              /* 40 .. 63 */
    tft180_show_string(64, y2, "W");
    tft180_show_int   (72, y2, (int32)pursuit_info.wide_rows, 2);               /* 72 .. 95 */
    tft180_show_string(96, y2, "M");
    tft180_show_int   (104, y2, (int32)pursuit_info.mark_rows, 2);              /* 104 .. 119 */

    /* 第 9 行（y = 132）：F 整行全白的行数 | D 舵机当前档位占空比（460/440/474/418/500） */
    y3 = (uint16)(y2 + 16);
    if((y3 + 16) <= tft180_height_max)
    {
        tft180_show_string(0,  y3, "F");
        tft180_show_int   (8,  y3, (int32)pursuit_info.white_rows, 2);          /* 8  .. 31 */
        tft180_show_string(32, y3, "D");
        tft180_show_int   (40, y3, Servo_Get_Duty(), 3);                        /* 40 .. 71 */
    }
#endif
}
