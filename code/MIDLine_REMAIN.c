/*
 * MIDLine_REMAIN.c
 *
 *  加权中线判定法（含互补滤波 + 十字路口多特征状态机）
 *
 *  一、逐行求中线
 *      left_col  = k_left  * row + b_left         （左侧边缘基线）
 *      right_col = k_right * row + b_right        （右侧边缘基线）
 *      mid       = (left_col + right_col) / 2     两侧都有效
 *      mid       = left_col  + width / 2          只有左侧有效（用上一帧统计的赛道宽度推算）
 *      mid       = right_col - width / 2          只有右侧有效
 *  存进 mid_line_list[row]，该行的权重由 midline_weight(row, turn) 给出（单侧推算时权重减半）
 *
 *  二、双段加权
 *      直道：row < 60 用 row+1（线性），row >= 60 用 (row-60)^2/4+61（二次），上限 200 —— 近景主导
 *      弯道：row > MIDLINE_TURN_HEIGHT 的行统一取固定小权重 8，row <= 40 的行用 (120-row) —— 远端主导
 *      mid_line = Σ(mid * w) / Σw
 *      conf     = Σw / Σw(full)  （满权重，0~100）
 *
 *  三、滤波与输出
 *      mid_line_value = last_mid_line * 0.2 + mid_line * 0.8
 *      error          = mid_line_value - MT9V03X_W / 2
 *
 *  四、十字路口（见 MIDLine_REMAIN.h 的说明）
 *      路口内保持进路口前的航向（绝不硬回正），出路口后冻结宽度学习，冷却结束再重新捕获
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */
#include "MIDLine_REMAIN.h"
#include "Camera_REMAIN.h"                               // base_image / camera_threshold

midline_struct midline_info;                            // 中线计算结果
uint8          mid_weight_list[MT9V03X_H];              // mid_weight_list[row]：权重增益（百分比）

/* ---- 逐行扫描的中间量 ---- */
static uint8  cross_mark_rows = 0;                      // 本帧有"短黑条"的行数（F5）
static uint8  turn_flag       = 0;                      // 弯道滞回状态

/* ---- 十字路口状态机 ---- */
static uint8  cross_state  = MIDLINE_CROSS_ST_NONE;     // 当前状态
static uint16 cross_ticks  = 0;                         // INSIDE / EXIT / COOLDOWN 剩余帧数
static uint8  cross_hit    = 0;                         // 连续命中帧数（进路口用）
static uint8  cross_clean  = 0;                         // 连续无特征帧数（出路口用）
static uint8  cross_hint   = 0;                         // 上一帧的路口提示：本帧算中线前就要生效
static uint8  cross_open_cnt = 0;                       // 近端宽度连续超限的帧数（路口"快速通道"）
static float  cross_hold_center = 0.0f;                 // 路口内的中线（逐帧回中，不是死锁值）
static float  cross_hold_snap   = 0.0f;                 // 进路口那一帧的中线（补线平移基准）
static uint8  cross_hold_line[MT9V03X_H];               // 进路口前的中线快照（路口内补线用）

/* ---- 直角对（十字路口专供）：近处两个直角 + 对面两个直角 = 两条补线基线 ---- */
static midline_corner_struct corner_left;               // 左侧：近处左直角 <-> 对面左直角
static midline_corner_struct corner_right;              // 右侧：近处右直角 <-> 对面右直角
static uint8  corner_state_now = 0;                     // 本帧检测结果：0-无  1-只有近处  2-近处+对面
static uint8  corner_ready     = 0;                     // 补线是否可用（过了防抖）
static uint8  corner_hit       = 0;                     // 连续成立帧数
static uint8  corner_keep      = 0;                     // 保持计数（0.5s = 25 帧，丢帧后倒计时）
static uint8  corner_slide     = 0;                     // 基线下滑量（帧）：保持期基线要跟着车往车头方向走
static uint8  corner_run       = 0;                     // 补线已经连续用了多少帧（总时长上限用）
static uint8  corner_far_ok    = 0;                     // 本轮连续段里是否见过"对面两个直角"（决定确认门槛）

/* ---- 逐行继承补线（解决路口丢线）：真实边界快照 + 补线标记 ---- */
static uint8  fill_flag[MT9V03X_H];                     // 1-该行是补出来的（加权时打折）
static uint8  real_left[MT9V03X_H];                     // 补线前的真实左边界（路口特征打分只能用这份）
static uint8  real_right[MT9V03X_H];                    // 补线前的真实右边界
static uint8  out_left[MT9V03X_H];                      // 1-该行左侧是界外（左转直角张开的那一侧）
static uint8  out_right[MT9V03X_H];                     // 1-该行右侧是界外（右转直角张开的那一侧）
static uint8  outside_cnt = 0;                          // 本帧判为界外的行数
static uint8  right_angle = 0;                          // 1-本帧是"直角弯/单侧张开"：只看近处，不按远端打方向
static uint8  blind_cnt = 0;                            // 连续"两侧基线同时失效"的帧数（赛道比视野还宽）

/* ---- 中线历史环形缓冲：基点行本身就丢线时，取"三帧前"的成品兜底（参考补线法 line.c） ---- */
typedef struct
{
    uint8  valid;                                       // 1-这一帧的中线是算出来的（不是丢线帧）
    uint8  line[MT9V03X_H];                             // 该帧逐行中线
    uint8  mid_line;                                    // 该帧加权中线（滤波前）
    uint8  mid_line_value;                              // 该帧滤波后的输出
    uint8  row_start, row_end;                          // 该帧的计算区间
    uint8  conf;                                        // 该帧置信度
    float  center;                                      // 该帧滤波后中线（浮点）
    float  last_mid_line;                               // 该帧的滤波状态
} midline_hist_struct;

static midline_hist_struct hist_buf[MIDLINE_HIST_DEPTH];
static uint8  hist_head = 0;                            // 下一个写入位置（环形）
static uint8  hist_cnt  = 0;                            // 已存帧数（<= MIDLINE_HIST_DEPTH）
static uint8  hist_used = 0;                            // 连续靠历史兜底的帧数（超过 MIDLINE_HIST_MAX 就不再仗着老数据）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把"上一帧算完的成品"压进中线历史环形缓冲
// 参数说明     void
// 返回参数     void
// 备注信息     在 Find_MIDLine() 把 midline_info 清空（valid=0 / line[] 置无效）之前调用
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Hist_Push(void)
{
    midline_hist_struct *p = &hist_buf[hist_head];

    /* 只有"真算出来的帧"才配进历史：软兜底/回中式输出的 conf 只有 25，低于 MIDLINE_RAW_CONF，
     * 把它们也存进来，三帧后就会拿一条假中线去当真中线用，等于把误差一路带下去 */
    p->valid          = ((midline_info.conf >= MIDLINE_RAW_CONF) ? midline_info.valid : ((uint8)0));
    p->mid_line       = midline_info.mid_line;
    p->mid_line_value = midline_info.mid_line_value;
    p->row_start      = midline_info.row_start;
    p->row_end        = midline_info.row_end;
    p->conf           = midline_info.conf;
    p->center         = midline_info.center;
    p->last_mid_line  = midline_info.last_mid_line;

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        p->line[i] = midline_info.line[i];
    }

    hist_head = (uint8)((hist_head + 1) % MIDLINE_HIST_DEPTH);
    if(hist_cnt < MIDLINE_HIST_DEPTH)
    {
        hist_cnt ++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取 back 帧前的成品当作本帧输出（基点行丢线专用）
// 参数说明     back    回退帧数，1 = 上一帧，MIDLINE_HIST_BACK = 丢线帧(倒数第1帧)往前数第 5 帧
// 返回参数     uint8   1-已输出  0-历史不够深或那帧本身就是丢线帧（调用者继续往下兜底）
// 备注信息     为什么不用"上一帧"：单帧丢线多是反光、阴影、赛道外杂物，
//              上一帧很可能已经踩进脏数据里；往前退 5 帧（100ms）取一条确定算得出来的中线，
//              比沿着上一帧做软收敛更干净，也不会把最后那个错误角度一直保持下去
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Hist_Output(uint8 back)
{
    midline_hist_struct *p;
    uint8 idx;

    if(back >= MIDLINE_HIST_DEPTH)                      // 回退超过深度，直接放弃
    {
        return (0);
    }
    if(hist_cnt < (uint8)(back + 1))                    // 历史还没攒够（刚上电）
    {
        return (0);
    }

    idx = (uint8)((hist_head + MIDLINE_HIST_DEPTH - back) % MIDLINE_HIST_DEPTH);
    p   = &hist_buf[idx];

    if(0 == p->valid)                                   // 那一帧本身就在丢线，不能拿来用
    {
        return (0);
    }

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i] = p->line[i];
    }

    midline_info.mid_line       = p->mid_line;
    midline_info.mid_line_value = p->mid_line_value;
    midline_info.row_start      = p->row_start;
    midline_info.row_end        = p->row_end;
    midline_info.conf           = p->conf;
    midline_info.center         = p->center;
    midline_info.last_mid_line  = p->last_mid_line;
    midline_info.error          = p->center - ((float)MT9V03X_W * 0.5f);
    midline_info.valid          = 1;

    return (1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     图像坐标 -> TFT180 显示坐标（与 tft180_show_gray_image 的缩放一致）
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint16 midline_to_dis_x(uint8 col)
{
    /* 与 tft180_show_gray_image 的采样方向严格互逆：
     * 驱动第 i 列画的是源图 i*188/128 列，所以源图 col 列要落在 col*128/188 列 */
    return ((uint16)(CAMERA_DIS_X0 + ((uint32)col * CAMERA_DIS_W / MT9V03X_W)));
}

static uint16 midline_to_dis_y(uint8 row)
{
    return ((uint16)(CAMERA_DIS_Y0 + ((uint32)row * CAMERA_DIS_H / MT9V03X_H)));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     单行权重：双段式 line+1 / line^2+1，弯道时权重方向翻转
// 参数说明     row     行号（0 最远，MT9V03X_H-1 最近车头）
//              turn    1-弯道（远端主导）  0-直道（近景主导）
// 返回参数     uint32  该行权重（已乘 mid_weight_list[] 的百分比增益）
// 备注信息     1. 直道：d = row，越靠近车头越大，二次段放大近景
//              2. 弯道：row > MIDLINE_TURN_HEIGHT 的行固定 MIDLINE_TURN_NEAR_W，
//                       row <= MIDLINE_TURN_HEIGHT 用 (MT9V03X_H-1-row)，远端权重最大
//              3. 想手工干预某一行：改 mid_weight_list[row]（100 = 不加不减）
//-------------------------------------------------------------------------------------------------------------------
static uint32 midline_weight(int row, uint8 turn)
{
    uint32 gain = (uint32)mid_weight_list[row];
    uint32 w;

    if(turn)
    {
        if(row > MIDLINE_TURN_HEIGHT)
        {
            w = (uint32)MIDLINE_TURN_NEAR_W;            // 近景平台，只保留连续性
        }
        else
        {
            w = (uint32)(MT9V03X_H - 1 - row) + 1u;     // 远端最大
        }
        return ((w * gain) / 100u);
    }

    if(row < MIDLINE_WEIGHT_SEG_A_END)                  // 线性段：line+1
    {
        w = (uint32)row + 1u;
    }
    else                                                // 二次段：line^2+1
    {
        uint32 d = (uint32)(row - MIDLINE_WEIGHT_SEG_A_END);
        w = ((d * d) / MIDLINE_WEIGHT_QUAD_DIV) + (uint32)MIDLINE_WEIGHT_SEG_A_END + 1u;
        if(w > (uint32)MIDLINE_WEIGHT_MAX)
        {
            w = (uint32)MIDLINE_WEIGHT_MAX;             // 近景平台，防止一行吃掉全部权重
        }
    }
    return ((w * gain) / 100u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐行真实边界扫描（十字特征的数据源，顺带统计短黑条）
// 参数说明     void
// 返回参数     void
// 备注信息     1. 种子取上一帧的 mid_line_list[row]，首帧取画面中心——时序连续，不怕横臂
//              2. 从种子向左右走，遇到"薄黑条"(<= MIDLINE_RAW_MARK_MAX px) 就跳过去继续走，
//                 遇到"厚黑块"才算路基外的深色地板 = 边界（这一步就是过路口的补线）
//              3. 结果写 midline_info.raw_left[] / raw_right[]，无效为 MIDLINE_INVALID_COL
//              4. 必须在清空 mid_line_list 之前调用（要用上一帧的中线做种子）
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Row_Scan(void)
{
    for(uint16 row = 0; row < MT9V03X_H; row ++)
    {
        midline_info.raw_left[row]  = MIDLINE_INVALID_COL;
        midline_info.raw_right[row] = MIDLINE_INVALID_COL;
        out_left[row]               = 0;
        out_right[row]              = 0;
    }
    cross_mark_rows = 0;
    outside_cnt     = 0;

    for(int row = 2; row <= (MT9V03X_H - 3); row ++)
    {
        const uint8 *p = base_image[row];
        int seed = ((midline_info.line[row] != MIDLINE_INVALID_COL) ?
                    ((int)midline_info.line[row]) : (MT9V03X_W / 2));

        if((seed < 2) || (seed > (MT9V03X_W - 3)))      // 种子跑出画面就用中心
        {
            seed = MT9V03X_W / 2;
        }
        /* 路口里上一帧的中线可能被横臂带偏，种子留在偏侧会把左右边界一路走歪；
         * 这时直接拉回画面中路，从中间往两边找，左右两条边先撞到哪就是哪。 */
        if((1 == cross_hint) &&
           ((seed < (MT9V03X_W / 4)) || (seed > ((MT9V03X_W * 3) / 4))))
        {
            seed = MT9V03X_W / 2;
        }
        if(0 == p[seed])                                // 种子落在黑点上，附近找白点
        {
            int k = 1;
            while(k <= 6)
            {
                if((seed + k < MT9V03X_W) && (0 != p[seed + k])) { seed += k; break; }
                if((seed - k >= 0)        && (0 != p[seed - k])) { seed -= k; break; }
                k ++;
            }
            if(0 == p[seed])
            {
                continue;                               // 整段都黑：本行放弃
            }
        }

        int left  = -1;
        int right = -1;
        uint8 edge_l = 0;                               // 1-左侧一路扫到画面左沿（整行发白 = 可能界外）
        uint8 edge_r = 0;                               // 1-右侧一路扫到画面右沿

        // ---- 向左走：薄黑条跳过，厚黑块是边界 ----
        {
            int x    = seed;
            uint8 n  = 0;
            while(x > 0)
            {
                if(0 == p[x])
                {
                    int x0 = x;
                    while((x0 > 0) && (0 == p[x0]))
                    {
                        x0 --;
                    }
                    int w = x - x0;                     // 黑段宽度
                    if((w <= MIDLINE_RAW_MARK_MAX) && (n < MIDLINE_RAW_SKIP_MAX) && (x0 > 3))
                    {
                        n ++;                           // 薄黑条：跳过去
                        x = x0;
                        continue;
                    }
                    left = x + 1;                       // 厚黑块：右沿就是左边界
                    break;
                }
                x --;
            }
            if(x <= 0 && left < 0)
            {
                left   = 0;                             // 整行都白：贴到图像左沿（横臂张开）
                edge_l = 1;                             // 先记下来，下面再判断是"贴边"还是"界外"
            }
        }

        // ---- 向右走 ----
        {
            int x   = seed;
            uint8 n = 0;
            while(x < (MT9V03X_W - 1))
            {
                if(0 == p[x])
                {
                    int x1 = x;
                    while((x1 < (MT9V03X_W - 1)) && (0 == p[x1]))
                    {
                        x1 ++;
                    }
                    int w = x1 - x;                     // 黑段宽度
                    if((w <= MIDLINE_RAW_MARK_MAX) && (n < MIDLINE_RAW_SKIP_MAX) && (x1 < (MT9V03X_W - 4)))
                    {
                        n ++;
                        x = x1;
                        continue;
                    }
                    right = x - 1;                      // 厚黑块：左沿就是右边界
                    break;
                }
                x ++;
            }
            if(x >= (MT9V03X_W - 1) && right < 0)
            {
                right  = MT9V03X_W - 1;
                edge_r = 1;
            }
        }

        /* ---- 直角弯的界外判定（本轮新增）----
         * 直角张开的那一侧，整行是连成一片的白，扫描会一路撞到画面边缘（left=0 / right=W-1）。
         * 那一片已经是"界外"（横臂、支路、赛道外面的空地），不是我们的赛道边界：
         *   右转直角 -> 右侧继续往右是界外 -> 右侧判无效
         *   左转直角 -> 左侧继续往左是界外 -> 左侧判无效
         * 只有"一侧贴边、另一侧还好好的"才算界外；两侧都贴边是路面宽出画面（正常近景），
         * 维持原样（中线取画面中心），否则直道也会被误伤。 */
#if (1 == MIDLINE_OUTSIDE_EN)
        /* 只有"已经在路口策略里"才允许判界外。
         * 直道/弯道上车稍微偏一点，一侧边界本来就会撞出画面（赛道比画面宽），
         * 那不是界外，是正常近景 —— 判成界外会把那条正常基线作废，
         * 中线立刻退化成"另一侧 ± 学习半宽"，只要宽度学习值有偏差，巡线立刻飘。
         * 直角弯会被"宽度快速通道"提前拉进 cross_hint，所以仍然判得到。 */
        if((edge_l != edge_r) && ((1 == cross_hint) || (MIDLINE_CROSS_ST_INSIDE == cross_state)))
        {
            if(1 == edge_l)
            {
                out_left[row] = 1;                      // 左侧界外
                left          = -1;
            }
            else
            {
                out_right[row] = 1;                     // 右侧界外
                right         = -1;
            }
            if(outside_cnt < 255)
            {
                outside_cnt ++;
            }
        }
#endif

        if((left < 0) && (right < 0))
        {
            continue;                                   // 两侧都不成立：本行没有可用边界
        }
        if((left >= 0) && (right >= 0) && (right <= left))
        {
            continue;                                   // 左右交叉：本行边界无效
        }

        midline_info.raw_left[row]  = (left  >= 0) ? ((uint8)left)  : (MIDLINE_INVALID_COL);
        midline_info.raw_right[row] = (right >= 0) ? ((uint8)right) : (MIDLINE_INVALID_COL);

#if (1 == MIDLINE_CROSS_MARK_EN)
        // ---- 统计这一行里落在路面内的短黑条（路口两排横条），行区间避开斑马线扫描带 ----
        if((left >= 0) && (right >= 0) &&
           (row >= MIDLINE_CROSS_MARK_ROW_TOP) && (row <= MIDLINE_CROSS_MARK_ROW_BOT))
        {
            int runs = 0;
            int x    = left;
            while(x < right)
            {
                if(0 == p[x])
                {
                    int x1 = x;
                    while((x1 < right) && (0 == p[x1]))
                    {
                        x1 ++;
                    }
                    int w = x1 - x;
                    if((w >= 1) && (w <= MIDLINE_RAW_MARK_MAX))
                    {
                        runs ++;
                    }
                    x = x1;
                }
                else
                {
                    x ++;
                }
            }
            // 2~4 根 = 路口短横条；>=5 根是斑马线，交给 Zebra 模块，这里不算
            if((runs >= MIDLINE_CROSS_MARK_RUN_MIN) && (runs <= MIDLINE_CROSS_MARK_RUN_MAX))
            {
                if(cross_mark_rows < 255)
                {
                    cross_mark_rows ++;
                }
            }
        }
#endif
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐行继承补线：把十字路口左右边界的"丢线"填掉
// 参数说明     width_ref   本帧学习到的赛道宽度（单侧丢线时判断"上次宽度"还可不可信）
// 返回参数     void
// 备注信息     1. 从车头那侧（row 大）往远端（row 小）逐行走，和参考代码"从种子行往上扫"同序：
//                 近处边界最可信，越远越不可靠，所以继承必须由近及远
//              2. 只丢一侧：用真实那一侧 + 上次有效宽度把另一侧顶出来（保宽度比独立外推稳）
//              3. 两侧都丢（或左右交叉 l >= r）：按最近两行真实边界的步进做斜率外推；
//                 上次宽度超过 学习宽度x1.6 说明那几行是横臂张开，不按宽度补，直接走斜率
//              4. 补出来的行写回 raw_left/raw_right 并置 fill_flag=1（加权打折）；
//                 补线前的真实边界先存进 real_left/real_right，路口特征打分只认这一份
//              5. 连续补够 MIDLINE_FILL_MAX_ROWS 行、或外推出画面/交叉就停手——
//                 再往上的行全是编造，宁可空着交给回中式输出
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Edge_Fill(float width_ref)
{
    int   est_l    = -1;                                // 当前推算的左边界
    int   est_r    = -1;
    int   real_l   = -1;                                // 最近一次真实左边界
    int   real_r   = -1;
    int   real_row = -1;                                // 最近一次真实边界所在行
    int   slope_l  = 0;                                 // 真实边界逐行步进（斜率外推用）
    int   slope_r  = 0;
    int   width    = 0;                                 // 最近一次真实宽度
    uint8 fill_run = 0;

    midline_info.fill_use = 0;

    for(int row = 0; row < MT9V03X_H; row ++)           // 真实边界快照（打分要用这一份）
    {
        real_left[row]  = midline_info.raw_left[row];
        real_right[row] = midline_info.raw_right[row];
        fill_flag[row]  = 0;
    }

#if (1 == MIDLINE_FILL_EN)
    for(int row = (MT9V03X_H - 3); row >= 2; row --)
    {
        int l = (MIDLINE_INVALID_COL == real_left[row])  ? (-1) : ((int)real_left[row]);
        int r = (MIDLINE_INVALID_COL == real_right[row]) ? (-1) : ((int)real_right[row]);

        /* 界外带：那一侧是被直角张开顶出去的，不是"丢线"。
         * 补线是给丢线用的，不许把界外补成一条边线——补了就等于凭空造出一条赛道边。 */
        if((1 == out_left[row]) || (1 == out_right[row]))
        {
            continue;
        }

        if((l >= 0) && (r >= 0) && (l < r))             // ---- 真实行：刷新参考 + 取步进 ----
        {
            if(real_row == (row + 1))                   // 相邻两行才取步进，跳行说明中间断过
            {
                slope_l = l - real_l;
                slope_r = r - real_r;
                if(slope_l >  MIDLINE_FILL_STEP_MAX) { slope_l =  MIDLINE_FILL_STEP_MAX; }
                if(slope_l < -MIDLINE_FILL_STEP_MAX) { slope_l = -MIDLINE_FILL_STEP_MAX; }
                if(slope_r >  MIDLINE_FILL_STEP_MAX) { slope_r =  MIDLINE_FILL_STEP_MAX; }
                if(slope_r < -MIDLINE_FILL_STEP_MAX) { slope_r = -MIDLINE_FILL_STEP_MAX; }
            }
            else
            {
                slope_l = 0;
                slope_r = 0;
            }
            real_l   = l;
            real_r   = r;
            real_row = row;
            width    = r - l;
            est_l    = l;
            est_r    = r;
            fill_run = 0;
            continue;
        }

        if(real_l < 0)                                  // 还没抓到真实边界：没有参考，空着
        {
            continue;
        }
        if(fill_run >= MIDLINE_FILL_MAX_ROWS)           // 补够了，再往上没意义
        {
            break;
        }

        int nl;
        int nr;

        if((l >= 0) && (r < 0))                         // ---- 只丢右侧 ----
        {
            nl = l;
            nr = l + width;
            if((float)width > (width_ref * MIDLINE_FILL_WIDTH_RATIO))
            {
                nr = est_r + slope_r;                   // 上次宽度不可信（横臂行）：退回斜率外推
            }
        }
        else if((r >= 0) && (l < 0))                    // ---- 只丢左侧 ----
        {
            nr = r;
            nl = r - width;
            if((float)width > (width_ref * MIDLINE_FILL_WIDTH_RATIO))
            {
                nl = est_l + slope_l;
            }
        }
        else                                            // ---- 两侧都丢 / 左右交叉 ----
        {
            nl = est_l + slope_l;
            nr = est_r + slope_r;
        }

        if((nl < MIDLINE_FILL_EDGE_MARGIN) ||
           (nr > (MT9V03X_W - 1 - MIDLINE_FILL_EDGE_MARGIN)) ||
           (nl >= nr))                                  // 推出画面或左右交叉：停手
        {
            break;
        }

        midline_info.raw_left[row]  = (uint8)nl;
        midline_info.raw_right[row] = (uint8)nr;
        fill_flag[row]              = 1;
        est_l   = nl;
        est_r   = nr;
        fill_run ++;
        if(midline_info.fill_use < 255)
        {
            midline_info.fill_use ++;
        }
    }
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取直角连线上某一行的列号（带合法性检查）
// 参数说明     p       一侧的直角对
//              row     行号
//              col     输出列号（浮点）
// 返回参数     uint8   1-合法  0-这条连线在这一行已经跑出画面 / 该侧不可用
// 备注信息     补线必须逐行验算：斜率是两个拐点定出来的，中间某侧完全可能越界
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Corner_Col(const midline_corner_struct *p, int row, float *col)
{
    float c;

    if(0 == p->valid)
    {
        return (0);
    }

    c = (p->k * (float)row) + p->b;
    if((c < (float)MIDLINE_EDGE_MARGIN) || (c > (float)(MT9V03X_W - 1 - MIDLINE_EDGE_MARGIN)))
    {
        return (0);
    }

    *col = c;
    return (1);
}

#if (1 == MIDLINE_CORNER_EN)
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     当前可补线的行区间（保持期往车头方向延伸，并给出基线的行偏移）
// 参数说明     row_hi      输出：靠车头的一端（图像下方，row 大）
//              row_lo      输出：远端（图像上方，row 小）
//              row_slide   输出：基线下滑量，查线时用 (row - row_slide)
// 返回参数     void
// 备注信息     保持期里"近端直角以下"那段是路口中央，本来就没有真实边线，
//              正好用两条连线的延长段补上；越出画面时 MIDLine_Corner_Col 会自己判无效
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Corner_Range(int *row_hi, int *row_lo, int *row_slide)
{
    int slide = ((int)corner_slide) * MIDLINE_CORNER_SLIDE_PX;

    *row_lo    = (int)corner_left.far_row;
    *row_hi    = (int)corner_left.near_row + slide;
    *row_slide = slide;

    if(1 == MIDLINE_CORNER_FILL_LOW_EN)
    {
        *row_hi = MT9V03X_H - 1;                        // 一路补到画面最下面（车头前面）
    }
    if(*row_hi > (MT9V03X_H - 1))
    {
        *row_hi = MT9V03X_H - 1;
    }
    if(*row_lo < 0)
    {
        *row_lo = 0;
    }
    return;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直角补线兜底：两侧基线全断时，直接用四个直角连出来的两条线自己算中线
// 参数说明     center      输出：加权中线值
// 返回参数     uint8       1-算出来了  0-补线不可用
// 备注信息     权重仍按"远端主导"，补线行再乘 MIDLINE_CORNER_WEIGHT 打折
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Corner_Center(float *center)
{
    uint32 sum_mid = 0;
    uint32 sum_w   = 0;
    uint8  num     = 0;
    int    row;
    int    row_hi  = 0;
    int    row_lo  = 0;
    int    row_slide = 0;

    if((0 == corner_ready) || (0 == corner_left.valid) || (0 == corner_right.valid))
    {
        return (0);
    }

    MIDLine_Corner_Range(&row_hi, &row_lo, &row_slide);

    for(row = row_hi; row >= row_lo; row --)
    {
        float  cl = 0.0f;
        float  cr = 0.0f;
        uint32 w;
        int    r_line = row - row_slide;                // 保持期基线已经往车头滑过去了

        if((0 == MIDLine_Corner_Col(&corner_left,  r_line, &cl)) ||
           (0 == MIDLine_Corner_Col(&corner_right, r_line, &cr)) || (cr <= cl))
        {
            continue;
        }
        w = (uint32)((float)midline_weight(row, 1) * MIDLINE_CORNER_WEIGHT);
        if(0u == w)
        {
            w = 1u;
        }
        sum_mid += (uint32)(((cl + cr) * 0.5f) * (float)w);
        sum_w   += w;
        num     ++;
    }

    if((num < MIDLINE_MIN_ROWS) || (0u == sum_w))
    {
        return (0);
    }

    *center = (float)sum_mid / (float)sum_w;
    return (1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把直角补线直接写进中线数组（写完全帧就能用，不用等下一帧）
// 参数说明     center      加权中线值
// 返回参数     void
// 备注信息     只在"两条基线全断"的兜底路径里用；正常路径由行循环逐行覆盖
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Corner_Fill(float center)
{
    int   row;
    int   row_hi    = 0;
    int   row_lo    = 0;
    int   row_slide = 0;
    uint8 num       = 0;

    MIDLine_Corner_Range(&row_hi, &row_lo, &row_slide);

    for(row = row_hi; row >= row_lo; row --)
    {
        float cl = 0.0f;
        float cr = 0.0f;
        int   r_line = row - row_slide;                 // 保持期基线往车头滑过去

        if((1 == MIDLine_Corner_Col(&corner_left,  r_line, &cl)) &&
           (1 == MIDLine_Corner_Col(&corner_right, r_line, &cr)) && (cr > cl))
        {
            midline_info.line[row] = (uint8)(((cl + cr) * 0.5f) + 0.5f);
            if(num < 255)
            {
                num ++;
            }
        }
    }

    if(0 == num)                                        // 一条都没补出来：不冒充有效帧
    {
        return;
    }

    midline_info.center         = center;
    midline_info.last_mid_line  = center;
    midline_info.mid_line_value = (uint8)(center + 0.5f);
    midline_info.error          = center - (MT9V03X_W * 0.5f);
    midline_info.num            = num;
    midline_info.valid          = 1;
    midline_info.corner_use     = num;
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直角对识别：近处两个直角 + 对面两个直角
// 参数说明     width_ref   学习到的赛道宽度（像素）
// 返回参数     void
// 备注信息     1. 数据源是 midline_info.raw_left/raw_right（逐行真实边界），不是拟合基线
//              2. 判据用"宽度突变"而不是"列跳变"：宽段（横臂）的两个端点就是两对拐点所在的行，
//                 拐点外侧那一行的左右边界列就是直角点 —— 比追列跳变抗噪得多
//              3. 四个直角要同时满足：左右同行、左右都外跳、宽度接近学习宽度、连线斜率合理、左右不交叉；
//                 任何一条不满足就退回纯特征打分（宁可不补，也不补错）
//              4. 看不到对面主路时（宽段一直顶到画面上沿），用近端主路的拟合斜率先把线延伸出去
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Corner_Detect_One(float width_ref)
{
    uint8  wide[MT9V03X_H];
    uint8  have[MT9V03X_H];
    int    run_near = -1;                               // 宽段最靠车头的行（row 最大）
    int    run_far  = -1;                               // 宽段最靠对面的一行（row 最小）
    int    near_row;
    int    far_row;
    int    hole = 0;
    int    i;
    uint8  ok_left;
    uint8  ok_right;
    float  near_w;
    float  far_w;
    int    jump_l;
    int    jump_r;

    corner_left.valid  = 0;
    corner_right.valid = 0;
    corner_state_now   = 0;
    midline_info.corner_use = 0;

    // ---- 1. 标出"横臂行"：两侧都有数据、且宽度明显超过学习宽度 ----
    for(i = 0; i < MT9V03X_H; i ++)
    {
        uint8 l = midline_info.raw_left[i];
        uint8 r = midline_info.raw_right[i];

        if((MIDLINE_INVALID_COL == l) || (MIDLINE_INVALID_COL == r) || (r <= l))
        {
            have[i] = 0;
            wide[i] = 0;
        }
        else
        {
            have[i] = 1;
            wide[i] = (((float)(r - l)) > (width_ref * MIDLINE_CORNER_WIDE_RATIO)) ? (1) : (0);
        }
    }

    // ---- 2. 从近往远找最靠车头的一段连续宽行（允许断 MIDLINE_CORNER_HOLE 行）----
    for(i = MT9V03X_H - 4; i >= 3; i --)
    {
        if(1 == wide[i])
        {
            if(run_near < 0)
            {
                run_near = i;
            }
            run_far = i;
            hole    = 0;
        }
        else if(run_near >= 0)
        {
            hole ++;
            if(hole > MIDLINE_CORNER_HOLE)
            {
                break;
            }
        }
    }

    if(run_near < 0)
    {
        return;                                         // 这段路面没有横臂
    }
    if(run_near > MIDLINE_CORNER_NEAR_BOT)
    {
        return;                                         // 横臂已经压到车头：近端拐点看不见了
    }
    if((run_near - run_far + 1) < MIDLINE_CORNER_MIN_LEN)
    {
        return;                                         // 宽段太短：是宽弯，不是路口
    }
    if((run_near - run_far + 1) > MIDLINE_CORNER_MAX_LEN)
    {
        return;                                         // 宽段太长：整幅画面发白，不是路口
    }

    // ---- 3. 近处两个直角 = 宽段近端外面那一行的左右边界 ----
    near_row = run_near + 1;
    if((near_row >= (MT9V03X_H - 1)) || (0 == have[near_row]))
    {
        return;
    }

    near_w = (float)midline_info.raw_right[near_row] - (float)midline_info.raw_left[near_row];
    if((near_w < (width_ref * MIDLINE_CORNER_WIDTH_MIN)) || (near_w > (width_ref * MIDLINE_CORNER_WIDTH_MAX)))
    {
        return;                                         // 拐点外侧不是正常宽度的主路：判据不成立
    }

    jump_l = (int)midline_info.raw_left[near_row]  - (int)midline_info.raw_left[run_near];
    jump_r = (int)midline_info.raw_right[run_near] - (int)midline_info.raw_right[near_row];
    if((jump_l < MIDLINE_CORNER_JUMP_MIN) || (jump_r < MIDLINE_CORNER_JUMP_MIN))
    {
        return;                                         // 只外跳一侧 = 单边丢线 / 缓弯，不是两个直角
    }

    // ---- 4. 对面两个直角 = 宽段远端外面那一行的左右边界 ----
    far_row = run_far - 1;
    if((far_row >= MIDLINE_CORNER_FAR_TOP) && (1 == have[far_row]))
    {
        far_w = (float)midline_info.raw_right[far_row] - (float)midline_info.raw_left[far_row];
        if((far_w < (width_ref * MIDLINE_CORNER_WIDTH_MIN)) || (far_w > (width_ref * MIDLINE_CORNER_WIDTH_MAX)))
        {
            far_row = -1;                               // 对面那一行不是主路：按看不见处理
        }
    }
    else
    {
        far_row = -1;                                   // 对面主路被截断
    }

    // ---- 5. 连线：左直角连左直角、右直角连右直角，绝不交叉 ----
    corner_left.near_row  = (uint8)near_row;
    corner_left.near_col  = midline_info.raw_left[near_row];
    corner_right.near_row = (uint8)near_row;
    corner_right.near_col = midline_info.raw_right[near_row];

    if(far_row >= 0)                                    // 对面两个直角都对上：两点定一条线
    {
        float den = (float)(far_row - near_row);        // 负数（far_row < near_row）

        corner_left.far_row  = (uint8)far_row;
        corner_left.far_col  = midline_info.raw_left[far_row];
        corner_right.far_row = (uint8)far_row;
        corner_right.far_col = midline_info.raw_right[far_row];

        corner_left.k  = ((float)corner_left.far_col  - (float)corner_left.near_col)  / den;
        corner_left.b  = (float)corner_left.near_col  - (corner_left.k  * (float)near_row);
        corner_right.k = ((float)corner_right.far_col - (float)corner_right.near_col) / den;
        corner_right.b = (float)corner_right.near_col - (corner_right.k * (float)near_row);
    }
    else
    {
#if (1 == MIDLINE_CORNER_FALLBACK_EN)
        corner_left.far_row  = (uint8)MIDLINE_CORNER_FAR_TOP;
        corner_right.far_row = (uint8)MIDLINE_CORNER_FAR_TOP;
        corner_left.k  = (1 == bedline_left.valid)  ? (bedline_left.k)  : (0.0f);
        corner_right.k = (1 == bedline_right.valid) ? (bedline_right.k) : (0.0f);
        corner_left.b  = (float)corner_left.near_col  - (corner_left.k  * (float)near_row);
        corner_right.b = (float)corner_right.near_col - (corner_right.k * (float)near_row);
#else
        return;
#endif
    }

    if((corner_left.k > MIDLINE_CORNER_K_MAX) || (corner_left.k < -MIDLINE_CORNER_K_MAX) ||
       (corner_right.k > MIDLINE_CORNER_K_MAX) || (corner_right.k < -MIDLINE_CORNER_K_MAX))
    {
        return;                                         // 连线太斜：补出来一定是垃圾
    }

    // ---- 6. 逐行验算：区间内每一行左右都在画面里，而且左列 < 右列 ----
    corner_left.valid  = 1;
    corner_right.valid = 1;
    ok_left  = 1;
    ok_right = 1;
    for(i = (int)corner_left.far_row; i <= (int)corner_left.near_row; i ++)
    {
        float cl = 0.0f;
        float cr = 0.0f;

        if((1 == ok_left) && (0 == MIDLine_Corner_Col(&corner_left, i, &cl)))
        {
            ok_left = 0;
        }
        if((1 == ok_right) && (0 == MIDLine_Corner_Col(&corner_right, i, &cr)))
        {
            ok_right = 0;
        }
        if((1 == ok_left) && (1 == ok_right) && (cr <= cl))
        {
            ok_left  = 0;                               // 左右交叉 = 补线不成立
            ok_right = 0;
        }
    }
    if((0 == ok_left) || (0 == ok_right))
    {
        corner_left.valid  = 0;
        corner_right.valid = 0;
        return;                                         // 只连上一侧不算"直角对"：宁可不补，也不半补
    }
    corner_left.valid  = 1;
    corner_right.valid = 1;

    // ---- 7. 调试信息 ----
    corner_state_now = (far_row >= 0) ? (2) : (1);
    midline_info.corner_state      = corner_state_now;
    midline_info.corner_near_row   = (uint8)near_row;
    midline_info.corner_far_row    = corner_left.far_row;
    midline_info.corner_near_left  = corner_left.near_col;
    midline_info.corner_near_right = corner_right.near_col;
    midline_info.corner_far_left   = (far_row >= 0) ? (midline_info.raw_left[far_row])  : (MIDLINE_INVALID_COL);
    midline_info.corner_far_right  = (far_row >= 0) ? (midline_info.raw_right[far_row]) : (MIDLINE_INVALID_COL);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直角对识别（带"上一帧基线备份"的外壳）
// 参数说明     width_ref   学习到的赛道宽度（像素）
// 返回参数     void
// 备注信息     Detect_One 一进来就把 corner_left/right 的 valid 清 0，失败时会把上一帧的基线也冲掉；
//              但"保持 0.5s"恰恰要求失败帧继续用上一帧的连线，所以这里备份-失败-还原
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Corner_Detect(float width_ref)
{
    midline_corner_struct bak_l = corner_left;
    midline_corner_struct bak_r = corner_right;

    MIDLine_Corner_Detect_One(width_ref);

    if(0 == corner_state_now)                           // 这一帧没看到拐角：上一帧的基线留着给保持窗口用
    {
        corner_left  = bak_l;
        corner_right = bak_r;
    }
    return;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     弯道判定（滞回）：远端中线 - 近端中线 的横向差
// 参数说明     void
// 返回参数     uint8   1-弯道（权重重心要上移）  0-直道
// 备注信息     1. 只统计两侧边界都有效的行；任一段有效行不足 4 行就按直道处理
//              2. 路口内强制按直道（路口已经单独处理，别和弯道抢权重）
//              3. 距离 > MIDLINE_TURN_DIFF 判弯，回落到 MIDLINE_TURN_DIFF_EXIT 以下才退出
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Turn_Detect(void)
{
    float far_sum  = 0.0f;
    float near_sum = 0.0f;
    uint8 far_n    = 0;
    uint8 near_n   = 0;
    float d;

    int far_top  = 4;
    int far_bot  = far_top + MIDLINE_TURN_SEG - 1;
    int near_bot = MT9V03X_H - 10;
    int near_top = near_bot - MIDLINE_TURN_SEG + 1;

    if(MIDLINE_CROSS_ST_INSIDE == cross_state)          // 路口内不抢权重
    {
        turn_flag = 0;
        return (0);
    }

    for(int row = far_top; row <= far_bot; row ++)
    {
        uint8 l = midline_info.raw_left[row];
        uint8 r = midline_info.raw_right[row];
        if((l != MIDLINE_INVALID_COL) && (r != MIDLINE_INVALID_COL) && (r > l))
        {
            far_sum += ((float)l + (float)r) * 0.5f;
            far_n ++;
        }
    }
    for(int row = near_top; row <= near_bot; row ++)
    {
        uint8 l = midline_info.raw_left[row];
        uint8 r = midline_info.raw_right[row];
        if((l != MIDLINE_INVALID_COL) && (r != MIDLINE_INVALID_COL) && (r > l))
        {
            near_sum += ((float)l + (float)r) * 0.5f;
            near_n ++;
        }
    }

    if((far_n < 4) || (near_n < 4))
    {
        turn_flag = 0;
        return (0);
    }

    d = (far_sum / (float)far_n) - (near_sum / (float)near_n);
    if(d < 0.0f)
    {
        d = -d;
    }

    if(0 == turn_flag)
    {
        if(d > MIDLINE_TURN_DIFF)
        {
            turn_flag = 1;
        }
    }
    else
    {
        if(d < MIDLINE_TURN_DIFF_EXIT)
        {
            turn_flag = 0;
        }
    }
    return (turn_flag);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     十字路口五特征打分
// 参数说明     width_ref   学习到的赛道宽度（像素）
//              dev_now     本帧测量到的中线偏差（像素，正=偏右）
// 返回参数     uint8       0~4 分（>= MIDLINE_CROSS_SCORE_NEED 视为路口）
// 备注信息     F1 宽行 / F2 丢线带 / F3 双边界外扩 / F4 断带上方仍有线（F2 的闸门）/ F5 短黑条
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Cross_Score(float width_ref, float dev_now)
{
    int   wide_cnt  = 0;
    int   rise_cnt  = 0;
    int   above_cnt = 0;
    int   gap_run   = 0;
    int   gap_max   = 0;
    int   gap_top   = -1;
    uint8 score     = 0;
    float wide_lim  = width_ref * MIDLINE_CROSS_WIDTH_RATIO;
    float dev       = (dev_now < 0.0f) ? (-dev_now) : (dev_now);

    // ---- F1 宽行 + F2 丢线带 ----
    for(int row = 2; row <= (MT9V03X_H - 3); row ++)
    {
        uint8 l  = real_left[row];                      // 只认补线前的真实边界
        uint8 r  = real_right[row];
        uint8 ok = (uint8)((l != MIDLINE_INVALID_COL) && (r != MIDLINE_INVALID_COL) && (r > l));

        if(ok)
        {
            if((float)(r - l) > wide_lim)
            {
                wide_cnt ++;
            }
            gap_run = 0;
        }
        else
        {
            gap_run ++;
            if(gap_run > gap_max)
            {
                gap_max = gap_run;
                gap_top = row - gap_run + 1;
            }
        }
    }

    // ---- F4 断带上方（更远处）还有没有线：有 = 路口，没有 = 出弯丢线 ----
    if(gap_max >= MIDLINE_CROSS_GAP_MIN)
    {
        for(int row = 2; row < gap_top; row ++)
        {
            uint8 l = real_left[row];
            uint8 r = real_right[row];
            if((l != MIDLINE_INVALID_COL) && (r != MIDLINE_INVALID_COL) && (r > l))
            {
                above_cnt ++;
            }
        }
    }
    else
    {
        above_cnt = 99;                                 // 没有断带：F4 不参与闸门
    }

    // ---- F3 相邻两行双边界同时向外跳（横向路面突然张开） ----
    for(int row = (MT9V03X_H - 2); row > 2; row --)
    {
        uint8 l0 = real_left[row];
        uint8 r0 = real_right[row];
        uint8 l1 = real_left[row - 1];
        uint8 r1 = real_right[row - 1];

        if((l0 == MIDLINE_INVALID_COL) || (r0 == MIDLINE_INVALID_COL) ||
           (l1 == MIDLINE_INVALID_COL) || (r1 == MIDLINE_INVALID_COL))
        {
            continue;
        }
        if((l0 + MIDLINE_CROSS_RISE_D <= l1) &&      // 左边界继续往左
           (r0 >= r1 + MIDLINE_CROSS_RISE_D))        // 右边界继续往右
        {
            rise_cnt ++;
        }
    }

    // ---- 打分 ----
    if(wide_cnt >= MIDLINE_CROSS_MIN_WIDE_ROWS)
    {
        score ++;
    }
    if((gap_max >= MIDLINE_CROSS_GAP_MIN) && (above_cnt >= MIDLINE_CROSS_ABOVE_MIN))
    {
        score ++;
    }
    if(rise_cnt >= MIDLINE_CROSS_RISE_MIN)
    {
        score ++;
    }
#if (1 == MIDLINE_CROSS_MARK_EN)
    if((cross_mark_rows >= MIDLINE_CROSS_MARK_MIN_ROWS) &&
       (cross_mark_rows <= MIDLINE_CROSS_MARK_MAX_ROWS))
    {
        score ++;
    }
#endif

    if(dev > MIDLINE_CROSS_TOL)                         // 中线已经偏出中心：是弯道/斜入，不是正对路口
    {
        score = 0;
    }

    midline_info.cross_feat[0] = (uint8)((wide_cnt  > 255) ? (255) : (wide_cnt));
    midline_info.cross_feat[1] = (uint8)((gap_max   > 255) ? (255) : (gap_max));
    midline_info.cross_feat[2] = (uint8)((rise_cnt  > 255) ? (255) : (rise_cnt));
    midline_info.cross_feat[3] = (uint8)((above_cnt > 255) ? (255) : (above_cnt));
    midline_info.cross_feat[4] = cross_mark_rows;

    return (score);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路口状态机推进（每帧必须调用一次，丢线帧也要推进，否则出不了路口）
// 参数说明     score       本帧打分
// 返回参数     void
// 备注信息     NONE -> CONFIRM -> INSIDE -> EXIT -> COOLDOWN -> NONE
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Cross_Tick(uint8 score)
{
    if(score >= MIDLINE_CROSS_SCORE_NEED)
    {
        cross_clean = 0;
        if(cross_hit < 255)
        {
            cross_hit ++;
        }
    }
    else
    {
        cross_hit = 0;
        if(cross_clean < 255)
        {
            cross_clean ++;
        }
    }

    switch(cross_state)
    {
        case MIDLINE_CROSS_ST_NONE:
            if(cross_hit >= 1)
            {
                cross_state = MIDLINE_CROSS_ST_CONFIRM;
            }
            break;

        case MIDLINE_CROSS_ST_CONFIRM:
            if(cross_hit >= MIDLINE_CROSS_CONFIRM)      // 连续命中：认账，切到路口策略
            {
                cross_state       = MIDLINE_CROSS_ST_INSIDE;
                cross_ticks       = MIDLINE_CROSS_KEEP;
                cross_hold_center = midline_info.last_mid_line;
                cross_hold_snap   = midline_info.last_mid_line;   // 补线平移基准
                for(uint16 i = 0; i < MT9V03X_H; i ++)
                {
                    cross_hold_line[i] = midline_info.line[i];
                }
            }
            else if(0 == cross_hit)
            {
                cross_state = MIDLINE_CROSS_ST_NONE;    // 只是抖了一下
            }
            break;

        case MIDLINE_CROSS_ST_INSIDE:
            if(cross_ticks > 0)
            {
                cross_ticks --;
            }
            if((0 == cross_ticks) && (cross_clean >= MIDLINE_CROSS_CLEAN))
            {
                cross_state = MIDLINE_CROSS_ST_EXIT;    // 出路口：冻结宽度学习
                cross_ticks = MIDLINE_CROSS_CLEAN;
#if (1 == MIDLINE_CORNER_EN)
                /* 出路口立刻放掉直角基线：路口已经过去，再挂着补线会把出路口的
                 * 第一帧中线按"路口几何"算歪，而那一帧正是控制环最敏感的时刻。 */
                corner_keep        = 0;
                corner_ready       = 0;
                corner_slide       = 0;
#endif
            }
            break;

        case MIDLINE_CROSS_ST_EXIT:
            if(cross_ticks > 0)
            {
                cross_ticks --;
            }
            if(0 == cross_ticks)
            {
                cross_state = MIDLINE_CROSS_ST_COOLDOWN;
                cross_ticks = MIDLINE_CROSS_COOLDOWN;
            }
            break;

        case MIDLINE_CROSS_ST_COOLDOWN:
            if(cross_ticks > 0)
            {
                cross_ticks --;
            }
            if(0 == cross_ticks)
            {
                cross_state = MIDLINE_CROSS_ST_NONE;    // 冷却结束，重新捕获
            }
            break;

        default:
            cross_state = MIDLINE_CROSS_ST_NONE;
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     路口内输出（有可信行就跟着走，没有就逐帧回中）
// 参数说明     center_now  本帧由"远端主导"算出的中线（has_line = 1 时有效）
//              has_line    1-本帧还有可信行  0-画面里只剩横臂的白色开口
// 返回参数     void
// 备注信息     1. 老做法是"冻结进路口前的偏差"，等于一直按同一个角度打舵，车必然沿一段圆弧拐走；
//              2. 现在改成：有可信行就跟着它（车头会自己回正），一行可信的都没有就让偏差指数回中，
//                 穿过路口走的是直线而不是圆弧；
//              3. 中线快照按当前中线整体平移后补齐，屏显/差速看到的是连续的一条中线。
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Cross_Output(float center_now, uint8 has_line)
{
    float target;

    if(has_line)
    {
        /* 路口内只给"进路口那一刻的中线"很小的修正权限：
         * 远端行里混进来的横臂开口、对面路口的错位、直角补线的斜率误差，
         * 一旦把中线拉出 ±MIDLINE_CROSS_TRUST_PX 就当噪声丢掉。
         * 这是"部分路口把车带弯"的直接对策：路口里航向基本不变，只做小幅修正。 */
        float dev = center_now - cross_hold_snap;

        if(dev > MIDLINE_CROSS_TRUST_PX)
        {
            dev = MIDLINE_CROSS_TRUST_PX;
        }
        if(dev < -MIDLINE_CROSS_TRUST_PX)
        {
            dev = -MIDLINE_CROSS_TRUST_PX;
        }
        target = cross_hold_snap + dev;
    }
    else
    {
        target = cross_hold_center +
                 (((float)MT9V03X_W * 0.5f) - cross_hold_center) * MIDLINE_CROSS_DECAY;
    }

    cross_hold_center = target;

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        if(midline_info.line[i] == MIDLINE_INVALID_COL)
        {
            if(cross_hold_line[i] != MIDLINE_INVALID_COL)
            {
                float v = ((float)cross_hold_line[i]) + (target - cross_hold_snap);
                if(v < 0.0f)
                {
                    v = 0.0f;
                }
                if(v > (float)(MT9V03X_W - 1))
                {
                    v = (float)(MT9V03X_W - 1);
                }
                midline_info.line[i] = (uint8)(v + 0.5f);
            }
            else
            {
                midline_info.line[i] = (uint8)(target + 0.5f);
            }
        }
    }

    midline_info.center         = target;
    midline_info.last_mid_line  = target;
    midline_info.mid_line       = (uint8)(target + 0.5f);
    midline_info.mid_line_value = (uint8)(target + 0.5f);
    midline_info.error          = target - ((float)MT9V03X_W * 0.5f);
    midline_info.conf           = has_line ? (60) : (20);
    midline_info.valid          = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完全丢线时的输出（偏差指数回中，绝不保持最后的偏差）
// 参数说明     void
// 返回参数     void
// 备注信息     1. 一帧都看不见赛道时没有任何横向信息，"保持最后一个偏差"等于闭着眼睛按固定角打舵；
//              2. 这里让偏差每帧向画面中心收一点，车头自己回正，重新看见线时不会带着大偏差冲出去。
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Lost_Output(void)
{
    cross_hold_center = cross_hold_center +
                        (((float)MT9V03X_W * 0.5f) - cross_hold_center) * MIDLINE_LOST_DECAY;

    midline_info.center         = cross_hold_center;
    midline_info.last_mid_line  = cross_hold_center;
    midline_info.mid_line       = (uint8)(cross_hold_center + 0.5f);
    midline_info.mid_line_value = (uint8)(cross_hold_center + 0.5f);
    midline_info.error          = cross_hold_center - ((float)MT9V03X_W * 0.5f);
    midline_info.conf           = 0;
    midline_info.valid          = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     赛道比视野还宽（边线全在画面外）时的软兜底输出
// 参数说明     void
// 返回参数     void
// 备注信息     1. 赛道宽度大于视野时，左右边界都在画面之外，Badpoint 取不到种子，两条基线同时失效；
//                这种画面不是"丢线"，而是"看得见的全是赛道"，此时按 MIDLINE_LOST_DECAY 回中等于撒手不管；
//              2. 这里沿上一帧中线、只允许以 MIDLINE_BLIND_DECAY 极慢地往画面中心收，
//                车按原方向滑过去，重新看见边界时自然接回正常拟合；
//              3. conf 给 MIDLINE_BLIND_CONF 低值，控制层按置信度自动降速；
//              4. 连续 MIDLINE_BLIND_MAX 帧（0.8s）仍不见边界，才真正判丢线。
//-------------------------------------------------------------------------------------------------------------------
static void MIDLine_Soft_Output(void)
{
    float c = midline_info.last_mid_line;

    if((c < 0.0f) || (c > (float)(MT9V03X_W - 1)))
    {
        c = (float)MIDLINE_DEFAULT_CENTER;
    }

    c = c + (((float)MT9V03X_W * 0.5f) - c) * (1.0f - MIDLINE_BLIND_DECAY);

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i] = (uint8)(c + 0.5f);       // 软兜底：整幅画一条竖直中线，屏上能直接看到
    }

    midline_info.center         = c;
    midline_info.last_mid_line  = c;
    midline_info.mid_line       = (uint8)(c + 0.5f);
    midline_info.mid_line_value = (uint8)(c + 0.5f);
    midline_info.error          = c - ((float)MT9V03X_W * 0.5f);
    midline_info.conf           = MIDLINE_BLIND_CONF;
    midline_info.valid          = 1;
    midline_info.row_start      = (uint8)(MT9V03X_H - 1);   // 软兜底是整幅竖直线，屏上覆盖全高
    midline_info.row_end        = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空中线信息，并按行生成权重增益表
// 参数说明     void
// 返回参数     void
// 使用示例     MIDLine_Init();
// 备注信息     1. 权重本身由 midline_weight() 按行按弯道实时计算，这里是"增益表"，默认 100（不加不减）
//              2. 想手工干预某一行直接改 mid_weight_list[]，例如
//                 mid_weight_list[0] = 0;     // 最远一行不参与
//                 mid_weight_list[110] = 50;  // 车头附近的脏数据减半
//-------------------------------------------------------------------------------------------------------------------
void MIDLine_Init(void)
{
    memset(&midline_info, 0, sizeof(midline_info));

    midline_info.mid_line       = MIDLINE_DEFAULT_CENTER;
    midline_info.mid_line_value = MIDLINE_DEFAULT_CENTER;
    midline_info.last_mid_line  = (float)MIDLINE_DEFAULT_CENTER;
    midline_info.center         = (float)MIDLINE_DEFAULT_CENTER;
    midline_info.conf           = 100;

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i]      = MIDLINE_INVALID_COL;
        midline_info.raw_left[i]  = MIDLINE_INVALID_COL;
        midline_info.raw_right[i] = MIDLINE_INVALID_COL;
        mid_weight_list[i]        = 100;                // 权重增益百分比
        cross_hold_line[i]        = MIDLINE_INVALID_COL;
    }

    cross_state       = MIDLINE_CROSS_ST_NONE;
    cross_ticks       = 0;
    cross_hit         = 0;
    cross_clean       = 0;
    cross_hint        = 0;
    cross_open_cnt    = 0;
    cross_mark_rows   = 0;
    turn_flag         = 0;
    cross_hold_center = (float)MIDLINE_DEFAULT_CENTER;
    cross_hold_snap   = (float)MIDLINE_DEFAULT_CENTER;

    /* ---- 直角弯界外状态 ---- */
    outside_cnt = 0;
    right_angle = 0;
    blind_cnt   = 0;
    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        out_left[i]  = 0;
        out_right[i] = 0;
    }

    /* ---- 中线历史环形缓冲（基点行丢线时取三帧前的成品） ---- */
    memset(hist_buf, 0, sizeof(hist_buf));
    hist_head = 0;
    hist_cnt  = 0;
    hist_used = 0;

    /* ---- 直角对状态 ---- */
    corner_left.valid  = 0;    corner_right.valid = 0;
    corner_state_now   = 0;
    corner_ready       = 0;
    corner_hit         = 0;
    corner_keep        = 0;
    corner_slide       = 0;
    corner_run         = 0;
    corner_far_ok      = 0;
    midline_info.corner_state      = 0;
    midline_info.corner_use        = 0;
    midline_info.corner_near_row   = MIDLINE_INVALID_COL;
    midline_info.corner_far_row    = MIDLINE_INVALID_COL;
    midline_info.corner_near_left  = MIDLINE_INVALID_COL;
    midline_info.corner_near_right = MIDLINE_INVALID_COL;
    midline_info.corner_far_left   = MIDLINE_INVALID_COL;
    midline_info.corner_far_right  = MIDLINE_INVALID_COL;

    /* ---- 逐行继承补线状态 ---- */
    midline_info.fill_use = 0;
    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        fill_flag[i]  = 0;
        real_left[i]  = MIDLINE_INVALID_COL;
        real_right[i] = MIDLINE_INVALID_COL;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     不依赖 Bedline 的"裸边线"中线：只用逐行扫描出来的 raw_left / raw_right
// 参数说明     width_ref       学习到的赛道宽度（px）
// 参数说明     center          输出：加权中线列
// 参数说明     row_hi / row_lo 输出：参与计算的行区间
// 返回参数     uint8           1-算出来了  0-没有足够的成对边线
// 备注信息     1. 赛道比视野还宽时 Badpoint 取不到种子，两条基线失效，但逐行扫描照样能扫到
//                "中间几行有边、最底下几行全白"的边界，这部分信息不该被浪费；
//              2. 只认宽度落在 学习宽度 x[WIDTH_MIN, WIDTH_MAX] 的行，横臂的超宽行、界外行一律不算；
//              3. 权重沿用 midline_weight()，和其他路径保持同一套远近原则。
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Raw_Center(float width_ref, float *center, int *row_hi, int *row_lo)
{
    float sum_w = 0.0f;
    float sum_c = 0.0f;
    uint16 num  = 0;
    int   hi    = -1;
    int   lo    = -1;

    for(int row = (MT9V03X_H - 2); row >= 2; row --)
    {
        int   l   = (int)midline_info.raw_left[row];
        int   r   = (int)midline_info.raw_right[row];
        float w_row;

        if((l < 0) || (r < 0) || (l >= r))
        {
            continue;
        }
        if((1 == out_left[row]) || (1 == out_right[row]))
        {
            continue;                                   // 界外行（直角张开那一侧）不参与
        }

        w_row = (float)(r - l);
        if((w_row < (width_ref * MIDLINE_CORNER_WIDTH_MIN)) || (w_row > (width_ref * MIDLINE_CORNER_WIDTH_MAX)))
        {
            continue;                                   // 太窄=噪声，太宽=横臂/赛道占满，都不是我们的边线
        }

        {
            uint32 w = midline_weight(row, midline_info.turn);

            if(0 == w)
            {
                w = 1;
            }
            sum_c += (((float)l + (float)r) * 0.5f) * (float)w;
            sum_w += (float)w;
            num   ++;
            if(hi < 0) { hi = row; }
            lo = row;
        }
    }

    if((num < MIDLINE_MIN_ROWS) || (sum_w < 1.0f))
    {
        return (0);
    }

    *center = sum_c / sum_w;
    *row_hi = hi;
    *row_lo = lo;

    return (1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用裸边线直接输出一帧中线（Bedline 两条基线都失效时的第一顺位兜底）
// 参数说明     width_ref       学习到的赛道宽度（px）
// 返回参数     uint8           1-已输出  0-凑不出成对边线
// 备注信息     1. 内部调用 MIDLine_Raw_Center()，输出同样过 MIDLINE_MAX_JUMP_PX 跨帧限幅；
//              2. conf 给 MIDLINE_RAW_CONF，控制层据此降速；
//              3. 行区间写回 r_hi / r_lo，屏上绿线只覆盖真正参与计算的那几行。
//-------------------------------------------------------------------------------------------------------------------
static uint8 MIDLine_Raw_Output(float width_ref)
{
    float cc   = 0.0f;
    int   r_hi = 0;
    int   r_lo = 0;

    if(0 == MIDLine_Raw_Center(width_ref, &cc, &r_hi, &r_lo))
    {
        return (0);
    }

    {
        float anchor = midline_info.last_mid_line;       // 跨帧限幅：裸边线也不许把车头抽一下
        float dv     = cc - anchor;

        if(dv > MIDLINE_MAX_JUMP_PX)
        {
            dv = MIDLINE_MAX_JUMP_PX;
        }
        if(dv < -MIDLINE_MAX_JUMP_PX)
        {
            dv = -MIDLINE_MAX_JUMP_PX;
        }
        cc = anchor + dv;
    }

    for(int i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i] = (uint8)(cc + 0.5f);
    }

    midline_info.center         = cc;
    midline_info.last_mid_line  = cc;
    midline_info.mid_line       = (uint8)(cc + 0.5f);
    midline_info.mid_line_value = (uint8)(cc + 0.5f);
    midline_info.error          = cc - ((float)MT9V03X_W * 0.5f);
    midline_info.conf           = MIDLINE_RAW_CONF;
    midline_info.valid          = 1;
    midline_info.row_start      = (uint8)r_hi;
    midline_info.row_end        = (uint8)r_lo;
    blind_cnt = 0;
    MIDLine_Cross_Tick(0);
    midline_info.cross       = 0;
    midline_info.cross_state = cross_state;
    midline_info.cross_score = 0;

    return (1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逐行扫描 + 加权求中线 + 互补滤波 + 十字路口判定
// 参数说明     void
// 返回参数     uint8           0-成功  1-失败（边线无效或有效行数不足）
// 使用示例     Find_MIDLine();
// 备注信息     1. 依赖 Bedline_REMAIN 的 bedline_left / bedline_right，必须先调用 Find_Bedline()
//              2. 行区间取左右两条基线的公共部分；只有一侧有效时用那一侧的行区间
//              3. 单侧推算用的赛道宽度取上一帧的加权宽度，首帧用 MIDLINE_DEFAULT_WIDTH
//              4. 结果看 midline_info.mid_line_value（滤波后）、error、conf、cross
//-------------------------------------------------------------------------------------------------------------------
uint8 Find_MIDLine(void)
{
    float width_ref = (midline_info.width > 1.0f) ? (midline_info.width) : (MIDLINE_DEFAULT_WIDTH);
    int   row_start = -1;
    int   row_end   = -1;
    uint8 score;

    /* ---- 上一帧的路口提示：必须在算中线之前生效 ----
     * 打分要等中线算完才出来，等它出来，这一帧的中线已经被横臂拉偏、舵机已经打错角。
     * 所以用上一帧的状态/打分提前切到路口策略，时序上早一帧，等价于零延迟。 */
    cross_hint = (uint8)(((cross_state != MIDLINE_CROSS_ST_NONE) &&
                          (cross_state != MIDLINE_CROSS_ST_COOLDOWN)) ? (1) : (0));
    if(midline_info.cross_score >= MIDLINE_CROSS_HINT_SCORE)
    {
        cross_hint = 1;
    }

    /* ---- 0. 逐行真实边界扫描（用上一帧的中线做种子） + 弯道判定 ---- */
#if (1 == MIDLINE_RAW_EN)
    MIDLine_Row_Scan();
#endif
    midline_info.turn = MIDLine_Turn_Detect();

    /* ---- 0.5 直角对识别：看到近处两个直角，马上去找对面两个左右直角，连成基线 ----
     * 这一步必须在 Row_Scan 之后、行循环之前：补线要在本帧的行计算里就用上（零延迟）。 */
#if (1 == MIDLINE_CORNER_EN)
    uint8 confirm_need;
    MIDLine_Corner_Detect(width_ref);
    if(corner_state_now >= 1)                           // 近处两个直角看到了（对面可有可无）
    {
        if(corner_hit < 255)
        {
            corner_hit ++;
        }
        if(corner_state_now >= 2)
        {
            corner_far_ok = 1;                          // 这一轮连续段里见过对面两个直角
        }
        corner_keep  = MIDLINE_CORNER_KEEP;             // 基线保持 0.5s：还在看就一直续
        corner_slide = 0;                               // 拐角是本帧的，基线不用往下滑
    }
    else
    {
        corner_hit    = 0;                              // 连续段断了，下次要重新确认
        corner_far_ok = 0;
        if(corner_keep > 0)
        {
            corner_keep --;                             // 看不到拐角：开始 0.5s 倒计时
        }
        if(corner_slide < 255)
        {
            corner_slide ++;                            // 基线跟着车往车头方向滑
        }
        if(0 == corner_keep)
        {
            corner_left.valid  = 0;
            corner_right.valid = 0;
            corner_slide = 0;
            corner_run   = 0;
            midline_info.corner_state = 0;
        }
    }

    /* 确认门槛：四个直角同时对上 2 帧就够（硬证据）；只有近处两个直角要 5 帧（对面被挡，几何靠外推） */
    confirm_need = (1 == corner_far_ok) ? (MIDLINE_CORNER_CONFIRM) : (MIDLINE_CORNER_CONFIRM_NEAR);
    corner_ready = (uint8)(((corner_hit >= confirm_need) && (corner_keep > 0) &&
                            (0 != corner_left.valid) && (0 != corner_right.valid)) ? (1) : (0));

    if(1 == corner_ready)
    {
        if(corner_run < 255)
        {
            corner_run ++;
        }
        if(corner_run > MIDLINE_CORNER_RUN_MAX)          // 挂太久了（车可能卡在路口里）：退回正常巡线
        {
            corner_ready  = 0;
            corner_keep   = 0;
            corner_hit    = 0;
            corner_far_ok = 0;
            corner_slide  = 0;
            corner_run    = 0;
            corner_left.valid  = 0;
            corner_right.valid = 0;
            midline_info.corner_state = 0;
        }
        else
        {
            cross_hint = 1;                             // 直角对成立：本帧立刻按路口策略走
        }
    }
    else
    {
        corner_run = 0;
    }
#endif

    /* ---- 0.6 逐行继承补线：把丢线的行（边界丢失 / 左右交叉）用下面的近处边界顶上去 ----
     * 必须在直角对识别之后调：直角对要靠"真断带"找拐点，先补就找不到断了；
     * 必须在行循环之前调：这一帧的加权就要用补好的边界。 */
    MIDLine_Edge_Fill(width_ref);

    /* 直角弯/单侧张开：界外行数够多，说明这不是"两边都张开的十字路口"，而是单侧的直角弯。
     * 这种情况远端根本没有我们的赛道（那边是界外），权重必须压回近处，
     * 否则路口策略的"远端主导"会让车提前按拐角打方向 —— 这就是"直角弯不要转向"。 */
    right_angle = (((outside_cnt >= MIDLINE_OUTSIDE_MIN_ROWS) && (1 == cross_hint)) ?
                   ((uint8)1) : ((uint8)0));

#if (1 == MIDLINE_HIST_EN)
    /* 先把"上一帧算完的成品"压进历史环形缓冲 —— 此刻 midline_info 里还是上一帧的结果，
     * 必须放在下面的 valid=0 / line[] 清零之前，否则压进去的全是无效帧。
     * 这一帧若碰上"基点行本身就丢线"，兜底时就取三帧前的成品，而不是沿用上一帧。 */
    MIDLine_Hist_Push();
#endif

    midline_info.valid      = 0;
    midline_info.num        = 0;
    midline_info.weight_sum = 0;

    for(uint16 i = 0; i < MT9V03X_H; i ++)
    {
        midline_info.line[i] = MIDLINE_INVALID_COL;     // mid_line_list 全部置无效
    }

    if(!bedline_left.valid && !bedline_right.valid)     // 两侧边线都无效，无法求中线
    {
#if (1 == MIDLINE_CORNER_EN)
        /* 车正压在路口中央时典型就是这个状态：横臂把两条基线全带走。
         * 这时候四个直角已经是唯一还连着的信息，直接拿连出来的两条线当中线用。 */
        {
            float cc = 0.0f;

            if(1 == MIDLine_Corner_Center(&cc))
            {
                /* 和 MIDLine_Cross_Output 用同一道闸：不管这条补出来的中线多漂亮，
                 * 相对"上一帧的中线 / 进路口那一刻的中线"都不许跳出去。
                 * 否则四个直角只要有一个点歪，中线就直接被拽到横臂方向。 */
                float anchor = (MIDLINE_CROSS_ST_INSIDE == cross_state) ?
                               (cross_hold_snap) : (midline_info.last_mid_line);
                float dv     = cc - anchor;

                if(dv > MIDLINE_CROSS_TRUST_PX)
                {
                    dv = MIDLINE_CROSS_TRUST_PX;
                }
                if(dv < -MIDLINE_CROSS_TRUST_PX)
                {
                    dv = -MIDLINE_CROSS_TRUST_PX;
                }
                cc = anchor + dv;

                MIDLine_Corner_Fill(cc);
                MIDLine_Cross_Tick(MIDLINE_CROSS_SCORE_NEED);
                midline_info.cross       = 1;
                midline_info.cross_state = cross_state;
                midline_info.cross_score = MIDLINE_CROSS_SCORE_NEED;
                blind_cnt = 0;                          // 直角对补出了中线，软兜底计数清零
                return (0);
            }
        }
#endif
        if(MIDLINE_CROSS_ST_INSIDE == cross_state)      // 路口内丢线：回中 + 补线
        {
            MIDLine_Cross_Output(cross_hold_center, 0);
            MIDLine_Cross_Tick(0);
            midline_info.cross       = 1;
            midline_info.cross_state = cross_state;
            midline_info.cross_score = 0;
            return (0);
        }
#if (1 == MIDLINE_RAW_CENTER_EN)
        /* 两侧基线都没了，但逐行扫描可能仍扫到边（最底下几行全白、中间几行有边）。
         * 先用"裸边线"凑一帧真中线，算不出来再退到软兜底 —— 专治"赛道比视野还宽"的路段。 */
        if(1 == MIDLine_Raw_Output(width_ref))
        {
            return (0);
        }
#endif
#if (1 == MIDLINE_HIST_EN)
        /* 基点行本身就丢线（最底行找不到跳变 → Badpoint 无效 → 两侧基线一起废）。
         * 参考补线法 line.c：这一帧不沿用"上一帧结果"，改为取"5 帧前的结果"。
         * 计数口径：出现丢线的那一帧算倒数第 1 帧，再往前数 5 帧（N-5，100ms 前）的那一帧。
         * 单帧丢线大多是反光、阴影、赛道外杂物，上一帧很可能已经踩脏；
         * 退 5 帧拿一条确定算得出来的中线，既不撒手，也不会把最后那个错误角度按住不放。 */
        if((hist_used < MIDLINE_HIST_MAX) && (1 == MIDLine_Hist_Output(MIDLINE_HIST_BACK)))
        {
            hist_used ++;
            MIDLine_Cross_Tick(0);
            midline_info.cross       = 0;
            midline_info.cross_state = cross_state;
            midline_info.cross_score = 0;
            return (0);
        }
#endif
#if (1 == MIDLINE_BLIND_EN)
        /* 两侧基线同时失效，且不在路口内：先分清"看不见赛道"还是"赛道比视野还宽"。
         * 赛道占满整幅画面时没有横向信息，但方向信息还在（车没偏），软兜底比撒手更稳。 */
        if(blind_cnt < MIDLINE_BLIND_MAX)
        {
            blind_cnt ++;
            MIDLine_Soft_Output();
            MIDLine_Cross_Tick(0);
            midline_info.cross       = 0;
            midline_info.cross_state = cross_state;
            midline_info.cross_score = 0;
            return (0);
        }
#endif
        MIDLine_Lost_Output();                          // 完全丢线：偏差回中，别按死最后一个角打舵
        MIDLine_Cross_Tick(0);
        midline_info.cross       = 0;
        midline_info.cross_state = cross_state;
        midline_info.cross_score = 0;
        return (1);
    }

    // 1. 计算行区间：取两条基线的公共区间（row 大的一端靠车头）
    if(bedline_left.valid)
    {
        row_start = (int)bedline_left.row_start;
        row_end   = (int)bedline_left.row_end;
    }
    if(bedline_right.valid)
    {
        if(row_start < 0)
        {
            row_start = (int)bedline_right.row_start;
            row_end   = (int)bedline_right.row_end;
        }
        else
        {
            if((int)bedline_right.row_start > row_start)
            {
                row_start = (int)bedline_right.row_start;
            }
            if((int)bedline_right.row_end < row_end)
            {
                row_end = (int)bedline_right.row_end;
            }
        }
    }

    if((row_start < 0) || ((row_start - row_end + 1) < MIDLINE_MIN_ROWS))
    {
        if(MIDLINE_CROSS_ST_INSIDE == cross_state)
        {
            MIDLine_Cross_Output(cross_hold_center, 0);
            MIDLine_Cross_Tick(0);
            midline_info.cross       = 1;
            midline_info.cross_state = cross_state;
            midline_info.cross_score = 0;
            return (0);
        }
#if (1 == MIDLINE_RAW_CENTER_EN)
        if(1 == MIDLine_Raw_Output(width_ref))          // 行区间退化：裸边线还扫得到就用它
        {
            return (0);
        }
#endif
#if (1 == MIDLINE_BLIND_EN)
        /* 行区间退化成 0（基线只覆盖了贴车头的几行）：同样属于"没有横向信息但没离道"，
         * 给它软兜底，别在这一帧把车头判成回正。 */
        if(blind_cnt < MIDLINE_BLIND_MAX)
        {
            blind_cnt ++;
            MIDLine_Soft_Output();
            MIDLine_Cross_Tick(0);
            midline_info.cross       = 0;
            midline_info.cross_state = cross_state;
            midline_info.cross_score = 0;
            return (0);
        }
#endif
        MIDLine_Lost_Output();                          // 有效行不够：偏差回中
        MIDLine_Cross_Tick(0);
        midline_info.cross       = 0;
        midline_info.cross_state = cross_state;
        midline_info.cross_score = 0;
        return (1);
    }

    /* 1.5 路口"快速通道"：近端宽度被横臂顶到 学习宽度xOPEN_RATIO 以上就直接按路口策略走 ----
     * 五特征打分要"连成片"才成立，遇到边线断得不干净的窄臂路口会漏判；
     * 而漏判的那几帧，恰恰就是白色开口把中线往臂的方向拉、车开始拐的时候。 */
#if (1 == MIDLINE_CROSS_OPEN_EN)
    {
        uint8 open_now = 0;

        if((1 == bedline_left.valid) && (1 == bedline_right.valid))
        {
            float l_near = (bedline_left.k  * (float)row_start) + bedline_left.b;
            float r_near = (bedline_right.k * (float)row_start) + bedline_right.b;

            if((r_near - l_near) > (width_ref * MIDLINE_CROSS_OPEN_RATIO))
            {
                open_now = 1;
            }
        }

        if(1 == open_now)
        {
            if(cross_open_cnt < 255)
            {
                cross_open_cnt ++;
            }
            cross_hint = 1;                             // 本帧加权立刻按路口策略走（远端主导 + 丢宽行）
        }
        else
        {
            cross_open_cnt = 0;                         // 断续的不算：横臂一出现就是持续的宽
        }
    }
#endif

    // 2. 逐行求中线，按 midline_weight(row, turn) 加权累加
    uint32 weight_midline_sum = 0;                      // 加权中线累加值 Σ(mid * w)
    uint32 weight_sum         = 0;                      // 权重累加值     Σw
    uint32 weight_full        = 0;                      // 满权重 Σw(full)
    float  sum_w_width        = 0.0f;                   // Σ(w * 赛道宽度)
    float  sum_w_both         = 0.0f;                   // 两侧都有效的行权重和
    uint8  num                = 0;

    for(int row = row_start; row >= row_end; row --)
    {
        float left_col  = 0.0f;
        float right_col = 0.0f;
        uint8 ok_left   = 0;
        uint8 ok_right  = 0;
#if (1 == MIDLINE_CORNER_EN)
        uint8 bridge_row = 0;                           // 1-本行的边界来自直角补线
#endif

        if(bedline_left.valid)                          // 左边界：基线在该行的列号
        {
            float c = (bedline_left.k * (float)row) + bedline_left.b;
            if((c >= 0.0f) && (c <= (float)(MT9V03X_W - 1)))
            {
                left_col = c;
                ok_left  = 1;
            }
        }
        if(bedline_right.valid)                         // 右边界
        {
            float c = (bedline_right.k * (float)row) + bedline_right.b;
            if((c >= 0.0f) && (c <= (float)(MT9V03X_W - 1)))
            {
                right_col = c;
                ok_right  = 1;
            }
        }

        /* ---- 直角弯界外带（本轮新增）----
         * 判为界外的那一侧，拟合出来的列号只是"把下面那条直线往上延长"的产物，
         * 它落在界外区里，没有任何物理意义。这一侧直接作废，交给单侧逻辑
         * （另一侧 + 半宽）或者直角对补线，绝不能让界外的那一侧参与取中点。 */
#if (1 == MIDLINE_OUTSIDE_EN)
        if(1 == out_left[row])
        {
            ok_left = 0;
        }
        if(1 == out_right[row])
        {
            ok_right = 0;
        }
#endif

        /* ---- 直角对补基线：把"近处直角 -> 对面直角"的连线直接当这一行的边界用 ----
         * 故意放在真实基线之后：路口区里横臂把左右边界推到画面两侧，
         * 那个区间里"扫描出来的真实边界"本来就没有物理意义，几何连线才是可靠的那一个。 */
#if (1 == MIDLINE_CORNER_EN)
        if(1 == corner_ready)
        {
            int   r_line  = row - (((int)corner_slide) * MIDLINE_CORNER_SLIDE_PX);   // 保持期基线往车头滑
            uint8 in_band = (uint8)((r_line <= (int)corner_left.near_row) &&
                                    (r_line >= (int)corner_left.far_row));
            uint8 in_low  = (uint8)((1 == MIDLINE_CORNER_FILL_LOW_EN) &&
                                    (r_line > (int)corner_left.near_row));

            /* 横臂带里：连线无条件接管（那一带扫描出来的真实边界本来就没有物理意义）。
             * 近端直角以下（车已经开进路口中央）：只有真实边线断了才让基线去补，真实边线永远优先。 */
            if((1 == in_band) || ((1 == in_low) && ((0 == ok_left) || (0 == ok_right))))
            {
                float c_l   = 0.0f;
                float c_r   = 0.0f;
                uint8 got_l = MIDLine_Corner_Col(&corner_left,  r_line, &c_l);
                uint8 got_r = MIDLine_Corner_Col(&corner_right, r_line, &c_r);

                if((1 == got_l) && (1 == got_r) && (c_r > c_l))   // 两条线都得在、且不交叉
                {
                    left_col   = c_l;
                    right_col  = c_r;
                    ok_left    = 1;
                    ok_right   = 1;
                    bridge_row = 1;                     // 本行是补出来的
                    if(midline_info.corner_use < 255)
                    {
                        midline_info.corner_use ++;
                    }
                }
            }
        }
#endif

        /* 路口区把权重翻转成远端主导：近景的白色开口不携带"车相对赛道"的信息，
         * 只有远处还连着主路边线的行才说明车头朝向 */
        uint8  turn_use = ((1 == cross_hint) && (1 == MIDLINE_CROSS_FAR_EN)) ?
                          ((uint8)1) : (midline_info.turn);
        if(1 == right_angle)                            // 直角弯：远端是界外，权重压回近处（不提前转向）
        {
            turn_use = 0;
        }
        uint32 w_row = midline_weight(row, turn_use);   // 该行权重（双段式 + 弯道/路口翻转）
        uint32 w_fun = w_row;                           // 满权重（算置信度用）
        float  mid   = 0.0f;

#if (1 == MIDLINE_CORNER_EN)
        if(1 == bridge_row)                             // 补线行：权重略打折，别压过真实远端行
        {
            w_row = (uint32)((float)w_row * MIDLINE_CORNER_WEIGHT);
            if(0u == w_row)
            {
                w_row = 1u;
            }
        }
#endif
#if (1 == MIDLINE_FILL_EN)
        if(1 == fill_flag[row])                         // 继承补线补出来的行
        {
            /* 路口帧里这些行一条都不能用：补线是拿近处边线按"上次斜率"往上顶出来的，
             * 斜率的误差会随行数放大（16 行 x 3 px = 48 px），而路口恰恰是边线断得最多的地方。
             * 路口内的正确做法是保持进路口那一刻的航向，不是把补出来的线当中线。 */
            if(1 == cross_hint)
            {
                continue;
            }
            w_row = ((w_row * (uint32)MIDLINE_FILL_WEIGHT) / 100u);
            if(0u == w_row)
            {
                w_row = 1u;
            }
        }
#endif
        if(ok_left && ok_right && (right_col > left_col))                   // 两侧都有效：取中点
        {
            float w_now = right_col - left_col;

            mid = (left_col + right_col) * 0.5f;

            // 宽度学习做分位数过滤：横臂/丢边把宽度带飞的行不参与
            if((w_now >= (width_ref * MIDLINE_WIDTH_LEARN_MIN)) &&
               (w_now <= (width_ref * MIDLINE_WIDTH_LEARN_MAX)))
            {
                sum_w_width += ((float)w_row * w_now);
                sum_w_both  += (float)w_row;
            }

            /* 宽行 = 横臂开口。这一行的"中点"由开口决定，不由赛道决定：
             *   - 路口区：直接丢掉（留着就会把中线钉死在画面中心，车必然往横臂方向拐）
             *   - 平常  ：只打折，保留连续性，防止弯道两侧同时外扩时误伤 */
            if(w_now > (width_ref * MIDLINE_CROSS_WIDTH_RATIO))
            {
#if (1 == MIDLINE_CROSS_NEAR_OFF_EN)
                if(1 == cross_hint)
                {
                    continue;
                }
#endif
                w_row = (uint32)((float)w_row * MIDLINE_WIDE_SCALE);
                if(0u == w_row)
                {
                    w_row = 1u;
                }
            }
        }
        else if(ok_left || ok_right)                                        // 只有单侧：用半宽推算
        {
#if (1 == MIDLINE_EDGE_INVALID_EN)
            /* 边界贴到画面边缘：外侧的深色地板已经被横臂"吃"进画面，
             * 拿这个边界加/减半宽推出来的中点完全失真，宁可不要这一行 */
            if(((1 == ok_left)  && (left_col  <= (float)MIDLINE_EDGE_MARGIN)) ||
               ((1 == ok_right) && (right_col >= (float)(MT9V03X_W - 1 - MIDLINE_EDGE_MARGIN))))
            {
                continue;
            }
#endif
            float half_width = width_ref * 0.5f;
            mid   = ok_left ? (left_col + half_width) : (right_col - half_width);
            w_row = (uint32)((float)w_row * MIDLINE_ONE_SIDE_SCALE);        // 可信度打折
        }
        else
        {
            continue;                                                       // 本行两侧都无效
        }

        if(mid < 0.0f)                                  // 限幅到图像范围
        {
            mid = 0.0f;
        }
        if(mid > (float)(MT9V03X_W - 1))
        {
            mid = (float)(MT9V03X_W - 1);
        }

        midline_info.line[row] = (uint8)mid;            // mid_line_list[row]
        weight_midline_sum    += (uint32)((uint8)mid) * w_row;
        weight_sum            += w_row;
        weight_full           += w_fun;
        num ++;
    }

    // 3. 加权求和得到本次中线
    if((num < MIDLINE_MIN_ROWS) || (0 == weight_sum))
    {
        if(MIDLINE_CROSS_ST_INSIDE == cross_state)      // 路口内一行可信的都没有
        {
            MIDLine_Cross_Output(cross_hold_center, 0); // 偏差回中，走直线穿过去
            MIDLine_Cross_Tick(0);
            midline_info.cross       = 1;
            midline_info.cross_state = cross_state;
            midline_info.cross_score = 0;
            return (0);
        }
        MIDLine_Lost_Output();                          // 丢线：偏差回中
        MIDLine_Cross_Tick(0);
        midline_info.cross       = 0;
        midline_info.cross_state = cross_state;
        midline_info.cross_score = 0;
        return (1);
    }

    uint8 mid_line = (uint8)(weight_midline_sum / weight_sum);

    // 4. 互补滤波（一阶低通）
    float center_now = (midline_info.last_mid_line * MIDLINE_FILTER_OLD) +
                       ((float)mid_line * MIDLINE_FILTER_NEW);

    /* 单帧跳变限幅：横臂一进画面，中线可能一帧跳几十像素，舵机会跟着抽一下。
     * 每帧最多移动 MIDLINE_MAX_JUMP_PX，剩下的量留给后面几帧慢慢收。
     * 路口帧（含"快速通道"刚触发、还没进 INSIDE 的那几帧）把限幅收到 MIDLINE_CROSS_TRUST_PX：
     * 白色开口把中线往外拽的那一下，绝对不允许它落到输出上。 */
    {
        float jump_lim = (1 == cross_hint) ? (MIDLINE_CROSS_TRUST_PX) : (MIDLINE_MAX_JUMP_PX);
        float jump_now = center_now - midline_info.last_mid_line;

        if(jump_now > jump_lim)
        {
            center_now = midline_info.last_mid_line + jump_lim;
        }
        else if(jump_now < -jump_lim)
        {
            center_now = midline_info.last_mid_line - jump_lim;
        }
    }

    float width_now  = (sum_w_both > 0.0f) ? (sum_w_width / sum_w_both) : (width_ref);
    float dev_now    = center_now - ((float)MT9V03X_W * 0.5f);

    midline_info.num        = num;
    midline_info.weight_sum = weight_sum;
    midline_info.weight_full = weight_full;
    midline_info.mid_line   = mid_line;
    midline_info.row_start  = (uint8)row_start;
    midline_info.row_end    = (uint8)row_end;
    midline_info.conf       = (uint8)((weight_full > 0u) ?
                                      ((weight_sum * 100u) / weight_full) : (0u));

    // 5. 十字路口判定 + 状态机
#if (1 == MIDLINE_CROSS_EN)
    score = MIDLine_Cross_Score(width_ref, dev_now);
#if (1 == MIDLINE_CORNER_EN)
    if(1 == corner_ready)                               // 直角对是比特征打分更硬的证据：直接保底
    {
        if(score < MIDLINE_CROSS_SCORE_NEED)
        {
            score = MIDLINE_CROSS_SCORE_NEED;
        }
    }
#endif
#if (1 == MIDLINE_CROSS_OPEN_EN)
    if(cross_open_cnt >= MIDLINE_CROSS_OPEN_FRAMES)     // 近端宽度连续超限：白色开口已经是既成事实
    {
        if(score < MIDLINE_CROSS_SCORE_NEED)
        {
            score = MIDLINE_CROSS_SCORE_NEED;
        }
    }
#endif
    MIDLine_Cross_Tick(score);
#else
    score = 0;
    (void)score;
#endif
    midline_info.cross_score = score;
    midline_info.cross_state = cross_state;
    midline_info.cross       = ((MIDLINE_CROSS_ST_INSIDE == cross_state) ||
                                (MIDLINE_CROSS_ST_EXIT   == cross_state)) ? (1) : (0);

    if(MIDLINE_CROSS_ST_INSIDE == cross_state)          // 路口内：只信"远端主导"算出来的中线
    {
        /* 硬回正 = 舵机突然回中，车会沿斜线冲进路口四个角（老方案的头号问题）；
         * 死锁偏差 = 一直按同一个角打舵，车沿一段圆弧拐走（本轮要解决的问题）。
         * 现在的做法：本帧还有可信行（远端还连着主路）就跟着它走，车头自然回正；
         *             一行可信的都没有时，上面几个分支已经让偏差指数回中，走直线穿过去。 */
        MIDLine_Cross_Output(center_now, 1);
        midline_info.width = width_ref;                 // 路口内不学赛道宽度
        blind_cnt = 0;                                  // 本帧有横向信息，软兜底计数清零
        return (0);
    }

    midline_info.center         = center_now;
    midline_info.last_mid_line  = center_now;
    midline_info.mid_line_value = (uint8)(center_now + 0.5f);
    midline_info.error          = dev_now;

    if((MIDLINE_CROSS_ST_EXIT == cross_state) ||        // 刚出路口 / 冷却：先别学宽度
       (MIDLINE_CROSS_ST_COOLDOWN == cross_state))
    {
        midline_info.width = width_ref;
    }
    else
    {
        midline_info.width = width_now;
    }

    midline_info.valid = 1;
    blind_cnt = 0;                                      // 本帧有横向信息，软兜底计数清零
    hist_used = 0;                                      // 正常帧到了，历史兜底计数也归零
    return (0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在屏幕上画出中线/真实边界，并显示中线值、偏差、路口状态
// 参数说明     void
// 返回参数     void
// 使用示例     MIDLine_Show_On_TFT180();
// 备注信息     1. 绿点 = 加权中线，蓝点 = 逐行真实边界；必须排在 tft180_show_gray_image 之后
//              2. Y4 行：C 中线  E 偏差  X 路口状态字母
//              3. Y=100 行：路口状态机 / 打分 / 置信度 / 弯道标志
//              4. Y=116 行：五个特征计数（F1宽行 F2丢线带 F3外扩 F4上方有效行 F5短黑条行）
//-------------------------------------------------------------------------------------------------------------------
void MIDLine_Show_On_TFT180(void)
{
    tft180_set_font(TFT180_8X16_FONT);                  // 下面的 8x16 数字排版依赖这个字体
#if (1 == CAMERA_DIS_IMAGE)
    if(midline_info.valid)                              // 绿点画在图像上，图像关了就没有意义
    {
        for(int row = (int)midline_info.row_start; row >= (int)midline_info.row_end; row --)
        {
            if(midline_info.line[row] == MIDLINE_INVALID_COL)
            {
                continue;
            }

            uint16 x = midline_to_dis_x(midline_info.line[row]);
            uint16 y = midline_to_dis_y((uint8)row);
            if((x < tft180_width_max) && (y < tft180_height_max))
            {
                tft180_draw_point(x, y, RGB565_GREEN);
            }
        }
#if (1 == MIDLINE_RAW_EN)
        /* 蓝点只画在 row >= MIDLINE_DIS_BLUE_TOP（=H/5）的区域：
         * 图像上五分之一是远景，蓝点在那里糊成一片、还挡住屏幕上方，直接不画。
         * 注意只是不"显示"，raw_left/raw_right 的扫描与判定完全不受影响。 */
        for(int row = MIDLINE_DIS_BLUE_TOP; row <= (MT9V03X_H - 3); row ++)
        {
            uint16 y = midline_to_dis_y((uint8)row);
            if(y >= tft180_height_max)
            {
                continue;
            }
            if(midline_info.raw_left[row] != MIDLINE_INVALID_COL)
            {
                uint16 x = midline_to_dis_x(midline_info.raw_left[row]);
                if(x < tft180_width_max)
                {
                    tft180_draw_point(x, y, RGB565_BLUE);
                }
            }
            if(midline_info.raw_right[row] != MIDLINE_INVALID_COL)
            {
                uint16 x = midline_to_dis_x(midline_info.raw_right[row]);
                if(x < tft180_width_max)
                {
                    tft180_draw_point(x, y, RGB565_BLUE);
                }
            }
        }
#endif

        /* 红色 = 边缘基线（bedline 的最小二乘拟合线 col = k*row + b）
         * 蓝点是逐行原始边界采样，红线是"基线"本身；两条几乎重合说明基线跟得稳，
         * 只有极端路口才会出现红蓝分叉，这时候就是补线/继承在起作用。 */
        if(bedline_left.valid)
        {
            for(int row = (int)bedline_left.row_start; row >= (int)bedline_left.row_end; row --)
            {
                float c = (bedline_left.k * (float)row) + bedline_left.b;
                if((c >= 0.0f) && (c <= (float)(MT9V03X_W - 1)))
                {
                    uint16 x = midline_to_dis_x((uint8)(c + 0.5f));
                    uint16 y = midline_to_dis_y((uint8)row);
                    if((x < tft180_width_max) && (y < tft180_height_max))
                    {
                        tft180_draw_point(x, y, RGB565_RED);
                    }
                }
            }
        }

        if(bedline_right.valid)
        {
            for(int row = (int)bedline_right.row_start; row >= (int)bedline_right.row_end; row --)
            {
                float c = (bedline_right.k * (float)row) + bedline_right.b;
                if((c >= 0.0f) && (c <= (float)(MT9V03X_W - 1)))
                {
                    uint16 x = midline_to_dis_x((uint8)(c + 0.5f));
                    uint16 y = midline_to_dis_y((uint8)row);
                    if((x < tft180_width_max) && (y < tft180_height_max))
                    {
                        tft180_draw_point(x, y, RGB565_RED);
                    }
                }
            }
        }
    }
#endif

#if (1 == CAMERA_DIS_INFO)                          // CAMERA_DIS_INFO = 0 时只留图像（绿中线 + 蓝边界），不画任何数字
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    /* Y4 行：C 中线值  E 偏差  X 路口状态（屏幕宽 128，8x16 字体，x 必须 < 128，字段按 8px 排） */
    tft180_show_string(0, CAMERA_INFO_Y4, "C");                                     // 滤波后的中线值
    tft180_show_int(10, CAMERA_INFO_Y4, (int32)midline_info.mid_line_value, 3);
    tft180_show_string(36, CAMERA_INFO_Y4, "E");                                    // 偏差
    tft180_show_float(46, CAMERA_INFO_Y4, (double)midline_info.error, 2, 1);
    tft180_show_string(88, CAMERA_INFO_Y4, "X");                                    // 路口状态字母
    tft180_show_int(98, CAMERA_INFO_Y4, (int32)midline_info.cross_state, 1);

    /* Y=100：S 状态机  P 打分  Q 置信度  T 弯道标志 */
    tft180_show_string(0, 100, "S");
    tft180_show_int(10, 100, (int32)midline_info.cross_state, 1);
    tft180_show_string(26, 100, "P");
    tft180_show_int(36, 100, (int32)midline_info.cross_score, 1);
    tft180_show_string(56, 100, "Q");
    tft180_show_int(66, 100, (int32)midline_info.conf, 3);
    tft180_show_string(96, 100, "T");
    tft180_show_int(106, 100, (int32)midline_info.turn, 1);

    /* Y=116：五特征计数（F1 宽行 / F2 丢线带 / F3 外扩对称行 / F5 短黑条行），四位 3 位数字刚好排满 128 */
    tft180_show_int(0, 116, (int32)midline_info.cross_feat[0], 3);
    tft180_show_int(32, 116, (int32)midline_info.cross_feat[1], 3);
    tft180_show_int(64, 116, (int32)midline_info.cross_feat[2], 3);
    tft180_show_int(96, 116, (int32)midline_info.cross_feat[4], 3);

#if (1 == MIDLINE_CORNER_EN)
    /* Y=132 直角对调试行：N 近端直角行 / F 对面直角行 / 左、右近端直角列 / 补线行数 / 保持剩余帧 */
    tft180_show_string(0,   MIDLINE_DIS_CORNER_Y, "N");
    tft180_show_int(8,      MIDLINE_DIS_CORNER_Y, (int32)midline_info.corner_near_row, 2);
    tft180_show_string(24,  MIDLINE_DIS_CORNER_Y, "F");
    tft180_show_int(32,     MIDLINE_DIS_CORNER_Y, (int32)midline_info.corner_far_row, 2);
    tft180_show_int(48,     MIDLINE_DIS_CORNER_Y, (int32)corner_left.near_col, 3);
    tft180_show_int(70,     MIDLINE_DIS_CORNER_Y, (int32)corner_right.near_col, 3);
    tft180_show_int(92,     MIDLINE_DIS_CORNER_Y, (int32)midline_info.corner_use, 2);
    tft180_show_int(108,    MIDLINE_DIS_CORNER_Y, (int32)corner_keep, 2);   // 保持窗口剩余帧（0.5s = 25 帧，0 = 基线已释放）
#endif

    /* Y=144：继承补线调试行（L = 本帧补出来的行数，0 = 这一帧没丢线）——末行 y+15 = 159 < 160 */
    tft180_show_string(0, 144, "L");
#if (1 == MIDLINE_FILL_EN)
    tft180_show_int(8, 144, (int32)midline_info.fill_use, 2);
#else
    tft180_show_int(8, 144, (int32)0, 2);
#endif

    /* Y=144：直角弯界外判别（O = 本帧判为界外的行数，R = 1 表示按直角弯处理：只看近处、不提前转向） */
    tft180_show_string(40, 144, "O");
#if (1 == MIDLINE_OUTSIDE_EN)
    tft180_show_int(48, 144, (int32)outside_cnt, 2);
#else
    tft180_show_int(48, 144, (int32)0, 2);
#endif
    tft180_show_string(72, 144, "R");
    tft180_show_int(80, 144, (int32)right_angle, 1);
#endif                                              // CAMERA_DIS_INFO
}
