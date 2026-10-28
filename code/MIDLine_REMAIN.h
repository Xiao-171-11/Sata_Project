/*
 * MIDLine_REMAIN.h
 *
 *  加权中线判定法 + 十字路口多特征判定
 *
 *  一、加权中线（替代逆透视 + 纯追踪）
 *    1) 每一行由左右两条边缘基线各得到一个边界列，两者取中点 = 该行的中线点，存进 mid_line_list
 *    2) 每一行有一个权重，双段式：远段 line+1（线性），近段 line^2+1（二次，近景主导），上限 200
 *       权重数组 mid_weight_list[] 保留为"百分比增益表"（默认 100），可直接改某一行来手工干预
 *    3) 弯道时权重方向翻转：只让 row <= MIDLINE_TURN_HEIGHT 的远端行主导，提前入弯
 *    4) 与上一帧结果做互补滤波：mid_line_value = last * 0.2 + new * 0.8
 *    5) 顺带输出置信度 conf（0~100）= Σw / 满权重，供控制层降速、降增益
 *
 *  二、十字路口判定（多特征打分 + 状态机）
 *    判据全部取"逐行真实边界" raw_left/raw_right（不是拟合基线），5 个物理特征：
 *      F1 wide  ：宽度 > 学习宽度 x 1.30 的行数         （横臂张开）
 *      F2 gap   ：两侧同时丢线的连续行数                 （横臂断带 / 白路面中断）
 *      F3 rise  ：相邻两行双边界同时向外跳 >= 6px 的行数 （横向路面突然出现）
 *      F4 above ：断带"上方"仍有有效行的行数             （=0 说明是出弯丢线，不是路口）
 *      F5 mark  ：一行里 2~4 根 1~10px 短黑条的行数      （路口两排短横条；>=5 根是斑马线，已排除）
 *    打分 = F1 + F2 + F3 + F5，>= MIDLINE_CROSS_SCORE_NEED 且中线居中才进 CONFIRM；
 *    状态机：NONE -> CONFIRM -> INSIDE -> EXIT -> COOLDOWN -> NONE（带冷却，防抖）
 *
 *  三、十字路口直角对补线（补出来的线直接当基线用）
 *    1) 横臂行 = 两侧都有数据、且宽度 > 学习宽度 x 1.35 的行；从车头往远处找最靠前的一段连续横臂
 *    2) 近处两个直角 = 横臂近端外面那一行的左右边界（左右必须同时向外跳 >= 5px 才算两个直角）
 *    3) 对面两个直角 = 横臂远端外面那一行的左右边界；看不到对面主路时用近端基线斜率外推
 *    4) 连线一律"左连左、右连右"：col = k * row + b，两点定线，逐行验算不越界、左右不交叉
 *    5) 补线区间 [far_row, near_row] 内直接用连线替换该行边界，权重乘 MIDLINE_CORNER_WEIGHT 打折；
 *       两侧基线全断（车正压路口中央）时，用两条连线自己加权算中线，不需要基线
 *    6) 四个直角是硬证据：成立时把路口打分直接保底到 NEED，状态机不等特征计数
 *    7) 基线保持：一旦连成，基线连同"路口中央没有真实边线"的行一起用，并保持 MIDLINE_CORNER_HOLD_MS（0.5s）；
 *       保持期内基线每帧按 MIDLINE_CORNER_SLIDE_PX 往车头方向滑（车在动，画面里的拐角也在动）
 *    8) 总时长有上限 MIDLINE_CORNER_RUN_MAX 帧，防止车卡在路口里时基线一直挂着
 *
 *  Created on: 2026年10月21日
 *      Author: Freed
 */

#ifndef CODE_MIDLINE_REMAIN_H_
#define CODE_MIDLINE_REMAIN_H_
#include "zf_common_headfile.h"
#include "Bedline_REMAIN.h"                             // 需要左右边缘基线

/* ================================ 可调参数 ================================ */
#define MIDLINE_DEFAULT_CENTER  (MT9V03X_W / 2)         // 中线默认值（图像中心列）
#define MIDLINE_INVALID_COL     (0xFF)                  // mid_line_list[] 中表示该行无效

/* ---- 一、逐行权重（双段式 line+1 / line^2+1） ---- */
#define MIDLINE_WEIGHT_SEG_A_END    (60)                // 权重分段点：row < 60 用线性 row+1，否则用二次
#define MIDLINE_WEIGHT_MAX          (200)               // 权重上限（近景平台，避免一行吃掉全部权重）
#define MIDLINE_WEIGHT_QUAD_DIV     (4)                 // 二次段除数：w = (row-60)^2/4 + 61
#define MIDLINE_TURN_EN             (1)                 // 1-启用弯道权重翻转（看得更远，提前入弯）
#define MIDLINE_TURN_HEIGHT         (40)                // 弯道权重平台行：row > 40 的行权重取平台值
#define MIDLINE_TURN_NEAR_W         (8)                 // 弯道时近景行(row > TURN_HEIGHT)的固定小权重
#define MIDLINE_TURN_DIFF           (10.0f)             // 远端中线 - 近端中线 超过它判为弯道（像素）
#define MIDLINE_TURN_DIFF_EXIT      (6.0f)              // 回落到它以下才退出弯道（滞回）
#define MIDLINE_TURN_SEG            (12)                // 取远端/近端各多少行做弯道比较

#define MIDLINE_FILTER_NEW      (0.8f)                  // 互补滤波：本次值的系数
#define MIDLINE_FILTER_OLD      (0.2f)                  // 互补滤波：上次值的系数（两者和为 1）

#define MIDLINE_MIN_ROWS        (5)                     // 参与加权计算的最少行数
#define MIDLINE_ONE_SIDE_SCALE  (0.50f)                 // 只有单侧边线时用赛道半宽推算，权重打 5 折
#define MIDLINE_WIDTH_LEARN_MIN (0.70f)                 // 单行宽度 < 学习宽度 x0.7 不参与宽度学习
#define MIDLINE_WIDTH_LEARN_MAX (1.40f)                 // 单行宽度 > 学习宽度 x1.4 不参与宽度学习（防横臂带飞）
#define MIDLINE_DEFAULT_WIDTH   (110.0f)                // 尚未统计出赛道宽度时使用的默认宽度（像素）

/* ---- 二、逐行真实边界扫描（十字特征的数据源） ---- */
#define MIDLINE_RAW_EN              (1)                 // 1-做逐行真实边界扫描（十字判定必须）
#define MIDLINE_RAW_MARK_MAX        (10)                // 扫描时允许跳过的短黑条最大宽度（px）
#define MIDLINE_RAW_SKIP_MAX        (6)                 // 一行里最多跳几段短黑条（再多就不是短横条了）

/* ---- 三、十字路口：多特征打分 ---- */
#define MIDLINE_CROSS_EN            (1)                 // 1-开路口识别（路口内保持航向，限幅打角）
#define MIDLINE_CROSS_WIDTH_RATIO   (1.30f)             // F1：真实宽度 > 学习宽度 x 1.30 记一行"宽"
#define MIDLINE_CROSS_MIN_WIDE_ROWS (6)                 // F1：至少这么多行判"宽"才算命中
#define MIDLINE_CROSS_GAP_MIN       (3)                 // F2：两侧同时丢线连续 >= 这么多行才算命中
#define MIDLINE_CROSS_RISE_D        (6)                 // F3：相邻两行边界外跳 >= 这么多像素记一次
#define MIDLINE_CROSS_RISE_MIN      (4)                 // F3：至少这么多次才算命中
#define MIDLINE_CROSS_ABOVE_MIN     (2)                 // F4：断带上方至少还有这么多行有效（否则是出弯）
#define MIDLINE_CROSS_MARK_EN       (1)                 // F5：启用短黑条（路口两排横条）特征
#define MIDLINE_CROSS_MARK_MIN_ROWS (2)                 // F5：至少这么多行有短黑条
#define MIDLINE_CROSS_MARK_MAX_ROWS (8)                 // F5：超过这么多行 = 斑马线，不算路口
#define MIDLINE_CROSS_MARK_RUN_MIN  (2)                 // F5：一行里最少几根短黑条
#define MIDLINE_CROSS_MARK_RUN_MAX  (4)                 // F5：一行里最多几根短黑条（>=5 根交给斑马线模块）
#define MIDLINE_CROSS_MARK_ROW_TOP  (18)                // F5：只在这个行区间数短黑条
#define MIDLINE_CROSS_MARK_ROW_BOT  (78)                //     避开斑马线模块的 82~118 扫描带
#define MIDLINE_CROSS_SCORE_NEED    (3)                 // 打分阈值（最高 4 分，F4 是 F2 的闸门不计分）
#define MIDLINE_CROSS_CONFIRM       (2)                 // 连续这么多帧命中才认账
#define MIDLINE_CROSS_KEEP          (30)                // INSIDE 至少保持帧数（约 0.6 s @50fps）
#define MIDLINE_CROSS_CLEAN         (6)                 // 连续这么多帧无特征才允许退出
#define MIDLINE_CROSS_TOL           (34.0f)             // 进路口前中线必须在这个偏差内（防止斜入弯误判）
#define MIDLINE_CROSS_COOLDOWN      (20)                // 出路口后的冷却帧数（防抖，二次误判）

/* ---- 三点五、路口专用：加权中线本身的优化 ----
 *  路口内近景全是白色开口（横臂），常规"近景权重大"的加权会把中线拉向横臂方向，
 *  车就会顺着横臂转 90 度开出去 —— 这一组参数专门治这个。 */
#define MIDLINE_CROSS_FAR_EN        (1)                 // 1-路口区权重翻转成远端主导（只看还连着主路边线的行）
#define MIDLINE_CROSS_NEAR_OFF_EN   (1)                 // 1-路口区的宽行（横臂开口）直接丢弃，不参与中线
#define MIDLINE_WIDE_SCALE          (0.20f)             // 非路口区宽行的权重折扣（保留连续性，防误伤弯道）
#define MIDLINE_CROSS_HINT_SCORE    (2)                 // 上一帧打分 >= 它，本帧就提前走路口策略
#define MIDLINE_EDGE_INVALID_EN     (1)                 // 1-单侧边界贴在画面边缘的行判为不可信
#define MIDLINE_EDGE_MARGIN         (2)                 // 贴边判定余量（px）
#define MIDLINE_OUTSIDE_EN          (1)                 // 1-单侧"直角张开"（整行扫到画面边缘）把那一侧判为界外
#define MIDLINE_OUTSIDE_MIN_ROWS    (4)                 // 界外行数达到它就认定是直角弯：只看近处，不按远端打方向
#define MIDLINE_CROSS_DECAY         (0.22f)             // 路口内没有任何可信行时，偏差每帧回中系数
#define MIDLINE_LOST_DECAY          (0.35f)             // 完全丢线时，偏差每帧回中系数
/* ---- 赛道比视野还宽时的软兜底 ----
 *  边线全在画面外 -> Badpoint 取不到种子 -> 两条基线同时失效。
 *  这不是"看不见赛道"，而是"看得见的全是赛道"：此时判丢线（偏差回中）等于撒手不管，
 *  车会按回正后的车头直着冲出去。这里沿上一帧中线只允许极慢地回中，让车按原方向滑过去。 */
#define MIDLINE_BLIND_EN            (1)                 // 1-两侧基线同时失效时启用软兜底
#define MIDLINE_BLIND_MAX           (40)                // 软兜底最长连续帧数（40 x 20ms = 0.8s）
#define MIDLINE_BLIND_DECAY         (0.97f)             // 软兜底期间中线每帧保留系数（越小越早回中）
#define MIDLINE_BLIND_CONF          (25)                // 软兜底的置信度：控制层按它自动降速
#define MIDLINE_RAW_CENTER_EN       (1)                 // 1-两侧基线失效时，用逐行扫描的裸边线自己凑一帧中线
#define MIDLINE_RAW_CONF            (40)                // 裸边线中线的置信度（比正常帧低，但比软兜底可信）

/* ---- 基点行丢线：本帧不沿用"上一帧"，改用"三帧前"的成品（参考补线法 line.c） ---- */
#define MIDLINE_HIST_EN             (1)                 // 1-启用中线历史环形缓冲
#define MIDLINE_HIST_DEPTH          (8)                 // 环形缓冲深度（必须 > MIDLINE_HIST_BACK + 1）
#define MIDLINE_HIST_BACK           (5)                 // 回退帧数：取 5 帧前的结果（5 x 20ms = 100ms）
                                                        // 计数口径：出现丢线的那一帧算倒数第 1 帧，
                                                        //           再往前数 5 帧的那一帧（即 N-5）就是它
#define MIDLINE_HIST_MAX            (5)                 // 连续靠历史兜底的最多帧数（5 x 20ms = 100ms），超了退回归中式输出
#define MIDLINE_MAX_JUMP_PX         (20.0f)             // 中线单帧最大跳变（防路口里突然抽一下）
#define MIDLINE_CROSS_TRUST_PX      (8.0f)              // 路口内中线相对"进路口那一刻"的允许偏离：超了就视为噪声
#define MIDLINE_CROSS_OPEN_EN       (1)                 // 1-近端宽度 > 学习宽度 xOPEN_RATIO 就直接进路口策略（不走五特征打分）
#define MIDLINE_CROSS_OPEN_RATIO    (1.45f)             // 快速通道的宽度倍数（横臂把边界推到两侧才会到这个数）
#define MIDLINE_CROSS_OPEN_FRAMES   (3)                 // 近端宽度连续这么多帧超限，就直接算一次"命中"

/* ---- 四、十字路口：直角对识别 + 直角连线补基线（本节是本轮路口专项的核心）----
 *  几何事实（图像 row 小 = 远，row 大 = 近）：
 *      近景主路(窄)  ->  横臂白色宽带(极宽)  ->  对面主路(窄)
 *  · 两个"近处直角"   = 主路左右边线 与 横臂近沿 的两个交点，两点同行（行号 near_row）
 *  · 两个"对面直角"   = 主路左右边线 与 横臂远沿 的两个交点，两点同行（行号 far_row）
 *  · 同一侧的两个直角连一条直线 = 这一侧在路口区里的**基线**，直接替代拟合边线用
 *    左直角<->左直角、右直角<->右直角，绝不交叉；补出来的左右列再取中点 = 路口内的中线
 *  为什么这样补：路口里横臂把左右边界推到画面两侧，(左+右)/2 恒等于画面中心，
 *  任何"按真实边界加权"的做法都会被横臂钉死；只有几何连线还原了"主路往哪走"。 */
#define MIDLINE_CORNER_EN           (0)                 // 1-启用直角对识别 + 连线补基线（几何外推，风险最高，默认关）
#define MIDLINE_CORNER_WIDE_RATIO   (1.35f)             // 宽度 > 学习宽度 x 1.35 记一行"横臂行"
#define MIDLINE_CORNER_MIN_LEN      (3)                 // 横臂宽段最少行数（太短是普通宽弯，不是路口）
#define MIDLINE_CORNER_MAX_LEN      (72)                // 横臂宽段最多行数（再长就是整幅画面发白）
#define MIDLINE_CORNER_HOLE         (2)                 // 宽段里允许断开的行数（黑条/噪点导致整行失败）
#define MIDLINE_CORNER_NEAR_BOT     (MT9V03X_H - 6)     // 宽段最靠车的行超过这里：横臂已压到车头，近端拐点不可见
#define MIDLINE_CORNER_FAR_TOP      (4)                 // 对面主路至少要露出这么多行（row >= 4 才算看得见）
#define MIDLINE_CORNER_JUMP_MIN     (5)                 // 拐点处边界至少外跳这么多像素（缓弯不算直角）
#define MIDLINE_CORNER_K_MAX        (2.50f)             // 补线斜率上限 |k|（超了说明连线要飞出画面）
#define MIDLINE_CORNER_WIDTH_MIN    (0.60f)             // 拐点行的宽度必须落在 学习宽度 x0.6 ~ x1.5（验证是主路）
#define MIDLINE_CORNER_WIDTH_MAX    (1.50f)
#define MIDLINE_CORNER_CONFIRM      (2)                 // 连续这么多帧"近处+对面"成立才允许补线（防单帧噪点）
#define MIDLINE_CORNER_CONFIRM_NEAR (5)                 // 只有近处两个直角（对面被挡）时，要连续这么多帧才补线
#define MIDLINE_FRAME_MS            (20)                // 主循环 50fps，一帧 20ms（算保持时间用）
#define MIDLINE_CORNER_HOLD_MS      (500)               // 连成的基线保持时间：0.5s
#define MIDLINE_CORNER_KEEP         ((MIDLINE_CORNER_HOLD_MS) / (MIDLINE_FRAME_MS))   // = 25 帧 = 0.5s
#define MIDLINE_CORNER_RUN_MAX      (50)                // 补线总时长的帧数上限（=1s，防止卡在路口里一直挂基线）
#define MIDLINE_CORNER_SLIDE_PX     (1)                 // 保持期每帧把基线往车头方向平移的像素（车速快就调大）
#define MIDLINE_CORNER_FILL_LOW_EN  (1)                 // 1-保持期用基线补"近端以下（车已进路口）没有真实边线"的行
#define MIDLINE_CORNER_WEIGHT       (0.85f)             // 补线行的权重折扣（略低于真实边线）
#define MIDLINE_CORNER_FALLBACK_EN  (1)                 // 1-看不到对面主路时，用近端斜率先把线延伸出去
#define MIDLINE_DIS_CORNER_Y        (132)               // 直角对调试信息的屏显行（y+15 < 160）
/* ---- 屏显：蓝色扫描点（逐行真实边界）的最高一行 ----
 *  行号 0 是图像最远（最上）一行。上半部分本来就是"远景"，蓝点画上去只会糊成一片，
 *  看不出边界，还盖住屏幕上方。这里只画 row >= MT9V03X_H/5 的蓝点，
 *  即图像上五分之一完全不打蓝线；远端照样参与计算，只是不显示。 */
#define MIDLINE_DIS_BLUE_TOP        (MT9V03X_H / 5)     // =24 行（120 行图像的上五分之一）

/* ---- 五、逐行继承补线：解决十字路口"丢线"（edge 丢失 / 左右交叉）----
 *  做法参考常见的"从种子行往上逐行扫 + 找不到就继承上一行"的补线：某一行找不到边界，
 *  就用最近一行的有效边界顶上去，把这一行的坑填掉，中线就不会因为中间几行空了而跳变。
 *  这里比"整行继承"多做两件事：一是按最近两行的步进做斜率外推（贴线而不是台阶），
 *  二是补出来的行权重打折，真实边线永远压得住补线。 */
#define MIDLINE_FILL_EN             (1)                 // 1-启用逐行继承补线
#define MIDLINE_FILL_MAX_ROWS       (16)                // 连续补线的最大行数（再多就是编造）
#define MIDLINE_FILL_STEP_MAX       (3)                 // 斜率外推的单行步进上限（像素/行）
#define MIDLINE_FILL_EDGE_MARGIN    (2)                 // 补线允许贴到画面左右边缘的余量
#define MIDLINE_FILL_WEIGHT         (70)                // 补线行的权重百分比（70 = 打七折）
#define MIDLINE_FILL_WIDTH_RATIO    (1.60f)             // 单侧丢线时按"上次宽度"补；宽度超过 学习宽度x1.6 就不按宽度补（横臂行）

/* ---- 状态机编号 ---- */
#define MIDLINE_CROSS_ST_NONE       (0)                 // 直道/弯道
#define MIDLINE_CROSS_ST_CONFIRM    (1)                 // 可疑，累计帧数中
#define MIDLINE_CROSS_ST_INSIDE     (2)                 // 路口内（补线 + 保持航向）
#define MIDLINE_CROSS_ST_EXIT       (3)                 // 刚出路口（宽度冻结 + 权重回抓）
#define MIDLINE_CROSS_ST_COOLDOWN   (4)                 // 冷却

/* ================================ 数据结构 ================================ */
/* 一侧的直角对：近处直角 + 对面直角，两点连线就是这一侧在路口区里的基线 */
typedef struct
{
    uint8 valid;                                        // 1-本侧直角对可用（两点都在画面里且连线合法）
    uint8 near_row, far_row;                            // 近端直角行（靠车头，row 大）/ 对面直角行（row 小）
    uint8 near_col, far_col;                            // 两个直角点的列号
    float k, b;                                         // 连线基线：col = k * row + b
} midline_corner_struct;

typedef struct
{
    uint8  valid;                                       // 1-本帧中线计算成功
    uint8  num;                                         // 参与加权的行数
    uint8  row_start, row_end;                          // 计算区间（row_start 靠车头）
    uint32 weight_sum;                                  // 权重累加值 Σw
    uint32 weight_full;                                 // 满权重 Σw(full)，算置信度用
    uint8  mid_line;                                    // 本次加权中线值（滤波前）
    uint8  mid_line_value;                              // 互补滤波后的中线输出（给舵机用这个）
    float  last_mid_line;                               // 上一帧的中线输出（滤波状态）
    float  center;                                      // 滤波后的中线（浮点，画图/调试用）
    float  error;                                       // 偏差 = center - MT9V03X_W/2（正=偏右）
    float  width;                                       // 本帧加权平均赛道宽度（像素）
    uint8  conf;                                        // 置信度 0~100（Σw / 满权重）
    uint8  turn;                                        // 1-本帧判为弯道（权重重心已上移）
    uint8  cross;                                       // 1-本帧判为十字路口（已保持航向）
    uint8  cross_state;                                 // 路口状态机 MIDLINE_CROSS_ST_xxx
    uint8  cross_score;                                 // 本帧特征打分 0~4
    uint8  cross_feat[5];                               // F1宽行 F2丢线带 F3外扩 F4上方有效行 F5短黑条行
    uint8  corner_state;                                // 直角对状态：0-无  1-只有近处两个直角  2-近处+对面都对上
    uint8  corner_use;                                  // 本帧有多少行是用直角连线补出来的（调试用）
    uint8  corner_near_row;                             // 近处两个直角所在行
    uint8  corner_far_row;                              // 对面两个直角所在行
    uint8  corner_near_left;                            // 近处左直角列
    uint8  corner_near_right;                           // 近处右直角列
    uint8  corner_far_left;                             // 对面左直角列
    uint8  corner_far_right;                            // 对面右直角列
    uint8  fill_use;                                    // 本帧有多少行是"继承补线"补出来的（调试用）
    uint8  line[MT9V03X_H];                             // mid_line_list：每一行的中线列
    uint8  raw_left[MT9V03X_H];                         // 逐行真实左边界（MIDLINE_INVALID_COL 无效）
    uint8  raw_right[MT9V03X_H];                        // 逐行真实右边界
} midline_struct;

extern midline_struct midline_info;

/* 权重增益表：mid_weight_list[row]，百分比（默认 100），想手工干预某一行就改这里 */
extern uint8 mid_weight_list[MT9V03X_H];

/* ================================ 接口声明 ================================ */
void    MIDLine_Init            (void);                 // 清空中线信息 + 生成权重增益表
uint8   Find_MIDLine            (void);                 // 逐行扫描 + 加权求中线 + 互补滤波 + 路口判定
void    MIDLine_Show_On_TFT180  (void);                 // 画出中线并显示中线值/偏差/路口状态

/* ============================== 一键保守档 ==============================
 * 实车出问题时，把 MIDLINE_SAFE_MODE 改成 1 再编译：
 *   逐行继承补线  关（MIDLINE_FILL_EN   = 0）
 *   直角对补基线  关（MIDLINE_CORNER_EN = 0）
 *   直角弯判界外  关（MIDLINE_OUTSIDE_EN= 0）
 * 只留：逐行扫描 + 加权中线 + 五特征路口判定 + 路口内回中式输出 + 路口内直行。
 * 这一档就是"第一轮 + 第二轮 + 闸一~闸四"的组合，用来确认巡线本身没问题；
 * 确认之后再逐项把开关打开，一次只开一个，出问题立刻知道是哪一层。
 * 注意：必须放在文件末尾 —— 要盖掉上面所有模块开关的定义。 */
#ifndef MIDLINE_SAFE_MODE
#define MIDLINE_SAFE_MODE           (0)                 // 1-保守档：关掉三层高风险几何层
#endif

#if (1 == MIDLINE_SAFE_MODE)
#undef  MIDLINE_FILL_EN
#define MIDLINE_FILL_EN             (0)
#undef  MIDLINE_CORNER_EN
#define MIDLINE_CORNER_EN           (0)
#undef  MIDLINE_OUTSIDE_EN
#define MIDLINE_OUTSIDE_EN          (0)
#endif

#endif /* CODE_MIDLINE_REMAIN_H_ */
