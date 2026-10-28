/*
 * Pursuit_REMAIN.h
 *
 *  纯追踪（pure pursuit）：在逆透视俯视图上找前瞻点 -> 算曲率 -> 算前轮转角 -> 舵机偏移
 *
 *  和原来"中线加权偏差 -> 分段 PD 打角"的区别：
 *    原来：偏差是整幅图像素误差的加权平均，单位是"像素"，同一辆车同一赛道，
 *          近处 1 像素和远处 1 像素代表的横向距离差几十倍，增益永远调不到两头都好。
 *    现在：先把图像变成地面俯视图（每个格点 6.7 x 11.8 mm，恒定比例），
 *          在车前方 Ld 处取一个目标点，用几何公式直接算出"车要沿多大的圆弧过去"，
 *          再换算成前轮转角。曲率是**物理量**（1/mm），与安装高度、镜头、分辨率全部解耦。
 *
 *  几何公式（后轴中心为参考点，前视距离 Ld，目标点横向偏移 Xt）：
 *      kappa = 2 * Xt / (Xt^2 + Yt^2)          圆弧曲率，右转为正
 *      delta = atan(kappa * L)                 前轮转角，L 为轴距
 *      ratio = kappa * W / 2                   后轮差速比例，W 为后轮距
 *
 *  三个"防冲出赛道"的机制（针对用户反馈的路口/弯道冲出）：
 *    1. 十字路口：左右边界同时消失 -> 判定横臂开口 -> 保持进路口前的航向并逐拍衰减，
 *       同时按 open_rows / one_side_rows 区分"路口"和"弯道外侧丢线"，两者绝不混淆。
 *    2. 弯道：曲率直接参与减速，转角越大速度越低；并且单侧丢线时用学习到的半赛道宽
 *       推算另一侧，弯道里外道边界跑出画面也不会把中线拽到沟里。
 *    3. 冲出赛道：最近几行的中线偏离超过半赛道宽 -> off_track 置位，
 *       控制层会强制降速，并且直接朝赛道中心打角。
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */

#ifndef CODE_PURSUIT_REMAIN_H_
#define CODE_PURSUIT_REMAIN_H_
#include "zf_common_headfile.h"
#include "IPM_REMAIN.h"

/* ============================ 边界提取 ============================ */
/* 本赛道的边界不是"黑线"，而是"白路面 -> 暗地板"的跳变（白胶带铺在蓝色地板上）。
 * 所以边界提取的规则是：从预测中线往两侧扫，跳过路面里那些细的暗条纹
 * （斑马线、横臂、反光缝，厚度 <= PURSUIT_LINE_W_MAX），
 * 遇到第一片"够厚"的暗区，它的内侧第一个格子就是赛道边界。 */
#define PURSUIT_SEARCH_COLS     (42)                    // 从预测中线向左右各搜多少列
#define PURSUIT_LINE_W_MAX      (8)                     // 路面内暗条纹的最大厚度（格）；比这更厚才算地板边界
#define PURSUIT_TRACK_W_MIN     (22)                    // 有效赛道宽度下限（格点，约 148mm）
#define PURSUIT_TRACK_W_MAX     (92)                    // 有效赛道宽度上限（格点，约 620mm）
#define PURSUIT_WIDTH_TOL       (0.50f)                 // 实测宽度和学习的宽度差超过 50% 就当不可信
#define PURSUIT_SINGLE_TOL      (10.0f)                 // 单侧推算结果与预测中线差多少格以内才敢用
#define PURSUIT_WIDE_RATIO      (1.45f)                 // 实测宽度 > 学习宽度 x 这个倍数 = 路面变宽（十字/岔口）
#define PURSUIT_SLOPE_MAX       (1.20f)                 // 每往上一行，中线最多横移几格（限制斜率，防一路滑进支路）
#define PURSUIT_JUMP_MAX        (8.00f)                 // 单行中线相对预测值突跳超过这么多格 -> 该行不可信
#define PURSUIT_XT_CLAMP_MM     (IPM_X_HALF_MM * 0.92f) // 前瞻点横向偏移的硬上限（mm）：俯视图就 320mm 宽，超出去一定是杂点
#define PURSUIT_TARGET_ROW_MIN  (8)                     // 前瞻行往下找可信行时最低找到第几行（≈ 250mm）

/* ============================ 纯追踪几何参数 ============================ */
#define PURSUIT_WHEELBASE_MM    (130.0f)                // 轴距（前后轮中心距），量一下填这里
#define PURSUIT_REAR_TRACK_MM   (120.0f)                // 后轮距（两个驱动轮中心距）
#define PURSUIT_DELTA_MAX_DEG   (32.0f)                 // 前轮最大转角（机械极限 30~35 度）

/* 前视距离随速度线性变化：慢的时候看近一点（不会切内道），快的时候看远一点（不会摆） */
#define PURSUIT_LD_MIN_MM       (220.0f)
#define PURSUIT_LD_MAX_MM       (700.0f)                // 不要超过 IPM_Y_FAR_MM；弯道转不过来就继续减小（650/600）
#define PURSUIT_SPEED_NORM_MIN  (0.30f)                 // 速度归一化下限（发车前也至少看 415mm）

/* ============================ 弯道减速 ============================ */
#define PURSUIT_CURV_SLOW_K     (0.0180f)                // 每 1 度转角降多少速（0.014 -> 0.018：弯里更慢、更咬得住）
#define PURSUIT_CURV_SLOW_MIN   (0.4200f)               // 最慢降到 42%

/* ============================ 十字路口 ============================ */
/* 判据的核心只有一条：路面在中间"消失一段，又在上方回来"。
 *    band_rows ：第一段连续不可信行的长度（路面断掉的那一段）
 *    above_rows：这段不可信行"上方"又看清的行数
 * 十字/岔口：带子有长度、且上方又能看清 -> 判路口
 * 弯道出画：外侧边线一旦出画，上方永远是盲区 -> above = 0 -> 绝不判路口
 * 另外把 wide_rows（路面整体变宽）当第二条独立判据。 */
#define PURSUIT_CROSS_MIN_WIDE  (5)                     // 至少 5 行"路面变宽"才算路口/岔口
#define PURSUIT_CROSS_MIN_OPEN  (6)                     // 或者至少 6 行左右同时丢才算横臂开口
#define PURSUIT_CROSS_MIN_BAND  (4)                     // 中断带至少 4 行（约 47mm 的路面缺口）
#define PURSUIT_CROSS_MIN_ABOVE (4)                     // 中断带上方至少又能看清 4 行 -> 路面"回来了"
#define PURSUIT_CROSS_MIN_VALID (8)                     // 至少有 8 行看清楚才算可信（否则摄像头有问题）
#define PURSUIT_CROSS_TICKS     (32)                    // 路口保持 32 帧（50fps 下 0.64 s，够开过 640mm 宽的横向路）
#define PURSUIT_CROSS_CLEAN     (8)                     // 连续 8 帧没有开口才提前退出（太少会没过完就退出）
#define PURSUIT_CROSS_REJECT    (20)                    // 出路口后 20 帧内不再判路口（防抖）
#define PURSUIT_CROSS_MAX_KAPPA (0.0025f)               // 进路口前曲率超过这个值 -> 是弯道不是路口
#define PURSUIT_CROSS_DELTA_MAX (6.0f)                  // 路口内前轮转角最多 6 度（绝不许横着拐进支路）
#define PURSUIT_CROSS_LOOKAHEAD_MM (400.0f)             // 路口内的前瞻距离：固定 400mm，只沿车道轴走
#define PURSUIT_CROSS_NEAR_ROWS (8)                     // 路口内用第 1~8 行（车头前 170~310mm）当车道轴参考

/* 黑色横条标记：这个赛道的十字路口是在白路面上横贴两排黑色短横条。
 *   横条厚度 1~2 格，不够厚，pursuit_find_floor() 不会把它当边界（边界照样能找对），
 *   但"一行里数出 >= PURSUIT_MARK_MIN_RUNS 段短暗条"就是路口的正面证据 —— 弯道永远没有。
 *   这条判据带否决权（在其它否决之后生效），是全套判据里最硬的一条。 */
#define PURSUIT_MARK_MIN_RUNS   (2)                     // 一行里至少 2 段短暗条才算"标记行"
#define PURSUIT_MARK_MIN_ROWS   (2)                     // 至少 2 行有标记 -> 判定为路口

/* 整行全白：白路面铺在深色地板上，正常赛道每一行的两侧一定有深色地板；
 *   只有"横向大路面"（十字/岔口）才会出现整行 96 格一个暗点都没有的情形。
 *   弯道不会出现（弯外永远是地板），所以这条判据对弯道零误伤、对路口很稳。
 *   注意：出界格点在俯视图里是白色，判定前必须用 IPM_Lut_Row() 排掉含出界点的行。 */
#define PURSUIT_FULL_WHITE_MIN  (3)                     // 至少 3 行整行全白 -> 判定为路口


/* ============================ 冲出赛道 ============================ */
#define PURSUIT_OFFTRACK_MM     (190.0f)                // 最近 6 行中线偏离超过这个距离 = 车已经在赛道边缘
#define PURSUIT_RECOVER_GAIN    (0.2200f)               // 救车增益：每 mm 偏离给多少舵机偏移

/* ============================ 视野丢失保持 ============================ */
#define PURSUIT_HOLD_FRAMES     (5)                     // 看得清的行数不够时，沿用上一份快照最多 5 帧（100 ms）

/* ============================ 屏幕显示 ============================ */
/* TFT180 默认 TFT180_PORTAIT：宽 128、高 160，字体 TFT180_8X16（一个字符 8x16 像素）。
 * 屏幕上半部分 Y1~Y6 六行已经被相机/基点/基线/中线/控制占满（y = 2/18/34/50/68/84），
 * 所以纯追踪的信息画在第 7、8 行（y = 100 / 116），并且运行时再判一次屏幕够不够高。
 * 注意：zf 库里 show_float 的字符串长度是 num + pointnum + 2，show_int 是 num + 1，
 * 都是固定长度（不足补空格）。算 x 的时候必须按这个长度算，否则最后一格会画到 x = 128，
 * 触发 tft180.c:460 的 zf_assert(x < tft180_width_max)。 */
#define PURSUIT_DIS_EN          (1)                     // 1 = 在屏幕上显示纯追踪状态

/* ============================ 快照 ============================ */
typedef struct
{
    uint8  valid;                                       // 1 = 本次快照可用于控制
    uint8  cross;                                       // 1 = 正在过路口
    uint8  off_track;                                   // 1 = 车压到赛道边缘了
    uint8  valid_rows;                                  // 看得清的行数
    uint8  open_rows;                                   // 左右同时丢的行数
    uint8  one_side_rows;                               // 只有一侧丢的行数
    uint8  wide_rows;                                   // 路面变宽的行数（十字/岔口的正面特征）
    uint8  band_rows;                                   // 中断带长度（路面断掉的那一段有多长）
    uint8  above_rows;                                  // 中断带上方又看清的行数（>0 且够多 = 路口；=0 = 弯道）
    uint8  mark_rows;                                   // 路面内有黑色横条标记的行数（十字最硬特征）
    uint8  white_rows;                                  // 整行全白（96 格一个暗点都没有）的行数：十字/岔口必然出现
    int16  target_row;                                  // 前瞻点所在行
    int16  target_col;                                  // 前瞻点所在列
    float  kappa;                                       // 曲率（1/mm），右转为正
    float  delta_deg;                                   // 前轮转角（度）
    float  steer_out;                                   // 给舵机的偏移（正=向右 = 占空比减小，与 Servo_Set_Offset 同号）
    float  slow;                                        // 速度系数（0~1）
    float  ratio;                                       // 后轮差速比例
    float  lookahead_mm;                                // 本次前视距离
    float  target_x_mm;                                 // 前瞻点横向（相对车轴中心，右为正）
    float  target_y_mm;                                 // 前瞻点纵向
    float  near_off_mm;                                 // 最近 6 行的中线横向偏移
    float  half_width_mm;                               // 学习到的半赛道宽（mm），调参时看这个判断标定对不对
} pursuit_struct;

extern pursuit_struct pursuit_info;                     // 主循环用（显示、调试）
extern int16 pursuit_center_row[];                       // 每行中线列号（显示用），长度 = IPM_H

void  Pursuit_Init(void);                               // 复位状态机
void  Pursuit_Process(void);                            // 每帧调用：提取边界 -> 十字判定 -> 纯追踪
uint8 Pursuit_Get(pursuit_struct *out);                 // 控制中断里取快照（双缓冲，不会读到半截数据）
void  Pursuit_Show_On_TFT180(void);                     // 屏幕显示 K/D/C/S

#endif /* CODE_PURSUIT_REMAIN_H_ */
