/*
 * Camera_REMAIN.h
 *
 *  Created on: 2026年10月20日
 *      Author: Freed
 */

#ifndef CODE_CAMERA_REMAIN_H_
#define CODE_CAMERA_REMAIN_H_

#include "zf_common_headfile.h"

/* ================================ 显示参数 ================================ */
/* 总钻风原图 188*120，TFT180 竖屏可用区域 128*160                       */
/* 显示宽度不能超过 tft180_width_max，显示高度不能超过 tft180_height_max   */
/* 显示面积：按 188:120 等比缩放，112 x 71（112*120/188 = 71.5 → 71）
 *
 * 【为什么宽度绝对不能 ≥128 —— 这是"图像崩坏"的真凶】
 *   tft180_set_region() 在竖屏分支里给 ST7735 写窗口时是带固定偏移的：
 *       x: 0x00, (x1 + 2), 0x00, (x2 + 2)
 *       y: 0x00, (y1 + 1), 0x00, (y2 + 1)
 *   x2 = X0 + W - 1。取 W = 128、X0 = 0 → x2 = 127 → x2 + 2 = 129，
 *   而 129 已经超过面板列地址上限 127，控制器的列结束寄存器被写坏，
 *   于是每写一行就多走 2 个像素，逐行横向累计错位 —— 屏幕上就是斜着撕裂的"崩坏"。
 *   限制条件：X0 + W - 1 + 2 ≤ 127  →  X0 + W ≤ 126
 *             Y0 + H - 1 + 1 ≤ 159  →  Y0 + H ≤ 159
 *   126x80（原始默认值）刚好卡在 X0+W = 126 上，所以它一直是好的。
 *
 * 112x71 = 7952 像素/帧，比 126x80(10080) 还小两成，SPI 传输更快，
 * 也不会再碰窗口边界。                                                          */
#define CAMERA_DIS_W            (112)                   // 显示宽度：X0 + W ≤ 126
#define CAMERA_DIS_H            (71)                    // 显示高度：等比 112*120/188
#define CAMERA_DIS_X0           (8)                     // 图像左上角（水平居中：(128-112)/2）
#define CAMERA_DIS_Y0           (2)
#define CAMERA_DIS_INFO         (0)                     // 0-屏幕只放图像：调试数字/字母一律不画（本轮需求）

/* 屏幕下方信息区行坐标（8x16 字体，最后一行 y+15 必须小于 tft180_height_max=160） */
/* 图像总开关：1-铺满整个屏幕显示二值化图像（含绿中线/蓝边界叠加）
                0-不显示图像，只显示参数。省掉每帧 128x160 = 20480 个像素的刷屏时间，
                  帧率和控制周期都会更宽裕（调试期改成 0，平时看画面用 1）          */
#define CAMERA_DIS_IMAGE        (1)

/* 参数区起点：显示图像时整屏都是图像，参数行会被推到屏幕外（所以 CAMERA_DIS_INFO 必须为 0）；
               不显示图像时直接从屏幕顶部开始（y=0） */
#define CAMERA_INFO_TOP         ((1 == CAMERA_DIS_IMAGE) ? (CAMERA_DIS_H) : (0))

#define CAMERA_INFO_Y1          (CAMERA_INFO_TOP + 2)      // 阈值 TH
#define CAMERA_INFO_Y2          (CAMERA_INFO_TOP + 18)     // 左右基点 BP
#define CAMERA_INFO_Y3          (CAMERA_INFO_TOP + 34)     // 基线斜率 K
#define CAMERA_INFO_Y4          (CAMERA_INFO_TOP + 50)     // 中线 C / 偏差 E
#define CAMERA_INFO_Y5          (CAMERA_INFO_TOP + 68)     // 轮速 V/R（6x8 小字）
#define CAMERA_INFO_Y6          (CAMERA_INFO_TOP + 84)     // 前馈标定结果 gL/gR bL/bR（6x8 小字）
#if (1 == CAMERA_DIS_IMAGE) && (1 == CAMERA_DIS_INFO)
#error "整屏显示图像时没有信息区：请把 CAMERA_DIS_INFO 置 0（只显示图像），或把 CAMERA_DIS_IMAGE 置 0（只显示参数）"
#endif
/* 窗口偏移守卫：ST7735 竖屏分支的 x 偏移是 +2、y 偏移是 +1，越界不会报错，
 * 只会把列/行结束寄存器写坏，屏幕上表现为逐行斜着错位（"图像崩坏"）。
 * 126x80 是原始默认值，正好卡在上限；想放大请先改 X0/Y0，别再超过这两个上限。 */
#if ((CAMERA_DIS_X0 + CAMERA_DIS_W) > 126) || ((CAMERA_DIS_Y0 + CAMERA_DIS_H) > 159)
#error "显示窗口超出 ST7735 地址范围：必须满足 CAMERA_DIS_X0 + CAMERA_DIS_W <= 126 且 CAMERA_DIS_Y0 + CAMERA_DIS_H <= 159"
#endif
/* ================================ 大津法参数 ================================ */
/* 二值化阈值一律由大津法（OTSU，最大类间方差法）给出，快速版是默认路径。
 * 下面四个开关决定"有多快、有多准"，动完必须重新上车看阈值和边线的稳定性。    */

/* 快速大津法总开关
 *   1-Camera_FastOtsu_Threshold()（默认：采样 + 区间收缩 + 粗搜精搜）
 *   0-Camera_Otsu_Threshold()（原始逐像素、逐灰度版本，留作对拍基准）        */
#define CAMERA_OTSU_FAST_EN     (1)

/* 直方图采样步长（行、列同一个步长）
 * 原理：大津法只看灰度分布的"形状"，不看像素长在哪。均匀隔点取样相当于把整条直方图
 *       按同一比例缩小，argmax 位置基本不动 —— 但取样越狠，阈值抖得越明显。
 * 实测代价（200 张实拍型合成赛道图，噪声 σ = 3~9 灰度）：
 *   1-全图 188*120 = 22560 点，二值化结果与传统大津法逐像素一致（默认）
 *   2-5640 点，直方图耗时约 1/4；阈值平均偏 2.2 灰度，翻转像素平均 0.18%、最差帧 2.8%
 *   4-3528 点，翻转像素平均 0.50%，边线开始有毛刺，只在算力实在不够时用
 * 提示：先把 COARSE_STEP 的便宜占了（那一刀不影响结果），再考虑动这里。          */
#define CAMERA_OTSU_SAMPLE_STEP (1)

/* 候选阈值粗搜步长（灰度单位）
 * 传统版要把 t = 0..255 全算一遍：256 个候选 × 每个候选 2 次 float 除法。
 * 这里先按 step 粗搜，再在最优点 ±step 邻域内逐灰度精搜：
 * 候选数 ≈ 256/step + 2*step，step = 8 时 49 个候选、98 次除法（原 512 次）。
 * TC264 没有硬件浮点单元，float 除法走软件实现，这一刀比采样更值钱。
 *   1-退化成逐灰度精搜（配合 SAMPLE_STEP = 1 与传统版逐位一致）
 *   8-默认                                                                  */
#define CAMERA_OTSU_COARSE_STEP (8)

/* 两版对拍开关（开发期专用，出车前必须置 0）
 *   1-每帧额外跑一遍传统大津法，结果写 camera_otsu_ref_thr；与快速版不同则
 *     camera_otsu_diff_cnt 自增。不屏显、不参与控制，只用来验证"改快了以后阈值没跑偏"
 *   0-不编译对拍代码（默认）                                                 */
#define CAMERA_OTSU_CHECK       (0)

/* 参数守卫：写错了不会报错，只会在车上表现成"阈值莫名偏移"，所以拦在编译期 */
#if (CAMERA_OTSU_SAMPLE_STEP < 1) || (CAMERA_OTSU_SAMPLE_STEP > 8)
#error "CAMERA_OTSU_SAMPLE_STEP 必须是 1~8 的整数（1 = 全图逐像素，2 = 隔点采样）"
#endif
#if (CAMERA_OTSU_COARSE_STEP < 1) || (CAMERA_OTSU_COARSE_STEP > 64)
#error "CAMERA_OTSU_COARSE_STEP 必须是 1~64 的整数（1 = 逐灰度精搜，与传统大津法一致）"
#endif
/* 想验证"改造没引入逻辑错误"，把 SAMPLE_STEP=1、COARSE_STEP=1、CHECK=1 跑一段：
 * 这组配置下快速版与传统版逐位一致，camera_otsu_diff_cnt 必须一直是 0。
 * （验证完记得把三个值改回来，逐像素 + 逐灰度是最慢的组合）                  */

/* base_image：二值化结果 0-黑(赛道黑线)  255-白(背景)                     */
extern uint8 base_image[MT9V03X_H][MT9V03X_W];
extern uint8 camera_threshold;                              // 本帧算出的阈值（斑马线模块要用，别重复算）
#if (1 == CAMERA_OTSU_CHECK)
extern uint8  camera_otsu_ref_thr;                          // 对拍用：同一帧传统大津法算出的阈值
extern uint16 camera_otsu_diff_cnt;                         // 对拍用：两版阈值不一致的帧数（上电清零，只增）
#endif

/* ================================ 接口声明 ================================ */
void    Camera_LCD_Init          (void);                                            // 摄像头 + TFT180 初始化

/* 二值化阈值一律由大津法计算，不使用固定阈值；Camera_Process_Once() 默认走快速版 */
uint8   Camera_FastOtsu_Threshold(const uint8 *image, uint32 width, uint32 height);  // 快速大津法求阈值（默认路径）
uint8   Camera_Otsu_Threshold    (const uint8 *image, uint32 width, uint32 height);  // 传统大津法求阈值（对拍基准）
void    Camera_Binarize          (uint8 threshold);                                 // 按阈值二值化到 base_image

void    Camera_LCD_Dis_Show      (void);                                            // 显示 base_image 二值化图像
void    Camera_LCD_Dis_Raw       (void);                                            // 显示原始灰度图（排查用）
void    Camera_Process_Once      (void);                                            // 大津法阈值 + 二值化 + 显示（一帧完整流程）

#endif /* CODE_CAMERA_REMAIN_H_ */
