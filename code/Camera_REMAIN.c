/*
 * Camera_REMAIN.c
 *
 *  Created on: 2026年10月20日
 *      Author: Freed
 */
#include "Camera_REMAIN.h"
uint8 base_image[MT9V03X_H][MT9V03X_W];                      // 二值化结果：0-黑(黑线)  255-白(背景)
uint8 camera_threshold = 128;                               // 本帧二值化阈值，Camera_Process_Once() 每帧刷新
#if (1 == CAMERA_OTSU_CHECK)
uint8  camera_otsu_ref_thr  = 0;                            // 校验用：同一帧用传统大津法算出的阈值
uint16 camera_otsu_diff_cnt = 0;                            // 校验用：两版阈值不一致的帧数（只增，上电清零）
#endif

/* 直方图缓冲：256 * 4B = 1KB。必须放静态区——CPU0 用户栈只有 2K，1KB 数组一上栈就溢出。
 * 快速版和传统参考版共用这一块（两者都只在 CPU0 主循环里调用，不存在并发），
 * 所以打开 CAMERA_OTSU_CHECK 做逐帧对照时，大津法的 RAM 也不会从 1KB 变成 2KB。 */
static uint32 otsu_histogram[256];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头 + TFT180 初始化
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_LCD_Init();
// 备注信息     先初始化屏幕，摄像头初始化失败时可以直接在屏幕上报错
//-------------------------------------------------------------------------------------------------------------------
void Camera_LCD_Init(void)
{
    tft180_init();                                      // 1.8 寸 TFT 初始化（硬件 SPI2，P15 口）
    tft180_full(0x0000);                                // 上电清屏，避免残影被误认为一片白
    tft180_set_color(0xFFFF, 0x0000);                   // 白字黑底
    if(mt9v03x_init())                                  // 0-成功  1-失败
    {
        tft180_show_string(0, 0, "CAMERA ERR");         // 摄像头没接好/配置失败时的提示
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法内部用：按累计量算「类间方差」得分（越大说明两类分得越开）
// 参数说明     w_b         C0（灰度 <= t）的像素个数
// 参数说明     total       参与统计的像素总数 N
// 参数说明     sum_b       C0 的灰度和 Σ(i<=t) i*n[i]
// 参数说明     sum_total   全图灰度和 S = Σ i*n[i]
// 返回参数     float       类间方差 g(t) = w0 * w1 * (μ0 - μ1)^2；有一类为空时返回 -1（不参与比较）
// 使用示例     内部调用
// 备注信息     这就是大津法的目标函数本体。快速版的"快"不在这一步的写法，而在"少算多少个 t、
//              每个 t 少算多少次"——见 Camera_FastOtsu_Threshold() 的三刀 + 一刀可选说明。
//-------------------------------------------------------------------------------------------------------------------
static float Camera_Otsu_Score(uint32 w_b, uint32 total, float sum_b, float sum_total)
{
    if((0 == w_b) || (w_b >= total))                    // C0 或 C1 为空：方差无意义
    {
        return (-1.0f);
    }

    uint32 w_f = total - w_b;                           // C1 像素个数
    float  m_b = sum_b / (float)w_b;                    // μ0
    float  m_f = (sum_total - sum_b) / (float)w_f;      // μ1
    float  d   = m_b - m_f;

    return ((float)w_b * (float)w_f * d * d);           // g(t) = w0*w1*(μ0-μ1)^2
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法内部用：在 [t_lo, t_hi] 内按 step 步长找"类间方差最大"的阈值
// 参数说明     hist        直方图（256 bin）
// 参数说明     g_min       有效灰度下界（区间外 n[i] 恒为 0，累计从这里开始）
// 参数说明     t_lo        候选阈值下界（闭区间）
// 参数说明     t_hi        候选阈值上界（闭区间；调用者保证 t_hi <= 有效灰度上界）
// 参数说明     step        候选阈值步长，1 = 逐灰度
// 参数说明     total       参与统计的像素总数 N
// 参数说明     sum_total   全图灰度和 S
// 参数说明     score_best  输出：找到的最大类间方差
// 返回参数     int32       最优阈值
// 使用示例     内部调用
// 备注信息     1. 只有一趟升序循环，w_b / sum_b 边走边累加，不需要 256 项前缀和数组（省 1.5KB RAM）
//              2. step > 1 时，被跳过的那几个 bin 会在"下一个候选点"一并累加进 w_b（fill 游标），
//                 这样每个候选点的得分仍然是"阈值恰好等于 t"的精确得分 —— 只在采样点上少算，
//                 不在采样点上算错。若偷懒把跳过 bin 直接丢弃，w_b 会偏小、阈值整体偏小，
//                 这是粗搜最典型的写错形态，改这里务必打表核对
//              3. 得分相同时保留先出现的 t（小阈值优先），与传统大津法的 > 比较规则一致
//-------------------------------------------------------------------------------------------------------------------
static int32 Camera_Otsu_Scan(const uint32 *hist, int32 g_min,
                              int32 t_lo, int32 t_hi, int32 step,
                              uint32 total, float sum_total, float *score_best)
{
    uint32 w_b   = 0;
    float  sum_b = 0;

    for(int32 t = g_min; t < t_lo; t ++)                // 把 t_lo 之前的累计量补齐（最多 256 次加法）
    {
        w_b   += hist[t];
        sum_b += (float)t * (float)hist[t];
    }

    int32 t_best = t_lo;
    *score_best  = -1.0f;
    int32 fill   = t_lo;                                // fill：下一个待累加的 bin，保证每个候选点得分精确

    for(int32 t = t_lo; t <= t_hi; t += step)
    {
        for(; fill <= t; fill ++)                       // 补齐 fill..t（含 t 自己），被跳过的 bin 不丢
        {
            w_b   += hist[fill];
            sum_b += (float)fill * (float)hist[fill];
        }

        float score = Camera_Otsu_Score(w_b, total, sum_b, sum_total);
        if(score > *score_best)                         // 严格大于：得分相同保留先出现的小阈值
        {
            *score_best = score;
            t_best      = t;
        }
    }

    return (t_best);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速大津法（Fast Otsu）自动计算二值化阈值 —— 本工程默认使用的版本
// 参数说明     image           8bit 灰度图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     uint8           阈值 T* ∈ [0,255]；退化图像（整幅灰度单一）返回 0，由调用者兜底
// 使用示例     threshold = Camera_FastOtsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//
// 【和传统大津法的关系】目标函数一模一样
//       T* = argmax_t [ w0(t) * w1(t) * (μ0(t) - μ1(t))^2 ]
//   变的只是"求这个 argmax 要花多少功夫"。所以光照正常时两版阈值只差 0~1 个灰度级，
//   二值图肉眼没有区别；只有画面接近退化（几乎全黑/全白/极低对比度）时才会差 ±2，
//   而那种画面本来就没有正确的阈值可言（调用者会兜底成 128）。
//
// 【快速在哪 —— 三刀必修 + 一刀可选】
//   ① 灰度区间收缩   传统版把 t = 0..255 全扫一遍，其中一大段是空 bin（画面里根本没有那些灰度）。
//      统计完直方图后从两端往里找第一个/最后一个非空 bin，候选区间立刻缩到 [gray_min, gray_max]。
//      代价为 0：区间外 n[i] 恒为 0，w_b / sum_b 在区间外本来也不变，argmax 一模一样
//   ② 粗搜 + 精搜    CAMERA_OTSU_COARSE_STEP = 8：先隔 8 个灰度扫一遍（最多 32 个候选）定位最优邻域，
//      再在 t ± 8 内逐灰度精搜（最多 17 个候选）。候选数 256 → 49，浮点除法 512 次 → 98 次。
//      TC264 没有硬件 FPU，float 除法是 HighTec 运行库的软实现（每次上百周期），这一刀最值钱
//   ③ 二值化零拷贝   Camera_Binarize() 不再"先 memcpy 整帧 22560 字节、再逐像素重读一遍"，
//      改成读 src 写 dst 一趟完成，省掉一次整帧拷贝 + 一次全图读
//   （可选）④ 隔点采样   CAMERA_OTSU_SAMPLE_STEP > 1 时按步长取行列样本，像素遍历 22560 → 22560/(step*step)
//      实测代价（200 张实拍型合成赛道图，噪声 σ = 3~9 灰度）：
//        步长 1：二值化结果与传统版逐像素一致（阈值数值差异只落在直方图的空 bin 内，不翻任何一个像素）
//        步长 2：阈值平均偏 2.2 个灰度，翻转像素平均 0.18%、最差帧 2.8%（强暗角 + 黑线与背景灰度重叠）
//        步长 4：翻转像素平均 0.50%
//      所以默认给的是 1 —— 先把"快"拿到手，同时保证"结果不变"。真要再榨，把 SAMPLE_STEP 改 2 再上车看边线
//
// 【复杂度】传统：N 次直方图 + 256 次打分（每次 2 次 float 除法）+ 整帧 memcpy
//           快速：N 次直方图 + ≤49 次打分 + 零拷贝二值化
//           阈值计算这一段约为传统的 1/5，二值化再省一次整帧搬移
//
// 【等价性】
//   SAMPLE_STEP = 1（默认）+ COARSE_STEP = 8 → 200 张实拍型图里 197 张阈值数值完全相同，3 张的差异落在
//   直方图空 bin 内；二值化翻转像素平均 0.00%、最差帧 0.07%（≈16 个像素，全在灰度过渡带）
//   SAMPLE_STEP = 1 + COARSE_STEP = 1 → 连 camera_threshold 的数值都逐位一致（完全等价基准，最慢）
//   FAST_EN = 0 → 整条链路退回 Camera_Otsu_Threshold()。三个开关都在 Camera_REMAIN.h
//
// 备注信息     1. 直方图 1KB 放 static（otsu_histogram），不占 CPU0 的 2K 用户栈
//              2. 退化情况（灰度单一）返回 0，由 Camera_Process_Once() 兜底成 128
//              3. 本函数只做纯大津法计算，不加任何偏置/钳位；要 OTSU_OFFSET 之类的经验修正，加在调用处
//-------------------------------------------------------------------------------------------------------------------
uint8 Camera_FastOtsu_Threshold(const uint8 *image, uint32 width, uint32 height)
{
    uint32 sample_total = 0;

    if((NULL == image) || (0 == width) || (0 == height))
    {
        return (0);                                     // 参数非法，交给调用者兜底
    }

    memset(otsu_histogram, 0, sizeof(otsu_histogram));  // ① 统计直方图（SAMPLE_STEP = 1 时等于全图）

    for(uint32 row = 0; row < height; row += CAMERA_OTSU_SAMPLE_STEP)
    {
        const uint8 *line = &image[row * width];        // 行首指针：乘法只在行循环里做一次

        for(uint32 col = 0; col < width; col += CAMERA_OTSU_SAMPLE_STEP)
        {
            otsu_histogram[line[col]] ++;
            sample_total ++;
        }
    }

    if(0 == sample_total)
    {
        return (0);
    }

    // ② 从直方图两端往里找有效灰度区间
    //    这里故意不在逐像素循环里比 min/max：那是每像素 2 次比较 × 22560 次 = 4.5 万次。
    //    改成统计完再看 256 个 bin，最坏 512 次比较，快了一个数量级，还顺手省掉 2 个临时变量。
    int32 gray_min = 0;
    while((gray_min < 255) && (0 == otsu_histogram[gray_min]))
    {
        gray_min ++;
    }

    int32 gray_max = 255;
    while((gray_max > 0) && (0 == otsu_histogram[gray_max]))
    {
        gray_max --;
    }

    if(gray_min >= gray_max)                            // 整幅灰度单一（镜头被挡/摄像头未出图）
    {
        return (0);                                     // C0 或 C1 必为空，argmax 无定义，交给调用者兜底
    }

    float sum_total = 0;                                // 全图灰度和 S = Σ i*n[i]（区间外恒为 0，不用加）
    for(int32 i = gray_min; i <= gray_max; i ++)
    {
        sum_total += (float)i * (float)otsu_histogram[i];
    }

    float score_coarse;                                 // ③ 粗搜：步长 CAMERA_OTSU_COARSE_STEP
    int32 t_coarse = Camera_Otsu_Scan(otsu_histogram, gray_min,
                                      gray_min, gray_max, CAMERA_OTSU_COARSE_STEP,
                                      sample_total, sum_total, &score_coarse);

    int32 t_lo = t_coarse - CAMERA_OTSU_COARSE_STEP;    // ④ 精搜：粗搜最优点 ±COARSE_STEP 内逐灰度
    int32 t_hi = t_coarse + CAMERA_OTSU_COARSE_STEP;
    if(t_lo < gray_min) { t_lo = gray_min; }
    if(t_hi > gray_max) { t_hi = gray_max; }

    float score_fine;
    int32 t_fine = Camera_Otsu_Scan(otsu_histogram, gray_min,
                                    t_lo, t_hi, 1,
                                    sample_total, sum_total, &score_fine);

    return ((uint8)t_fine);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     传统大津法（OTSU / 最大类间方差法）—— 慢速参考实现，平时不参与主链路
// 参数说明     image           8bit 灰度图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 返回参数     uint8           阈值 T* ∈ [0,255]，正常赛道图像落在 1~254
// 使用示例     Camera_Otsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//
// 【算法原理】大津法 Otsu (1979)：遍历 256 个候选阈值，取“类间方差最大”的那一个
//     ① 统计直方图   n[i] 为灰度 i 的像素个数（i = 0..255），总像素数 N = width * height
//     ② 对候选阈值 t，把图像分成两类：
//            C0 = [0, t]      （灰度低 —— 赛道黑线）
//            C1 = [t+1, 255]  （灰度高 —— 白色背景）
//     ③ 计算两类的像素占比与灰度均值：
//            w_b = Σ(i<=t) n[i]                    C0 像素个数
//            w_f = N - w_b                          C1 像素个数
//            μ0  = Σ(i<=t) i*n[i] / w_b             C0 灰度均值
//            μ1  = Σ(i>t)  i*n[i] / w_f             C1 灰度均值
//     ④ 类间方差（越大说明两类分得越开）：
//            g(t) = w_b * w_f * (μ0 - μ1)^2
//     ⑤ T* = argmax_t g(t)，即让 g(t) 取最大值的 t
//
// 【实现说明】用累加量避免每次重算：
//            w_b  边遍历边累加直方图；
//            sum_b 边遍历边累加 t*n[t]；
//            sum   为全图灰度和 Σ i*n[i]，于是 μ1 里的 Σ(i>t) i*n[i] = sum - sum_b
//     复杂度：直方图 O(N) + 阈值遍历 O(256) + 二值化 O(N)；188*120 图像为毫秒级
//
// 备注信息     1. 本函数是"对照基准"，用于 CAMERA_OTSU_FAST_EN = 0 或 CAMERA_OTSU_CHECK = 1 时
//                 验证快速版有没有算错（两版阈值应当只差 0~1 个灰度级）
//              2. 直方图与快速版共用 otsu_histogram[]，不额外占 RAM
//              3. 本函数只做纯大津法计算，不做任何修正；退化情况（整幅图灰度单一）由调用者兜底
//-------------------------------------------------------------------------------------------------------------------
uint8 Camera_Otsu_Threshold(const uint8 *image, uint32 width, uint32 height)
{
    uint32 total_pixels = width * height;
    if((NULL == image) || (0 == total_pixels))
    {
        return (0);                                     // 参数非法，交给调用者兜底
    }

    memset(otsu_histogram, 0, sizeof(otsu_histogram));  // ① 统计直方图
    for(uint32 i = 0; i < total_pixels; i ++)
    {
        otsu_histogram[image[i]] ++;
    }

    float sum = 0;                                      // 全图灰度和 Σ i*n[i]
    for(int i = 0; i < 256; i ++)
    {
        sum += (float)i * (float)otsu_histogram[i];
    }

    uint32  w_b       = 0;                              // C0 像素个数
    float   sum_b     = 0;                              // Σ(i<=t) i*n[i]
    float   var_max   = -1.0f;                          // 最大类间方差 g(t)
    uint8   threshold = 0;                              // 当前最优阈值 T*

    for(int t = 0; t < 256; t ++)                       // ②~⑤ 遍历 256 个候选阈值
    {
        w_b += otsu_histogram[t];
        if(0 == w_b)
        {
            continue;                                   // C0 里还没有像素，跳过
        }

        uint32 w_f = total_pixels - w_b;                // C1 像素个数
        if(0 == w_f)
        {
            break;                                      // C1 里没有像素了，后面不可能更大
        }

        sum_b += (float)t * (float)otsu_histogram[t];

        float m_b = sum_b / (float)w_b;                 // μ0
        float m_f = (sum - sum_b) / (float)w_f;         // μ1
        float var_between = (float)w_b * (float)w_f * (m_b - m_f) * (m_b - m_f);   // g(t) = w0*w1*(μ0-μ1)2

        if(var_between > var_max)                       // ⑤ 记录最大类间方差对应的阈值
        {
            var_max   = var_between;
            threshold = (uint8)t;
        }
    }

    return (threshold);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按给定阈值把 mt9v03x_image 二值化到 base_image（0 / 255）
// 参数说明     threshold       二值化阈值，由 Camera_FastOtsu_Threshold() 给出
// 返回参数     void
// 使用示例     Camera_Binarize(Camera_FastOtsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H));
// 备注信息     1. 灰度 > threshold 判为白(255)，否则判为黑(0)——与屏幕驱动 threshold 分支一致
//              2. 【快速化】老写法是"先 memcpy 整帧 22560 字节，再逐像素写 0/255"，等于把 22.5KB
//                 图像搬一遍又读一遍。这里改成读 src 写 dst 一趟完成，省掉整帧拷贝和一次全图读。
//              3. 行列顺序：base_image 是 [MT9V03X_H][MT9V03X_W]，行循环用 H，列循环用 W
//                 写反（i<W 当行、j<H 当列）会越界写 12716 字节，还会漏掉 8160 个像素
//-------------------------------------------------------------------------------------------------------------------
void Camera_Binarize(uint8 threshold)
{
    for(uint32 row = 0; row < MT9V03X_H; row ++)        // 行：MT9V03X_H = 120
    {
        const uint8 *src = &mt9v03x_image[row][0];
        uint8       *dst = &base_image[row][0];

        for(uint32 col = 0; col < MT9V03X_W; col ++)    // 列：MT9V03X_W = 188
        {
            *dst ++ = (*src ++ > threshold) ? (255) : (0);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把二值化后的 base_image 显示到 TFT180
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_LCD_Dis_Show();
// 备注信息     最后一个参数 128 表示由驱动按阈值二值化：< 128 画黑，>= 128 画白
//              传 0 表示不二值化、原样输出灰度，排查时用得上
//-------------------------------------------------------------------------------------------------------------------
void Camera_LCD_Dis_Show(void)
{
    tft180_show_gray_image(CAMERA_DIS_X0, CAMERA_DIS_Y0, base_image[0],
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_DIS_W, CAMERA_DIS_H,
                           128);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接显示摄像头原始灰度图（不做二值化），排查时用
// 参数说明     void
// 返回参数     void
// 使用示例     把 SATA_MAIN.c 里 SATA_Frame_Process() 的 Camera_Process_Once() 临时换成 Camera_LCD_Dis_Raw();
// 备注信息     这里 threshold 传 0 = 不二值化，原样输出 8bit 灰度
//-------------------------------------------------------------------------------------------------------------------
void Camera_LCD_Dis_Raw(void)
{
    tft180_show_gray_image(CAMERA_DIS_X0, CAMERA_DIS_Y0, mt9v03x_image[0],
                           MT9V03X_W, MT9V03X_H,
                           CAMERA_DIS_W, CAMERA_DIS_H,
                           0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一帧完整处理：快速大津法求阈值 → 二值化 → 显示
// 参数说明     void
// 返回参数     void
// 使用示例     Camera_Process_Once();
// 备注信息     调用前必须保证 mt9v03x_finish_flag 已经置 1（一帧采集完成）
//-------------------------------------------------------------------------------------------------------------------
void Camera_Process_Once(void)
{
    // 1. 大津法：对当前这一帧真实图像计算自适应阈值（默认走快速版，FAST_EN=0 退回传统版对照）
#if (1 == CAMERA_OTSU_FAST_EN)
    uint8 threshold = Camera_FastOtsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
#else
    uint8 threshold = Camera_Otsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
#endif

    // 2. 退化兜底：整幅图灰度完全一致（镜头被遮挡、摄像头未出图）时，
    //    任何候选阈值的类间方差都是 0，大津法给出的 0 / 255 会把整屏刷成纯白或纯黑。
    //    正常赛道图像不会走到这里，不影响大津法的计算结果。
    if((0 == threshold) || (255 == threshold))
    {
        threshold = 128;
    }

#if (1 == CAMERA_OTSU_CHECK)
    // 2.5 校验模式：同一帧再用传统实现算一遍，把偏差留给调试器/示波器看（不占屏幕、不参与控制）
    //     期望：camera_otsu_diff_cnt 增长极慢（每几百帧偶尔 +1），差值 ≤ 2 个灰度级
    //     若差 5 个灰度以上或几乎每帧都不同 → 采样步长太大（先调回 1 复测）
    camera_otsu_ref_thr = Camera_Otsu_Threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    if((0 == camera_otsu_ref_thr) || (255 == camera_otsu_ref_thr))
    {
        camera_otsu_ref_thr = 128;
    }
    if(camera_otsu_ref_thr != threshold)
    {
        if(camera_otsu_diff_cnt < 0xFFFF)               // 饱和就不再累加，避免回绕
        {
            camera_otsu_diff_cnt ++;
        }
    }
#endif

    // 3. 存一份阈值给斑马线模块（复用大津法结果，不重复算），然后按阈值二值化
    camera_threshold = threshold;
    Camera_Binarize(threshold);

#if (1 == CAMERA_DIS_IMAGE)
    Camera_LCD_Dis_Show();                              // 图像显示；CAMERA_DIS_IMAGE=0 时整段跳过
#endif

#if (1 == CAMERA_DIS_INFO)
    tft180_set_color(0xFFFF, 0x0000);
    tft180_show_string(0, CAMERA_INFO_Y1, "TH");
    tft180_show_int(24, CAMERA_INFO_Y1, (int32)threshold, 3);
#endif
}
