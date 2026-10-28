/*
 * IPM_REMAIN.c
 *
 *  逆透视变换：摄像头图像 -> 地面俯视图（指针查表实现）
 *
 *  核心思想只有一句话：地面上每个格点，上电时算一次"它落在摄像头第几行第几列"，
 *  把那个字节的地址直接存下来，以后每帧只做一次指针解引用。
 *  所以运行时没有三角函数、没有矩阵乘法、没有除法，连"这个点出界了没"都不用判。
 *
 *  数学推导（针孔模型，车辆坐标系 X右 Y前 Z上）：
 *      相机光心 (0, 0, h)，俯角 theta（向下为正）
 *      相机三轴：x_c = (1, 0, 0)               向右
 *                y_c = (0, -sin, -cos)        图像向下
 *                z_c = (0,  cos, -sin)        光轴向前下
 *      地面点 (X, Y, 0) 在相机系下：
 *              x_c = X
 *              y_c = h*cos - Y*sin
 *              z_c = Y*cos + h*sin
 *      投到像平面（f 为焦距，单位像素）：
 *              u = cx + f * x_c / z_c
 *              v = cy + f * y_c / z_c
 *      z_c 就是"该点有多远"，只要 z_c 很小（接近地平线）就判为无效点。
 *
 *  为什么这里是"正向"畸变而不是 undistort：
 *      我们已知地面点在真实世界的位置，算出的是"理想针孔相机"会把它拍在第几像素；
 *      而真实镜头实际把它拍在 Distort(理想像素) 处。要采样到那条边线的像素，
 *      本来就该去查 Distort(理想像素)，方向和 Brown-Conrady 正向模型完全一致，
 *      所以不需要迭代求逆 —— 这一步能整个折进上电建的表里。
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */
#include "IPM_REMAIN.h"

#define IPM_PI_F                (3.14159265f)

/* 指针表：每个格点指向"源图里那个字节"，出界格点指向 ipm_white_pixel。
 * 64 x 96 x 4B = 24 KB（下标表是 12 KB），换来的收益见 IPM_Process() 注释。
 * dsram1 里 .bss 目前只用到 0x60006938 附近，到 heap 起点 0x6001A600 还有 80 KB 余量。 */
static const uint8 *ipm_lut[IPM_H][IPM_W];
static const uint8 *ipm_src = NULL;                     // 当前指针表绑定在哪张源图上

uint8  ipm_white_pixel = IPM_WHITE;                     // 出界格点统一指向这里
uint8  ipm_row_valid[IPM_H];                            // 每行有效格点数（0 = 整行出界）
ipm_struct ipm_info;

uint8  ipm_image[IPM_H][IPM_W];

static uint8 ipm_inited = 0;                            // IPM_Init() 是否已经跑过
static uint8 ipm_geo_ok = 0;                            // 只由几何(安装参数)决定的那一半判定

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     算相机内参（fx fy cx cy 像素）
// 参数说明     fx/fy/cx/cy      输出，单位像素
// 返回参数     void
// 使用示例     内部函数
// 备注信息     1. 宏给负数表示"由 IPM_CAM_FOV_H_DEG 自动算"，默认值下与改造前的 focal 完全相同
//              2. 有 OpenCV 标定结果时直接填 IPM_CAM_FX_PX / FY / CX / CY，比猜 FOV 准
//-------------------------------------------------------------------------------------------------------------------
static void IPM_Compute_Intrinsics(float *fx, float *fy, float *cx, float *cy)
{
    float fov_rad  = IPM_CAM_FOV_H_DEG * IPM_PI_F / 180.0f;
    float focal_auto = (((float)MT9V03X_W) * 0.5f) / tanf(fov_rad * 0.5f);

    *fx = (IPM_CAM_FX_PX > 0.0f) ? IPM_CAM_FX_PX : focal_auto;
    *fy = (IPM_CAM_FY_PX > 0.0f) ? IPM_CAM_FY_PX : (*fx);
    *cx = (IPM_CAM_CX_PX >= 0.0f) ? IPM_CAM_CX_PX : ((((float)MT9V03X_W) * 0.5f) - 0.5f);
    *cy = (IPM_CAM_CY_PX >= 0.0f) ? IPM_CAM_CY_PX : ((((float)MT9V03X_H) * 0.5f) - 0.5f);
}

#if (1 == IPM_UNDIS_EN)
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Brown-Conrady 正向畸变：理想像素 -> 实际成像像素
// 参数说明     u/v             输入输出，单位像素（就地改写）
// 返回参数     void
// 使用示例     内部函数，只在 IPM_UNDIS_EN=1 时参与建表
// 备注信息     1. 与 OpenCV / RUBO-UDIPM 的畸变模型同式：先归一化，径向 + 切向，再乘回焦距
//              2. 我们没有做"逆"这一步：真值在地面上，本来就要往畸变后的位置去采样
//-------------------------------------------------------------------------------------------------------------------
static void IPM_Distort(float *u, float *v)
{
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float xn = 0.0f;
    float yn = 0.0f;
    float r2 = 0.0f;
    float radial = 0.0f;
    float xt = 0.0f;
    float yt = 0.0f;

    IPM_Compute_Intrinsics(&fx, &fy, &cx, &cy);

    xn = ((*u) - cx) / fx;
    yn = ((*v) - cy) / fy;

    r2     = (xn * xn) + (yn * yn);
    radial = 1.0f + (IPM_DIST_K1 * r2) + (IPM_DIST_K2 * r2 * r2) + (IPM_DIST_K3 * r2 * r2 * r2);

    xt = (xn * radial) + (2.0f * IPM_DIST_P1 * xn * yn) + (IPM_DIST_P2 * (r2 + (2.0f * xn * xn)));
    yt = (yn * radial) + (IPM_DIST_P1 * (r2 + (2.0f * yn * yn))) + (2.0f * IPM_DIST_P2 * xn * yn);

    *u = (xt * fx) + cx;
    *v = (yt * fy) + cy;
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     建表：把 96 x 64 个地面格点映射成源图像素指针
// 参数说明     image           源图首地址（本工程 = base_image[0]，二值化图）
// 返回参数     void
// 使用示例     内部函数，由 IPM_Process() 在"换缓冲"时自动调用
// 备注信息     1. 只在第一帧（或换缓冲）时跑一次，6144 次浮点运算，随便多久
//              2. 顺手统计 invalid_cnt / ipm_row_valid[]，供自检和整行跳过用
//-------------------------------------------------------------------------------------------------------------------
static void IPM_Bind(const uint8 *image)
{
    float  pitch_rad = IPM_CAM_PITCH_DEG * IPM_PI_F / 180.0f;
    float  sin_p     = sinf(pitch_rad);
    float  cos_p     = cosf(pitch_rad);
    float  cam_h     = IPM_CAM_HEIGHT_MM;
    float  fx        = 0.0f;
    float  fy        = 0.0f;
    float  cx        = 0.0f;
    float  cy        = 0.0f;
    uint16 invalid   = 0;
    uint8  valid_row = 0;
    uint8  r         = 0;
    uint8  c         = 0;

    IPM_Compute_Intrinsics(&fx, &fy, &cx, &cy);

    for(r = 0; r < IPM_H; r++)
    {
        float  y_ground  = IPM_Y_NEAR_MM + (((float)r) * ipm_info.dy_mm);
        float  z_c       = (y_ground * cos_p) + (cam_h * sin_p);
        float  y_c       = (cam_h * cos_p) - (y_ground * sin_p);
        uint8  row_valid = 0;
        int16  u_prev    = -1;                          /* 本行上一格的列号（单调守卫用） */

        for(c = 0; c < IPM_W; c++)
        {
            float x_ground = (((float)c) - (((float)(IPM_W - 1)) * 0.5f)) * ipm_info.dx_mm;
            float u_px     = 0.0f;
            float v_px     = 0.0f;
            int16 u_i      = 0;
            int16 v_i      = 0;

            ipm_lut[r][c] = IPM_FILL_PTR;               /* 先按出界处理，下面通了再改 */

            if(z_c <= 1.0f)                             /* 该点在地平线上，除不得 */
            {
                continue;
            }

            u_px = cx + ((fx * x_ground) / z_c);
            v_px = cy + ((fy * y_c) / z_c);

#if (1 == IPM_UNDIS_EN)
            IPM_Distort(&u_px, &v_px);                  /* 理想像素 -> 实际成像像素 */
#endif

            if((u_px < -0.5f) || (u_px > (((float)MT9V03X_W) - 0.5f)) ||
               (v_px < -0.5f) || (v_px > (((float)MT9V03X_H) - 0.5f)))
            {
                continue;                               /* 该点落在摄像头画面外 */
            }

            u_i = (int16)(u_px + 0.5f);
            v_i = (int16)(v_px + 0.5f);

            /* 横向单调守卫：同一行里列号只许往前走，不许回头。
               畸变多项式在"标定有效半径"之外会翻折（radial 过零后再反号），
               翻折后的格点虽然数值上落在画面内，采到的却是镜像像素，
               在俯视图上会凭空长出一条反方向的假边线。
               实测（100 度广角、fx≈79px）：纯径向 k1 到 -0.4 都不翻折；
               把别人 320x240 标定的五系数照搬过来（k2=+0.284 / k3=-0.109）就会翻折 48 个格点。 */
            if(u_i < u_prev)
            {
                continue;
            }
            u_prev = u_i;

            if(u_i < 0)                u_i = 0;
            if(u_i > (MT9V03X_W - 1))  u_i = (MT9V03X_W - 1);
            if(v_i < 0)                v_i = 0;
            if(v_i > (MT9V03X_H - 1))  v_i = (MT9V03X_H - 1);

            ipm_lut[r][c] = &image[(v_i * MT9V03X_W) + u_i];
            row_valid ++;
        }

        ipm_row_valid[r] = row_valid;
        invalid         += (uint16)(IPM_W - row_valid);
        if(0 != row_valid)
        {
            valid_row ++;
        }
    }

    ipm_info.invalid_cnt   = invalid;
    ipm_info.valid_row_cnt = valid_row;

    /* 参数自检：出界格点超过 1/4 -> 俯视图大部分是空白，参数肯定配错了 */
    ipm_info.ok = ipm_geo_ok;
    if((invalid * 4) >= (IPM_H * IPM_W))
    {
        ipm_info.ok = 0;
    }

    ipm_src = image;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     逆透视初始化（上电调一次）
// 参数说明     void
// 返回参数     void
// 使用示例     IPM_Init();
// 备注信息     1. 只需要 IPM_CAM_HEIGHT_MM / IPM_CAM_PITCH_DEG / IPM_CAM_FOV_H_DEG 三个参数，
//                 换镜头、改安装高度都只改宏，不重新标定
//              2. 这里只算与源图无关的几何量（内参、格点间距、地平线行）；
//                 真正建指针表在 IPM_Process() 第一次被调用时做，因为那时才知道源图地址
//              3. 改了安装参数后重新调一次 IPM_Init() 会强制重建表
//-------------------------------------------------------------------------------------------------------------------
void IPM_Init(void)
{
    float pitch_rad = IPM_CAM_PITCH_DEG * IPM_PI_F / 180.0f;
    float fx        = 0.0f;
    float fy        = 0.0f;
    float cx        = 0.0f;
    float cy        = 0.0f;
    float horizon   = 0.0f;
    uint8 r         = 0;

    IPM_Compute_Intrinsics(&fx, &fy, &cx, &cy);

    ipm_info.focal_px     = fx;
    ipm_info.dx_mm        = (2.0f * IPM_X_HALF_MM) / ((float)(IPM_W - 1));
    ipm_info.dy_mm        = (IPM_Y_FAR_MM - IPM_Y_NEAR_MM) / ((float)(IPM_H - 1));
    ipm_info.undis_en     = (uint8)IPM_UNDIS_EN;
    ipm_info.invalid_cnt  = 0;
    ipm_info.valid_row_cnt = 0;

    /* 地平线所在行：Y 趋于无穷时 v = cy - f*tan(theta)。
       这一行以上没有地面，行号越小说明相机压得越平（俯角越小）。 */
    horizon = cy - (fx * tanf(pitch_rad));
    if(horizon < 0.0f)
    {
        ipm_info.horizon_row = 0;
    }
    else if(horizon > ((float)MT9V03X_H))
    {
        ipm_info.horizon_row = (uint8)MT9V03X_H;
    }
    else
    {
        ipm_info.horizon_row = (uint8)(horizon + 0.5f);
    }

    /* 地平线跑到画面里太靠上（<5 行）或者跑到画面外（>115 行）-> 俯角明显不对 */
    ipm_geo_ok = 1;
    if(((float)ipm_info.horizon_row < 5.0f) || ((float)ipm_info.horizon_row > (((float)MT9V03X_H) - 5.0f)))
    {
        ipm_geo_ok = 0;
    }

    ipm_info.ok = ipm_geo_ok;

    /* 俯视图先全填白色，第一帧来之前就是一张白纸 */
    for(r = 0; r < IPM_H; r++)
    {
        ipm_row_valid[r] = 0;
        memset(&ipm_image[r][0], IPM_WHITE, IPM_W);
    }

    ipm_src    = NULL;                                  /* 强制下一次 IPM_Process() 重建指针表 */
    ipm_inited = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一帧逆透视重采样
// 参数说明     image           二值化图像首地址（base_image[0]，0=黑 255=白）
// 返回参数     uint8           0-成功 1-参数无效/空指针
// 使用示例     IPM_Process(base_image[0]);
// 备注信息     1. 热循环就是 dst[c] = *lut[c]，每个格点 1 次取指针 + 1 次取字节 + 1 次存字节；
//                 改造前是"取 uint16 下标 + 比较 + 分支 + 源图基址加下标 + 取字节"，
//                 每格点省 1 次加法、去掉 1 个数据相关分支（出界比例高的行最吃亏）
//              2. 整行出界的行（地平线以上）直接 memset 铺白，省掉 96 次解引用
//              3. 6144 个格点，每格点比改造前少 1 次地址加法和 1 个数据相关分支；
//                 量级是几十微秒（不是数量级提速），真正的收益是把去畸变从"每帧多扫
//                 两万多个点"变成上电时多算一遍，运行时 0 成本
//-------------------------------------------------------------------------------------------------------------------
uint8 IPM_Process(const uint8 *image)
{
    uint8 r = 0;

    if(NULL == image)
    {
        return (1);
    }

    if(0 == ipm_inited)
    {
        IPM_Init();                                     /* 忘了初始化也能自己补上 */
    }

    if(image != ipm_src)
    {
        IPM_Bind(image);                                /* 第一帧 / 换缓冲：建表，之后不再进这里 */
    }

    if(0 == ipm_info.ok)
    {
        return (1);
    }

    for(r = 0; r < IPM_H; r++)
    {
        const uint8 *const *lut = &ipm_lut[r][0];
        uint8              *dst = &ipm_image[r][0];
        uint8               c   = 0;

#if (1 == IPM_SKIP_EMPTY_ROW)
        if(0 == ipm_row_valid[r])
        {
            memset(dst, IPM_WHITE, IPM_W);              /* 整行在画面外：铺白，不查表 */
            continue;
        }
#endif

        for(c = 0; c < IPM_W; c++)
        {
            dst[c] = *lut[c];                           /* 出界格点在建表时已指向白哨兵，不用判 */
        }
    }

    return (0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示俯视图（调试用）
// 参数说明     void
// 返回参数     void
// 使用示例     IPM_Show_On_TFT180();
// 备注信息     1. 只有 IPM_DIS_EN=1 才真正画图，平时是空函数
//              2. 标定的时候打开：赛道两条边线在俯视图里应该是平行的竖线，
//                 上宽下窄说明 FOV 给小了、下宽上窄说明 FOV 给大了
//              3. TFT180 默认竖屏 128x160，俯视图 96x64 画在屏幕最下面（y = 160-64 = 96），
//                 不盖住上面几行文字；高度不够就退回 (0,0)。
//-------------------------------------------------------------------------------------------------------------------
void IPM_Show_On_TFT180(void)
{
#if (1 == IPM_DIS_EN)
    uint16 y = 0;

    if(tft180_height_max > IPM_H)
    {
        y = (uint16)(tft180_height_max - IPM_H);
    }
    tft180_show_gray_image(0, y, ipm_image[0], IPM_W, IPM_H, IPM_W, IPM_H, 128);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取某一行的逆透视指针表（只读）
// 参数说明     row             俯视图行号 0 ~ IPM_H-1（0 = 车头最近）
// 返回参数     该行 IPM_W 个格点指向的源图像素地址；越界行返回 NULL
// 使用示例     const uint8 *const *lut = IPM_Lut_Row(r);
//              if(!IPM_Lut_Valid(lut[c])) { /* 这一格在画面外 */ }
// 备注信息     供 Pursuit_REMAIN 判断"整行全白"用：出界格点在俯视图里是白色，
//              只看 ipm_image 会把地平线附近的出界行误判成"整行全白"。
//-------------------------------------------------------------------------------------------------------------------
const uint8 *const * IPM_Lut_Row(uint8 row)
{
    if(row >= IPM_H)
    {
        return (NULL);
    }
    return (&ipm_lut[row][0]);
}
