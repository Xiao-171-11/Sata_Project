/*
 * IPM_REMAIN.h
 *
 *  逆透视变换（Inverse Perspective Mapping）：把摄像头看到的斜视图重采样成地面俯视图
 *
 *  为什么需要：
 *    1. 摄像头是斜着看赛道的：画面最下边 1 像素 = 2 mm 地面，画面上边 1 像素 = 100 mm，
 *       直接在图像坐标上算偏差，等于"近处放大、远处缩小"，权重全是乱的。
 *    2. 纯追踪（pure pursuit）必须在地面坐标系里选前瞻点、算曲率，所以先要有俯视图。
 *
 *  实现方式：查表 + 指针（快速大津法同一套路：把能提前算的全挪到上电）。
 *    上电时 IPM_Init() 算出内参和地面格点间距；第一帧 IPM_Process() 再把
 *    "96 x 64 个地面格点该采摄像头哪一字节"直接存成指针表 ipm_lut[][]，
 *    之后每帧只是 6144 次指针解引用，没有下标加法、没有分支、没有浮点。
 *    这就是 RUBO-UDIPM（losehu）下位机模板的写法：uint8_t *PerImg_ip[ROW][COL]，
 *    出界格点统一指向一个常量字节，用指针相等来判断出界。
 *
 *  去畸变也一样折进这张表：
 *    我们本来就是"地面点 -> 针孔模型算出理想像素"，而镜头的实际成像是
 *    理想像素再经一次 Brown-Conrady 畸变（fx/fy/cx/cy + k1 k2 k3 p1 p2）。
 *    所以建表时直接补一步正向畸变就行，运行时零成本；
 *    不需要 OpenCV undistort() 那种"逐像素迭代求逆"的预处理，那一趟要多扫两万多个点。
 *
 *  坐标系（车辆坐标系）：X 向右为正，Y 向前为正，Z 向上为正；
 *    原点是两个后轮中点正下方的地面，相机光心在 (0, 0, IPM_CAM_HEIGHT_MM)。
 *
 *  标定方法（没有尺子也能做）：
 *    1. 把车放在赛道上，正对前方，量一下镜头离地高度 -> IPM_CAM_HEIGHT_MM
 *    2. 量一下镜头光轴与水平面的夹角 -> IPM_CAM_PITCH_DEG（俯角，向下为正）
 *    3. IPM_CAM_FOV_H_DEG 是横向视场角，总钻风 MT9V03X 配 2.8mm 镜头约 100 度
 *    4. 打开 IPM_DIS_EN，屏幕上直接看俯视图：赛道两条边线应该是平行的竖线；
 *       不平行就微调 FOV（大了边线向内收，小了向外张），不在画面中间就调 PITCH
 *    5. 直道边线在俯视图里"中间直、两头往里弯"就是桶形畸变没去干净，
 *       打开 IPM_UNDIS_EN 并填标定得到的 k1 k2（见下面"去畸变参数"）
 *
 *  Created on: 2026年10月28日
 *      Author: Freed
 */

#ifndef CODE_IPM_REMAIN_H_
#define CODE_IPM_REMAIN_H_
#include "zf_common_headfile.h"

/* ============================ 俯视图尺寸 ============================ */
/* 96 列 x 6.7mm 横向覆盖 +-320mm（够 450mm 赛道左右各留 95mm 余量）。
 * 64 行 x 19.8mm 从 150mm 看到 1400mm：路口上方必须能看到"路面又回来了"，
 * 所以远端不能只做到 800mm（800mm 处只剩源图第 52 行，路口一挡就什么都看不见）。 */
#define IPM_H                   (64)
#define IPM_W                   (96)

/* ============================ 地面覆盖范围（mm） ============================ */
/* 最近 150mm：总钻风装 240mm 高、俯角 22 度时，画面最下沿大约对应车头前 144mm，
 *             所以 150mm 是最靠近车的有效行，再近就是画面外了。 */
#define IPM_Y_NEAR_MM           (150.0f)
/* 最远 1400mm：22 度俯角下对应源图第 42 行，再往后源图只剩 10 行且地面分辨率
 *             已经退化到 90mm/像素以上（模糊但没有坏处，判"有没有路面"够用）。 */
#define IPM_Y_FAR_MM            (1400.0f)
#define IPM_X_HALF_MM           (320.0f)

/* ============================ 相机安装参数（换车就改这三个） ============================ */
#define IPM_CAM_HEIGHT_MM       (240.0f)                // 镜头光心离地高度
#define IPM_CAM_PITCH_DEG       (22.0f)                 // 光轴俯角（向下为正）
#define IPM_CAM_FOV_H_DEG       (100.0f)                // 横向视场角（2.8mm 镜头约 100 度）

/* ============================ 去畸变参数（默认关，标定后打开） ============================ */
/* 默认 0：不填系数时几何和改造前完全一致（等价性验证过的"零风险默认值"）。
 * 打开后建表时对每个理想像素补一次正向畸变，运行时开销不变（表是上电建的）。
 *
 * 系数怎么来（任选一种）：
 *   a. 直接抄同款镜头的标定结果，形如 OpenCV 的
 *        cameraMatrix = {{296.482, 0, 152.665}, {0, 286.375, 104.540}, {0, 0, 1}}
 *        distCoeffs  = {-0.45995, 0.28368, 0.00230, 0.00257, -0.10927}  (k1 k2 p1 p2 k3)
 *      把 fx fy cx cy 填到下面四个宏、k1 k2 k3 p1 p2 填到下面五个宏即可。
 *      RUBO-UDIPM 模板（smartcar Demo\2_去畸变下位机.c）用的就是这一组。
 *   b. 自己标：打印一张棋盘格贴在平板上，用 OpenCV calibrateCamera 标一次，
 *      只取 fx fy cx cy 和 5 个畸变系数。
 *   c. 偷懒法：本工程没有标定条件时先保持 IPM_UNDIS_EN=0，
 *      只靠 IPM_CAM_FOV_H_DEG 拟合视场角（改造前的画法），几何仍然可用。
 *
 * 注意：MT9V03X 输出 188x120 是裁剪后的窗口，OpenCV 标定矩阵一般是 188x120 分辨率下的值，
 *       分辨率不一致要先按 比例 缩放 fx fy cx cy（k1 k2 p1 p2 k3 是无量纲的，不用改）。
 */
#define IPM_UNDIS_EN            (0)                     // 1 = 建表时补去畸变
#define IPM_CAM_FX_PX           (-1.0f)                 // <0 = 由 FOV 自动算 fx（= fy）
#define IPM_CAM_FY_PX           (-1.0f)                 // <0 = 与 fx 相同
#define IPM_CAM_CX_PX           (-1.0f)                 // <0 = 自动取 (W-1)/2 = 93.5
#define IPM_CAM_CY_PX           (-1.0f)                 // <0 = 自动取 (H-1)/2 = 59.5
#define IPM_DIST_K1             (0.0f)                  // 径向一阶
#define IPM_DIST_K2             (0.0f)                  // 径向二阶
#define IPM_DIST_K3             (0.0f)                  // 径向三阶
#define IPM_DIST_P1             (0.0f)                  // 切向 x
#define IPM_DIST_P2             (0.0f)                  // 切向 y

/* ============================ 快速选项 ============================ */
/* 1 = 整行都在画面外的行直接 memset 铺白，跳过 96 次解引用。
 * 俯视图最上面几行常常整行出界（地平线以上），这一刀是纯赚。 */
#define IPM_SKIP_EMPTY_ROW      (1)

/* 查表哨兵：出界格点在表里指向这个变量（值恒为 IPM_WHITE），运行时不必再判空 */
extern uint8 ipm_white_pixel;                           // 恒等于 IPM_WHITE(255) 的白像素
#define IPM_FILL_PTR            (&ipm_white_pixel)
#define IPM_Lut_Valid(p)        ((p) != IPM_FILL_PTR)   // 该指针是否真的指向摄像头像素
#define IPM_BLACK               (0)
#define IPM_WHITE               (255)

/* 1 = 屏幕上直接显示俯视图（调试标定用，会盖住上面几行信息） */
#define IPM_DIS_EN              (0)

#if (0 != IPM_UNDIS_EN) && (1 != IPM_UNDIS_EN)
#error "IPM_UNDIS_EN 只能是 0 或 1"
#endif
#if (0 != IPM_SKIP_EMPTY_ROW) && (1 != IPM_SKIP_EMPTY_ROW)
#error "IPM_SKIP_EMPTY_ROW 只能是 0 或 1"
#endif
#if (IPM_H > 255) || (IPM_W > 255)
#error "行号列号用 uint8 存，IPM_H / IPM_W 不能超过 255"
#endif

typedef struct
{
    uint8  ok;                                          // 1 = LUT 有效（参数算出来在画面内）
    uint8  horizon_row;                                 // 地平线所在行（越小说明俯角越大）
    uint16 invalid_cnt;                                 // 画面外的格点数（>20% 说明参数配错了）
    float  focal_px;                                    // 等效焦距（像素）
    float  dx_mm;                                       // 每个格点横向代表多少 mm
    float  dy_mm;                                       // 每个格点纵向代表多少 mm
    uint8  undis_en;                                    // 建表时是否做了去畸变（回读宏的值）
    uint8  valid_row_cnt;                               // 至少有 1 个有效格点的行数
} ipm_struct;

extern ipm_struct ipm_info;
extern uint8 ipm_image[IPM_H][IPM_W];                   // 俯视二值图：0=黑线 255=白
extern uint8 ipm_row_valid[IPM_H];                      // 每行有效格点数（0 = 整行出界）

void  IPM_Init(void);                                   // 算几何参数（上电调一次）
uint8 IPM_Process(const uint8 *image);                  // 每帧调用；返回 0-成功 1-参数无效
void  IPM_Show_On_TFT180(void);                         // IPM_DIS_EN=1 时显示俯视图

/* 取某一行的指针表（只读）：调用方可以据此判断"这个格点是不是真的采到了摄像头像素"。
 * 俯视图里出界的格点被填成白色(255)，如果只看像素值会误以为"整行全白"，
 * 所以判"整行全白"之前必须先问 LUT 这一行有没有出界点。
 * 注意：指针表绑定在"第一帧传进来的那张图"上（本工程是 base_image），
 *       换缓冲重新调 IPM_Init() 或直接传新缓冲给 IPM_Process() 都会自动重建。 */
const uint8 *const * IPM_Lut_Row(uint8 row);            // 返回该行 IPM_W 个源像素指针（越界行返回 NULL）

#endif /* CODE_IPM_REMAIN_H_ */
