# Sata_Project —— TC264 摄像头循迹智能车

> 主控：英飞凌 **TC264D**（AURIX 双核）　·　库：**逐飞 TC264 开源库**（GPL3.0）
> 影像：总钻风 **MT9V03X**（188×120，灰度）　·　执行器：**舵机转向（前轮）+ 双电机驱动（后轮）**
> 开发环境：**AURIX Development Studio v1.10.2**（Eclipse CDT + TASKING VX-toolset）

---

## 1. 工程在干什么

整车的活儿分在两处跑，互不阻塞：

| 运行位置 | 周期 | 干的事 |
|---|---|---|
| `code/SATA_MAIN.c` 主循环（core0） | 约 50 fps（一帧 20 ms） | 取一帧图像 → 大津法二值化 → 斑马线 → 边缘基点 → 边缘基线 → 逐行真实边界 + 加权中线（含十字路口五特征打分）→ TFT180 显示 |
| `user/isr.c` 的 `cc60_pit_ch0_isr` | 5 ms（200 Hz） | 编码器测速 → 弯道/置信度/路口降速 → 后轮差速 → 速度环（前馈 + PID）→ 电机 PWM；中线偏差 → 分档打角 → 舵机 PWM |

图像层和控制层之间只通过一个 `midline_struct midline_info`（`error` / `conf` / `turn` / `cross`）交换数据，
所以图像算法怎么改都不会破坏控制节的实时性。

---

## 2. 目录结构

```
Seekfree_TC264_Opensource_Library/
├─ code/                        ★ 本项目全部业务代码（15 个模块，约 8000 行）
│  ├─ SATA_MAIN.c/.h            主程序：外设初始化 + 主循环 + 单帧图像流水线（含版本号）
│  ├─ Camera_REMAIN.c/.h        总钻风 MT9V03X 采集 + 大津法阈值 + 二值化 + TFT180 显示
│  ├─ Badpoint_REMAIN.c/.h      边缘基点（左右边线起始点）
│  ├─ Bedline_REMAIN.c/.h       边缘基线（左右边线拟合直线）
│  ├─ MIDLine_REMAIN.c/.h       ★ 核心：逐行真实边界扫描 + 加权中线 + 十字路口状态机（最大模块）
│  ├─ Zebra_REMAIN.c/.h         斑马线识别与起停状态机
│  ├─ IPM_REMAIN.c/.h           逆透视变换（已撤出主流水线，保留对比用）
│  ├─ Pursuit_REMAIN.c/.h       纯追踪 pure pursuit（已撤出主流水线，保留对比用）
│  ├─ Control_REMAIN.c/.h       ★ 控制总成：速度环 / 差速 / 减速 / 舵机打角需求（参数都在 .h 顶部）
│  ├─ Motor_REMAIN.c/.h         左右电机：PWM + 方向脚，有符号占空比，带限幅
│  ├─ Servo_REMAIN.c/.h         舵机：50 Hz PWM，中值 460，分档打角 + 线性化 + 出弯保持
│  ├─ Encoder_REMAIN.c/.h       编码器测速（脉冲/5ms，一阶低通）
│  ├─ PIT_REMAIN.c/.h           5 ms 控制节拍（CCU60 CH0）
│  └─ LCD_REMAIN.c/.h           屏幕小接口
├─ user/                        cpu0_main / cpu1_main / isr.c / isr_config.h
├─ libraries/                   逐飞开源库（zf_common / zf_driver / zf_device / infineon iLLD）
├─ Lcf_Tasking_Tricore_Tc.lsl   链接脚本
├─ .cproject / .project         ADS(Eclipse) 工程文件
└─ 推荐IO分配.txt / 尽量不要使用的引脚.txt
```

**源文件编码是 GBK(936) + CRLF**（ADS/TASKING 工具链习惯）。`.gitattributes` 里已关掉所有自动转换，
所以 clone 下来字节级一致，中文注释不会乱码。

---

## 3. 图像链路（code/SATA_MAIN.c）

```
mt9v03x_finish_flag ──► Camera_Process_Once()   大津法算阈值 → 二值化（base_image[0]）
                    ──► Zebra_Scan()            斑马线判定（必须在二值化之后）
                    ──► Find_Badpoint()         左右边缘基点
                    ──► Find_Bedline()          左右边缘基线（拟合直线）
                    ──► Find_MIDLine()          逐行真实边界 → 加权中线 → 路口判定 → midline_info
                    ──► MIDLine_Show_On_TFT180() 绿线 = 加权中线，蓝线 = 真实边界（只画下 4/5）
```

`MIDLine` 的几层设计：

* **加权中线**：越靠近车头的行权重越大（前 60 行线性 `row+1`，之后二次 `(row-60)²/4+61`，上限 200）；
  弯道时权重自动翻转成远端主导（`MIDLINE_TURN_EN`），提前入弯。
* **十字路口判定**：五特征打分（F1 宽度突变 / F2 两侧同时丢线 / F3 边界外跳 / F4 断带上方有效行 / F5 短黑条），
  `SCORE >= 3` 且连续 2 帧才认账；进入后保持 30 帧、连续 6 帧无特征才退出，出路口后冷却 20 帧。
* **丢线三层兜底**：逐行扫描重算 → 5 帧前历史中线 → 软收敛回中（`MIDLINE_BLIND_*`）。
* 撤掉逆透视/纯追踪后，`IPM_REMAIN.c` `Pursuit_REMAIN.c` 仍在，`SATA_PURSUIT_EN` 改回 1 即可对比。

---

## 4. 控制链路（code/Control_REMAIN.c，5 ms 一拍）

```
Encoder_Get_Speed(L/R)                    编码器 脉冲/5ms，带一阶低通
   │
error = midline_info.error  (像素，正 = 赛道偏右)
   │
   ├─► 弯道减速  slow = 1 - 0.008·|error|，夹到 [0.45,1]，再 ×1.21 收窄降速（直道夹回 1.0）
   ├─► 置信度减速 conf < 70 时按比例降到最低 0.55
   ├─► 路口减速 ×0.80（差速同时打折 ×0.35，路口最怕横摆）
   │
base_speed = CONTROL_BASE_SPEED · slow
   ├─► 后轮差速 ratio = 0.0040 · error（上限 ±0.50），外侧轮 ×(1+ratio)、内侧轮 ×(1-ratio)
   │      target ∈ [CONTROL_SPEED_MIN, CONTROL_SPEED_MAX]
   │
   └─► 速度环：duty = 前馈(gain·target + offset) + PID 修正（Kp 20 / Ki 0.5 / Kd 0，输出限幅）
          └─► Motor_Set_Duty()  →  PWM 5 kHz，占空比 0~10000（MOTOR_DUTY_MAX = 8000 留余量）
```

同一拍里还有两件事：

* **舵机打角**：`error(px) × 0.45°/px` 得到打角需求（直道再乘 0.9025、过 EMA 低通 + 4.5 px 死区、
  按中线形状 `bend` 与置信度 `conf` 动态缩放），交给 `Servo_Set_Step()` 输出 **5 个固定档位**：
  `460`（中位/路口）、`452/468`（微调 ±4 码）、`440/474`（小档）、`418/500`（大档/弯道）、`400/530`（急弯）。
  每档带滞环和最短停留时间，路口强制回中位。
* **保护**：编码器掉线（占空比够大但轮子不转 1 s → 停车 0.5 s 重试，连续 5 次放弃并显示 `E`）、
  超速看门狗（实测远超目标 0.5 s → 停车重新标定）、前馈自动标定（`CONTROL_FF_LEARN_EN`）。

### 电机硬件映射（code/Motor_REMAIN.h）

| 名称 | 引脚 | 说明 |
|---|---|---|
| `Motor_L_pwm` / `Motor_L_turn` | `ATOM0_CH4_P02_4` / `P02_5` | 后左（驱动轮） |
| `Motor_R_pwm` / `Motor_R_turn` | `ATOM0_CH6_P02_6` / `P02_7` | 后右（驱动轮） |
| `SERVO_PWM_PIN` | `ATOM0_CH1_P33_9` | 舵机，50 Hz，中值 460，机械限 395~534 |

---

## 5. 关键可调参数（都在头文件里，改完直接编译）

| 参数 | 位置 | 现值 | 含义 |
|---|---|---|---|
| `CONTROL_BASE_SPEED` | `code/Control_REMAIN.h` | `99.25f` | 基础速度（编码器 脉冲/5ms） |
| `CONTROL_DUTY_BASE` | `code/Control_REMAIN.h` | `794` | 跑到基础速度的前馈占空比 |
| `CONTROL_SPEED_MAX` / `MIN` | `code/Control_REMAIN.h` | `148.875f` / `29.775f` | 差速后内外轮限幅 |
| `CONTROL_PID_OUT_MAX` | `code/Control_REMAIN.h` | `661.5f` | 速度环修正量限幅 |
| `CONTROL_CURVE_SLOW_K` | `code/Control_REMAIN.h` | `0.0080f` | 弯道降速斜率 |
| `CONTROL_DIFF_K` | `code/Control_REMAIN.h` | `0.0040f` | 差速比例（每像素） |
| `CONTROL_STRAIGHT_STEER_SCALE` | `code/Control_REMAIN.h` | `0.9025f` | 直道打角折扣 |
| `SERVO_DUTY_*` 档位表 | `code/Servo_REMAIN.h` | `400~530` | 分档打角 |
| `MIDLINE_CROSS_SCORE_NEED` | `code/MIDLine_REMAIN.h` | `3` | 路口判定打分阈值 |

---

## 6. 怎么编译下载

1. 打开 **AURIX Development Studio**，`File → Import → Existing Projects into Workspace`，
   选中本目录（`Seekfree_TC264_Opensource_Library`）。
2. 工具栏选 `Debug` 配置 → `Build`（生成 `Debug/*.elf` / `*.hex`）。
3. 接上逐飞 TC264 核心板 + DAP 仿真器，`Run → Debug` 下载。
4. 上电后：电机先停、舵机回中，`CONTROL_START_DELAY_MS` 延时后才发车。

> 编译产物目录 `Debug/` 已在 `.gitignore` 里排除（单次编译约 75 MB 中间文件）；
> 需要固件请自行编译，或用 `AURIX修改工程名称.bat` / `删除临时文件.bat` 辅助。

---

## 7. 版本

当前 **V4.6 / 2026-10-26**（见 `code/SATA_MAIN.h` 顶部注释块，含每次改动的原因与还原方法）：
加权中线判定法（移除逆透视 + 纯追踪）、十字路口五特征打分、丢线三层兜底、
舵机按中线形状/置信度动态缩放、直道打角 ×0.9025、电机 BASE 99.25 / DUTY 794。

---

## 8. 许可

`libraries/` 下为成都逐飞科技 TC264 开源库，遵循 **GPL3.0**（见 `libraries/doc/GPL3_permission_statement.txt`）。
业务代码（`code/`、`user/`）为本项目作者编写。
