# agent_link · gc2145-camera 学习专注度监控

[English](README-EN.md) | [简体中文](README.zh-CN.md)

一个跑在 ESP32-S3 上的 Physical AI 末端：用摄像头定时抓拍学习者照片，上传云端 Agent 分析「是否在认真学」，分心时语音提醒，专注时静默不打扰。设备管番茄钟与拍照时机，云端管图像理解与决策。

> 评委快速入口：[最短验证路径](#最短验证路径一次完整闭环) · [接线表](#接线说明) · [编译烧录](#编译烧录配置与运行) · [密钥管理](#配置与密钥管理)

---

## 目录

- [项目概述与核心 Physical AI 闭环](#项目概述与核心-physical-ai-闭环)
- [硬件清单与接线](#硬件清单与接线)
- [固件、SDK 与主要依赖版本](#固件sdk-与主要依赖版本)
- [目录结构与主要模块职责](#目录结构与主要模块职责)
- [环境准备与依赖安装步骤](#环境准备与依赖安装步骤)
- [编译、烧录、配置和运行步骤](#编译烧录配置与运行)
- [Agent、Skill、Tool 或 Custom Service 的配置方式](#agentskilltool-或-custom-service-的配置方式)
- [最短验证路径：一次完整闭环](#最短验证路径一次完整闭环)
- [已知限制与未完成部分](#已知限制与未完成部分)
- [配置与密钥管理](#配置与密钥管理)
- [评委访问](#评委访问)
- [许可证](#许可证)

---

## 项目概述与核心 Physical AI 闭环

`agent_link` 是把嵌入式设备接入 Deotaland Agent 平台的连接层 SDK（[components/agent_link/](components/agent_link/)）。`gc2145-camera` 是其中一块参考板，定位为**学习专注度监控器**：用户说出学习计划 → Agent 拆解为番茄钟 → 设备本地计时，每分钟抓拍一张照片上传 → Agent 判断专注度 → 分心时设备放行喇叭播报提醒，专注时设备保持安静。

### 核心 Physical AI 闭环

```
        ┌─────────────── 感知 (Sense) ───────────────┐
        │  GC2145 摄像头抓拍 JPEG  →  send_image 上行  │
        │  ES8311 麦克风 PTT 语音 →  push_voice 上行   │
        └──────────────────┬───────────────────────────┘
                           │ 上行 (Uplink: BLE / WiFi)
                           ▼
   ┌──────────┐  BLE  ┌──────────┐  云端推理  ┌──────────────────┐
   │ ESP32-S3  │ <──> │ 手机 App  │ <───────> │ Deotaland Agent   │
   │ 设备       │      │ (中继)    │           │ (图像理解 + 番茄钟)│
   └──────────┘      └──────────┘           └──────────────────┘
        ▲                                    │ 下行 (Downlink)
        │              ┌──────── 执行 (Act) ────────┐
        └──────────────│ on_audio_out 喇叭提醒          │
                       │ SetLed WS2812 状态灯          │
                       │ on_actuate 番茄钟控制/拍照触发  │
                       └──────────────────────────────┘
```

**一次完整闭环（学习监控）**：
1. 用户按住按键说「我要复习两小时数学」→ 语音上行。
2. Agent 拆解为番茄钟计划，调用 `study_session` 端点（MCP Tool）下发。
3. 设备本地开始学习阶段，摄像头上电，每 60s 自动抓拍 JPEG 上传 + 发分析 prompt。
4. Agent 看图判断，调用 `study_verdict`（true=专注 / false=分心）。
5. 专注 → 设备什么也不做（安静）；分心 → 设备放行喇叭播报「请认真学习」，LED 变红。
6. 学习阶段结束 → 进入休息，摄像头断电；循环直至计划完成。

**设计要点**：设备本地管时钟与拍照门控，云端只做图像理解与决策。摄像头**仅在番茄钟学习阶段上电**，其余时间（休息/暂停/空闲）断电不抓拍。喇叭在专注时强制静音（设备侧保证，不依赖 Agent 自觉），仅在分心提醒窗口或用户刚发完问的回复窗口放行。

---

## 硬件清单与接线

### 开发板

- **型号**：`gc2145-camera`（ESP32-S3 模组，16MB Flash，八线 PSRAM）
- **声明能力**：`CAMERA | SCREEN | LED | MIC | SPEAKER | RECORDING | BUTTON`（见 [boards/gc2145-camera/gc2145_camera.cc](boards/gc2145-camera/gc2145_camera.cc) `Capabilities()`）
- **硬件构成**：GC2145 DVP 摄像头 + ST7789 240×240 SPI LCD + WS2812 NeoPixel LED + ES8311 codec（麦+喇叭）+ 1 个 PTT 按键

### 接线说明

引脚定义全部在 [boards/gc2145-camera/config.h](boards/gc2145-camera/config.h)。所有 GPIO 为 **3.3V 电平**。

#### 摄像头（GC2145，DVP 8-bit + SCCB + XCLK）

| 信号 | 引脚 | 说明 |
|---|---|---|
| XCLK | GPIO15 | 主时钟输出，20MHz，经 LEDC 产生 |
| SCCB SDA（SIOD） | GPIO4 | I2C 配置数据 |
| SCCB SCL（SIOC） | GPIO5 | I2C 配置时钟 |
| D7 / D6 / D5 / D4 | GPIO16 / 17 / 18 / 12 | 像素 Y9/Y8/Y7/Y6 |
| D3 / D2 / D1 / D0 | GPIO10 / 8 / 9 / 11 | 像素 Y5/Y4/Y3/Y2 |
| VSYNC | GPIO6 | 帧同步 |
| HREF | GPIO7 | 行参考 |
| PCLK | GPIO13 | 像素时钟 |
| PWDN / RESET | -1 | 未接（软件控制） |

> 注意：D2=GPIO8 同时也是其它板子的 PTT 引脚，本板 PTT 走 GPIO3，不冲突。SCCB 用 I2C 端口 1，与下方 ES8311 的 I2C 端口 0 隔离。

#### 显示屏（ST7789，240×240，SPI）

| 信号 | 引脚 | 说明 |
|---|---|---|
| SPI 主机 | SPI2_HOST | — |
| SCK | GPIO21 | 40MHz |
| MOSI | GPIO47 | 数据 |
| CS | GPIO44 | 片选 |
| DC | GPIO43 | 数据/命令 |
| RST / BL | -1 | 未接（软件复位 / 常亮） |
| SPI 模式 | 0 | `invert_color=true` |

> 待机画面是一张线稿猫（开机时一次性画入 PSRAM 帧缓冲），不显示摄像头实时预览。

#### 音频（ES8311，麦克风 + 喇叭，I2S + I2C）

| 信号 | 引脚 | 说明 |
|---|---|---|
| I2S MCLK | GPIO46 | 主时钟 |
| I2S BCLK | GPIO39 | 位时钟 |
| I2S WS / LRCK | GPIO2 | 字选择 |
| I2S DIN ← 麦克风 | GPIO40 | ES8311 ASDOUT，录音 |
| I2S DOUT → 喇叭 | GPIO38 | ES8311 DAC，播放 TTS |
| PA_EN | NC | 无外接功放 |
| I2C SDA | GPIO41 | ES8311 配置 |
| I2C SCL | GPIO42 | ES8311 配置 |
| ES8311 地址 | 0x18 | AD0 低；AD0 高时为 0x19 |

> 采样率固定 16kHz / 16bit / 单声道（与云端 ASR/TTS 对齐）。麦克风增益 30，喇叭音量 80。

#### 按键与 LED

| 信号 | 引脚 | 电压 / 说明 |
|---|---|---|
| PTT 按键 | GPIO3 | 3.3V，**高电平有效**（内部下拉，按下为高）。按住说话，松开发送。GPIO3 不与任何外设共用，电平轮询即可 |
| WS2812 LED | GPIO48 | 3.3V，经 RMT 驱动。状态：暗绿=学习/空闲，红=分心，橙=休息，暗白=暂停，灭=会话结束 |

#### 接线注意事项

1. **PSRAM 必需**：摄像头双帧缓冲（RGB565 240×240）与猫咪待机画面都放在 PSRAM，构建时需开启 Octal PSRAM（`sdkconfig.defaults.esp32s3` 已配）。
2. **I2C 端口隔离**：摄像头 SCCB 用 I2C 端口 1，ES8311 用端口 0，互不干扰。
3. **XCLK 由 LEDC 产生**：`esp32-camera` 内部用 LEDC Timer0/Channel0 输出 20MHz 给传感器，与 WS2812 的 RMT 是不同外设，不冲突。
4. **摄像头电源门控**：固件在番茄钟学习阶段 `esp_camera_init`、结束后 `esp_camera_deinit`，传感器物理断电而非仅忽略，确保非学习时段不抓拍。
5. **BLE 天线**：ESP32-S3 模组自带 PCB 天线，金属外壳会衰减信号。

---

## 固件、SDK 与主要依赖版本

| 项 | 版本 | 来源 |
|---|---|---|
| ESP-IDF | ≥ v5.0（已在 v5.5.4 验证） | [main/idf_component.yml](main/idf_component.yml) |
| agent_link SDK | v2.0.0 / 协议 v1 | [components/agent_link/include/agent_link.h](components/agent_link/include/agent_link.h) |
| Flash 布局 | 16MB，双 OTA 槽（各 3MB）+ FAT 存储 ~10MB | [partitions.csv](partitions.csv) |
| espressif/esp32-camera | ^2.0.4（锁定 2.1.7） | [main/idf_component.yml](main/idf_component.yml) |
| espressif/esp_codec_dev | ^1.5.0（锁定 1.6.2） | [main/idf_component.yml](main/idf_component.yml) |
| espressif/esp_jpeg | 1.3.1（JPEG 编码） | [FIRMWARE_FEATURES.md](FIRMWARE_FEATURES.md) |
| NimBLE 蓝牙协议栈 | 随 ESP-IDF | [sdkconfig.defaults](sdkconfig.defaults) |
| 板级额外依赖 | `esp32_camera`、`esp_driver_rmt` | [main/CMakeLists.txt](main/CMakeLists.txt) `EXTRA_REQUIRES` |

> 实际锁定版本见 [dependencies.lock](dependencies.lock)；首次构建时 ESP-IDF 组件管理器自动拉取到 `managed_components/`（已被 `.gitignore` 忽略，不入库）。

---

## 目录结构与主要模块职责

```
agent_link/
├── CMakeLists.txt               顶层工程：解析 boards/<板>/config.json，叠加 sdkconfig 片段
├── partitions.csv              分区表（nvs/otadata/phy_init + 双 OTA + FAT）
├── sdkconfig.defaults*         配置模板（仅功能开关，无密钥）
├── FIRMWARE_FEATURES.md        固件功能清单（最详尽状态快照）
│
├── main/                        共享应用层，与板无关
│   ├── app_main.cpp             入口：把 agent_link 回调绑定到 Board；5s 轮询电量
│   ├── board.h                  抽象 Board 接口 + DECLARE_BOARD 宏
│   ├── Kconfig.projbuild        Board Type / Transport 选择菜单
│   └── CMakeLists.txt           按所选板编译；gc2145-camera 追加 esp32_camera/esp_driver_rmt
│
├── boards/
│   ├── gc2145-camera/           ★ 本项目重点板
│   │   ├── config.h             全部引脚与学习监控参数（拍照间隔、提示语、番茄钟默认值）
│   │   ├── config.json          目标芯片 esp32s3
│   │   ├── gc2145_camera.cc     板类：摄像头/番茄钟/PTT/MCP 端点注册（991 行核心逻辑）
│   │   ├── st7789_lcd.{h,cc}     ST7789 SPI 驱动（自包含）
│   │   ├── ws2812_led.{h,cc}     WS2812 RMT 驱动
│   │   └── cat_screen.{h,cc}     线稿猫待机画面
│   ├── common/                  跨板复用驱动：es8311_audio / es_codec / bq27220 / 屏驱动
│   └── README.md                如何加一块板
│
├── components/
│   ├── agent_link/              可复用连接层 SDK（协议 + 传输 + 能力路由）
│   │   ├── include/             agent_link.h / caps / io / transport 接口
│   │   ├── src/                 agent_link.cpp / protocol.cpp / transport_ble.cpp / transport_wifi.cpp
│   │   └── portal/              WiFi 配网页面（编译期嵌入）
│   └── esp_lcd_sh8501/          其它板用到的屏驱动
│
└── managed_components/          （构建时生成，不入库）乐鑫官方组件
```

### gc2145-camera 板内职责

- **`gc2145_camera.cc`**：板类 `Gc2145CameraBoard`。初始化 LCD/LED/codec/摄像头；注册 5+1 个 MCP 端点；跑两个任务——`CaptureLoop`（抓拍+JPEG+上传）与 `PomodoroLoop`（番茄钟状态机）；PTT 任务处理按键说话；`PlayLoop` 把 TTS PCM 写入 ES8311 DAC。
- **`config.h`**：全部引脚 + 学习监控可调参数（`CAMERA_AUTO_CAPTURE_INTERVAL_MS`=60s、`CAMERA_ANALYSIS_PROMPT`、`STUDY_ALERT_PHRASE`、番茄钟时长、喇叭放行窗口等）。

---

## 环境准备与依赖安装步骤

### 1. 安装 ESP-IDF 工具链

```bash
# Linux/macOS
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh
. ./export.sh          # 每个新终端先 source
```

> Windows 推荐用 [ESP-IDF Tools Installer](https://docs.espressif.com/projects/esp-idf/zh_CN/v5.5.4/esp32s3/get-started/)，或在 PowerShell 中 `. $env:IDF_PATH\export.ps1`。

验证：`idf.py --version`

### 2. 克隆本仓库

```bash
git clone <仓库地址> agent_link
cd agent_link
```

> 本仓库**不使用 git submodule**，第三方组件由 ESP-IDF 组件管理器按 [dependencies.lock](dependencies.lock) 在首次构建时自动下载到 `managed_components/`，无需手动拉取。

### 3. Python 依赖

ESP-IDF 安装时已配好。若提示缺包，执行 `idf.py install`。

---

## 编译、烧录、配置和运行步骤

### 1. 选择目标芯片

```bash
idf.py set-target esp32s3
```

### 2. 选择板型与传输后端（配置）

```bash
idf.py menuconfig
# 菜单路径：Agent Link Device
#   → Board Type → ESP32-S3 GC2145 Camera (DVP camera preview on ST7789 LCD)
#   → Transport backend → BLE（默认；WiFi 仅配网可用，控制/数据面为桩，见「已知限制」）
```

### 3. 编译

```bash
idf.py build
```

首次构建自动下载 `esp32-camera` / `esp_codec_dev` / `esp_jpeg` 到 `managed_components/`。

### 4. 烧录与监控

```bash
idf.py -p <串口> flash monitor
# Windows：    idf.py -p COM5 flash monitor
# Linux/macOS：idf.py -p /dev/ttyACM0 flash monitor
```

### 5. 启动日志（成功标志）

```
agent_link.app: board = GC2145_CAMERA, caps = 0x023f
agent_link: init: name='GC2145_CAMERA' caps=0x023f proto=v1 transport=ble io=0
agent_link.ble: BLE started — Service C 0xFFC0 registered; advertising on sync
Gc2145Cam: WS2812 LED ready on GPIO48 (App endpoint: led0)
Gc2145Cam: ES8311 mic ready (16000 Hz)
Gc2145Cam: speaker ready: Agent TTS plays through the ES8311 DAC
Gc2145Cam: PTT ready: hold GPIO3 to talk, release to end
Gc2145Cam: camera sensor PID=0x2145 (GC2145 expected)
Gc2145Cam: camera ready (RGB565 240x240)
Gc2145Cam: camera OFF (deinit=ESP_OK)
[state] READY
```

看到 `advertising as 'GC2145_CAMERA'` + `[state] READY` + LED 暗绿，即链路就绪、摄像头已校验并断电待命。

---

## Agent、Skill、Tool 或 Custom Service 的配置方式

设备向云端暴露 **6 个 MCP 端点**（在 `agent_link_start()` 前用 `agent_link_register_io` 注册，连接时序列化为 manifest 下发，**云端据此自动生成 MCP 工具/资源，无需手写云端配置**）：

| 端点 id | 方向 | 类型 | 作用 |
|---|---|---|---|
| `study_session` | OUT（Tool） | str(JSON) | Agent 调用启动番茄钟。JSON：`{total_cycles, study_mins, short_break_mins, long_break_mins}`，字段均可选，设备侧钳位下限 |
| `study_control` | OUT（Tool） | str | 控制运行中的会话：`pause` / `resume` / `stop` |
| `study_verdict` | OUT（Tool） | bool | 每张照片的专注度结论。true=专注（设备静默）；false=分心（设备放行喇叭播 `STUDY_ALERT_PHRASE`，LED 变红） |
| `camera0` | OUT（Tool） | bool | Agent 主动触发一次抓拍（仅在番茄钟学习阶段生效，否则拒绝） |
| `study_status` | IN（资源） | str | 设备上报阶段切换：`study_started` / `break_started` / `cycle_completed` / `session_completed` / `paused` / `resumed` |
| `study_watch` | IN（资源） | str | 设备每次自动抓拍后镜像 `CAMERA_ANALYSIS_PROMPT`，作为 MCP 资源供 Agent 读取 |

### 关键机制

- **触发用 prompt，不用 reading**：`0x19 IoReading` 只是数据更新，不会启动 Agent 一轮对话；`0x04 Prompt`（`agent_link_push_prompt`）才被 App 原样转发给 Agent 触发回答。每次自动抓拍**同时发两份**——reading 更新 MCP 资源值，prompt 真正触发分析。
- **图像走 L2CAP，prompt 走控制面**：`agent_link_send_image` 异步流式上传 JPEG（~1-2s），prompt 延迟 2s 发送（`CAMERA_ANALYSIS_PROMPT_DELAY_MS`），确保 Agent 收到 prompt 时图像已到。
- **设备管时钟**：番茄钟计划一旦下发，阶段切换在设备本地完成，云端断连也不影响计时。Agent 只在每个切换点收到 `0x04` prompt 状态通知，可据此辅导用户。
- **喇叭门控（设备侧保证）**：学习阶段默认静音，仅两个窗口放行——分心提醒窗口（`STUDY_ALERT_WINDOW_MS`=15s）与用户发问后的回复窗口（`STUDY_REPLY_WINDOW_MS`=30s，TTS 播完即关）。非学习阶段音频正常播放。

---

## 最短验证路径：一次完整闭环

以验证「拍照→分析→提醒」核心闭环为例。

### 前置条件

- 硬件：`gc2145-camera` 开发板，USB 数据线，板载按键/摄像头/LED 正常
- 软件：ESP-IDF v5.0+ 工具链
- 账号：**Deotaland 手机 App**（完整闭环需要；App 中继到云端 Agent 并持有云端凭证，App 侧需已配置可进行图像理解的 Agent）

### 步骤

```bash
# 1. 烧录
idf.py set-target esp32s3
idf.py menuconfig      # Agent Link Device → Board Type → GC2145 Camera
idf.py build
idf.py -p <串口> flash monitor
```

```
# 2. 观察设备启动
#    - 日志出现 advertising as 'GC2145_CAMERA'  ← BLE 就绪
#    - 日志出现 camera sensor PID=0x2145         ← 摄像头校验通过
#    - LED 暗绿                                   ← 板子就绪、摄像头待命
#    - [state] READY
```

```
# 3. 链路冒烟验证（无需 App，1 分钟）
#    手机装 nRF Connect / LightBlue，扫描：
#    - 能按名搜到 'GC2145_CAMERA'
#    - 连接后可见控制服务 0xFFC0、标准电量 0x180F / 设备信息 0x180A
#    ⇒ 看到即设备侧链路正常
```

```
# 4. 完整学习监控闭环（需 Deotaland App）
#    a. App 扫描连接设备 → 设备日志 [state] CONNECTED → [state] READY
#    b. 按住板载按键（GPIO3）说「我要复习两小时数学」→ 松开
#       设备日志：PTT pressed, starting voice uplink → PTT released, ending voice
#    c. Agent 拆解计划，调用 study_session 端点
#       设备日志：study_session queued → STUDY phase started (cycle 1/N, 25 mins)
#       现象：LED 转亮绿，摄像头 ON，开始计时
#    d. 约 60s 后自动抓拍上传
#       设备日志：snapshot #1 (timer): 240x240 -> ~XXkB jpeg, send=ESP_OK
#                analysis prompt -> Agent (0x04)
#    e. Agent 分析后调用 study_verdict
#       - 专注：设备日志 verdict: attentive — no device reaction，LED 保持绿
#       - 分心：设备日志 verdict: DISTRACTED — speaker unmuted，LED 变红，
#              喇叭播报「请认真学习」
#    f. 25 分钟学习结束 → 进入休息
#       设备日志：SHORT BREAK started (5 mins)，LED 转橙，摄像头 OFF
#    ⇒ 看到拍照上传 + Agent 判定 + 分心提醒 = 核心闭环成功
```

### 验证成功标志

| 环节 | 日志/现象 |
|---|---|
| 设备就绪 | `advertising as 'GC2145_CAMERA'` + LED 暗绿 + `camera sensor PID=0x2145` |
| 语音上行 | `PTT pressed, starting voice uplink` |
| 会话启动 | `STUDY phase started` + LED 转绿 + `camera ON (study phase)` |
| 抓拍上传 | `snapshot #1 (timer): ... send=ESP_OK` + `analysis prompt -> Agent` |
| 专注判定 | `verdict: attentive — no device reaction`（LED 保持绿） |
| 分心提醒 | `verdict: DISTRACTED` + LED 变红 + 喇叭播报「请认真学习」 |
| 阶段切换 | `SHORT BREAK started` + LED 转橙 + `camera OFF` |

---

## 已知限制与未完成部分

| 部分 | 状态 | 说明 |
|---|---|---|
| BLE 控制面（广播、`0xFFC0`、事件、状态回调） | ✅ 完成 | 已验证 |
| 摄像头抓拍 + JPEG 上传（L2CAP image 通道） | ✅ 完成 | `send_image`，q80 JPEG，240×240 |
| 番茄钟状态机 + 5 MCP 端点 | ✅ 完成 | 设备本地计时，云端决策 |
| 语音上行 PTT（`push_voice`） | ⚠️ 已实现，未真机验证 | 按 MTU 切片 + 背压 |
| TTS 下行播放（`on_audio_out` → ES8311） | ⚠️ 已实现，未真机验证 | L2CAP，含喇叭门控 |
| WiFi STA 接入 + SoftAP 配网 | ✅ 可用 | 凭据存 NVS，配网自愈 |
| WiFi 控制面 / 数据面 | ❌ 桩函数 | WiFi 模式连不上云端，需用 BLE |
| 视频（`push_video`） | ❌ 仅接口 | 依赖未完成的 WiFi/WebRTC 后端 |
| 录音落盘、文件/OTA 传输 | ❌ 未实现 | 传输层已预留，未接业务 |
| `report_selected_agent` | ❌ 未实现 | `0x16` 事件格式待定 |
| 摄像头实时预览 | ❌ 不实现 | 设计上不预览，仅抓拍；屏幕只显示待机猫与状态 |
| SD 卡 / OTA 升级 | ❌ 未实现 | 分区表预留双 OTA 槽，固件无 `esp_ota_*` 调用 |

完整功能快照见 [FIRMWARE_FEATURES.md](FIRMWARE_FEATURES.md)。

---

## 配置与密钥管理

本仓库是**设备侧固件**，遵循「不提交任何真实凭证」原则。

### 1. 仓库内不含任何 API Key / Token / 密码

已对源码全量搜索确认：无硬编码的 `api_key` / `token` / `secret` / `password` / `credential`。云端平台的 API Key 与 Agent 鉴权 Token 由 **Deotaland 手机 App / 云端**持有，不在本设备固件仓库内。

### 2. 配置模板（不含真实凭证）

本仓库提交的配置模板为 [sdkconfig.defaults](sdkconfig.defaults)、[sdkconfig.defaults.esp32s3](sdkconfig.defaults.esp32s3)、[sdkconfig.defaults.esp32p4](sdkconfig.defaults.esp32p4)，仅含功能开关（BT 启用、Flash 大小、PSRAM 模式、分区表等），**不含任何密钥或凭证**。板级可调参数（拍照间隔、提示语、番茄钟时长）在 [boards/gc2145-camera/config.h](boards/gc2145-camera/config.h) 中以 `#define` 形式提供，均为业务常量无密钥。

> 说明：ESP-IDF 固件不使用 `.env` 文件。设备侧无需 API Key；唯一运行期敏感数据是 WiFi 凭据，通过手机配网页面注入 NVS（见下）。故不提供 `.env.example`，改以上述 `sdkconfig.defaults*` 作为不含真实凭证的配置模板。

### 3. 三层配置机制（均无凭证入库）

| 配置层 | 内容 | 是否入库 | 注入方式 |
|---|---|---|---|
| 构建期 | 板型、传输后端、Flash/PSRAM 等 | ✅ 入库（`sdkconfig.defaults*`，无密钥） | `idf.py menuconfig` |
| 运行期 · WiFi 凭据 | SSID / 密码 | ❌ 不入库 | 手机连 SoftAP 配网页面输入 → 校验成功后写 NVS；密码错误不入库 |
| 运行期 · 云端端点/Token | `agent_wifi_config_t` 的 `endpoint` / `token` | ❌ 不入库 | 仅 WiFi 模式需要，运行时通过 `agent_link_config_t` 传入，可为 NULL |

### 4. 防泄露机制

- [.gitignore](.gitignore) 已忽略：`sdkconfig`（本地配置）、`build/`、`managed_components/`、`.vscode/` 等。
- WiFi 凭据只在「设备确认能拿到 IP」后才落盘 NVS，错误密码永不保存。
- 固件不读取任何 `.env` 文件——ESP-IDF 固件通过 menuconfig 与 NVS 配网注入。

> 评委检查：仓库中无真实凭证；所有配置通过上述受控机制注入；`sdkconfig.defaults*` 仅含功能开关不含密钥。

---

## 评委访问

- **仓库链接**：提交前已确认仓库为公开可访问状态，评委账号具备读权限。
- **无子模块**：本仓库不使用 git submodule，所有第三方组件（`esp32-camera`、`esp_codec_dev`、`esp_lcd_co5300`、`esp_jpeg`）由 ESP-IDF 组件管理器在首次 `idf.py build` 时按 [dependencies.lock](dependencies.lock) 自动下载，无需手动拉取。
- **下载链接可用性**：上述组件均来自乐鑫官方组件库，长期稳定。
- **复现所需**：`gc2145-camera` 开发板 + ESP-IDF v5.0+ + USB 数据线 + Deotaland 手机 App（完整闭环）。
- **协议与设计说明**：详见 [FIRMWARE_FEATURES.md](FIRMWARE_FEATURES.md) 与 [components/agent_link/README.md](components/agent_link/README.md)。

---

## 许可证

本项目以 MIT 许可证发布，见 [LICENSE](LICENSE)。Copyright (c) 2026 DEOTALAND LIMITED（德奧塔文化科技有限公司）。

板级目录布局参考了 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)（同为 MIT）。[components/esp_lcd_sh8501/](components/esp_lcd_sh8501/) 与 `managed_components/` 下的乐鑫驱动为 Apache-2.0，各自保留其许可证声明。
