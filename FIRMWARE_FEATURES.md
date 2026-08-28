# 固件功能清单

> 快照基于工作区当前状态：`agent_link` v2.0.0 / ESP-IDF v5.5.4 / 版本号 `75fb505-dirty`。
> 当前 sdkconfig 选中：`BOARD_TYPE_ESP32S3_ROROLEE_BASIC` + `AGENT_LINK_TRANSPORT_BLE`（目标芯片 esp32s3）。

## 1. 整体定位

设备侧连接层 SDK。硬件只需「声明能力 + 注册回调」即可接入 Deotaland Agent 平台，
传输、协议、能力路由全部由 `components/agent_link` 承担；对话逻辑在云端，设备保持轻量。

- Agent → 设备：`agent_output_cb_t` 回调（语音 / 文本 / 图像 / 震动 / LED / 执行器）
- 设备 → Agent：`agent_link_push_*` 系列（语音 / 按键 / 传感器 / 电量 / 自定义事件）
- 应用层 `main/app_main.cpp` 只依赖抽象 `Board` 接口，换板子不动 SDK

## 2. 连接与传输

| 功能 | 状态 | 说明 |
|---|---|---|
| BLE（NimBLE）广播 + 可连接 | ✅ | 广播固定 128-bit 产品识别 UUID，App 可按此扫描过滤 |
| 控制服务 `0xFFC0` | ✅ | `0xFFC1` 命令通道（Write 收命令 / Notify 回响应）、`0xFFC4` 事件通道（Notify 推事件） |
| 语音上行服务 `0xFFA0` | ✅ | `0xFFA1` Notify 推 PCM（事件 `0x40 VoiceChunk`），按 MTU 自适应切片 + 背压队列 |
| L2CAP CoC（PSM `0x0081`） | ✅ | 双向：App→设备 TTS 下行；设备→App 录音/ASR 上行 |
| 标准电池服务 `0x180F` | ✅ | `0x2A19` 电量可读 + Notify |
| 标准设备信息服务 `0x180A` | ✅ | 厂商 `0x2A29` / 型号 `0x2A24` / 固件版本 `0x2A26`，可在 init 时配置 |
| WiFi STA 接入 | ✅ | esp_netif + 驱动初始化、自动重连（2s 起指数退避，封顶 30s）、凭据存 NVS |
| WiFi 配网（SoftAP 强制门户） | ✅ | 无凭据时开放 AP `<name>-XXXX`；内置 DNS 劫持 + HTTP 服务，手机自动弹配网页 |
| 配网自愈 | ✅ | 已存凭据连续 8 次连不上（从未拿到 IP）→ 重新开门户；密码错误不落盘 |
| 配网页面可定制 | ✅ | `portal/portal.html`、`portal/connecting.html` 编译期嵌入，改样式/翻译不动 C 代码 |
| WiFi 控制面 / 数据面 | ⚠️ 桩 | `wifi_send_ctrl` / `stream_*` / `is_ready` 尚未实现，只有 STA + 配网是可用的 |
| `AGENT_TRANSPORT_BOTH`（BLE 控制 + WiFi 媒体） | ⚠️ 未实现 | 当前回退为纯 BLE |

## 3. 控制面协议

6 字节定长头 + 载荷：`version(1)=0x01 | msg_type(1) | command_id(1) | sequence(1) | payload_len(2, LE)`，
`msg_type` 低 7 位 `0x01 命令 / 0x02 响应 / 0x03 事件`，最高位 `0x80` 为加密标志。

**SDK 直接处理的下行命令**

| 命令 | 名称 | 说明 |
|---|---|---|
| `0x03` | GetChargingStatus | 由 SDK 用电量缓存直接应答，无需板级参与 |
| `0x05` | VoiceReply | `status=3` 表示播放结束 → 触发 `on_audio_end` |
| `0x33` | IoActuate | 按 endpoint id 路由到注册的执行器回调 |
| `0x34` | GetIoManifest | App 主动拉取，重发能力清单 |
| `0x35` | GetReading | 从缓存返回最新读数 + 数据新鲜度 `age_ms` |
| `0x36` | SetReadingConfig | 按 endpoint 设置上报策略（关闭 / 定频 / 变化上报） |
| `0x3C` / `0x3D` | Start/StopCapture | App 发起的「开始/停止听」→ `on_listen` 回调 |

未识别的命令走 `on_command`（需要回数据）或 `on_custom`（fire-and-forget，SDK 自动回空 ACK）。

**上行事件**

| 事件 | 内容 |
|---|---|
| `0x01/0x02/0x03` | 按键 / 传感器 / 唤醒词 |
| `0x14` | PowerStatus：`{充电状态, 电量}`，充电状态含 `0x02` 低电告警 |
| `0x18` | IoManifest：能力清单 JSON，按 MTU 分片下发 |
| `0x19` | IoReading：单条传感器读数 |
| `0x1A` | ManifestChanged：运行期增删 endpoint 后通知 App 重新拉取 |
| `0x40` | VoiceChunk：语音上行分片 |
| `0x52` / `0x53` | 录音/ASR 流开始 / 结束（带 status + valid_bytes） |
| `0x64` | 板级私有事件，payload 由板子自定义 |

## 4. 能力模型（`agent_cap_t`）

`MIC` / `SPEAKER` / `SCREEN` / `BUTTON` / `HAPTIC` / `BATTERY` / `LED` / `SENSOR` / `ACTUATOR` / `RECORDING` / `CAMERA`，按位或声明。

**通用 I/O（自描述端点）**

- `agent_link_register_io()` 注册端点：id / 方向 / 语义 kind / 数据类型 / 单位 / 量程 / 建议频率 /
  参数 schema / 展示名 / 可见性（AI 可调 vs 仅用户可见）/ 枚举值 / 默认值 / 上报语义
- 数据类型：`bool / i32 / u16 / f32 / vec2 / vec3 / rgb / blob / str`
- 连接时序列化为 manifest（`{proto, rev, caps, io:[...]}`）推给云端，云端据此生成 MCP 工具/资源
- 传感器统一走 `agent_link_push_reading(id, value)`，执行器按 id 路由回调 —— 新增传感器无需改 SDK
- SDK 自动为已接线的富输出补注册合成端点：`led0` / `motor0` / `screen0`，让 Agent 用同一条 `0x33` 通路驱动

**上报策略与去重**

- 电量：仅在数值变化 / 充电状态翻转 / 跌破 5% 低电边沿时上报；发送失败不更新缓存以便重试；连接后首次强制同步
- 读数：默认透传，可按端点切「关闭 / 定频 / 变化上报」，SDK 内部维护最近值缓存供 `0x35` 查询

## 5. 媒体通路

| 通路 | 状态 |
|---|---|
| TTS 下行（L2CAP → `on_audio_out` / `on_audio_end`），PCM16 16kHz 单声道 | ✅ |
| 语音上行 `push_voice` / `voice_end`（GATT Notify） | ✅ |
| 实时 ASR 录音流 `asr_start` / `asr_push` / `asr_end`（L2CAP，透明管道，一次一路） | ✅ |
| 文本显示 `on_show_text` / 图像显示 `on_show_image`（RGB565 BE） | 接口就绪，板级实现程度不一 |
| 视频 `push_video` / `on_video_out` | ⚠️ 仅接口，依赖未完成的 WiFi 后端 |
| 录音文件落盘 + 上传、文件/OTA 传输流 | ⚠️ 未实现（`AGENT_STREAM_RECORDING` / `AGENT_STREAM_FILE` 已在传输层预留） |

## 6. 板级支持

| 板子 | 芯片 | 声明能力 | 硬件 |
|---|---|---|---|
| `rorolee-basic` **（当前选中）** | esp32s3 | MIC / SPEAKER / SCREEN / BUTTON / HAPTIC / BATTERY / RECORDING | SH8501 屏、ES8311+ES7210 codec、BQ27220 电量计、BOOT 键、外设电源域 |
| `rorolee-s3` | esp32s3 | 同上 | 同款，带外部 32.768kHz 晶振配置 |
| `es8311-voice` | esp32s3 | MIC / SPEAKER / BUTTON / RECORDING | 单 codec 音频示例 |
| `es8311-asr` | esp32s3 | MIC / BUTTON / RECORDING | 最小实时 ASR 示例 |
| `gc2145-camera` | esp32s3 | CAMERA / SCREEN | GC2145 DVP 摄像头 + ST7789 实时预览 |
| `tem_monitor` | esp32s3 | SENSOR | SHT30 温湿度 + SPA06 气压 + ST7789 |
| `esp32p4Waveshare` | esp32p4 | SCREEN | CO5300 466×466 AMOLED |

板级配置由 `boards/<board>/config.json` 的 `sdkconfig_append` 驱动，顶层 CMake 解析后叠加到 `sdkconfig.defaults`，
`BOARD=<name> idf.py build` 即可切板。

### 当前板 `rorolee-basic` 的具体行为

- **单键三手势**（GPIO0，轮询 + ~40ms 去抖，单任务串行驱动 codec 避免并发）
  - 单击：开始录音，再按停止（流标签 `voice`）
  - 双击（400ms 内两下）：开始「环境光/氛围」口述录音（流标签 `light_env`）
  - 长按 ≥1s：标记任务完成，不录音
  - 录音中任意按下：停止录音
- **私有事件**（`0x64`，携带显式状态 + LE32 序号，丢帧不会导致状态错位）
  - `[0x01][state][kind][seq]` 录音开始/停止
  - `[0x02][seq]` 任务完成
  - `[0x03][rms LE16]` 每 2s 上报环境响度（录音中取已推帧均值，空闲时临时开麦取一帧）
- **音频**：20ms/帧 PCM16@16k 推 ASR 通道；TTS 走 StreamBuffer + 独立播放任务，回调非阻塞，缓冲满则丢帧告警
- **电量**：`app_main` 每 5s 轮询 BQ27220 → `report_battery`（内部去重后才真正发包）
- **占位项**：`ShowText` 目前只填充纯色（无字体渲染）；`Vibrate` 仅打日志（LEDC 电机未接）；
  SD 卡（SDIO 4-bit）只在 `config.h` 定义了引脚，代码未实现

## 7. 分区与烧录

- 自定义分区表 `partitions.csv`：`nvs 16K` / `otadata 8K` / `phy_init 4K` / `ota_0 3M` / `ota_1 3M` / `storage(FAT) ~10M`，16MB Flash
- 双 OTA 槽位与 `ota_data_initial.bin` 已就位，但**固件内没有任何 OTA 升级代码**（无 `esp_ota_*` / `esp_https_ota` 调用），属预留
- 依赖组件：`espressif/esp32-camera 2.1.7`、`espressif/esp_codec_dev 1.6.2`、`espressif/esp_jpeg 1.3.1`

## 8. 已知缺口

1. WiFi 传输后端只有链路层，控制面/数据面是桩函数 —— WiFi 模式下连不上云端
2. 视频上行/下行仅有 API 定义
3. 录音落盘、文件传输、OTA 全部未实现
4. `report_selected_agent` 未实现（`0x16` 事件格式待定）
5. 屏幕文本渲染缺字体，图像下行 `on_show_image` 各板未接
6. README 的 Status 表已略滞后：L2CAP 上行与 ASR 通道实际已完成，表中仍写「Not implemented」
