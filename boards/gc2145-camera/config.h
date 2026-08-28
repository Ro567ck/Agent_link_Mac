#ifndef _AGENT_LINK_BOARD_CONFIG_H_
#define _AGENT_LINK_BOARD_CONFIG_H_

#include <driver/gpio.h>

// GC2145 Camera: live DVP camera preview on an ST7789 240x240 SPI LCD

// Camera: DVP 8-bit + SCCB(I2C) + XCLK. -1 = not wired
#define CAM_PIN_PWDN            -1              // power-down
#define CAM_PIN_RESET           -1              // reset
#define CAM_PIN_XCLK            GPIO_NUM_15     // master clock out to sensor
#define CAM_PIN_SIOD            GPIO_NUM_4      // SCCB SDA
#define CAM_PIN_SIOC            GPIO_NUM_5      // SCCB SCL
#define CAM_PIN_D7              GPIO_NUM_16     // Y9
#define CAM_PIN_D6              GPIO_NUM_17     // Y8
#define CAM_PIN_D5              GPIO_NUM_18     // Y7
#define CAM_PIN_D4              GPIO_NUM_12     // Y6
#define CAM_PIN_D3              GPIO_NUM_10     // Y5
#define CAM_PIN_D2              GPIO_NUM_8      // Y4  (also PTT_BUTTON_PIN below — see the warning there)
#define CAM_PIN_D1              GPIO_NUM_9      // Y3
#define CAM_PIN_D0              GPIO_NUM_11     // Y2
#define CAM_PIN_VSYNC           GPIO_NUM_6
#define CAM_PIN_HREF            GPIO_NUM_7
#define CAM_PIN_PCLK            GPIO_NUM_13

#define CAM_XCLK_FREQ_HZ        (20 * 1000 * 1000)

// The board's one button, used for push-to-talk: hold to record voice, release to send.
// Active-HIGH: idles low with the internal pull-down, driven high while pressed. GPIO3 is not shared
// with any other peripheral, so it can simply be polled by level (no frame-data detection needed).
#define PTT_BUTTON_PIN          GPIO_NUM_3

// Automatic snapshot interval (ms). The timer only fires during active study periods
// (when a Pomodoro study phase is running — not during breaks).
// Only fires while the link is READY; the button and the App's "camera0" endpoint work regardless.
#define CAMERA_AUTO_CAPTURE_INTERVAL_MS (60 * 1000)

// What the Agent should do with each auto snapshot. Sent as a 0x04 Prompt event (the channel the App
// forwards verbatim to the Agent) right after the photo goes up, so image + instruction land as one
// logical request. Also mirrored onto the "study_watch" MCP endpoint as a reading, for data semantics.
// Keep it short: a prompt must fit one BLE notify (ATT_MTU-9, ~238B at MTU 247). UTF-8 Chinese = 3B/char.
// The detailed rules (what to do with each outcome) live in the study_verdict endpoint description, which
// has far more room and lands in the model's context via the manifest — this only has to trigger the check.
#define CAMERA_ANALYSIS_PROMPT "分析照片：他在认真学习吗？按study_verdict端点的规则回复"

// Spoken when the Agent reports the user is NOT studying attentively. The Agent is asked to say exactly
// this; the device only unmutes the speaker for it. Attentive -> the device stays silent, no reaction.
#define STUDY_ALERT_PHRASE "请认真学习"

// How long the speaker stays unmuted after a "distracted" verdict (ms). The alert TTS has to arrive
// within this window or it is muted like anything else — this is what stops a late/unrelated reply from
// slipping through the gate. Generous enough for cloud TTS round-trip.
#define STUDY_ALERT_WINDOW_MS 15000

// How long the speaker stays unmuted after the user releases the PTT button (ms). Their own question has
// to get an audible answer even in the middle of a study phase, or the device just looks broken. Covers
// ASR + Agent + TTS round-trip; the window also closes as soon as the reply finishes playing.
#define STUDY_REPLY_WINDOW_MS 30000

// How long to wait after queueing the JPEG before sending the prompt (ms). send_image returns as soon
// as the bytes are queued; a background worker then streams them over L2CAP, which takes ~1-2s for a
// 240x240 q80 frame. The prompt is control-plane and would otherwise overtake the image, so the Agent
// could start answering before it has anything to look at. 0 = send the prompt immediately.
#define CAMERA_ANALYSIS_PROMPT_DELAY_MS 2000

// ── Pomodoro Timer Defaults (Agent can override these when starting a session) ──
#define POMODORO_STUDY_MINUTES     25    // Study phase duration
#define POMODORO_SHORT_BREAK_MINUTES 5   // Short break after each pomodoro
#define POMODORO_LONG_BREAK_MINUTES 15   // Long break after every 4 pomodoros
#define POMODORO_CYCLES_BEFORE_LONG 4    // How many pomodoros before a long break

// Onboard WS2812 (NeoPixel) RGB LED on GPIO48, driven over RMT. Exposed to the App as the "led0" endpoint.
#define WS2812_LED_PIN          GPIO_NUM_48

// The sensor's top rows are noisy; overpaint them with the first clean row below so the flickering
// band does not end up in the snapshot the Agent analyses.
#define CAMERA_MASK_TOP_ROWS 8

// Idle screen (line-art cat) pixel byte order. The camera fed the panel with no swap, so 0 matches
// what worked there. Set to 1 if red and blue look swapped on the idle screen.
#define CAT_SCREEN_BYTE_SWAP 0

// Display: ST7789 240x240
#define DISPLAY_SPI_HOST        SPI2_HOST
#define DISPLAY_SCK_PIN         GPIO_NUM_21     // SCL
#define DISPLAY_MOSI_PIN        GPIO_NUM_47     // SDA
#define DISPLAY_DC_PIN          GPIO_NUM_43     //
#define DISPLAY_CS_PIN          GPIO_NUM_44     //
#define DISPLAY_RST_PIN         (-1)
#define DISPLAY_BL_PIN          (-1)
#define DISPLAY_WIDTH           240
#define DISPLAY_HEIGHT          240

#define DISPLAY_SPI_CLK_HZ      (40 * 1000 * 1000)
#define DISPLAY_SPI_MODE        0

// ── Audio: one ES8311 codec, mic-only on this board (speaker pins defined for completeness) ──
// Same wiring as boards/es8311-voice; all of these GPIOs are free on this board.
// The shared Es8311Codec driver (boards/common) takes these.
#define AUDIO_I2S_MCLK          GPIO_NUM_46   // MCK
#define AUDIO_I2S_BCLK          GPIO_NUM_39   // SCLK
#define AUDIO_I2S_WS            GPIO_NUM_2    // LR / LRCK
#define AUDIO_I2S_DIN           GPIO_NUM_40   // DO : mic  (ASDOUT on the codec)
#define AUDIO_I2S_DOUT          GPIO_NUM_38   // DI : speaker (the Agent's spoken replies)
#define AUDIO_PA_EN             GPIO_NUM_NC   // no external PA
#define AUDIO_I2C_SDA           GPIO_NUM_41
#define AUDIO_I2C_SCL           GPIO_NUM_42
#define ES8311_ADDR             0x18          // AD0 low; 0x19 if AD0 is tied high
#define AUDIO_SAMPLE_RATE       16000

#endif  // _AGENT_LINK_BOARD_CONFIG_H_
