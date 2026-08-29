// GC2145 Camera: a study-attention monitor on an ST7789 240x240 SPI LCD
//
// The screen shows a line-art cat (cat_screen.cc), not a camera preview: it is drawn once into a PSRAM
// framebuffer at boot and blitted, so standby costs one DrawBitmap and no CPU after that.
//
// The camera is OFF unless a study phase is running. esp_camera_init/deinit bracket each phase, so the
// sensor is not merely ignored while idle — it is powered down and holds no frame buffers. The sequence
// is: session starts -> InitCamera -> ~2s for AE to settle -> snapshots on request -> phase ends ->
// esp_camera_deinit. Breaks deinit too, so the camera is dark for every minute the user is not studying.
//
// Snapshots (JPEG -> App on the SDK's image channel) come from two triggers, both funnelled through one
// flag the capture task consumes: the CAMERA_AUTO_CAPTURE_INTERVAL_MS timer and the Agent's "camera0"
// endpoint. Both are gated on an active study phase, so neither can wake the camera on its own.
//
// Voice: the board's one button (GPIO3) is push-to-talk. Hold it to stream 16kHz mono PCM from the
// ES8311 mic up to the App (agent_link_push_voice), release to end the utterance (agent_link_voice_end);
// the App transcribes and forwards it to the Agent. GPIO3 is not shared with any other peripheral, so
// the key is simply polled by level.
//
// The cat is not quite static: while a study phase is running its whiskers wag, and they stop the moment
// the phase does. That is the only always-visible sign that the device is actually watching — the LED
// says the same thing, but a moving cat reads as "live" from across the room without decoding a colour.
//
// A distracted verdict is answered by the Agent SPEAKING STUDY_ALERT_PHRASE — that voice is the
// reminder. The device also carries the same line as PCM in flash (alert_voice.pcm) and speaks it on a
// timer: the first TTS chunk that reaches the DAC cancels it, so the local copy is only ever heard when
// the cloud failed to deliver. The device is therefore never mute about a distracted verdict, whatever
// the App does — which matters, because for a while the App was the only thing saying anything at all.
//
// Phase changes chime through that same speaker: two rising notes when a pomodoro starts, two falling
// ones when it ends, three rising when the whole session completes. The tones are synthesised into the
// play buffer instead of driving a buzzer, because this board has only two free GPIOs left. They are
// written to the buffer directly rather than through PlayAudio(), so the study-phase speaker gate below
// never swallows them — a phase change is exactly what the user must not miss.
//
// The study feature exposes five MCP endpoints — the SDK serializes every registered endpoint into the
// manifest it pushes on connect, and the cloud derives MCP tools/resources from it:
//   "study_session" (out) Agent-callable: start a session with the plan it worked out, as JSON
//                         {"total_cycles","study_mins","short_break_mins","long_break_mins"}. The user
//                         describes the task ("我要复习两小时数学"); the Agent breaks it down, decides the
//                         schedule, and calls this. Fields are optional and clamped device-side.
//   "study_control" (out) Agent-callable: "pause" / "resume" / "stop" on the running session.
//   "study_verdict" (out) Agent-callable: the result of each photo analysis (true=attentive, false=not).
//                         Attentive is the silent path — the device does nothing at all. Only a false
//                         verdict makes it react: the speaker is unmuted so the Agent's STUDY_ALERT_PHRASE
//                         is spoken, and the LED goes red.
//   "study_status"  (in)  phase changes reported by the device (study_started, break_started,
//                         session_completed, ...) with cycle progress.
//   "camera0"       (out) Agent-callable: take a snapshot now — only during an active study phase.
//   "study_watch"   (in)  which snapshot is currently awaiting a verdict, refreshed after each photo.
//
// The device owns the clock, not the Agent: once a plan is accepted, phase transitions happen locally, so
// the schedule survives a busy or briefly disconnected Agent. The Agent is told about each transition
// (0x04 prompt) so it can coach the user, but it is not in the loop for keeping time.
//
// Camera permission is tied to the session: photos are taken ONLY during a POMODORO_STUDYING phase.
// Idle, paused, and every break gate the camera off — including the Agent's own "camera0" calls.
//
// The speaker is gated the same way, in PlayAudio: while a study phase is running the DAC is muted, so
// "silent when the user is studying properly" is a device-side guarantee rather than a request the Agent
// has to honour — a chatty reply to a per-minute photo would otherwise interrupt exactly the focus this
// is supposed to protect. Outside study phases audio is untouched, so conversation still works.
//
// The gate opens on the QUESTION, not the answer. Every device-initiated prompt arms an answer window
// (STUDY_ANSWER_WINDOW_MS); an `attentive` verdict closes it immediately, a `distracted` one extends it.
// Gating on the verdict instead — waiting for "distracted" before unmuting — could not work: the reply's
// audio arrives over L2CAP while the verdict arrives over the GATT control channel, two independent
// channels the device cannot order, and the Agent's spoken text is usually produced before its
// bookkeeping tool call. Whenever the TTS won that race it was discarded, and the alert the whole
// feature exists for was silently lost. Arming on the question removes the race; closing on `attentive`
// keeps the silence.
//
// What actually makes the Agent answer is the 0x04 Prompt event (agent_link_push_prompt): the App
// forwards that text verbatim to the Agent as if the user had typed it. A 0x19 IoReading is only a data
// update — the App caches it and nothing starts a turn — so the reading alone would leave the photo
// sitting in the chat unanswered. Each snapshot therefore sends both, but they must not say the same
// thing: the prompt carries the instruction, the reading carries which snapshot it refers to. Echoing
// the instruction into the reading (as this once did) hands the model the same order twice, and it read
// as two separate requests. The prompt is deferred by CAMERA_ANALYSIS_PROMPT_DELAY_MS so the image
// (async, streamed over L2CAP) is there before the Agent is asked about it.

#include "board.h"
#include "config.h"
#include "cat_screen.h"
#include "st7789_lcd.h"
#include "ws2812_led.h"
#include "es8311_audio.h"

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "agent_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "cJSON.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

#define TAG "Gc2145Cam"

// The snapshot cadence (CAMERA_AUTO_CAPTURE_INTERVAL_MS) is the only clock for the study check now:
// each timed photo carries its own prompt. Photos are taken ONLY during active Pomodoro study phases,
// not during breaks or when no session is running.

// Pomodoro session states
enum PomodoroPhase : uint8_t {
    POMODORO_IDLE = 0,        // No active session
    POMODORO_STUDYING,        // Active study phase (camera monitoring ON)
    POMODORO_SHORT_BREAK,     // Short break (camera OFF)
    POMODORO_LONG_BREAK,      // Long break (camera OFF)
    POMODORO_PAUSED           // Session paused by user
};

struct PomodoroSession {
    uint8_t  total_cycles = 0;      // Total pomodoros planned
    uint8_t  completed_cycles = 0;  // Pomodoros finished so far
    uint16_t study_mins = POMODORO_STUDY_MINUTES;
    uint16_t short_break_mins = POMODORO_SHORT_BREAK_MINUTES;
    uint16_t long_break_mins = POMODORO_LONG_BREAK_MINUTES;
    uint8_t  cycles_before_long = POMODORO_CYCLES_BEFORE_LONG;

    PomodoroPhase phase = POMODORO_IDLE;
    PomodoroPhase paused_phase = POMODORO_IDLE;  // phase to return to on resume
    TickType_t phase_start_tick = 0;  // When the current phase started
    TickType_t phase_duration_ticks = 0;  // How long this phase should last
    TickType_t paused_remaining_ticks = 0;  // Time left when paused
};

static const char* PhaseToString(PomodoroPhase p) {
    switch (p) {
        case POMODORO_IDLE:        return "idle";
        case POMODORO_STUDYING:    return "studying";
        case POMODORO_SHORT_BREAK: return "short_break";
        case POMODORO_LONG_BREAK:  return "long_break";
        case POMODORO_PAUSED:      return "paused";
        default:                   return "unknown";
    }
}

// One whisker wag cycle, as Q7 fractions of CAT_WHISKER_WAG_PX. Shaped like a sine rather than a
// triangle so the tips ease at the extremes instead of snapping between them. One step per capture-loop
// tick (100ms) -> ~1.6s per full cycle, which reads as a calm twitch rather than a nervous one.
static const int8_t kWagCycle[] = {   0,   49,   90,  118,  127,  118,   90,   49,
                                      0,  -49,  -90, -118, -127, -118,  -90,  -49 };
static constexpr uint8_t kWagSteps = sizeof(kWagCycle) / sizeof(kWagCycle[0]);

class Gc2145CameraBoard : public Board {
public:
    Gc2145CameraBoard() {
        InitLed();                 // independent of the camera; also doubles as a bring-up/fault indicator
        if (lcd_.Init(MakeLcdConfig()) != ESP_OK) { ESP_LOGE(TAG, "LCD init failed"); return; }
        lcd_ok_ = true;

        // Voice comes up FIRST and is not conditional on the camera. The two share no peripheral (SCCB is
        // on I2C port 1, the codec on port 0; DVP capture uses LCD_CAM+GDMA, the codec uses I2S), so a
        // camera fault is no reason to leave the user without a microphone. An earlier arrangement had
        // these after a camera check that could return early, which took voice down with it.
        InitCodec();               // ES8311: mic for push-to-talk + DAC for the Agent's spoken replies
        InitPtt();                 // GPIO3: hold = record voice, release = send

        // Idle screen. Falls back to a solid fill if PSRAM is tight — a blank screen is better than none.
        if (cat_.Init(lcd_.Width(), lcd_.Height()) == ESP_OK) ShowIdleScreen();
        else (void)lcd_.FillSolid(rgb565::kWhite);

        // Bring the camera up once to prove the wiring and log the sensor PID, then shut it back down.
        // Costs ~200ms of boot and means a miswired sensor shows up now rather than at the first session.
        // A failure here disables study monitoring only — voice above is already running.
        if (InitCamera() == ESP_OK) {
            CameraPowerDown();               // idle state: sensor off until a study phase starts
            cam_ok_ = true;
            (void)led_.SetColor(0, 16, 0);   // dim green = board up, camera idle
        } else {
            ESP_LOGE(TAG, "camera init failed — study monitoring unavailable, voice still works");
            (void)led_.SetColor(16, 0, 0);   // dim red = camera fault
        }

        RegisterPomodoroEndpoints(); // Pomodoro session management
        RegisterCaptureEndpoint();   // App/LLM -> snapshot (MCP actuator "camera0")
        RegisterAnalysisEndpoint();  // device -> Agent: the prompt for each timed photo ("study_watch")
        xTaskCreate(&Gc2145CameraBoard::CaptureTaskEntry, "cam_capture", 4096, this, 5, &capture_task_);
        xTaskCreate(&Gc2145CameraBoard::PomodoroTaskEntry, "pomodoro", 4096, this, 5, &pomodoro_task_);
    }

    const char* Name() const override { return "GC2145_CAMERA"; }

    // Hardware present: camera + screen + WS2812 LED + an ES8311 mic for push-to-talk.
    uint32_t Capabilities() const override {
        return AGENT_CAP_CAMERA | AGENT_CAP_SCREEN | AGENT_CAP_LED |
               AGENT_CAP_MIC | AGENT_CAP_SPEAKER | AGENT_CAP_RECORDING | AGENT_CAP_BUTTON;
    }

    // Agent -> Device: the SDK's synthetic "led0" endpoint routes here (0x00RRGGBB) -> drive the WS2812.
    void SetLed(uint32_t rgb) override { (void)led_.SetRgb(rgb); }

    // Agent's spoken reply (TTS downlink, PCM16/16k/mono): push into the play buffer and return
    // immediately — this callback runs in the transport task and must not block. PlayLoop drains it
    // to the DAC. If the buffer is full (link burst > realtime), drop and warn.
    void PlayAudio(const uint8_t* pcm16, size_t bytes) override {
        if (!play_buf_ || !pcm16 || bytes == 0) return;
        // Speaker gate. During a study phase the device stays silent unless something the device itself
        // did opened a window: it asked the Agent a question, or the user pressed PTT. Enforced here
        // rather than trusting the Agent to stay quiet, so "no audio while studying attentively" holds
        // even if the Agent narrates anyway.
        // Outside a study phase (idle / break / paused) audio plays normally — ordinary conversation.
        if (!SpeakerAllowed()) {
            if (!muted_notice_) { muted_notice_ = true; ESP_LOGI(TAG, "TTS muted — studying, no answer window open"); }
            return;
        }
        muted_notice_ = false;
        // The Agent is speaking, so the local fallback chime is not needed — the voice is the reminder.
        chime_due_tick_.store(0, std::memory_order_release);
        if (++play_chunks_ == 1) ESP_LOGI(TAG, "first TTS chunk arrived (%uB) — L2CAP downlink OK", (unsigned)bytes);
        const size_t sent = xStreamBufferSend(play_buf_, pcm16, bytes, 0);  // timeout 0 = non-blocking
        if (sent < bytes)
            ESP_LOGW(TAG, "play buffer full — dropped %u/%u bytes", (unsigned)(bytes - sent), (unsigned)bytes);
    }
    // End of a TTS segment: PlayLoop keeps playing whatever is left in the buffer (not flushed).
    void AudioEnd() override {
        ESP_LOGI(TAG, "AudioEnd (TTS segment done, %u chunk(s) played)", (unsigned)play_chunks_);
        // One question buys one spoken answer, not an open window afterwards. Only close on a segment
        // that actually played: a muted one must not consume the window a later answer still needs.
        if (play_chunks_ > 0) {
            answer_until_tick_.store(0, std::memory_order_release);
            reply_until_tick_.store(0, std::memory_order_release);
        }
        play_chunks_ = 0;
    }

private:
    static constexpr size_t kPlayBufBytes = 16 * 1024;  // ~0.5s @ 16k to absorb link bursts

    // What fired a snapshot: named in the log, and it decides whether the analysis prompt rides along.
    enum : uint8_t { kTrigNone = 0, kTrigApp = 2, kTrigTimer = 3 };   // no kTrigButton — the key is PTT

    // Session requests parked by the endpoint callbacks (host task) for the pomodoro task to apply.
    enum : uint8_t { kPendNone = 0, kPendStart, kPendPause, kPendResume, kPendStop };
    static const char* TrigName(uint8_t t) {
        return t == kTrigApp ? "App" : t == kTrigTimer ? "timer" : "?";
    }

    static St7789LcdConfig MakeLcdConfig() {
        St7789LcdConfig c = {};
        c.spi_host     = DISPLAY_SPI_HOST;
        c.pin_sck      = DISPLAY_SCK_PIN;
        c.pin_mosi     = DISPLAY_MOSI_PIN;
        c.pin_cs       = DISPLAY_CS_PIN;
        c.pin_dc       = DISPLAY_DC_PIN;
        c.pin_rst      = DISPLAY_RST_PIN;
        c.pin_bl       = DISPLAY_BL_PIN;
        c.width        = DISPLAY_WIDTH;
        c.height       = DISPLAY_HEIGHT;
        c.pclk_hz      = DISPLAY_SPI_CLK_HZ;
        c.spi_mode     = DISPLAY_SPI_MODE;
        c.invert_color = true;   // ST7789 default
        return c;
    }

    esp_err_t InitCamera() {
        camera_config_t cc = {};
        cc.pin_pwdn     = CAM_PIN_PWDN;
        cc.pin_reset    = CAM_PIN_RESET;
        cc.pin_xclk     = CAM_PIN_XCLK;
        cc.pin_sccb_sda = CAM_PIN_SIOD;
        cc.pin_sccb_scl = CAM_PIN_SIOC;
        cc.pin_d7       = CAM_PIN_D7;
        cc.pin_d6       = CAM_PIN_D6;
        cc.pin_d5       = CAM_PIN_D5;
        cc.pin_d4       = CAM_PIN_D4;
        cc.pin_d3       = CAM_PIN_D3;
        cc.pin_d2       = CAM_PIN_D2;
        cc.pin_d1       = CAM_PIN_D1;
        cc.pin_d0       = CAM_PIN_D0;
        cc.pin_vsync    = CAM_PIN_VSYNC;
        cc.pin_href     = CAM_PIN_HREF;
        cc.pin_pclk     = CAM_PIN_PCLK;
        cc.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
        cc.ledc_timer   = LEDC_TIMER_0;      // esp32-camera drives XCLK via LEDC
        cc.ledc_channel = LEDC_CHANNEL_0;
        cc.pixel_format = PIXFORMAT_RGB565;  // draw straight to the LCD, no decode
        cc.frame_size   = FRAMESIZE_240X240; // 1:1 with the screen
        cc.fb_count     = 2;                 // double-buffer in PSRAM
        cc.fb_location  = CAMERA_FB_IN_PSRAM;
        cc.grab_mode    = CAMERA_GRAB_LATEST;// always show the freshest frame
        esp_err_t r = esp_camera_init(&cc);
        if (r != ESP_OK) { ESP_LOGE(TAG, "esp_camera_init: %s", esp_err_to_name(r)); return r; }

        sensor_t* s = esp_camera_sensor_get();
        if (s) {
            ESP_LOGI(TAG, "camera sensor PID=0x%04x (GC2145 expected)", s->id.PID);
            // Orientation
            s->set_vflip(s, 0);
            s->set_hmirror(s, 0);
            RestartCaptureEngine(s);
        }
        cam_on_ = true;
        ESP_LOGI(TAG, "camera ready (RGB565 240x240)");
        return ESP_OK;
    }

    // Power the sensor up for a study phase. Idempotent.
    esp_err_t CameraPowerUp() {
        if (cam_on_) return ESP_OK;
        const esp_err_t r = InitCamera();
        if (r != ESP_OK) { ESP_LOGE(TAG, "camera power-up failed: %s", esp_err_to_name(r)); return r; }
        // The driver captures continuously once initialized, so AE/AWB converge on their own; give them
        // a moment before the first snapshot so photo #1 is not under/over-exposed.
        cam_ready_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(kCamSettleMs);
        ESP_LOGI(TAG, "camera ON (study phase) — settling %ums", (unsigned)kCamSettleMs);
        return ESP_OK;
    }

    // Release the sensor and its PSRAM frame buffers. Idempotent. After this the camera holds nothing
    // and cannot produce a frame until CameraPowerUp() runs again.
    void CameraPowerDown() {
        if (!cam_on_) return;
        cam_on_ = false;
        const esp_err_t r = esp_camera_deinit();
        ESP_LOGI(TAG, "camera OFF (deinit=%s)", esp_err_to_name(r));
    }

    // Pulsing CISCTL_restart_n (reg 0xfe bit4, active-low) low->high kicks it into streaming.
    static void RestartCaptureEngine(sensor_t* s) {
        if (!s->set_reg) return;
        s->set_reg(s, 0xfe, 0xff, 0x00);   // CISCTL restart asserted, page 0
        vTaskDelay(pdMS_TO_TICKS(20));
        s->set_reg(s, 0xfe, 0xff, 0x10);   // CISCTL restart released, page 0
        vTaskDelay(pdMS_TO_TICKS(20));
        ESP_LOGI(TAG, "GC2145 capture engine restarted");
    }

    // AEC/AWB are deliberately left running. The old code froze them once converged, which was the right
    // call for a live preview (a static scene otherwise "breathes"), but there is no preview now and only
    // one photo a minute — a snapshot late in a session should be exposed for the light in the room then,
    // not for whatever it was when the session started.

    // Endpoint id shared by the registration and every reading pushed to it.
    static constexpr const char* kAnalysisIoId = "study_watch";

    // The device -> Agent half of the periodic check, as an MCP resource: the manifest turns desc into the
    // resource description the LLM reads, and each photo refreshes the value (RequestAnalysis).
    // This endpoint is context, not the trigger — a reading is a data update the App may or may not
    // surface, so it can never start an Agent turn by itself. The 0x04 prompt sent alongside it does that.
    void RegisterAnalysisEndpoint() {
        static agent_link_io_desc_t desc = {};
        desc.id           = kAnalysisIoId;
        desc.dir          = AGENT_IO_IN;
        desc.kind         = "camera.analysis_request";
        desc.value        = AGENT_VAL_STR;
        desc.desc         = "Timed attention check: snapshot uploaded ~1min during study. Judge newest image, "
                            "call 'study_verdict'. Silent if focused, speak alert if distracted.";
        desc.display_name = "Study Attention Check";
        desc.event        = AGENT_EVT_PERIODIC;
        agent_link_register_io(&desc, nullptr, nullptr);   // IN endpoint: nothing to actuate
    }

    void RegisterPomodoroEndpoints() {
        // 1. Start a study session (Agent-callable)
        static agent_link_io_desc_t session_desc = {};
        session_desc.id           = "study_session";
        session_desc.dir          = AGENT_IO_OUT;
        session_desc.kind         = "study.pomodoro_session";
        session_desc.value        = AGENT_VAL_STR;
        session_desc.desc         = "Start a Pomodoro session. JSON: {total_cycles:u8, study_mins:u16, "
                                    "short_break_mins:u16, long_break_mins:u16}. The DEVICE owns the "
                                    "timer and announces each phase change itself. Those '[设备状态播报]' "
                                    "messages are notifications, NOT user requests — never call this "
                                    "endpoint in response to one. Ignored while a session is already "
                                    "running; call study_control 'stop' first to change the plan.";
        session_desc.display_name = "Start Study Session";
        agent_link_register_io(&session_desc, &Gc2145CameraBoard::OnStartSession, this);

        // 2. Control the session (pause/resume/stop)
        static agent_link_io_desc_t control_desc = {};
        control_desc.id           = "study_control";
        control_desc.dir          = AGENT_IO_OUT;
        control_desc.kind         = "study.control";
        control_desc.value        = AGENT_VAL_STR;
        control_desc.desc         = "Control session: 'pause', 'resume', or 'stop'.";
        control_desc.display_name = "Study Control";
        agent_link_register_io(&control_desc, &Gc2145CameraBoard::OnControlSession, this);

        // 3. Attention verdict for each analysed photo (Agent-callable)
        static agent_link_io_desc_t verdict_desc = {};
        verdict_desc.id           = "study_verdict";
        verdict_desc.dir          = AGENT_IO_OUT;
        verdict_desc.kind         = "study.attention_verdict";
        // A word, not a boolean: 'true' carried no hint of which way round it was, and it came back
        // inverted on every photo — see ParseVerdict. Spelling the outcome out removes the polarity.
        verdict_desc.value        = AGENT_VAL_STR;
        // Kept deliberately short. The first version of this spelled the rules out over four lines and
        // pushed the whole manifest from 1773B to 1939B — every endpoint's chunk boundaries moved with
        // it, and the App started failing calls to endpoints that had not changed at all. The manifest
        // is a shared budget: a description is not free just because it is only one endpoint's.
        verdict_desc.desc         = "Verdict for the newest photo, one word: 'focused' (silent) or "
                                    "'distracted' (device alerts). Not true/false. Call for EVERY photo.";
        verdict_desc.display_name = "Attention Verdict";
        agent_link_register_io(&verdict_desc, &Gc2145CameraBoard::OnVerdict, this);

        // 4. Session status (device -> Agent)
        static agent_link_io_desc_t status_desc = {};
        status_desc.id           = "study_status";
        status_desc.dir          = AGENT_IO_IN;
        status_desc.kind         = "study.status";
        status_desc.value        = AGENT_VAL_STR;
        status_desc.desc         = "Session status: 'study_started'|'break_started'|'cycle_completed'|'session_completed'|'paused'|'resumed'.";
        status_desc.display_name = "Study Status";
        status_desc.event        = AGENT_EVT_PERIODIC;
        agent_link_register_io(&status_desc, nullptr, nullptr);
    }

    // One JSON number -> a clamped uint16. Missing/garbage field keeps the default, so a partial plan
    // from the Agent still starts a sane session instead of a 0-minute one.
    static uint16_t JsonUint(const cJSON* root, const char* key, uint16_t dflt, uint16_t lo, uint16_t hi) {
        const cJSON* it = cJSON_GetObjectItemCaseSensitive(root, key);
        if (!cJSON_IsNumber(it)) return dflt;
        const double v = it->valuedouble;
        if (v < lo) return lo;
        if (v > hi) return hi;
        return static_cast<uint16_t>(v);
    }

    // Both endpoint callbacks run inside the NimBLE host task (GattAccess -> OnCtrlFrame -> cb), so they
    // only stash the request and return; the pomodoro task applies it on its next 1s tick. Doing the work
    // here would parse JSON and, worse, emit BLE notifies (ReportStatus) from inside the GATT write
    // handler. Same reason OnCaptureCmd just sets a flag. Cost is up to 1s of latency on start/pause.
    static void OnStartSession(const char* /*id*/, const uint8_t* args, size_t len, void* ctx) {
        auto* self = static_cast<Gc2145CameraBoard*>(ctx);
        if (!self || !args || len == 0) return;
        if (len > sizeof(self->pending_json_) - 1) {
            ESP_LOGW(TAG, "study_session: %uB payload too large, ignored", (unsigned)len);
            return;
        }
        {
            std::lock_guard<std::mutex> lk(self->pomo_mtx_);
            // A running session is never restarted from here, and that is a loop-breaker, not a nicety.
            // Every phase change is announced to the Agent as a 0x04 Prompt, which the App forwards
            // "verbatim, as if the user had typed it". A model reading "开始专注学习" sees an imperative
            // from the user and does the obvious thing: it calls this endpoint. That restarted the phase,
            // which announced again, which called again. The five identical announcements were that loop
            // — not a resend; the whole send path is single-shot with no retry.
            // Changing the plan mid-session goes through study_control "stop" first.
            if (self->pomo_.phase != POMODORO_IDLE) {
                ESP_LOGW(TAG, "study_session ignored — session already running (phase=%s); stop it first",
                         PhaseToString(self->pomo_.phase));
                return;
            }
            memcpy(self->pending_json_, args, len);
            self->pending_json_[len] = '\0';
        }
        self->pending_cmd_.store(kPendStart, std::memory_order_release);
        // The bytes, not just their count: a 4B "plan" is the App's placeholder, not a schedule, and
        // logging only the length hid that for an entire session's worth of snapshots.
        ESP_LOGI(TAG, "study_session queued (%uB plan): %.*s", (unsigned)len, (int)(len < 96 ? len : 96), (const char*)args);
    }

    // Parse the Agent's plan: {"total_cycles":4,"study_mins":25,"short_break_mins":5,"long_break_mins":15}.
    // Every field is optional and clamped — the Agent decides the plan, but it cannot set a 0-minute phase.
    // Runs on the pomodoro task. Returns false if the JSON was unusable (no session started).
    bool ApplyPlanFromJson() {
        char json[sizeof(pending_json_)];
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            memcpy(json, pending_json_, sizeof(json));
        }
        cJSON* root = cJSON_Parse(json);
        if (!root) { ESP_LOGE(TAG, "study_session: bad JSON, not started: %s", json); return false; }

        // `true`, `1` and `"start"` are all valid JSON, so cJSON_Parse accepts them and the field lookups
        // below then quietly return every default — which is how a session once started as "4 x 25min,
        // 5min, 15min, every 4" without the Agent having chosen a single one of those numbers. The plan
        // still falls back (a session the user asked for should start), but it says so, loudly, because
        // a placeholder arriving where a schedule belongs means the Agent's plan is being dropped in
        // transit and no amount of prompting will fix it.
        if (!cJSON_IsObject(root)) {
            ESP_LOGW(TAG, "study_session: payload '%s' is not a plan object — the Agent's schedule did not "
                          "arrive; starting on device defaults", json);
        }

        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            pomo_.total_cycles      = static_cast<uint8_t>(JsonUint(root, "total_cycles", 4, 1, 24));
            pomo_.study_mins        = JsonUint(root, "study_mins",       POMODORO_STUDY_MINUTES,       1, 120);
            pomo_.short_break_mins  = JsonUint(root, "short_break_mins", POMODORO_SHORT_BREAK_MINUTES, 1, 60);
            pomo_.long_break_mins   = JsonUint(root, "long_break_mins",  POMODORO_LONG_BREAK_MINUTES,  1, 60);
            pomo_.cycles_before_long= static_cast<uint8_t>(JsonUint(root, "cycles_before_long",
                                                                   POMODORO_CYCLES_BEFORE_LONG, 1, 12));
            pomo_.completed_cycles  = 0;
            ESP_LOGI(TAG, "Pomodoro plan: %u x %umin study, %umin short break, %umin long break every %u",
                     (unsigned)pomo_.total_cycles, (unsigned)pomo_.study_mins, (unsigned)pomo_.short_break_mins,
                     (unsigned)pomo_.long_break_mins, (unsigned)pomo_.cycles_before_long);
        }
        cJSON_Delete(root);
        return true;
    }

    // The value arrives as a bare string without a NUL (payload_len delimits it), so compare by length
    // rather than strcmp — and no std::string here, this runs on an SDK thread.
    static bool CmdIs(const uint8_t* args, size_t len, const char* want) {
        const size_t n = strlen(want);
        return len == n && memcmp(args, want, n) == 0;
    }

    enum class Verdict : uint8_t { kAttentive, kDistracted, kUnparsed };

    // The SDK hands 0x33 args through exactly as the App sent them — it does not decode by declared
    // value type (see the 0x33 branch in agent_link.cpp).
    //
    // This endpoint used to be declared BOOL, and that is what broke the whole feature: the Agent wrote
    // "请认真学习" in the chat and then called study_verdict with a 1-byte `true` on every single photo,
    // five for five. `true` is the attentive branch, which slams the answer window shut — so the alert
    // audio the App really did stream down arrived at a speaker that had just been muted, and the only
    // surviving trace was `AudioEnd (0 chunk(s) played)`. Whether the polarity was inverted by the model
    // or the value was collapsed to 1 in transit could not be told apart from a single byte, and it does
    // not matter: a bare boolean has no way to be self-evidently wrong. The endpoint now takes the words
    // 'focused' / 'distracted', which carry their meaning in the payload and cannot be silently flipped.
    //
    // The old encodings are still accepted so an App or model that has not caught up keeps working, and
    // anything else is refused rather than guessed — a wrong "attentive" is invisible, and invisible is
    // exactly what made this take five snapshots and a serial capture to find.
    static Verdict ParseVerdict(const uint8_t* args, size_t len) {
        if (len == 0) return Verdict::kUnparsed;
        if (len == 1 && args[0] <= 1) return args[0] ? Verdict::kAttentive : Verdict::kDistracted;

        // Lower-case, and drop the quotes/whitespace a JSON-ish serialiser may leave around the word.
        char t[16];
        size_t b = 0, e = len;
        while (b < e && (args[b] == ' ' || args[b] == '"' || args[b] == '\'')) ++b;
        while (e > b && (args[e-1] == ' ' || args[e-1] == '"' || args[e-1] == '\'' || args[e-1] == '\n' ||
                         args[e-1] == '\r' || args[e-1] == '\0')) --e;
        const size_t n = (e - b < sizeof(t) - 1) ? (e - b) : sizeof(t) - 1;
        for (size_t i = 0; i < n; ++i) {
            const uint8_t c = args[b + i];
            t[i] = static_cast<char>((c >= 'A' && c <= 'Z') ? c + 32 : c);
        }
        t[n] = '\0';

        // Only the word is trusted for `focused`, and the asymmetry is deliberate.
        //
        // The App does not forward the model's argument: it sends a fixed truthy value coerced to the
        // endpoint's declared type — 1 byte 0x01 while this was BOOL, the 4 bytes "true" now that it is
        // STR. study_session proves the same thing independently: it receives 4B where a plan object
        // should be, and every field falls back to its default. So a truthy payload here carries no
        // information at all — it is indistinguishable from the placeholder, and reading it as
        // "focused" is what silently muted the alert five times out of five.
        //
        // A false-ish payload has the opposite risk profile: the placeholder is never false, so it can
        // only come from something that meant it, and being wrong costs one spurious reminder — audible,
        // obvious, harmless. That one stays accepted. Truthy legacy encodings become undecodable, which
        // leaves the speaker gate open and dumps the bytes, i.e. fails loudly instead of silently.
        if (!strcmp(t, "focused"))
            return Verdict::kAttentive;
        if (!strcmp(t, "distracted") || !strcmp(t, "0") || !strcmp(t, "false") || !strcmp(t, "no")  || !strcmp(t, "off"))
            return Verdict::kDistracted;
        return Verdict::kUnparsed;
    }

    // 0x33 IoActuate for "study_verdict". Runs on the host task, so it only sets state — no sends, no
    // blocking, and the chime is handed to the capture task rather than synthesised here.
    static void OnVerdict(const char* /*id*/, const uint8_t* args, size_t len, void* ctx) {
        auto* self = static_cast<Gc2145CameraBoard*>(ctx);
        if (!self || !args) return;

        // Always log what actually arrived, not just what it was decoded as. The whole boolean fiasco
        // was invisible because the only record was "actuate 'study_verdict' (1B args)" — a length with
        // no content — while the decoded result read as a perfectly ordinary "attentive".
        ESP_LOGI(TAG, "study_verdict payload: %uB '%.*s'", (unsigned)len, (int)(len < 24 ? len : 24), (const char*)args);

        switch (ParseVerdict(args, len)) {
        case Verdict::kAttentive:
            // The quiet path. Shut the answer window so whatever the Agent is about to say is muted,
            // and clear a red left by an earlier verdict — the LED tracks current state, so leaving it
            // red would be stale rather than silent.
            self->answer_until_tick_.store(0, std::memory_order_release);
            if (self->led_red_.exchange(false, std::memory_order_acq_rel)) (void)self->led_.SetColor(0, 32, 0);
            ESP_LOGI(TAG, "verdict: attentive — speaker re-muted, no reaction");
            return;

        case Verdict::kDistracted:
            // Extend the window from now, in case the Agent took longer to answer than the question's
            // own window allowed, and arm the fallback deadline. The first TTS chunk that reaches the
            // DAC disarms it, so the chime only ever fires when the spoken alert did not turn up.
            self->answer_until_tick_.store(xTaskGetTickCount() + pdMS_TO_TICKS(STUDY_ALERT_WINDOW_MS),
                                           std::memory_order_release);
            self->chime_due_tick_.store(xTaskGetTickCount() + pdMS_TO_TICKS(STUDY_ALERT_FALLBACK_MS),
                                        std::memory_order_release);
            self->led_red_.store(true, std::memory_order_release);
            (void)self->led_.SetColor(48, 0, 0);           // red = not studying
            ESP_LOGW(TAG, "verdict: DISTRACTED — speaker open %ums for the alert voice (local chime in %ums if it never comes)",
                     (unsigned)STUDY_ALERT_WINDOW_MS, (unsigned)STUDY_ALERT_FALLBACK_MS);
            return;

        case Verdict::kUnparsed:
        default:
            // Leave every window exactly as it is: the answer this verdict belongs to is still allowed
            // through, and we have not faked a judgement in either direction. The payload is dumped
            // because the only thing worse than an undecodable verdict is one that cannot be diagnosed
            // from the log — that is what cost us the boolean bug above.
            ESP_LOGE(TAG, "verdict: cannot decode %uB payload (want 'focused' / 'distracted') — ignored",
                     (unsigned)len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, args, (len < 32) ? len : 32, ESP_LOG_ERROR);
            return;
        }
    }

    // Host task: classify and stash only (see OnStartSession).
    static void OnControlSession(const char* /*id*/, const uint8_t* args, size_t len, void* ctx) {
        auto* self = static_cast<Gc2145CameraBoard*>(ctx);
        if (!self || !args || len == 0) return;

        uint8_t cmd;
        if      (CmdIs(args, len, "pause"))  cmd = kPendPause;
        else if (CmdIs(args, len, "resume")) cmd = kPendResume;
        else if (CmdIs(args, len, "stop"))   cmd = kPendStop;
        else { ESP_LOGW(TAG, "study_control: unknown command (%uB), want pause/resume/stop", (unsigned)len); return; }

        self->pending_cmd_.store(cmd, std::memory_order_release);
    }

    void RegisterCaptureEndpoint() {
        static agent_link_io_desc_t desc = {};
        desc.id           = "camera0";
        desc.dir          = AGENT_IO_OUT;
        desc.kind         = "camera.capture";
        desc.value        = AGENT_VAL_BOOL;
        // Describes the shutter, nothing else. This used to append CAMERA_ANALYSIS_PROMPT, which put the
        // "analyse the photo and call study_verdict" instruction into the manifest as part of what this
        // tool DOES — a third copy of the same order, permanently in the model's context, attached to
        // the one endpoint that merely takes the picture.
        desc.desc         = "Take one photo now and upload it. Only works during an active study phase.";
        desc.display_name = "Camera Snapshot";
        agent_link_register_io(&desc, &Gc2145CameraBoard::OnCaptureCmd, this);
    }

    // 0x33 IoActuate for "camera0". Runs on an SDK thread, just flag the preview loop to do the capture + encode + send.
    // Gated by monitoring_active_: the camera can only be used when the user has granted permission.
    static void OnCaptureCmd(const char* /*id*/, const uint8_t* /*args*/, size_t /*len*/, void* ctx) {
        auto* self = static_cast<Gc2145CameraBoard*>(ctx);
        if (!self) return;
        if (!self->monitoring_active_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "camera0 DENIED — no active study phase (start a session first)");
            return;
        }
        self->capture_req_.store(kTrigApp, std::memory_order_release);
        ESP_LOGI(TAG, "snapshot requested (App)");
    }

    // Bring up the onboard WS2812 RGB LED (RMT). Safe if it fails — SetLed() just no-ops then.
    void InitLed() {
        if (led_.Init(WS2812_LED_PIN) != ESP_OK) { ESP_LOGE(TAG, "WS2812 init failed"); return; }
        (void)led_.Off();
        ESP_LOGI(TAG, "WS2812 LED ready on GPIO%d (App endpoint: led0)", (int)WS2812_LED_PIN);
    }

    // ── ES8311 codec, full duplex: mic for push-to-talk, DAC for TTS and the Pomodoro chime ──
    void InitCodec() {
        Es8311Config c = {};
        c.i2c_port    = I2C_NUM_0;
        c.pin_sda     = AUDIO_I2C_SDA;
        c.pin_scl     = AUDIO_I2C_SCL;
        c.pin_mclk    = AUDIO_I2S_MCLK;
        c.pin_bclk    = AUDIO_I2S_BCLK;
        c.pin_ws      = AUDIO_I2S_WS;
        c.pin_din     = AUDIO_I2S_DIN;    // mic
        c.pin_dout    = AUDIO_I2S_DOUT;   // speaker (unused)
        c.pin_pa_en   = AUDIO_PA_EN;
        c.es8311_addr = ES8311_ADDR;
        c.sample_rate = AUDIO_SAMPLE_RATE;
        c.mic_gain    = 30;
        c.out_volume  = 80;               // speaker on, for the Agent's spoken replies
        if (codec_.Init(c) != ESP_OK) { ESP_LOGE(TAG, "ES8311 init failed (PTT voice unavailable)"); return; }
        codec_ok_ = true;
        ESP_LOGI(TAG, "ES8311 mic ready (%d Hz)", AUDIO_SAMPLE_RATE);

        // Speaker side: buffer + a task that drains it to the DAC.
        play_buf_ = xStreamBufferCreate(kPlayBufBytes, /*trigger=*/64);
        if (!play_buf_) { ESP_LOGE(TAG, "play buffer alloc failed"); return; }
        xTaskCreate(&Gc2145CameraBoard::PlayTaskEntry, "spk_play", 8192, this, 5, &play_task_);
        ESP_LOGI(TAG, "speaker ready: Agent TTS plays through the ES8311 DAC");
    }

    static void PlayTaskEntry(void* arg) { static_cast<Gc2145CameraBoard*>(arg)->PlayLoop(); }

    // Pull PCM from the play buffer and write it to the codec DAC. Blocks on the buffer.
    void PlayLoop() {
        int16_t buf[256];
        bool first = true;
        while (true) {
            const size_t n = xStreamBufferReceive(play_buf_, buf, sizeof(buf), portMAX_DELAY);
            if (n >= sizeof(int16_t)) {
                if (codec_.WritePcm(buf, n / sizeof(int16_t)) == ESP_OK && first) {
                    first = false;
                    ESP_LOGI(TAG, "PlayLoop: first PCM written to DAC (%u samples)", (unsigned)(n / sizeof(int16_t)));
                }
            }
        }
    }

    // ── Pomodoro chime ──
    // Straight into play_buf_, NOT through PlayAudio(): a phase change is device-initiated feedback the
    // user has to hear, so it must bypass the study-phase mute gate. Writing to the buffer also keeps
    // PlayLoop the only task touching the codec, so a chime that lands mid-TTS queues behind it instead
    // of interleaving into garbage.

    // One square-wave note (freq_hz == 0 -> silence). The short fade in/out is not cosmetic: starting or
    // stopping a waveform at full amplitude puts a step into the DAC, which a small speaker reproduces as
    // a click louder than the note itself.
    void Tone(uint32_t freq_hz, uint32_t ms) {
        if (!play_buf_ || ms == 0) return;
        constexpr uint32_t sr = AUDIO_SAMPLE_RATE;
        const uint32_t total  = sr * ms / 1000u;
        const uint32_t fade   = (sr / 200u) ? (sr / 200u) : 1u;          // 5ms
        const uint32_t inc    = static_cast<uint32_t>((static_cast<uint64_t>(freq_hz) << 16) / sr);
        uint32_t phase = 0;                                              // Q16, wraps on its own
        int16_t chunk[128];
        for (uint32_t done = 0; done < total; ) {
            const uint32_t n = (total - done < 128u) ? (total - done) : 128u;
            for (uint32_t i = 0; i < n; ++i) {
                int32_t v = 0;
                if (freq_hz) {
                    v = (phase & 0x8000u) ? BEEP_AMPLITUDE : -BEEP_AMPLITUDE;
                    phase += inc;
                    const uint32_t idx = done + i;
                    uint32_t g = 256;                                    // Q8 envelope
                    if (idx < fade)              g = 256u * idx / fade;
                    else if (total - idx < fade) g = 256u * (total - idx) / fade;
                    v = v * static_cast<int32_t>(g) / 256;
                }
                chunk[i] = static_cast<int16_t>(v);
            }
            // The buffer holds ~0.5s, so this only waits when TTS is mid-stream. Bail rather than block
            // the pomodoro task indefinitely — a missed chime must not stall the phase clock.
            const size_t want = n * sizeof(int16_t);
            if (xStreamBufferSend(play_buf_, chunk, want, pdMS_TO_TICKS(200)) < want) {
                ESP_LOGW(TAG, "chime truncated — play buffer stayed full");
                return;
            }
            done += n;
        }
    }

    // The spoken alert, from flash. This used to be three high beeps, which said "something is wrong"
    // without saying what — and it only ever plays when the cloud has already failed to deliver the
    // line, so the device has to carry the words itself rather than gesture at them.
    //
    // Same route as the chimes: straight into play_buf_, bypassing the study-phase mute gate, with
    // PlayLoop still the only task touching the codec. ~1.3s of PCM through a ~0.5s buffer means this
    // blocks the caller for roughly a second; that is the capture task, whose only other job in that
    // window is animating whiskers, and an alert that gets spoken is worth a stalled whisker.
    void PlayAlertVoice() {
        extern const uint8_t alert_pcm_start[] asm("_binary_alert_voice_pcm_start");
        extern const uint8_t alert_pcm_end[]   asm("_binary_alert_voice_pcm_end");
        if (!play_buf_) return;
        const size_t total = static_cast<size_t>(alert_pcm_end - alert_pcm_start);
        ESP_LOGI(TAG, "speaking the local alert ('%s', %uB PCM from flash)", STUDY_ALERT_PHRASE, (unsigned)total);
        for (size_t off = 0; off < total; ) {
            const size_t want = ((total - off) < 1024u) ? (total - off) : 1024u;
            // Bail rather than block the capture task indefinitely, exactly as Tone() does: a truncated
            // alert is bad, a wedged snapshot clock is worse.
            if (xStreamBufferSend(play_buf_, alert_pcm_start + off, want, pdMS_TO_TICKS(300)) < want) {
                ESP_LOGW(TAG, "local alert truncated at %u/%uB — play buffer stayed full", (unsigned)off, (unsigned)total);
                return;
            }
            off += want;
        }
    }

    // Direction carries the meaning, so the three are distinguishable without being read as words:
    // rising = starting, falling = stopping, three rising = done for the day.
    void ChimeStudyStart() {
        Tone(BEEP_LOW_HZ, BEEP_TONE_MS);  Tone(0, BEEP_GAP_MS);  Tone(BEEP_HIGH_HZ, BEEP_TONE_MS);
    }
    void ChimeStudyEnd() {
        Tone(BEEP_HIGH_HZ, BEEP_TONE_MS); Tone(0, BEEP_GAP_MS);  Tone(BEEP_LOW_HZ, BEEP_TONE_MS);
    }
    void ChimeSessionDone() {
        Tone(BEEP_LOW_HZ, BEEP_TONE_MS);  Tone(0, BEEP_GAP_MS);
        Tone(BEEP_MID_HZ, BEEP_TONE_MS);  Tone(0, BEEP_GAP_MS);
        Tone(BEEP_HIGH_HZ, BEEP_TONE_MS * 2);
    }

    // ── Push-to-talk key (the board's one button) ──
    // ACTIVE-HIGH: idles low with the internal pull-down, drives high while pressed. GPIO3 is not
    // shared with any other peripheral, so a plain level poll is enough.
    void InitPtt() {
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << PTT_BUTTON_PIN;
        c.mode         = GPIO_MODE_INPUT;
        c.pull_up_en   = GPIO_PULLUP_DISABLE;
        c.pull_down_en = GPIO_PULLDOWN_ENABLE;   // active-high: idle low, pressed = high
        c.intr_type    = GPIO_INTR_DISABLE;      // polled, not interrupt-driven
        gpio_config(&c);
        if (!codec_ok_) { ESP_LOGW(TAG, "codec not ready; PTT voice task not started"); return; }
        xTaskCreate(&Gc2145CameraBoard::PttTaskEntry, "ptt_voice", 4096, this, 5, &ptt_task_);
        ESP_LOGI(TAG, "PTT ready: hold GPIO%d to talk, release to end", (int)PTT_BUTTON_PIN);
    }

    static void PttTaskEntry(void* arg) { static_cast<Gc2145CameraBoard*>(arg)->PttLoop(); }

    // Hold to talk: press starts capture, 20ms frames go up with push_voice; release ends the utterance
    // with voice_end. Same flow as boards/es8311-voice.
    void PttLoop() {
        constexpr size_t kFrame = AUDIO_SAMPLE_RATE / 50;  // 20ms @ 16k = 320 samples
        int16_t buf[kFrame];
        bool talking = false;
        ESP_LOGI(TAG, "PttLoop started, GPIO%d initial level=%d", (int)PTT_BUTTON_PIN, gpio_get_level(PTT_BUTTON_PIN));
        while (true) {
            if (!talking) {
                if (gpio_get_level(PTT_BUTTON_PIN) != 1) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }  // idle: not pressed
                vTaskDelay(pdMS_TO_TICKS(20));                      // debounce
                if (gpio_get_level(PTT_BUTTON_PIN) != 1) continue;  // bounce, ignore
                if (agent_link_state() != AGENT_STATE_READY) {
                    ESP_LOGW(TAG, "PTT pressed but Agent not connected, ignoring");
                    while (gpio_get_level(PTT_BUTTON_PIN) == 1) vTaskDelay(pdMS_TO_TICKS(50));  // wait for release
                    continue;
                }
                if (codec_.StartMic() != ESP_OK) { ESP_LOGE(TAG, "mic start failed"); continue; }
                talking = true;
                vframes_ = 0;
                ESP_LOGI(TAG, "PTT pressed, starting voice uplink");
            } else {
                if (agent_link_state() != AGENT_STATE_READY) {      // link dropped mid-talk
                    agent_link_voice_end(); codec_.StopMic(); talking = false;
                    ESP_LOGW(TAG, "connection lost, ending voice");
                    continue;
                }
                size_t got = 0;
                if (codec_.ReadPcm(buf, kFrame, &got) == ESP_OK && got > 0) {  // blocks ~20ms
                    const esp_err_t r = agent_link_push_voice(reinterpret_cast<const uint8_t*>(buf), got * sizeof(int16_t));
                    if (r != ESP_OK) ESP_LOGW(TAG, "push_voice failed: %s (frame %u)", esp_err_to_name(r), (unsigned)vframes_);
                    ++vframes_;
                }
                if (gpio_get_level(PTT_BUTTON_PIN) != 1) {          // released
                    vTaskDelay(pdMS_TO_TICKS(20));                  // debounce
                    if (gpio_get_level(PTT_BUTTON_PIN) != 1) {
                        agent_link_voice_end(); codec_.StopMic(); talking = false;
                        // The user just asked something — let the answer through even mid-study.
                        reply_until_tick_.store(xTaskGetTickCount() + pdMS_TO_TICKS(STUDY_REPLY_WINDOW_MS),
                                                std::memory_order_release);
                        ESP_LOGI(TAG, "PTT released, ending voice (%u frames sent), reply window open %ums",
                                 (unsigned)vframes_, (unsigned)STUDY_REPLY_WINDOW_MS);
                    }
                }
            }
        }
    }

    // Ask the Agent to analyse the snapshot that just went up. Two sends, different jobs:
    //   0x19 IoReading on "study_watch" — data update, keeps the MCP resource's value current.
    //   0x04 Prompt (push_prompt)       — the trigger. The App forwards this verbatim to the Agent, so
    //                                     this is the one that actually produces an answer.
    // Both strings go out without their NUL — the frame's payload_len delimits them.
    void RequestAnalysis() {
        const char* prompt = CAMERA_ANALYSIS_PROMPT;
        const size_t len   = strlen(prompt);

        // Data, not a restatement of the instruction below. This endpoint's job is to say WHICH photo is
        // outstanding; the prompt says what to do about it. Putting the instruction in both meant the
        // model received the same order down two channels and treated them as two requests.
        // Phrased for a human: the App surfaces this value in the UI, so it has to read as a status line
        // rather than a key=value pair, while still carrying the snapshot number.
        char v[64];
        const int vn = snprintf(v, sizeof v, "第%u张照片，等待判定", (unsigned)snapshot_seq_);
        const esp_err_t rr = (vn > 0) ? agent_link_push_reading(kAnalysisIoId, v, (size_t)vn) : ESP_FAIL;
        ESP_LOGI(TAG, "analysis reading -> %s ('%s', push=%s)", kAnalysisIoId, v, esp_err_to_name(rr));

        // INVALID_SIZE here = the prompt is longer than one BLE notify (see CAMERA_ANALYSIS_PROMPT):
        // shorten it, or the Agent never gets asked. Logged as an error because it silently breaks the feature.
        const esp_err_t rp = agent_link_push_prompt(prompt);
        if (rp == ESP_OK) { ArmAnswerWindow(); ESP_LOGI(TAG, "analysis prompt -> Agent (%uB, 0x04)", (unsigned)len); }
        else ESP_LOGE(TAG, "analysis prompt NOT sent (%uB): %s — Agent will not answer this photo",
                      (unsigned)len, esp_err_to_name(rp));
    }

    // Encode the current RGB565 frame to JPEG and hand it to the SDK's image channel
    // Must run on the ORIGINAL frame bytes, before any LCD byte-swap. send_image returns fast (async worker).
    // ~150ms of encode on the preview task, so the preview visibly hitches for one frame per snapshot.
    void SendSnapshot(camera_fb_t* fb, uint8_t trigger) {
        uint8_t* jpg = nullptr; size_t jpg_len = 0;
        if (!frame2jpg(fb, 80 /*quality*/, &jpg, &jpg_len)) { ESP_LOGE(TAG, "snapshot: frame2jpg failed"); return; }
        // send_image copies the bytes into the SDK's queue and returns; INVALID_STATE here means the App has
        // not opened the image L2CAP channel (or the previous snapshot is still going out) -> just skip this one.
        esp_err_t r = agent_link_send_image(jpg, jpg_len, AGENT_IMG_JPEG,
                                            static_cast<uint16_t>(fb->width), static_cast<uint16_t>(fb->height));
        const uint32_t seq = ++snapshot_seq_;
        ESP_LOGI(TAG, "snapshot #%u (%s): %ux%u -> %uB jpeg, send=%s",
                 (unsigned)seq, TrigName(trigger),
                 (unsigned)fb->width, (unsigned)fb->height, (unsigned)jpg_len, esp_err_to_name(r));
        free(jpg);
        // Device-initiated shots (the timer) carry the prompt; when the Agent itself called "camera0"
        // it already knows what it asked for, so don't talk over its own request.
        // Don't send it here: send_image only QUEUED the bytes, and the prompt is control-plane, so it
        // would overtake the image and the Agent would be asked about a photo it hasn't received.
        // Arm it instead — the preview loop fires it once the worker has had time to drain the stream.
        if (r == ESP_OK && trigger != kTrigApp) {
            prompt_due_tick_ = xTaskGetTickCount() + pdMS_TO_TICKS(CAMERA_ANALYSIS_PROMPT_DELAY_MS);
            prompt_pending_  = true;
            ESP_LOGI(TAG, "analysis prompt armed (+%ums, after the image finishes streaming)",
                     (unsigned)CAMERA_ANALYSIS_PROMPT_DELAY_MS);
        }
    }

    // Fire the armed prompt once its delay has elapsed. Called every frame from the preview loop, so the
    // wait costs nothing — the loop keeps drawing while the image streams. Dropped if the link went away
    // in the meantime: the image never made it, so there is nothing to ask about.
    void ServicePendingPrompt() {
        if (!prompt_pending_) return;
        // Signed difference, so the comparison stays correct across a TickType_t wraparound.
        if (static_cast<int32_t>(xTaskGetTickCount() - prompt_due_tick_) < 0) return;   // not due yet
        prompt_pending_ = false;
        if (agent_link_state() != AGENT_STATE_READY) { ESP_LOGW(TAG, "analysis prompt dropped — link not ready"); return; }
        RequestAnalysis();
    }

    // True when TTS may reach the DAC. Muted only while a study phase is running and no alert is armed;
    // idle / break / paused all play normally so the user can still just talk to the Agent.
    // Lock-free on purpose — this is called from the transport task on every downlink chunk.
    // Drop any armed alert + the red LED. Called on every phase transition so a verdict that lands late
    // (or was never followed by TTS) cannot unmute the next phase.
    void ClearAlert() {
        answer_until_tick_.store(0, std::memory_order_release);
        chime_due_tick_.store(0, std::memory_order_release);
        led_red_.store(false, std::memory_order_release);
    }

    // Signed difference so the comparison survives a TickType_t wraparound. 0 = closed.
    static bool WindowOpen(TickType_t until) {
        return until != 0 && static_cast<int32_t>(until - xTaskGetTickCount()) > 0;
    }

    // One device-initiated question buys one audible answer. Armed the moment the prompt goes out, so
    // the gate never has to guess whether the answer or the verdict will arrive first — see the header.
    void ArmAnswerWindow() {
        answer_until_tick_.store(xTaskGetTickCount() + pdMS_TO_TICKS(STUDY_ANSWER_WINDOW_MS),
                                 std::memory_order_release);
    }

    // True when TTS may reach the DAC. Lock-free — called from the transport task on every chunk.
    //
    // What this suppresses is UNSOLICITED audio during a study phase: the Agent receives a photo every
    // minute, and if it comments on each one the speaker talks over the very focus this is protecting.
    // What it must NOT suppress is an answer to something that was actually asked. Two windows open it:
    //   - answer window: armed when the DEVICE asks (photo analysis, phase announcement); closed by an
    //     `attentive` verdict, extended by a `distracted` one.
    //   - reply window:  armed when the user releases PTT, so their own question gets an audible answer.
    // Outside a study phase nothing is gated at all.
    bool SpeakerAllowed() const {
        if (!monitoring_active_.load(std::memory_order_acquire)) return true;   // not studying: normal audio
        return WindowOpen(answer_until_tick_.load(std::memory_order_acquire)) ||
               WindowOpen(reply_until_tick_.load(std::memory_order_acquire));
    }

    // ── Pomodoro Session Management ──
    // pomo_ is written from two threads: the SDK callback thread (study_session / study_control) and the
    // pomodoro task (phase advance). Every mutation goes through these methods holding pomo_mtx_.
    // monitoring_active_ stays atomic — the preview task reads it every frame and must not block.

    void StartStudyPhase() {
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            pomo_.phase = POMODORO_STUDYING;
            pomo_.phase_start_tick = xTaskGetTickCount();
            pomo_.phase_duration_ticks = pdMS_TO_TICKS((uint32_t)pomo_.study_mins * 60u * 1000u);
            ESP_LOGI(TAG, "STUDY phase started (cycle %u/%u, %u mins)",
                     (unsigned)(pomo_.completed_cycles + 1), (unsigned)pomo_.total_cycles, (unsigned)pomo_.study_mins);
        }
        ClearAlert();                      // a verdict from the previous phase must not carry over
        (void)CameraPowerUp();             // sensor on for this phase only
        auto_tick_ = xTaskGetTickCount();  // first photo one interval in, not immediately
        monitoring_active_.store(true, std::memory_order_release);
        (void)led_.SetColor(0, 32, 0);     // green = studying
        ChimeStudyStart();                 // a pomodoro begins
        ReportStatus("study_started", "已进入专注学习阶段");
    }

    void StartBreakPhase(bool is_long) {
        uint16_t mins;
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            pomo_.phase = is_long ? POMODORO_LONG_BREAK : POMODORO_SHORT_BREAK;
            pomo_.phase_start_tick = xTaskGetTickCount();
            mins = is_long ? pomo_.long_break_mins : pomo_.short_break_mins;
            pomo_.phase_duration_ticks = pdMS_TO_TICKS((uint32_t)mins * 60u * 1000u);
        }
        ClearAlert();
        monitoring_active_.store(false, std::memory_order_release);   // no photos on a break
        CameraPowerDown();                                            // and the sensor is off, not just idle
        (void)led_.SetColor(32, 16, 0);                               // orange = break
        ChimeStudyEnd();                                              // a pomodoro just ended
        ESP_LOGI(TAG, "%s BREAK started (%u mins)", is_long ? "LONG" : "SHORT", (unsigned)mins);
        char note[64];
        snprintf(note, sizeof(note), "已进入%s休息，共%u分钟", is_long ? "长" : "短", (unsigned)mins);
        ReportStatus(is_long ? "long_break_started" : "short_break_started", note);
    }

    void PauseSession() {
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            if (pomo_.phase == POMODORO_IDLE || pomo_.phase == POMODORO_PAUSED) return;
            const TickType_t elapsed = xTaskGetTickCount() - pomo_.phase_start_tick;
            pomo_.paused_remaining_ticks = (elapsed < pomo_.phase_duration_ticks)
                                         ? (pomo_.phase_duration_ticks - elapsed) : 0;
            pomo_.paused_phase = pomo_.phase;      // remember what to go back to
            pomo_.phase = POMODORO_PAUSED;
        }
        monitoring_active_.store(false, std::memory_order_release);
        CameraPowerDown();                         // paused counts as not studying
        (void)led_.SetColor(16, 16, 16);           // dim white = paused
        ESP_LOGI(TAG, "Session PAUSED");
        ReportStatus("paused", "已暂停");
    }

    // Resume back into whatever phase was interrupted, with the time that was left on it — resuming a
    // break into a study phase would silently skip the rest of the break.
    void ResumeSession() {
        bool study;
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            if (pomo_.phase != POMODORO_PAUSED) return;
            pomo_.phase = pomo_.paused_phase;
            pomo_.phase_start_tick = xTaskGetTickCount();
            pomo_.phase_duration_ticks = pomo_.paused_remaining_ticks;
            study = (pomo_.phase == POMODORO_STUDYING);
        }
        if (study) { (void)CameraPowerUp(); auto_tick_ = xTaskGetTickCount(); }
        monitoring_active_.store(study, std::memory_order_release);
        (void)led_.SetColor(study ? 0 : 32, study ? 32 : 16, 0);
        if (study) ChimeStudyStart();      // resuming into a study phase is a pomodoro starting
        ESP_LOGI(TAG, "Session RESUMED (%s)", study ? "studying" : "break");
        ReportStatus("resumed", study ? "已恢复到学习阶段" : "已恢复到休息阶段");
    }

    // done=true when the plan ran to completion; false when the user/Agent ended it early.
    void StopSession(bool done = false) {
        uint8_t made, want;
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            if (pomo_.phase == POMODORO_IDLE) return;   // already stopped: nothing happened, announce nothing
            pomo_.phase = POMODORO_IDLE;
            made = pomo_.completed_cycles;
            want = pomo_.total_cycles;
        }
        monitoring_active_.store(false, std::memory_order_release);
        CameraPowerDown();                         // session over: sensor off until the next one
        ClearAlert();
        (void)led_.SetColor(0, 0, 0);
        // The final pomodoro ends here, not in StartBreakPhase, so it needs its own chime. An early
        // stop is the user's own doing and gets none — they already know.
        if (done) ChimeSessionDone();
        ESP_LOGI(TAG, "Session %s (%u/%u cycles)", done ? "COMPLETE" : "STOPPED", (unsigned)made, (unsigned)want);
        char note[80];
        if (done) snprintf(note, sizeof(note), "全部%u个番茄钟已完成，学习结束", (unsigned)want);
        else      snprintf(note, sizeof(note), "学习已结束，完成%u/%u个番茄钟", (unsigned)made, (unsigned)want);
        ReportStatus(done ? "session_completed" : "session_stopped", note);
    }

    // Two sends per phase change, same split as the photo path: the reading keeps the MCP resource current,
    // the prompt is what actually reaches the Agent so it can coach the user at the transition.
    // `note` is the human-readable half — it goes to the Agent as the prompt.
    void ReportStatus(const char* status, const char* note) {
        uint8_t made, want;
        const char* phase_name;
        {
            std::lock_guard<std::mutex> lk(pomo_mtx_);
            made = pomo_.completed_cycles;
            want = pomo_.total_cycles;
            phase_name = PhaseToString(pomo_.phase);
        }

        // Belt and braces behind the guards above: if the exact same transition is reported twice in a
        // row, the second one carries no information and must not reach the Agent — an identical prompt
        // is what a restart loop looks like from the far end.
        if (strcmp(status, last_status_) == 0 && made == last_made_ && want == last_want_) {
            ESP_LOGW(TAG, "status '%s' unchanged since the last report — not announced again", status);
            return;
        }
        snprintf(last_status_, sizeof(last_status_), "%s", status);
        last_made_ = made;
        last_want_ = want;

        char buf[160];
        int n = snprintf(buf, sizeof(buf), "%s|phase=%s|cycle=%u/%u", status, phase_name, (unsigned)made, (unsigned)want);
        if (n > 0) (void)agent_link_push_reading("study_status", buf, (size_t)n);

        // Declarative, and labelled as a device notification. This text reaches the model through the
        // same channel as the user's own speech, so an imperative here is indistinguishable from an
        // order — which is exactly how the restart loop got started. Phrase it as something that has
        // already happened; the "do not act on this" rule lives in study_session's manifest desc, which
        // has room for it and does not eat into the single-notify budget every announcement pays.
        n = snprintf(buf, sizeof(buf), "[设备状态播报] %s（进度 %u/%u 个番茄钟）", note, (unsigned)made, (unsigned)want);
        if (n > 0) {
            const esp_err_t r = agent_link_push_prompt(buf);
            // Same rule as the analysis prompt: the device asked, so one answer may be spoken. Without
            // this, the Agent's reply to "开始专注学习" landed after monitoring_active_ had already gone
            // true and was muted — the user heard nothing at the one moment they were being addressed.
            if (r == ESP_OK) ArmAnswerWindow();
            else             ESP_LOGW(TAG, "status prompt not sent: %s", esp_err_to_name(r));
        }
    }

    static void PomodoroTaskEntry(void* arg) { static_cast<Gc2145CameraBoard*>(arg)->PomodoroLoop(); }

    // Advances the phase clock. Only this task calls the Start*/Stop transitions after a session begins,
    // so a phase can't be advanced twice; the control endpoint's pause/resume/stop mutate under the same lock.
    void PomodoroLoop() {
        ESP_LOGI(TAG, "Pomodoro task started");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));   // 1s resolution is plenty for minute-scale phases

            // Apply whatever the Agent asked for since the last tick (queued by the endpoint callbacks).
            switch (pending_cmd_.exchange(kPendNone, std::memory_order_acq_rel)) {
            case kPendStart:  if (ApplyPlanFromJson()) StartStudyPhase(); break;
            case kPendPause:  PauseSession();  break;
            case kPendResume: ResumeSession(); break;
            case kPendStop:   StopSession(/*done=*/false); break;
            default: break;
            }

            // Snapshot what we need under the lock, then act outside it (the transitions take it again).
            PomodoroPhase phase;
            bool          expired;
            bool          need_long = false;
            bool          last_cycle = false;
            {
                std::lock_guard<std::mutex> lk(pomo_mtx_);
                phase = pomo_.phase;
                if (phase == POMODORO_IDLE || phase == POMODORO_PAUSED) continue;
                expired = (xTaskGetTickCount() - pomo_.phase_start_tick) >= pomo_.phase_duration_ticks;
                if (expired && phase == POMODORO_STUDYING) {
                    pomo_.completed_cycles++;    // the cycle that just ended
                    last_cycle = pomo_.completed_cycles >= pomo_.total_cycles;
                    need_long  = !last_cycle && (pomo_.completed_cycles % pomo_.cycles_before_long) == 0;
                    ESP_LOGI(TAG, "Cycle %u/%u complete", (unsigned)pomo_.completed_cycles, (unsigned)pomo_.total_cycles);
                }
            }
            if (!expired) continue;

            if (phase == POMODORO_STUDYING) {
                if (last_cycle) StopSession(/*done=*/true);   // reports session_completed itself
                else            StartBreakPhase(need_long);
            } else {
                StartStudyPhase();                            // break over, back to work
            }
        }
    }

    // Blit the idle cat. Cheap enough to call on every transition; no-op if PSRAM was short at boot.
    void ShowIdleScreen() {
        if (!cat_.Ready()) return;
        (void)lcd_.DrawBitmap(0, 0, cat_.Width(), cat_.Height(), cat_.Pixels());
        DrawWhiskerBand(0);   // the framebuffer above has no whiskers in it; they live in the band
    }

    // ~40 rows instead of 240. DrawBitmap flushes before returning, so consecutive calls are safe and
    // the band's source buffer can be rebuilt immediately.
    void DrawWhiskerBand(int wag_px) {
        if (!cat_.Animated()) return;   // band alloc failed at boot: whiskers are baked in at rest
        const void* px = cat_.RenderBand(wag_px);
        if (px) (void)lcd_.DrawBitmap(0, cat_.BandTop(), cat_.Width(), cat_.BandHeight(), px);
    }

    // Wag only while a study phase is actually running. Breaks, pause and idle all leave the cat dead
    // still, so "nothing is moving" is a truthful signal that nothing is being watched.
    void ServiceWhiskers(bool studying) {
        if (!cat_.Animated()) return;
        if (!studying) {
            // Settle back to rest exactly once, or an interrupted wag leaves the whiskers frozen
            // mid-swing — which looks like a crashed screen rather than a stopped session.
            if (wag_step_ != 0) { wag_step_ = 0; DrawWhiskerBand(0); }
            return;
        }
        wag_step_ = static_cast<uint8_t>((wag_step_ + 1) % kWagSteps);
        DrawWhiskerBand(kWagCycle[wag_step_] * CAT_WHISKER_WAG_PX / 127);
    }

    static void CaptureTaskEntry(void* arg) { static_cast<Gc2145CameraBoard*>(arg)->CaptureLoop(); }

    // Idles at 100ms. It only ever calls esp_camera_fb_get when a snapshot was actually requested AND the
    // sensor is powered, so with the camera off this loop does nothing but tick. It is also the only task
    // that touches the screen after boot: it advances the whisker wag one step per tick, so the LCD keeps
    // a single writer and the animation costs one ~40-row blit rather than a full frame.
    void CaptureLoop() {
        ESP_LOGI(TAG, "capture task started (camera powered only during study phases)");
#if CAMERA_AUTO_CAPTURE_INTERVAL_MS > 0
        const TickType_t auto_period = pdMS_TO_TICKS(CAMERA_AUTO_CAPTURE_INTERVAL_MS);
        ESP_LOGI(TAG, "auto snapshot every %u ms while studying", (unsigned)CAMERA_AUTO_CAPTURE_INTERVAL_MS);
#endif
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));

            const bool studying = monitoring_active_.load(std::memory_order_acquire);
            ServiceWhiskers(studying);

            // Fallback chime, if the alert voice never arrived. Serviced here rather than in OnVerdict
            // because that runs on the NimBLE host task and Tone() waits on the play buffer.
            const TickType_t due = chime_due_tick_.load(std::memory_order_acquire);
            if (due != 0 && static_cast<int32_t>(xTaskGetTickCount() - due) >= 0) {
                chime_due_tick_.store(0, std::memory_order_release);
                ESP_LOGW(TAG, "no alert voice from the cloud within %ums — speaking the local clip instead",
                         (unsigned)STUDY_ALERT_FALLBACK_MS);
                PlayAlertVoice();
            }

#if CAMERA_AUTO_CAPTURE_INTERVAL_MS > 0
            // Timed snapshot. Only while studying, the link is up, and the sensor has finished settling —
            // otherwise drop the slot and re-arm so the next photo is one full interval away.
            if (studying && (xTaskGetTickCount() - auto_tick_) >= auto_period) {
                auto_tick_ = xTaskGetTickCount();
                if (agent_link_state() == AGENT_STATE_READY && CameraSettled()) {
                    capture_req_.store(kTrigTimer, std::memory_order_release);
                    ESP_LOGI(TAG, "snapshot requested (timer)");
                } else {
                    ESP_LOGD(TAG, "timed snapshot skipped — link not ready or camera still settling");
                }
            }
#endif
            const uint8_t trig = capture_req_.exchange(kTrigNone, std::memory_order_acq_rel);
            if (trig != kTrigNone) GrabAndSend(trig);

            // The prompt for the photo above, once the image has had time to go out (see SendSnapshot).
            ServicePendingPrompt();
        }
    }

    bool CameraSettled() const {
        return cam_on_ && static_cast<int32_t>(xTaskGetTickCount() - cam_ready_tick_) >= 0;
    }

    // Grab one frame, mask the noisy top rows, encode + send. Refuses if the sensor is not powered —
    // a stale request must never be what turns the camera on.
    void GrabAndSend(uint8_t trigger) {
        if (!cam_on_) { ESP_LOGW(TAG, "snapshot dropped — camera is off"); return; }

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { ESP_LOGW(TAG, "fb_get failed"); return; }

#if CAMERA_MASK_TOP_ROWS > 0
        // Overpaint the sensor's noisy top rows with the first clean row below, so the flickering
        // black/white band does not reach the Agent. Must happen before the JPEG encode.
        {
            uint8_t* b = static_cast<uint8_t*>(fb->buf);
            const size_t rb = static_cast<size_t>(fb->width) * 2u;
            for (int r = 0; r < CAMERA_MASK_TOP_ROWS; ++r)
                memcpy(b + static_cast<size_t>(r) * rb, b + static_cast<size_t>(CAMERA_MASK_TOP_ROWS) * rb, rb);
        }
#endif
        SendSnapshot(fb, trigger);
        esp_camera_fb_return(fb);
    }

    static constexpr uint32_t kCamSettleMs = 2000;  // AE/AWB convergence allowance after power-up

    St7789Lcd            lcd_;
    Ws2812Led            led_;
    Es8311Codec          codec_;
    CatScreen            cat_;                   // idle screen, drawn once at boot
    bool                 lcd_ok_       = false;
    bool                 cam_ok_       = false;  // sensor verified at boot (not "currently on")
    // Camera power state. Touched by the pomodoro task (phase transitions) and read by the capture task;
    // both are single writers in practice — only the pomodoro task calls PowerUp/PowerDown.
    volatile bool        cam_on_       = false;
    TickType_t           cam_ready_tick_ = 0;    // snapshots allowed once this tick passes
    bool                 codec_ok_     = false;
    uint32_t             vframes_      = 0;      // PCM frames pushed in the current PTT utterance
    uint32_t             play_chunks_  = 0;      // TTS chunks received in the current segment (diagnostic)
    StreamBufferHandle_t play_buf_     = nullptr;
    TaskHandle_t         play_task_    = nullptr;
    TaskHandle_t         capture_task_ = nullptr;
    TaskHandle_t         ptt_task_     = nullptr;
    TaskHandle_t         pomodoro_task_ = nullptr;
    std::atomic<uint8_t> capture_req_{kTrigNone};  // set by timer / BOOT button / App command; consumed by the preview loop
    uint32_t             snapshot_seq_ = 0;        // running snapshot number, reported in the 0x64 notice
    // Study monitoring control: camera is active ONLY during POMODORO_STUDYING phases
    std::atomic<bool>    monitoring_active_{false};
    TickType_t           auto_tick_ = 0;           // phase of the timed-snapshot clock (preview task only)
    uint8_t              wag_step_  = 0;           // index into kWagCycle; capture task only
    // Pomodoro session state. Guarded by pomo_mtx_: written by the SDK callback thread (study_session /
    // study_control) and by the pomodoro task (phase advance).
    PomodoroSession      pomo_;
    std::mutex           pomo_mtx_;
    // Endpoint callback -> pomodoro task handoff. pending_json_ holds the plan for kPendStart (guarded by
    // pomo_mtx_); pending_cmd_ is the atomic doorbell the loop drains each tick.
    std::atomic<uint8_t> pending_cmd_{kPendNone};
    char                 pending_json_[257] = {};
    // Last announced transition, for the dedup in ReportStatus (pomodoro task only).
    char                 last_status_[32] = {};
    uint8_t              last_made_ = 0xFF;
    uint8_t              last_want_ = 0xFF;
    // Speaker gate: 0 = muted while studying; otherwise the tick the answer window closes at. Armed
    // whenever the device sends a prompt, closed by an `attentive` verdict or once an answer has
    // played. Written from the pomodoro/capture tasks and the host task, read by PlayAudio on the
    // transport task — atomic, never locked.
    std::atomic<TickType_t> answer_until_tick_{0};
    // Fallback-chime deadline (0 = nothing pending). Armed by a distracted verdict on the host task,
    // disarmed by PlayAudio the moment real TTS gets through, fired by the capture task if it expires.
    std::atomic<TickType_t> chime_due_tick_{0};
    // Armed on PTT release: the user asked something, so the Agent's answer must be audible even during
    // a study phase. Without this the gate silently swallowed every reply the user asked for.
    std::atomic<TickType_t> reply_until_tick_{0};
    std::atomic<bool>    led_red_{false};      // a distracted verdict is showing on the LED
    bool                 muted_notice_ = false; // log "muted" once per run of dropped chunks, not per chunk
    // Deferred analysis prompt: armed by SendSnapshot, fired by ServicePendingPrompt once the image
    // has had CAMERA_ANALYSIS_PROMPT_DELAY_MS to stream out. Both touched only by the preview task.
    bool                 prompt_pending_  = false;
    TickType_t           prompt_due_tick_ = 0;
};

DECLARE_BOARD(Gc2145CameraBoard);
