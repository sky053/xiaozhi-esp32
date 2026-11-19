#include "wifi_board.h"
#include "adc_pdm_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <vector>
#include <functional>
#include <string_view>
#include <cstring>
#include <math.h>

#include "display/lcd_display.h"
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include "esp_lcd_ili9341.h"

#include "assets/lang_config.h"
#include "anim_player.h"
#include "emoji_display.h"
#include "servo_dog_ctrl.h"
#include "led_strip.h"
#include "driver/rmt_tx.h"
#include "device_state.h"
#include <esp_timer.h>
#include <esp_system.h>

#include "sdkconfig.h"

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
#include "esp_hi_web_control.h"
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED

#define TAG "ESP_HI"

// Named static payloads: C++ compound-literal pointers must outlive panel init.
static const uint8_t st7735_cmd_b1[] = {0x05, 0x3A, 0x3A};
static const uint8_t st7735_cmd_b2[] = {0x05, 0x3A, 0x3A};
static const uint8_t st7735_cmd_b3[] = {0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A};
static const uint8_t st7735_cmd_b4[] = {0x03};
static const uint8_t st7735_cmd_c0[] = {0x44, 0x04, 0x04};
static const uint8_t st7735_cmd_c1[] = {0xC0};
static const uint8_t st7735_cmd_c2[] = {0x0D, 0x00};
static const uint8_t st7735_cmd_c3[] = {0x8D, 0x6A};
static const uint8_t st7735_cmd_c4[] = {0x8D, 0xEE};
static const uint8_t st7735_cmd_c5[] = {0x08};
static const uint8_t st7735_cmd_e0[] = {0x0F, 0x10, 0x03, 0x03, 0x07, 0x02, 0x00, 0x02,
                                        0x07, 0x0C, 0x13, 0x38, 0x0A, 0x0E, 0x03, 0x10};
static const uint8_t st7735_cmd_e1[] = {0x10, 0x0B, 0x04, 0x04, 0x10, 0x03, 0x00, 0x03,
                                        0x03, 0x09, 0x17, 0x33, 0x0B, 0x0C, 0x06, 0x10};
static const uint8_t st7735_cmd_35[] = {0x00};
static const uint8_t st7735_cmd_3a[] = {0x05};
static const uint8_t st7735_cmd_36[] = {0xC8};

static const ili9341_lcd_init_cmd_t vendor_specific_init[] = {
    {0x11, NULL, 0, 120},  // Sleep out, Delay 120ms
    {0xB1, st7735_cmd_b1, sizeof(st7735_cmd_b1), 0},
    {0xB2, st7735_cmd_b2, sizeof(st7735_cmd_b2), 0},
    {0xB3, st7735_cmd_b3, sizeof(st7735_cmd_b3), 0},
    {0xB4, st7735_cmd_b4, sizeof(st7735_cmd_b4), 0},  // Dot inversion
    {0xC0, st7735_cmd_c0, sizeof(st7735_cmd_c0), 0},
    {0xC1, st7735_cmd_c1, sizeof(st7735_cmd_c1), 0},
    {0xC2, st7735_cmd_c2, sizeof(st7735_cmd_c2), 0},
    {0xC3, st7735_cmd_c3, sizeof(st7735_cmd_c3), 0},
    {0xC4, st7735_cmd_c4, sizeof(st7735_cmd_c4), 0},
    {0xC5, st7735_cmd_c5, sizeof(st7735_cmd_c5), 0},
    {0xE0, st7735_cmd_e0, sizeof(st7735_cmd_e0), 0},
    {0xE1, st7735_cmd_e1, sizeof(st7735_cmd_e1), 0},
    {0x35, st7735_cmd_35, sizeof(st7735_cmd_35), 0},
    {0x3A, st7735_cmd_3a, sizeof(st7735_cmd_3a), 0},
    {0x36, st7735_cmd_36, sizeof(st7735_cmd_36), 0},
    {0x29, NULL, 0, 0},  // Display on
    {0x2C, NULL, 0, 0},  // Memory write
};

static const led_strip_config_t bsp_strip_config = {
    .strip_gpio_num = GPIO_NUM_8,
    .max_leds = 4,
    .led_model = LED_MODEL_WS2812,
    .flags = {
        .invert_out = false
    }
};

static const led_strip_rmt_config_t bsp_rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 10 * 1000 * 1000,
    .flags = {
        .with_dma = false
    }
};

namespace {

// ---- Emotion -> light colour table --------------------------------------
// The screen and the LEDs must tell the same story. A face that arrives from
// the cloud (the server sends bare names like "angry") previously only changed
// the animation, so the light stayed on whatever colour it last had.
struct MoodColor { uint8_t r; uint8_t g; uint8_t b; };

struct MoodMapEntry {
    const char* emotion;
    MoodColor rgb;
};

static const MoodMapEntry kEmotionPalette[] = {
    { "happy",       {  20, 140,  40 } },
    { "laughing",    {  20, 145,  60 } },
    { "funny",       {  35, 150,  75 } },
    { "loving",      {  95,  35, 105 } },   // soft violet
    { "confident",   {  30, 130,  95 } },
    { "delicious",   { 125,  70,  20 } },   // amber
    { "silly",       {  60, 155,  90 } },
    { "winking",     {  90, 125,  30 } },
    { "relaxed",     {  26,  42,  22 } },
    { "thinking",    {  26,  42,  22 } },
    { "confused",    {  72,  32,  12 } },
    { "embarrassed", { 115,  45,  25 } },
    { "sad",         {   0,  16,  70 } },
    { "crying",      {   0,  14,  85 } },
    { "sleepy",      {   0,  12,  60 } },
    { "angry",       { 105,   0,   0 } },
    { "surprised",   { 125,  95,  20 } },
    { "shocked",     { 125,  60,  20 } },
    { "neutral",     {  14,  42,  22 } },
};

// Subclass of the stock emoji widget: it plays the animation exactly as before,
// then hands the emotion name to a hook so the firmware can drive the LEDs.
class MoodEmojiWidget : public anim::EmojiWidget {
public:
    using EmotionHook = std::function<void(const char*)>;

    MoodEmojiWidget(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io)
        : anim::EmojiWidget(panel, panel_io)
    {
    }

    bool SetEmotion(const char* emotion) override
    {
        const bool ok = anim::EmojiWidget::SetEmotion(emotion);
        if (hook_ != nullptr && emotion != nullptr) {
            hook_(emotion);
        }
        return ok;
    }

    void SetEmotionHook(EmotionHook hook)
    {
        hook_ = std::move(hook);
    }

private:
    EmotionHook hook_;
};

}  // namespace

class EspHi : public WifiBoard {
private:
    Button boot_button_;
    Button audio_wake_button_;
    Button move_wake_button_;
    anim::EmojiWidget* display_ = nullptr;
    // Same object as display_, kept separately so the class can reach the
    // emotion hook without a cast.
    MoodEmojiWidget* mood_display_ = nullptr;
    bool web_server_initialized_ = false;
    led_strip_handle_t led_strip_;
    bool led_on_ = false;
    MoodColor led_base_ = { 20, 140, 40 };   // colour the breathing effect fades around
    esp_timer_handle_t breathe_timer_ = nullptr;
    float breathe_phase_ = 0.0f;
    esp_timer_handle_t idle_anim_timer_ = nullptr;
    // Separate one-shot timer: after being woken from sleep the dog answers
    // first and only gets up a few seconds later, so it never jumps the instant
    // it hears its name.
    esp_timer_handle_t wake_stand_timer_ = nullptr;
    volatile bool idle_tick_busy_ = false;
    volatile bool dog_action_busy_ = false;
    bool mood_led_enabled_ = true;      // follow the user's light on/off intent
    int64_t last_wake_us_ = 0;          // last time it reacted to being woken
    uint8_t wake_burst_ = 0;            // repeated wake-ups in a short window
    int8_t last_emotion_idx_ = -1;
    int8_t last_action_idx_ = -1;

    // ---- Pose state machine ---------------------------------------------
    // The servo component has three postures that do NOT return to neutral
    // on their own: LAY_DOWN (resting) and RETRACT_LEGS (curled up asleep).
    // Everything else ends standing. We track which one we are in so a random
    // action is never played while the dog is lying or curled up.
    enum class DogPose { kStanding, kResting, kSleeping };
    DogPose pose_ = DogPose::kStanding;
    int64_t idle_entered_us_ = 0;   // when the device last settled into idle
    int64_t rest_until_us_ = 0;     // how long to stay lying down before standing
    int64_t sleep_entered_us_ = 0;  // silence window starts here
    int64_t last_activity_us_ = 0;  // last time the owner interacted with it
    bool lonely_shown_ = false;     // already did the "waiting for you" move

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                 int32_t event_id, void* event_data)
    {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {

            xTaskCreate(
                [](void* arg) {
                    EspHi* instance = static_cast<EspHi*>(arg);
                    
                    vTaskDelay(5000 / portTICK_PERIOD_MS);

                    if (!instance->web_server_initialized_) {
                        ESP_LOGI(TAG, "WiFi connected, init web control server");
                        esp_err_t err = esp_hi_web_control_server_init();
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to initialize web control server: %d", err);
                        } else {
                            ESP_LOGI(TAG, "Web control server initialized");
                            instance->web_server_initialized_ = true;
                        }
                    }

                    vTaskDelete(NULL);
                },
                "web_server_init",
                1024 * 10, arg, 5, nullptr);
        }
    }
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED

    void HandleMoveWakePressDown(int64_t current_time, int64_t &last_trigger_time, int &gesture_state)
    {
        int64_t interval = last_trigger_time == 0 ? 0 : current_time - last_trigger_time;
        last_trigger_time = current_time;

        if (interval > 1000) {
            gesture_state = 0;
        } else {
            switch (gesture_state) {
            case 0:
                break;
            case 1:
                if (interval > 300) {
                    gesture_state = 2;
                }
                break;
            case 2:
                if (interval > 100) {
                    gesture_state = 0;
                }
                break;
            }
        }
    }

    void HandleMoveWakePressUp(int64_t current_time, int64_t &last_trigger_time, int &gesture_state)
    {
        int64_t interval = current_time - last_trigger_time;

        if (interval > 1000) {
            gesture_state = 0;
        } else {
            switch (gesture_state) {
            case 0:
                if (interval > 300) {
                    gesture_state = 1;
                }
                break;
            case 1:
                break;
            case 2:
                if (interval < 100) {
                    ESP_LOGI(TAG, "gesture detected");
                    gesture_state = 0;
                    auto &app = Application::GetInstance();
                    app.ToggleChatState();
                }
                break;
            }
        }
    }

    void InitializeButtons()
    {
        static int64_t last_trigger_time = 0;
        static int gesture_state = 0;  // 0: init, 1: wait second long interval, 2: wait oscillation

        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode without reboot
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        audio_wake_button_.OnPressDown([this]() {
        });

        audio_wake_button_.OnPressUp([this]() {
        });

        move_wake_button_.OnPressDown([this]() {
            int64_t current_time = esp_timer_get_time() / 1000;
            HandleMoveWakePressDown(current_time, last_trigger_time, gesture_state);
        });

        move_wake_button_.OnPressUp([this]() {
            int64_t current_time = esp_timer_get_time() / 1000;
            HandleMoveWakePressUp(current_time, last_trigger_time, gesture_state);
        });
    }
    
    void InitializeLed() {
        ESP_LOGI(TAG, "BLINK_GPIO setting %d", bsp_strip_config.strip_gpio_num);

        ESP_ERROR_CHECK(led_strip_new_rmt_device(&bsp_strip_config, &bsp_rmt_config, &led_strip_));
        led_strip_set_pixel(led_strip_, 0, 0x00, 0x00, 0x00);
        led_strip_set_pixel(led_strip_, 1, 0x00, 0x00, 0x00);
        led_strip_set_pixel(led_strip_, 2, 0x00, 0x00, 0x00);
        led_strip_set_pixel(led_strip_, 3, 0x00, 0x00, 0x00);
        led_strip_refresh(led_strip_);
    }

    esp_err_t SetLedColor(uint8_t r, uint8_t g, uint8_t b) {
        esp_err_t ret = ESP_OK;

        ret |= led_strip_set_pixel(led_strip_, 0, r, g, b);
        ret |= led_strip_set_pixel(led_strip_, 1, r, g, b);
        ret |= led_strip_set_pixel(led_strip_, 2, r, g, b);
        ret |= led_strip_set_pixel(led_strip_, 3, r, g, b);
        ret |= led_strip_refresh(led_strip_);
        return ret;
    }

    // Writes a colour and remembers it as the base the breathing effect fades
    // around. Called for every mood change; the breathing timer then keeps
    // modulating that base so the light looks alive instead of a fixed lamp.
    void SetLedMood(MoodColor color)
    {
        led_base_ = color;
        if (!mood_led_enabled_) {
            return;
        }
        SetLedColor(color.r, color.g, color.b);
        led_on_ = true;
    }

    static void BreatheTimerCallback(void* arg)
    {
        EspHi* self = static_cast<EspHi*>(arg);
        if (self == nullptr || !self->mood_led_enabled_ || !self->led_on_) {
            return;
        }
        // One full breath every ~44 steps of 80 ms, i.e. about 3.5 s, the pace a
        // resting animal breathes at. 36 % .. 100 % brightness.
        const float kStep = 0.1428f;   // 2*pi/44
        self->breathe_phase_ += kStep;
        if (self->breathe_phase_ > 6.28318f) {
            self->breathe_phase_ -= 6.28318f;
        }
        const float wave = sinf(self->breathe_phase_);
        const float level = 0.68f + 0.32f * wave;    // 0.36 .. 1.00
        const uint8_t r = static_cast<uint8_t>(self->led_base_.r * level);
        const uint8_t g = static_cast<uint8_t>(self->led_base_.g * level);
        const uint8_t b = static_cast<uint8_t>(self->led_base_.b * level);
        self->SetLedColor(r, g, b);
    }

    void StartBreatheTimer()
    {
        if (breathe_timer_ != nullptr) {
            return;
        }
        esp_timer_create_args_t timer_args = {
            .callback = &BreatheTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "led_breathe",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&timer_args, &breathe_timer_) != ESP_OK) {
            breathe_timer_ = nullptr;
            ESP_LOGE(TAG, "Failed to create LED breathing timer");
            return;
        }
        esp_timer_start_periodic(breathe_timer_, 80 * 1000);
    }

    // A face that came from the server (or a voice command) has no explicit
    // mood colour of its own, so look it up in the table and drive the LEDs.
    void OnDisplayEmotion(const char* emotion)
    {
        if (emotion == nullptr || !mood_led_enabled_ || !led_on_) {
            return;   // the owner switched the light off, respect that
        }
        for (const auto& entry : kEmotionPalette) {
            if (strcmp(emotion, entry.emotion) == 0) {
                SetLedMood(entry.rgb);
                return;
            }
        }
    }

    static std::string_view PickRandomSound()
    {
        static const std::string_view sounds[] = {
            Lang::Sounds::OGG_POPUP,
            Lang::Sounds::OGG_SUCCESS,
            Lang::Sounds::OGG_VIBRATION,
        };
        const size_t count = sizeof(sounds) / sizeof(sounds[0]);
        return sounds[esp_random() % count];
    }

    void InitializeIot()
    {
        ESP_LOGI(TAG, "Initialize Iot");
        InitializeLed();
        SetLedColor(0x00, 0x00, 0x00);

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
        esp_event_loop_create_default();
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                                                 &wifi_event_handler, this));
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED
    }

    void InitializeSpi()
    {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        // Full-frame clear + emoji strips need room for one landscape row band.
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay()
    {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        static const ili9341_vendor_config_t vendor_config = {
            .init_cmds = vendor_specific_init,
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(ili9341_lcd_init_cmd_t),
        };

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void*)&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        // Software reset (RST is NC) needs extra settle time before init commands.
        vTaskDelay(pdMS_TO_TICKS(120));
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, false);
        esp_lcd_panel_set_gap(panel, 0, 24);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        ESP_LOGI(TAG, "LCD panel create success, %p", panel);

        esp_lcd_panel_disp_on_off(panel, true);

        // Clear residual GRAM so a failed assets load does not leave snow on screen.
        std::vector<uint16_t> black(DISPLAY_WIDTH * DISPLAY_HEIGHT, 0);
        esp_lcd_panel_draw_bitmap(panel, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, black.data());

        ESP_LOGI(TAG, "Create emoji widget, panel: %p, panel_io: %p", panel, panel_io);
        mood_display_ = new MoodEmojiWidget(panel, panel_io);
        display_ = mood_display_;
        if (mood_display_ != nullptr) {
            // Every face - local mood or cloud-sent emotion - goes through the
            // light as well, so the body cannot quietly disagree with the screen.
            mood_display_->SetEmotionHook([this](const char* emotion) {
                OnDisplayEmotion(emotion);
            });
        }

#if CONFIG_ESP_CONSOLE_NONE
        servo_dog_ctrl_config_t config = {
            .fl_gpio_num = FL_GPIO_NUM,
            .fr_gpio_num = FR_GPIO_NUM,
            .bl_gpio_num = BL_GPIO_NUM,
            .br_gpio_num = BR_GPIO_NUM,
        };

        servo_dog_ctrl_init(&config);
#endif
    }

    void InitializeTools()
    {
        auto& mcp_server = McpServer::GetInstance();
        
        // 基础动作控制
        mcp_server.AddTool("self.dog.basic_control", "机器人的基础动作。机器人可以做以下基础动作：\n"
            "forward: 向前移动\nbackward: 向后移动\nturn_left: 向左转\nturn_right: 向右转\nstop: 立即停止当前动作", 
            PropertyList({
                Property("action", kPropertyTypeString),
            }), [this](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                if (action == "forward") {
                    servo_dog_ctrl_send(DOG_STATE_FORWARD, NULL);
                } else if (action == "backward") {
                    servo_dog_ctrl_send(DOG_STATE_BACKWARD, NULL);
                } else if (action == "turn_left") {
                    servo_dog_ctrl_send(DOG_STATE_TURN_LEFT, NULL);
                } else if (action == "turn_right") {
                    servo_dog_ctrl_send(DOG_STATE_TURN_RIGHT, NULL);
                } else if (action == "stop") {
                    servo_dog_ctrl_send(DOG_STATE_IDLE, NULL);
                } else {
                    return false;
                }
                return true;
            });
        
        // 扩展动作控制
        mcp_server.AddTool("self.dog.advanced_control", "机器人的扩展动作。机器人可以做以下扩展动作：\n"
            "sway_back_forth: 前后摇摆\nlay_down: 趴下\nsway: 左右摇摆\nretract_legs: 收回腿部（睡觉）\n"
            "shake_hand: 握手/打招呼\nshake_back_legs: 伸懒腰\njump_forward: 向前跳跃\n"
            "turn_around: 原地转一整圈\nstand: 从趴下或收腿姿态慢慢站起来", 
            PropertyList({
                Property("action", kPropertyTypeString),
            }), [this](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                if (action == "sway_back_forth") {
                    servo_dog_ctrl_send(DOG_STATE_SWAY_BACK_FORTH, NULL);
                } else if (action == "lay_down") {
                    servo_dog_ctrl_send(DOG_STATE_LAY_DOWN, NULL);
                    pose_ = DogPose::kResting;
                    rest_until_us_ = esp_timer_get_time() + kRestHoldUs;
                } else if (action == "sway") {
                    dog_action_args_t args = {
                        .repeat_count = 4,
                    };
                    servo_dog_ctrl_send(DOG_STATE_SWAY, &args);
                } else if (action == "retract_legs") {
                    EnterSleep();
                } else if (action == "shake_hand") {
                    servo_dog_ctrl_send(DOG_STATE_SHAKE_HAND, NULL);
                } else if (action == "shake_back_legs") {
                    servo_dog_ctrl_send(DOG_STATE_SHAKE_BACK_LEGS, NULL);
                } else if (action == "jump_forward") {
                    servo_dog_ctrl_send(DOG_STATE_JUMP_FORWARD, NULL);
                } else if (action == "turn_around") {
                    /* Repeat twice the default so a single call really turns a
                     * full circle instead of shuffling three steps. */
                    dog_action_args_t args = {0};
                    args.repeat_count = 4;
                    args.speed = 80;
                    args.hold_time_ms = NOT_USE;
                    args.angle_offset = NOT_USE;
                    servo_dog_ctrl_send(DOG_STATE_TURN_RIGHT, &args);
                } else if (action == "stand") {
                    StandUpIfNeeded();
                } else {
                    return false;
                }
                return true;
            });

        // 睡眠 / 唤醒：只有用户明确要求时才收腿，其余时间一律保持站立
        mcp_server.AddTool("self.dog.sleep",
            "让机器狗收腿睡觉。只有当用户明确说“去睡觉”“睡觉吧”“歇着吧”时才调用；"
            "调用后它会收起四条腿、切到睡觉表情，直到听到唤醒词或按键才起身。", 
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                EnterSleep();
                return true;
            });

        mcp_server.AddTool("self.dog.wake",
            "让机器狗站起来。用户说“站起来”“起来吧”“别睡了”时调用。", 
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                StandUpIfNeeded();
                return true;
            });

        // 灯光控制
        mcp_server.AddTool("self.light.get_power", "获取灯是否打开", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            return led_on_;
        });

        mcp_server.AddTool("self.light.turn_on", "打开灯", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            mood_led_enabled_ = true;
            SetLedMood({ 255, 255, 255 });
            return true;
        });

        mcp_server.AddTool("self.light.turn_off", "关闭灯", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            // Switching the light off also parks the automatic mood colours, so
            // the dog does not light itself up again two seconds later.
            mood_led_enabled_ = false;
            led_on_ = false;
            SetLedColor(0x00, 0x00, 0x00);
            return true;
        });

        mcp_server.AddTool("self.light.set_rgb", "设置RGB颜色", PropertyList({
            Property("r", kPropertyTypeInteger, 0, 255),
            Property("g", kPropertyTypeInteger, 0, 255),
            Property("b", kPropertyTypeInteger, 0, 255)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int r = properties["r"].value<int>();
            int g = properties["g"].value<int>();
            int b = properties["b"].value<int>();

            mood_led_enabled_ = true;
            SetLedMood({ static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b) });
            return true;
        });
    }

    // ---- Idle-state liveliness + pose state machine -----------------------
    // A one-shot esp_timer wakes the main thread periodically. The timer is
    // always re-armed inside the callback itself, so it can never stall even
    // if the scheduled tick runs after the device has left the idle state.
    //
    // While idle the dog behaves like a pet:
    //   * every ~12-18 s it either changes its face or performs ONE complete
    //     in-place action;
    //   * occasionally it lies down (LAY_DOWN) and actually stays resting;
    //   * after a long stretch alone it curls up (RETRACT_LEGS) and sleeps,
    //     while the local wake-word engine keeps listening the whole time;
    //   * any real interaction stands it back up before anything else.

    static constexpr int64_t kFirstStartUs        = 15 * 1000 * 1000;   // 15 s after boot
    static constexpr int64_t kRecheckUs           = 6 * 1000 * 1000;    // slow check while standing
    static constexpr int64_t kSleepPollUs         = 1 * 1000 * 1000;    // poll fast while asleep/resting
    static constexpr int64_t kMinIdleDelayUs      = 12 * 1000 * 1000;
    static constexpr uint32_t kIdleDelayJitterMs  = 6000;
    static constexpr int64_t kRestHoldUs          = 30 * 1000 * 1000;   // stay lying ~30 s
    static constexpr int64_t kWakeStandDelayUs    = 3 * 1000 * 1000;    // answer first, stand 3 s later
    static constexpr int64_t kMoodBurstWindowUs   = 60 * 1000 * 1000;   // repeated wake-ups inside 1 min
    static constexpr int64_t kSleepSilenceUs      = 12 * 1000 * 1000;   // ignore wake-ups right after dozing off
    static constexpr int64_t kLonelyUs            = 30 * 60 * 1000LL * 1000;  // 30 min with nobody around

    // Small mood palette for the four WS2812 pixels. The dog carries its state
    // in the light as well as on the screen, the way a pet shows how it feels.
    static constexpr MoodColor kMoodHappy {  20, 140,  40 };  // warm green
    static constexpr MoodColor kMoodWake  {  90,  90,  90 };  // bright grey flash
    static constexpr MoodColor kMoodSleep {   0,  12,  60 };  // dim blue
    static constexpr MoodColor kMoodCalm  {  25,  40,  20 };  // low moss
    static constexpr MoodColor kMoodSad   {   0,  16,  70 };  // cold blue
    static constexpr MoodColor kMoodAngry { 100,   0,   0 };  // red

    void ArmIdleTimer(int64_t delay_us) {
        if (idle_anim_timer_ == nullptr) {
            return;
        }
        esp_timer_stop(idle_anim_timer_);
        esp_timer_start_once(idle_anim_timer_, delay_us);
    }

    int64_t NextIdleDelayUs() const {
        uint32_t jitter_ms = esp_random() % kIdleDelayJitterMs;
        return kMinIdleDelayUs + static_cast<int64_t>(jitter_ms) * 1000;
    }

    void StartIdleAnimTimer() {
        if (idle_anim_timer_ != nullptr) {
            return;
        }
        esp_timer_create_args_t timer_args = {
            .callback = &IdleTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "idle_anim",
            .skip_unhandled_events = false,
        };
        esp_err_t err = esp_timer_create(&timer_args, &idle_anim_timer_);
        if (err != ESP_OK || idle_anim_timer_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create idle anim timer: %d", err);
            idle_anim_timer_ = nullptr;
            return;
        }
        ArmIdleTimer(kFirstStartUs);
    }

    // Sets the face and, unless the user switched the light off, the mood colour.
    void SetMood(const char* emotion, MoodColor color) {
        if (display_ != nullptr && emotion != nullptr) {
            display_->SetEmotion(emotion);
        }
        SetLedMood(color);
    }

    void StartWakeStandTimer() {
        if (wake_stand_timer_ != nullptr) {
            return;
        }
        esp_timer_create_args_t timer_args = {
            .callback = &WakeStandCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "wake_stand",
            .skip_unhandled_events = false,
        };
        if (esp_timer_create(&timer_args, &wake_stand_timer_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create wake stand timer");
            wake_stand_timer_ = nullptr;
        }
    }

    static void WakeStandCallback(void* arg) {
        EspHi* self = static_cast<EspHi*>(arg);
        if (self == nullptr) {
            return;
        }
        // Stand on the main thread, where display and servo calls are safe.
        Application::GetInstance().Schedule([self]() { self->StandUpIfNeeded(); });
    }

    void ArmWakeStandTimer(int64_t delay_us) {
        if (wake_stand_timer_ == nullptr) {
            return;
        }
        esp_timer_stop(wake_stand_timer_);
        esp_timer_start_once(wake_stand_timer_, delay_us);
    }

    static void IdleTimerCallback(void* arg) {
        EspHi* self = static_cast<EspHi*>(arg);
        if (self == nullptr) {
            return;
        }
        auto& app = Application::GetInstance();

        const auto state = app.GetDeviceState();
        if (state != kDeviceStateIdle) {
            const int64_t now_us = esp_timer_get_time();
            self->last_activity_us_ = now_us;
            self->lonely_shown_ = false;

            /* Only "the owner is actually talking to me" counts as a wake-up.
             * Connecting and Listening are the two states that appear exclusively
             * when the device is woken; Speaking / Notifying / Upgrading are
             * produced by the dog itself (its own answer, its own alerts) and
             * must not count. Judging on "anything but Idle" was exactly what
             * made it hear its own "I'm going to sleep now" as a wake word. */
            const bool owner_talking = (state == kDeviceStateConnecting) ||
                                       (state == kDeviceStateListening);
            if (owner_talking && self->pose_ != DogPose::kStanding) {
                /* Silence window right after dozing off: the reply it gives to
                 * "go to sleep" must not bring it back up. */
                if (now_us - self->sleep_entered_us_ >= kSleepSilenceUs) {
                    app.Schedule([self]() { self->WakeUpResponse(); });
                }
            } else if (state == kDeviceStateListening &&
                       self->pose_ == DogPose::kStanding && self->dog_action_busy_) {
                // It listens to its owner: a random action is cut short the
                // moment somebody talks to it.
                servo_dog_ctrl_send(DOG_STATE_IDLE, NULL);
                self->dog_action_busy_ = false;
            }
            // Poll quickly so a sleeping dog reacts to the wake word within a
            // second instead of waiting for the next random slot.
            self->ArmIdleTimer(self->pose_ == DogPose::kStanding ? kRecheckUs : kSleepPollUs);
            return;
        }

        if (self->pose_ == DogPose::kSleeping) {
            // Asleep on request: no random actions, just keep polling so the
            // wake word can bring it back up.
            self->ArmIdleTimer(kSleepPollUs);
            return;
        }
        if (self->pose_ == DogPose::kResting) {
            // Lying down: poll faster so it stands up promptly when disturbed.
            self->ArmIdleTimer(kSleepPollUs);
            app.Schedule([self]() { self->IdleRandomTick(); });
            return;
        }

        // Re-arm first so the timer cannot stall, then run the tick on the
        // main thread where display/servo calls are safe.
        self->ArmIdleTimer(self->NextIdleDelayUs());
        app.Schedule([self]() { self->IdleRandomTick(); });
    }

    // Slowly unfolds from the curled-up pose. Safe to call repeatedly; it is a
    // no-op when already standing on all four legs.
    void StandUpIfNeeded() {
        idle_entered_us_ = 0;
        if (pose_ == DogPose::kStanding) {
            return;
        }
        const char* was = (pose_ == DogPose::kSleeping) ? "sleeping" : "resting";
        pose_ = DogPose::kStanding;
        rest_until_us_ = 0;
        // DOG_STATE_STAND walks every leg back frame by frame (~3.8 s). Using
        // DOG_STATE_IDLE here is what snapped the legs straight and then cut
        // the torque, so the dog never got back on its feet.
        servo_dog_ctrl_send(DOG_STATE_STAND, NULL);
        SetMood("happy", kMoodHappy);
        ESP_LOGI(TAG, "Stand up slowly: leaving %s posture", was);
    }

    // Reacts the moment it is woken up: a face and a short cue first, the actual
    // unfold is armed for a few seconds later so it does not snap upright.
    void WakeUpResponse() {
        const int64_t now = esp_timer_get_time();
        if (last_wake_us_ != 0 && now - last_wake_us_ < kMoodBurstWindowUs) {
            if (wake_burst_ < 255) {
                wake_burst_++;
            }
        } else {
            wake_burst_ = 1;
        }
        last_wake_us_ = now;
        const bool annoyed = wake_burst_ >= 3;

        if (pose_ == DogPose::kSleeping) {
            if (wake_burst_ <= 1) {
                /* First call: open the eyes and chirp once, but stay curled up.
                 * A dog that leaps to its feet on the first syllable is not
                 * believable, and at night it is plain annoying. */
                SetMood("surprised", kMoodWake);
                Application::GetInstance().PlaySound(PickRandomSound());
                ESP_LOGI(TAG, "First call while asleep: eyes only, staying curled");
            } else if (wake_burst_ == 2) {
                /* Second call within a minute: that is a real request, get up. */
                SetMood("happy", kMoodHappy);
                Application::GetInstance().PlaySound(PickRandomSound());
                ArmWakeStandTimer(kWakeStandDelayUs);
                ESP_LOGI(TAG, "Second call while asleep: answer now, stand in %llds",
                         static_cast<long long>(kWakeStandDelayUs / 1000000));
            } else {
                /* Third call inside the same window: it is being pestered. */
                SetMood("angry", kMoodAngry);
                Application::GetInstance().PlaySound(Lang::Sounds::OGG_EXCLAMATION);
                ArmWakeStandTimer(kWakeStandDelayUs);
                ESP_LOGI(TAG, "Third call while asleep: annoyed, standing anyway");
            }
        } else if (pose_ == DogPose::kResting) {
            pose_ = DogPose::kStanding;
            rest_until_us_ = 0;
            servo_dog_ctrl_send(DOG_STATE_IDLE, NULL);
            SetMood(annoyed ? "sad" : "happy", annoyed ? kMoodSad : kMoodHappy);
            ESP_LOGI(TAG, "Woken while resting, back on its feet");
        }
    }

    void IdleRandomTick() {
        if (idle_tick_busy_) {
            return;
        }
        auto& app = Application::GetInstance();
        if (app.GetDeviceState() != kDeviceStateIdle) {
            return;
        }
        idle_tick_busy_ = true;

        const int64_t now = esp_timer_get_time();
        if (idle_entered_us_ == 0) {
            idle_entered_us_ = now;
        }

        // Asleep on request: stay quiet and keep listening for the wake word.
        if (pose_ == DogPose::kSleeping) {
            idle_tick_busy_ = false;
            return;
        }

        // Lying down: hold the pose until the rest timer expires, then get up
        // properly so the next random action starts from all four legs.
        if (pose_ == DogPose::kResting) {
            if (now < rest_until_us_) {
                idle_tick_busy_ = false;
                return;
            }
            pose_ = DogPose::kStanding;
            servo_dog_ctrl_send(DOG_STATE_IDLE, NULL);
            SetMood("happy", kMoodHappy);
            ESP_LOGI(TAG, "Rest finished, stand up");
            idle_tick_busy_ = false;
            return;
        }

        // Nobody has spoken to it for half an hour: it gives one long sigh,
        // looks down and lies down to wait, the way a real dog does by the door.
        if (!lonely_shown_ && now - last_activity_us_ >= kLonelyUs) {
            lonely_shown_ = true;
            SetMood("sad", kMoodSad);
            Application::GetInstance().PlaySound(Lang::Sounds::OGG_POPUP);
            servo_dog_ctrl_send(DOG_STATE_LAY_DOWN, NULL);
            pose_ = DogPose::kResting;
            rest_until_us_ = now + kRestHoldUs;
            idle_tick_busy_ = false;
            ESP_LOGI(TAG, "Nobody around for 30 min: lie down and wait");
            return;
        }

        // ~60% a light face, ~40% one complete action carrying its own mood.
        if (esp_random() % 10 < 6) {
            PlayRandomEmotion();
        } else {
            PlayRandomGentleAction();
        }

        idle_tick_busy_ = false;
    }

    // Only ever reached on an explicit "go to sleep" request.
    void EnterSleep() {
        // Back to neutral first so RETRACT_LEGS always starts from the same
        // pose, then curl the legs up and settle into the sleeping mood.
        servo_dog_ctrl_send(DOG_STATE_IDLE, NULL);
        servo_dog_ctrl_send(DOG_STATE_RETRACT_LEGS, NULL);
        pose_ = DogPose::kSleeping;
        rest_until_us_ = 0;
        dog_action_busy_ = false;
        idle_entered_us_ = 0;
        sleep_entered_us_ = esp_timer_get_time();
        last_activity_us_ = sleep_entered_us_;
        lonely_shown_ = false;
        SetMood("sleepy", kMoodSleep);
        // Exactly one short cue when curling up, picked at random so it never
        // sounds like the same looped sample; then the dog stays quiet.
        Application::GetInstance().PlaySound(PickRandomSound());
        ESP_LOGI(TAG, "Sleep on request: legs retracted, quiet from now on");
    }

    void PlayRandomEmotion() {
        // Friendly, mild idle faces; the thoughtful/slant-eye looks stay in the
        // pool on purpose, they make it read as a real pet glancing around.
        struct FaceSpec {
            const char* emotion;
            MoodColor color;
        };
        static const FaceSpec faces[] = {
            { "happy",     kMoodHappy },
            { "laughing",  kMoodHappy },
            { "funny",     kMoodHappy },
            { "loving",    kMoodHappy },
            { "confident", kMoodHappy },
            { "silly",     kMoodHappy },
            { "winking",   kMoodHappy },
            { "relaxed",   kMoodCalm  },
            { "thinking",  kMoodCalm  },
            { "confused",  kMoodCalm  },
            { "sleepy",    kMoodSleep },
            { "angry",      kMoodAngry },          // red face, red light
            { "embarrassed", { 115, 45, 25 } },
            { "surprised",  { 125, 95, 20 } },
        };
        const int count = sizeof(faces) / sizeof(faces[0]);
        int idx = static_cast<int>(esp_random() % count);
        if (count > 1 && idx == last_emotion_idx_) {
            idx = (idx + 1) % count;
        }
        last_emotion_idx_ = static_cast<int8_t>(idx);
        SetMood(faces[idx].emotion, faces[idx].color);
    }

    void PlayRandomGentleAction() {
        // Every entry is one complete in-place movement and carries the mood it
        // belongs to, so body, face and light always tell the same story: a
        // stretch looks sleepy, a wave looks happy.
        struct ActionSpec {
            servo_dog_state_t state;
            int16_t repeat_count;   // >0: override, 0: use the table default
            const char* emotion;
            MoodColor color;
            bool lies_down;
        };
        static const ActionSpec actions[] = {
            { DOG_STATE_BOW,             0, "sleepy",    kMoodSleep, false },  // front-leg stretch
            { DOG_STATE_SHAKE_BACK_LEGS, 0, "sleepy",    kMoodSleep, false },  // hind-leg stretch
            { DOG_STATE_SHAKE_HAND,      0, "happy",     kMoodHappy, false },  // wave hello
            { DOG_STATE_SWAY_BACK_FORTH, 0, "laughing",  kMoodHappy, false },  // rock back and forth
            { DOG_STATE_SWAY,            0, "silly",     kMoodHappy, false },  // sway side to side
            { DOG_STATE_TURN_RIGHT,      4, "confident", kMoodHappy, false },  // turn in place
            { DOG_STATE_BOW_LEAN,        0, "funny",     kMoodHappy, false },  // bow + lean combo
            { DOG_STATE_LAY_DOWN,        0, "relaxed",   kMoodCalm,  true  },  // lie down and rest
        };
        const int count = sizeof(actions) / sizeof(actions[0]);
        int idx = static_cast<int>(esp_random() % count);
        if (count > 1 && idx == last_action_idx_) {
            idx = (idx + 1) % count;
        }
        last_action_idx_ = static_cast<int8_t>(idx);
        const ActionSpec& spec = actions[idx];

        SetMood(spec.emotion, spec.color);
        // One short cue at the start of the movement, so a random action is
        // a whole pet reaction (sound + face + light + body) instead of a
        // silent mechanical shuffle.
        Application::GetInstance().PlaySound(PickRandomSound());

        if (spec.repeat_count > 0) {
            dog_action_args_t args = {0};
            args.repeat_count = spec.repeat_count;
            args.speed = 80;
            args.hold_time_ms = NOT_USE;
            args.angle_offset = NOT_USE;
            servo_dog_ctrl_send(spec.state, &args);
        } else {
            // Pass NULL so the table's built-in defaults (speed, repeat, hold)
            // are used: a hand-built args struct would zero out speed and the
            // action body would be skipped entirely.
            servo_dog_ctrl_send(spec.state, NULL);
        }

        if (spec.lies_down) {
            pose_ = DogPose::kResting;
            rest_until_us_ = esp_timer_get_time() + kRestHoldUs;
        } else {
            dog_action_busy_ = true;
        }
    }
public:
    EspHi() : boot_button_(BOOT_BUTTON_GPIO),
        audio_wake_button_(AUDIO_WAKE_BUTTON_GPIO),
        move_wake_button_(MOVE_WAKE_BUTTON_GPIO)
    {
        InitializeButtons();
        InitializeIot();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeTools();
        StartIdleAnimTimer();
        StartWakeStandTimer();
        StartBreatheTimer();
        last_activity_us_ = esp_timer_get_time();
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        static AdcPdmAudioCodec audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_ADC_MIC_CHANNEL,
            AUDIO_PDM_SPEAK_P_GPIO,
            AUDIO_PDM_SPEAK_N_GPIO,
            AUDIO_PA_CTL_GPIO);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override
    {
        return display_;
    }
};

DECLARE_BOARD(EspHi);
