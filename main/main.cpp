// ================================================================
//  main.cpp
//  RoboMAZ - Entry point  (FreeRTOS multi-task architecture)
//  ESP32-S3 / ESP-IDF  |  C++20
//
//  Task layout:
//    app_main()       — one-shot: init hardware, create tasks, vTaskDelete(nullptr)
//    control_task     — BT polling, LED cycling, connection state  (50 Hz, core 1)
//    sensor_task      — compass / IMU reads, LCD updates           (10 Hz, core 1)
//    nimble_host_task — created internally by nimble_port_freertos_init() (core 0)
//
//  Stack sizes (bytes — ESP-IDF unit, NOT words):
//    control  → 8 192 B — BT update + command dispatch + LED refresh
//    sensor   → 4 096 B — I2C reads (compass + IMU) + LCD writes
//
//  Priorities:
//    control at 5 — BT commands and motor safety are time-critical
//    sensor  at 3 — sensor polling can tolerate a few ms jitter
//
//  Shared state:
//    g_motorState, g_led*, g_defaultSpeed are written only by
//    control_task (via BT command callback).
//    g_btConnected is std::atomic<bool> read by sensor_task to decide
//    whether it may write the LCD.
// ================================================================

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
}

#include "Motor/MecanumRobot.hpp"
#include "LCD/MAZLCD.hpp"
#include "Compass/MAZGY271.hpp"
#include "IMU/MAZMPU6050.hpp"
#include "Bluetooth/BluetoothComm.hpp"
#include "led_strip.h"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <atomic>

[[maybe_unused]] static const char *TAG = "RoboMAZ";

#define BlinkLED_GPIO     GPIO_NUM_46
#define FullColorLED_GPIO GPIO_NUM_48  // WS2812B — driven via RMT

// ── Speed constants ──────────────────────────────────────────────
[[maybe_unused]] static constexpr float DRIVE_SPEED  = 70.0f;
[[maybe_unused]] static constexpr float ROTATE_SPEED = 70.0f;

// ── Pin definitions ──────────────────────────────────────────────
//   Viewed from TOP, front of robot facing UP:
//
//       [FL]─────[FR]
//        │   TOP   │
//       [RL]─────[RR]
//
// Each MotorPins struct maps to one DRV8833 channel:
//   cw_gpio  → IN1 (clockwise when HIGH)
//   ccw_gpio → IN2 (counter-clockwise when HIGH)
static constexpr MotorPins PINS_FRONT_LEFT  = {.cw_gpio = 39, .ccw_gpio = 40};
static constexpr MotorPins PINS_FRONT_RIGHT = {.cw_gpio = 42, .ccw_gpio = 41};
static constexpr MotorPins PINS_REAR_LEFT   = {.cw_gpio = 2,  .ccw_gpio = 1};
static constexpr MotorPins PINS_REAR_RIGHT  = {.cw_gpio = 3,  .ccw_gpio = 4};

// ── Global instances ─────────────────────────────────────────────
// Initialised once in app_main before any task is created, so no race on init.
static MecanumRobot robot(PINS_FRONT_LEFT,
                          PINS_FRONT_RIGHT,
                          PINS_REAR_LEFT,
                          PINS_REAR_RIGHT);
static MAZLCD     lcd;
static MAZGY271   compass;
static MAZMPU6050 imu;
static led_strip_handle_t _rgb_strip = nullptr;

// ── Shared state ─────────────────────────────────────────────────
static MotorState g_motorState   = MotorState::IDLE;
static float      g_defaultSpeed = 70.0f;

// BT connection flag — written by control_task, read by sensor_task
static std::atomic<bool> g_btConnected{false};

// ── RGB LED state machine (control_task only — no mutex needed) ──
//
//  LED_DISCONNECTED  — blinking red (1 s period: 500 ms on, 500 ms off)
//  LED_CONNECT_BURST — first 3 s after BLE connect, fast color cycle 200 ms
//  LED_IDLE_CYCLE    — after burst, slow color cycle 1000 ms
//  LED_MOVING        — green while robot is moving (500 ms on/off blink)
//
enum class LedMode : uint8_t {
    DISCONNECTED, CONNECT_BURST, IDLE_CYCLE, MOVING
};

static LedMode  g_ledMode        = LedMode::DISCONNECTED;
static bool     g_ledManualHold  = false;   // CMD:LED manual override active
static uint32_t g_ledManualStart = 0;       // timestamp when manual override started
static uint32_t g_ledManualDurMs = 0;       // how long to hold manual color (ms)
static uint8_t  g_ledColorIndex  = 0;
static uint32_t g_ledLastCycleMs = 0;
static uint32_t g_ledConnectMs   = 0;       // timestamp of BLE connect
static uint32_t g_ledLastBlinkMs = 0;       // timestamp of last blink toggle
static bool     g_ledBlinkOn     = true;    // toggle state for blink modes

static constexpr uint32_t LED_BURST_DURATION_MS  = 3000;  // fast cycle duration
static constexpr uint32_t LED_BURST_INTERVAL_MS  = 200;   // fast cycle period
static constexpr uint32_t LED_IDLE_INTERVAL_MS   = 1000;  // slow cycle period
static constexpr uint32_t LED_DISCONNECT_BLINK_MS = 500;  // red blink half-period (1 s total)
static constexpr uint32_t LED_MOVING_BLINK_MS    = 500;   // green blink half-period while moving
static constexpr uint32_t LED_MANUAL_OFF_MS      = 2000;  // hold "off" for 2 s then resume
static constexpr uint32_t LED_MANUAL_COLOR_MS    = 5000;  // hold selected color for 5 s then resume

struct RGBColor { uint8_t r, g, b; const char* name; };
static constexpr RGBColor COLOR_CYCLE[] = {
    {255,   0,   0, "Red"},
    {  0, 255,   0, "Green"},
    {  0,   0, 255, "Blue"},
    {  0, 255, 255, "Cyan"},
    {255,   0, 255, "Magenta"},
    {255, 255,   0, "Yellow"},
    {255, 255, 255, "White"},
};
static constexpr uint8_t COLOR_COUNT = sizeof(COLOR_CYCLE) / sizeof(COLOR_CYCLE[0]);

// ── LCD connection state ─────────────────────────────────────────
static uint32_t g_lcdConnectMs    = 0;       // timestamp of BLE connect
static constexpr uint32_t LCD_CONNECTED_SHOW_MS = 2000;  // show "Connected" duration

// ── Task handles ─────────────────────────────────────────────────
// Stored globally so any module can suspend/resume/delete a task or query its
// stack high-water mark at runtime, e.g.:
//   UBaseType_t hwm = uxTaskGetStackHighWaterMark(h_control);  // bytes free
static TaskHandle_t h_control = nullptr;
static TaskHandle_t h_sensor  = nullptr;

// ── Helper ───────────────────────────────────────────────────────
static uint32_t millis()
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

// ── RGB LED helpers ──────────────────────────────────────────────
static void setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (_rgb_strip) {
        led_strip_set_pixel(_rgb_strip, 0, r, g, b);
        led_strip_refresh(_rgb_strip);
    }
}

[[maybe_unused]]
static void rgbOff()
{
    if (_rgb_strip) {
        led_strip_clear(_rgb_strip);
    }
}

// Non-blocking RGB LED state machine — called from control_task
static void updateLed()
{
    // CMD:LED manual override — hold for timed duration, then auto-release
    if (g_ledManualHold) {
        if (millis() - g_ledManualStart >= g_ledManualDurMs) {
            g_ledManualHold  = false;
            g_ledMode        = LedMode::IDLE_CYCLE;
            g_ledLastCycleMs = millis();
        }
        return;
    }

    uint32_t now = millis();

    switch (g_ledMode) {
        case LedMode::DISCONNECTED: {
            // Blinking red — 500 ms on, 500 ms off (1 s period)
            if (now - g_ledLastBlinkMs >= LED_DISCONNECT_BLINK_MS) {
                g_ledLastBlinkMs = now;
                g_ledBlinkOn = !g_ledBlinkOn;
            }
            if (g_ledBlinkOn) setRGB(255, 0, 0);
            else              rgbOff();
            break;
        }

        case LedMode::CONNECT_BURST: {
            // Fast color cycle (200 ms) for first 3 seconds after connect
            if (now - g_ledConnectMs >= LED_BURST_DURATION_MS) {
                g_ledMode = LedMode::IDLE_CYCLE;
                g_ledLastCycleMs = now;
                break;
            }
            if (now - g_ledLastCycleMs >= LED_BURST_INTERVAL_MS) {
                g_ledLastCycleMs = now;
                const auto& c = COLOR_CYCLE[g_ledColorIndex];
                setRGB(c.r, c.g, c.b);
                g_ledColorIndex = (g_ledColorIndex + 1) % COLOR_COUNT;
            }
            break;
        }

        case LedMode::IDLE_CYCLE: {
            // Slow color cycle (1000 ms)
            if (now - g_ledLastCycleMs >= LED_IDLE_INTERVAL_MS) {
                g_ledLastCycleMs = now;
                const auto& c = COLOR_CYCLE[g_ledColorIndex];
                setRGB(c.r, c.g, c.b);
                g_ledColorIndex = (g_ledColorIndex + 1) % COLOR_COUNT;
            }
            break;
        }

        case LedMode::MOVING: {
            // Blinking green — 500 ms on, 500 ms off while robot is moving
            if (now - g_ledLastBlinkMs >= LED_MOVING_BLINK_MS) {
                g_ledLastBlinkMs = now;
                g_ledBlinkOn = !g_ledBlinkOn;
            }
            if (g_ledBlinkOn) setRGB(0, 255, 0);
            else              rgbOff();
            break;
        }
    }
}

// Enter MOVING LED mode — green blink while robot moves (motor commands only)
static void triggerMovingLed()
{
    if (g_ledManualHold) return;
    g_ledMode        = LedMode::MOVING;
    g_ledLastBlinkMs = millis();
    g_ledBlinkOn     = true;
    setRGB(0, 255, 0);
}

// Return to idle cycle when motors stop (BRK / CST / STOP)
static void triggerIdleLed()
{
    if (g_ledManualHold) return;
    if (g_ledMode == LedMode::MOVING) {
        g_ledMode        = LedMode::IDLE_CYCLE;
        g_ledLastCycleMs = millis();
    }
}

// ── LCD formatted row ────────────────────────────────────────────
static void lcdRow(uint8_t row, const char* fmt, ...)
{
    char buf[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    size_t len = strlen(buf);
    while (len < 16) buf[len++] = ' ';
    buf[16] = '\0';

    lcd.setCursor(0, row);
    lcd.print(buf);
}

// ── LED init ─────────────────────────────────────────────────────
static void init_led()
{
    // GPIO46 — simple green LED
    gpio_reset_pin(BlinkLED_GPIO);
    gpio_set_direction(BlinkLED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BlinkLED_GPIO, 0);

    // GPIO48 — WS2812B RGB-LED via RMT
    led_strip_config_t strip_cfg = {};
    strip_cfg.strip_gpio_num        = (int)FullColorLED_GPIO;
    strip_cfg.max_leds              = 1;
    strip_cfg.led_model             = LED_MODEL_WS2812;
    strip_cfg.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    led_strip_rmt_config_t rmt_cfg = {};
    rmt_cfg.resolution_hz = 10 * 1000 * 1000;  // 10 MHz
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &_rgb_strip));
    led_strip_clear(_rgb_strip);
}

// ── Compass direction helper ─────────────────────────────────────
static const char* compassDir(float deg)
{
    static const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
    return dirs[(int)((deg + 22.5f) / 45.0f) % 8];
}

// ── Speed helper — clamp to [0,100] ──────────────────────────────
static float parseSpeed(const char* s)
{
    float v = (float)atof(s);
    if (v < 0.0f) v = 0.0f;
    if (v > 100.0f) v = 100.0f;
    return v;
}

// ================================================================
//  BT command handler  (runs on control_task via bt.update())
// ================================================================
static bool handleBtCommand(const ParsedCommand& cmd)
{
    auto& bt = btCommInstance();

    auto moveCmd = [&](const char* name, auto fn, MotorState state) -> bool {
        float spd = (cmd.nParams >= 1) ? parseSpeed(cmd.params[0]) : g_defaultSpeed;
        (robot.*fn)(spd);
        g_motorState = state;
        triggerMovingLed();  // green blink while robot is moving
        bt.sendAck(name);
        return true;
    };

    if (strcmp(cmd.cmd, "FWD") == 0)
        return moveCmd("FWD",  &MecanumRobot::moveForward,              MotorState::FWD);
    if (strcmp(cmd.cmd, "BWD") == 0)
        return moveCmd("BWD",  &MecanumRobot::moveBackward,             MotorState::BWD);
    if (strcmp(cmd.cmd, "SL") == 0)
        return moveCmd("SL",   &MecanumRobot::strafeLeft,               MotorState::SL);
    if (strcmp(cmd.cmd, "SR") == 0)
        return moveCmd("SR",   &MecanumRobot::strafeRight,              MotorState::SR);
    if (strcmp(cmd.cmd, "RCW") == 0)
        return moveCmd("RCW",  &MecanumRobot::rotateClockwise,          MotorState::RCW);
    if (strcmp(cmd.cmd, "RCCW") == 0)
        return moveCmd("RCCW", &MecanumRobot::rotateCounterClockwise,   MotorState::RCCW);
    if (strcmp(cmd.cmd, "DFL") == 0)
        return moveCmd("DFL",  &MecanumRobot::moveDiagonalFrontLeft,    MotorState::DFL);
    if (strcmp(cmd.cmd, "DFR") == 0)
        return moveCmd("DFR",  &MecanumRobot::moveDiagonalFrontRight,   MotorState::DFR);
    if (strcmp(cmd.cmd, "DRL") == 0)
        return moveCmd("DRL",  &MecanumRobot::moveDiagonalRearLeft,     MotorState::DRL);
    if (strcmp(cmd.cmd, "DRR") == 0)
        return moveCmd("DRR",  &MecanumRobot::moveDiagonalRearRight,    MotorState::DRR);

    if (strcmp(cmd.cmd, "R360CW") == 0)
        return moveCmd("R360CW",  &MecanumRobot::rotateClockwise,       MotorState::RCW);
    if (strcmp(cmd.cmd, "R360CCW") == 0)
        return moveCmd("R360CCW", &MecanumRobot::rotateCounterClockwise, MotorState::RCCW);

    if (strcmp(cmd.cmd, "BRK") == 0 || strcmp(cmd.cmd, "STOP") == 0) {
        robot.brake();
        g_motorState = MotorState::BRK;
        triggerIdleLed();  // motors stopped — back to color cycle
        bt.sendAck("BRK");
        return true;
    }
    if (strcmp(cmd.cmd, "CST") == 0) {
        robot.coast();
        g_motorState = MotorState::CST;
        triggerIdleLed();  // motors stopped — back to color cycle
        bt.sendAck("CST");
        return true;
    }

    if (strcmp(cmd.cmd, "SPD") == 0) {
        if (cmd.nParams < 1) { bt.sendError("BAD_PARAM:SPD"); return true; }
        g_defaultSpeed = parseSpeed(cmd.params[0]);
        robot.setGlobalSpeed(g_defaultSpeed);
        bt.sendAck("SPD");
        return true;
    }

    if (strcmp(cmd.cmd, "LED") == 0) {
        if (cmd.nParams < 3) { bt.sendError("BAD_PARAM:LED"); return true; }
        uint8_t r = (uint8_t)atoi(cmd.params[0]);
        uint8_t g = (uint8_t)atoi(cmd.params[1]);
        uint8_t b = (uint8_t)atoi(cmd.params[2]);
        g_ledManualHold  = true;
        g_ledManualStart = millis();
        // RGB off (0,0,0) → hold 2 s, any color → hold 5 s, then resume normal
        g_ledManualDurMs = (r == 0 && g == 0 && b == 0)
                           ? LED_MANUAL_OFF_MS : LED_MANUAL_COLOR_MS;
        setRGB(r, g, b);
        bt.sendAck("LED");
        return true;
    }

    if (strcmp(cmd.cmd, "LCD") == 0) {
        if (cmd.nParams < 2) { bt.sendError("BAD_PARAM:LCD"); return true; }
        uint8_t row = (uint8_t)atoi(cmd.params[0]);
        if (row > 1) { bt.sendError("BAD_PARAM:LCD"); return true; }
        lcdRow(row, "%s", cmd.params[1]);
        bt.sendAck("LCD");
        return true;
    }

    if (strcmp(cmd.cmd, "TEL") == 0) {
        bt.sendLine("TEL:CPUTEMP:0.0");

        float heading = 0.0f;
        if (compass.read(heading) == ESP_OK)
            bt.sendLine("TEL:HDG:%.1f:%s", heading, compassDir(heading));
        else
            bt.sendLine("TEL:HDG:0.0:N");

        float pitch = 0.0f, roll = 0.0f;
        if (imu.readAngles(pitch, roll) == ESP_OK) {
            bt.sendLine("TEL:PITCH:%.1f", pitch);
            bt.sendLine("TEL:ROLL:%.1f", roll);
        } else {
            bt.sendLine("TEL:PITCH:0.0");
            bt.sendLine("TEL:ROLL:0.0");
        }

        bt.sendLine("TEL:GPS:0.0:0.0");
        bt.sendLine("TEL:MOTOR:%s", motorStateStr(g_motorState));
        return true;
    }

    if (strcmp(cmd.cmd, "PING") == 0) {
        bt.sendPong();
        return true;
    }

    if (strcmp(cmd.cmd, "INFO") == 0) {
        bt.sendLine("TEL:INFO:FW:1.1.0-BLE-RTOS");
        return true;
    }

    return false;
}

// ================================================================
//  BT disconnect safety handler  (runs on control_task)
// ================================================================
static void handleBtDisconnect()
{
    ESP_LOGW(TAG, "BT disconnect — braking motors, LED blinking red");
    robot.brake();
    g_motorState     = MotorState::BRK;
    g_ledMode        = LedMode::DISCONNECTED;
    g_ledManualHold  = false;
    g_ledLastBlinkMs = millis();
    g_ledBlinkOn     = true;
    setRGB(255, 0, 0);
    lcdRow(1, "BT Disconnected");
}

// ================================================================
//  control_task — BT polling, LED cycling, connection state
//  50 Hz (20 ms period), pinned to core 1, priority 5
//
//  Owns: BT command dispatch, motor state, LED cycling, LCD row 1
//        when BT is connected.
// ================================================================
static void control_task(void * /*arg*/)
{
    auto& bt = btCommInstance();
    bool wasBtConnected = false;

    ESP_LOGI(TAG, "control_task started");

    while (true)
    {
        bt.update();

        bool nowConnected = bt.isConnected();

        // ── Connection state change ──────────────────────────────
        if (nowConnected && !wasBtConnected) {
            uint32_t now     = millis();
            g_ledMode        = LedMode::CONNECT_BURST;
            g_ledManualHold  = false;
            g_ledColorIndex  = 0;
            g_ledLastCycleMs = now;
            g_ledConnectMs   = now;
            g_lcdConnectMs   = now;
            g_motorState     = MotorState::IDLE;
            g_btConnected.store(true, std::memory_order_release);
            lcdRow(0, "  BT Connected!");
            lcdRow(1, "");
            ESP_LOGI(TAG, "BT connected — LED burst started");
        }
        else if (!nowConnected && wasBtConnected) {
            g_btConnected.store(false, std::memory_order_release);
        }
        wasBtConnected = nowConnected;

        // ── LED state machine ────────────────────────────────────
        updateLed();

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
}

// ================================================================
//  sensor_task — compass / IMU reads, LCD update when BT is off
//  10 Hz (100 ms period), pinned to core 1, priority 3
//
//  Owns: I2C sensor reads (compass, IMU), LCD rows 0 and 1 when
//        BT is NOT connected.  When BT is active, control_task
//        owns the LCD — sensor_task skips writes.
// ================================================================
static void sensor_task(void * /*arg*/)
{
    ESP_LOGI(TAG, "sensor_task started");

    while (true)
    {
        bool btOn = g_btConnected.load(std::memory_order_acquire);

        float heading = 0.0f;
        compass.read(heading);

        float pitch = 0.0f, roll = 0.0f;
        imu.readAngles(pitch, roll);

        // Update LCD:
        //   BT off  → show heading + pitch/roll
        //   BT on, first 3 s → "Connected" banner (written by control_task)
        //   BT on, after 3 s → show heading + pitch/roll
        if (!btOn) {
            lcdRow(0, "Hdg:%5.1f %s", heading, compassDir(heading));
            lcdRow(1, "P:%4d  R:%4d", (int)pitch, (int)roll);
        } else if (millis() - g_lcdConnectMs >= LCD_CONNECTED_SHOW_MS) {
            lcdRow(0, "Hdg:%5.1f %s", heading, compassDir(heading));
            lcdRow(1, "P:%4d  R:%4d", (int)pitch, (int)roll);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz
    }
}

// ================================================================
//  app_main — init hardware, create tasks, delete self
// ================================================================
//
// xTaskCreatePinnedToCore signature for reference:
//   xTaskCreatePinnedToCore(pvTaskCode, pcName, usStackDepth,
//                           pvParameters, uxPriority, pxCreatedTask, xCoreID)
//
// Core assignment:
//   Core 0 — reserved for NimBLE host task (created by nimble_port_freertos_init)
//   Core 1 — application tasks (control, sensor)
//
// Stack sizes:
//   control → 8 192 B — BT update, command dispatch, LED refresh, motor calls
//   sensor  → 4 096 B — I2C reads (compass + IMU), LCD writes
//   To measure actual usage: uxTaskGetStackHighWaterMark(h_control)
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "   Hello Robo MAZ! Starting up  ");
    ESP_LOGI(TAG, "   FreeRTOS multi-task mode      ");
    ESP_LOGI(TAG, "================================");

    // ── LED init ─────────────────────────────────────────────────
    init_led();
    setRGB(255, 0, 0);  // start blinking red until BLE connects

    // ── LCD init (SDA=GPIO05, SCL=GPIO06, address=0x27) ─────────
    lcd.init(GPIO_NUM_5, GPIO_NUM_6, 0x27);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("  RoboMAZ Ready!");

    // ── I2C sensors (share the bus the LCD already created) ──────
    if (compass.init(lcd.busHandle()) != ESP_OK) {
        ESP_LOGE(TAG, "Compass init FAILED — no HMC5883L or QMC5883L found");
    }
    imu.init(lcd.busHandle());

    // ── Motors ───────────────────────────────────────────────────
    robot.begin();
    vTaskDelay(pdMS_TO_TICKS(500));  // let power rails settle
    robot.coast();
    ESP_LOGI(TAG, "Hardware init complete.");

    // ── NVS init (required by NimBLE for bonding + RF calibration) ─
    {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
            ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        ESP_LOGI(TAG, "NVS flash initialised OK");
    }

    // ── BLE init ─────────────────────────────────────────────────
    // NimBLE host task is created internally on core 0
    auto& bt = btCommInstance();
    bt.begin("RoboMAZ-Explorer");
    bt.onCommand(handleBtCommand);
    bt.onDisconnect(handleBtDisconnect);

    lcdRow(1, "BT: Advertising");
    ESP_LOGI(TAG, "BLE initialised — creating RTOS tasks");

    // ── Create tasks — pinned to core 1 (core 0 = NimBLE) ───────
    xTaskCreatePinnedToCore(control_task, "control", 8192, nullptr, 5, &h_control, 1);
    xTaskCreatePinnedToCore(sensor_task,  "sensor",  4096, nullptr, 3, &h_sensor,  1);

    ESP_LOGI(TAG, "Tasks created — deleting app_main task");

    // Delete app_main's own task — frees its stack (~4 KB).
    // The application tasks are now fully owned by the FreeRTOS scheduler.
    vTaskDelete(nullptr);
}
