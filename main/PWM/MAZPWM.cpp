// ================================================================
//  MAZPWM.cpp
//  RoboMAZ - PWM Driver (IDF MCPWM high-level API)
//  ESP32-S3 / ESP-IDF v6  |  C++20
// ================================================================
#include "MAZPWM.hpp"

#include <cstdint>
#include <algorithm>
#include "esp_err.h"   // ESP_ERROR_CHECK

// ── Group mapping ────────────────────────────────────────────────
// Motors 0–2 share MCPWM group 0 (3 timers available).
// Motor  3   uses  MCPWM group 1 (independent hardware).
static constexpr int groupOf(uint8_t m) { return (m < 3) ? 0 : 1; }

// ── applyDuty() ─────────────────────────────────────────────────
// Controls one generator via the action table — no force level used.
//
// The TEZ (timer = 0) action is the only thing that changes:
//
//   0 %   → TEZ → LOW   : pin stays LOW every period (coast/off)
//   1–99% → TEZ → HIGH  : pin goes HIGH at start of period,
//                          compare fires at 'ticks' → LOW  (PWM)
//   100 % → TEZ → HIGH, compare = PWM_PERIOD_TICKS (1000):
//                          compare > timer max (999) so it never
//                          fires → pin stays HIGH every period (full on)
//
// Both the TEZ action change and the shadow compare transfer happen
// at the same TEZ boundary, so every new period starts clean.
static void applyDuty(mcpwm_gen_handle_t  gen,
                      mcpwm_cmpr_handle_t cmpr,
                      float               pct)
{
    if (pct <= 0.0f) {
        // TEZ → LOW: pin is always LOW
        mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_LOW));
    } else {
        // TEZ → HIGH: start each period high, compare → LOW
        const uint32_t ticks = (pct >= 100.0f)
            ? PWM_PERIOD_TICKS   // compare beyond timer range → never fires → 100 %
            : static_cast<uint32_t>((pct / 100.0f) * static_cast<float>(PWM_PERIOD_TICKS));
        mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                          MCPWM_TIMER_EVENT_EMPTY,
                                          MCPWM_GEN_ACTION_HIGH));
        mcpwm_comparator_set_compare_value(cmpr, ticks); // shadow → active at next TEZ
    }
}

// ================================================================
//  init()
// ================================================================
void MAZPWM::init()
{
    for (uint8_t i = 0; i < MAZ_MOTOR_COUNT; ++i) {

        // ── Timer ────────────────────────────────────────────────
        // The IDF driver allocates timers within the group in order,
        // so motors 0,1,2 get timers 0,1,2 in group 0 and motor 3
        // gets timer 0 in group 1.
        mcpwm_timer_config_t tcfg = {};
        tcfg.group_id      = groupOf(i);
        tcfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
        tcfg.resolution_hz = PWM_RES_HZ;       // 20 MHz → 1 µs per tick
        tcfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
        tcfg.period_ticks  = PWM_PERIOD_TICKS;  // 1 000 ticks → 50 µs = 20 kHz
        ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &_timer[i]));

        // ── Operator ─────────────────────────────────────────────
        mcpwm_operator_config_t ocfg = {};
        ocfg.group_id = groupOf(i);
        ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &_oper[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(_oper[i], _timer[i]));

        // ── Comparators (0 = CW, 1 = CCW) ───────────────────────
        // shadow → active at TEZ: new duty takes effect at the start
        // of the next full PWM period — glitch-free transitions.
        mcpwm_comparator_config_t ccfg = {};
        ccfg.flags.update_cmp_on_tez = true;
        for (int ch = 0; ch < 2; ++ch) {
            ESP_ERROR_CHECK(mcpwm_new_comparator(_oper[i], &ccfg, &_cmpr[i][ch]));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(_cmpr[i][ch], 0));
        }

        // ── Start timer ──────────────────────────────────────────
        // Generators are created later in attachMotorPin().
        // Timer runs continuously from here so comparator events
        // are ready the moment the first generator is attached.
        ESP_ERROR_CHECK(mcpwm_timer_enable(_timer[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(_timer[i], MCPWM_TIMER_START_NO_STOP));
    }
}

// ================================================================
//  attachMotorPin()
// ================================================================
void MAZPWM::attachMotorPin(uint8_t motorIdx, int cwGpio, int ccwGpio)
{
    const int gpios[2] = { cwGpio, ccwGpio };

    for (int ch = 0; ch < 2; ++ch) {
        // ── Generator ────────────────────────────────────────────
        mcpwm_generator_config_t gcfg = {};
        gcfg.gen_gpio_num = gpios[ch];
        ESP_ERROR_CHECK(mcpwm_new_generator(_oper[motorIdx], &gcfg, &_gen[motorIdx][ch]));

        // ── Action table: initial coast (both pins LOW) ──────────
        // TEZ (counter = 0) → pin LOW   (overridden to HIGH by applyDuty when duty > 0)
        // Compare match     → pin LOW
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            _gen[motorIdx][ch],
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,   // TEZ
                MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            _gen[motorIdx][ch],
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                _cmpr[motorIdx][ch],
                MCPWM_GEN_ACTION_LOW)));
    }
}

// ================================================================
//  setMotorDuty()
// ================================================================
void MAZPWM::setMotorDuty(uint8_t motorIdx, float cwDuty, float ccwDuty)
{
    applyDuty(_gen[motorIdx][0], _cmpr[motorIdx][0], cwDuty);
    applyDuty(_gen[motorIdx][1], _cmpr[motorIdx][1], ccwDuty);
}

// ================================================================
//  stopMotor() / stopAll()
// ================================================================
void MAZPWM::stopMotor(uint8_t motorIdx)
{
    setMotorDuty(motorIdx, 0.0f, 0.0f);
}

void MAZPWM::stopAll()
{
    for (uint8_t i = 0; i < MAZ_MOTOR_COUNT; ++i)
        stopMotor(i);
}
