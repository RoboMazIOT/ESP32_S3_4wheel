// ================================================================
//  MAZPWM.hpp
//  RoboMAZ - PWM Driver (IDF MCPWM high-level API)
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  Uses the ESP-IDF esp_driver_mcpwm component — no bare-metal
//  register access.  The driver handles clock enable, dual-core
//  safety, and register ordering internally.
//
//  Motor → peripheral mapping:
//    Motor 0 (FL) → MCPWM group 0, timer/operator 0
//    Motor 1 (FR) → MCPWM group 0, timer/operator 1
//    Motor 2 (RL) → MCPWM group 0, timer/operator 2
//    Motor 3 (RR) → MCPWM group 1, timer/operator 0
//
//  Each motor uses:
//    1 timer  ─ 20 kHz up-count, 1 000 ticks resolution
//    1 operator connected to that timer
//    2 comparators  (CW = index 0, CCW = index 1)
//    2 generators   (CW = index 0, CCW = index 1)
//
//  Duty control (DRV8833 truth table):
//    COAST   setMotorDuty( 0,   0  ) → both forced LOW
//    FORWARD setMotorDuty( n,   0  ) → CW = n % PWM,  CCW forced LOW
//    REVERSE setMotorDuty( 0,   n  ) → CW forced LOW, CCW = n % PWM
//    BRAKE   setMotorDuty(100, 100) → both forced HIGH (short-brake)
// ================================================================
#pragma once

#include <cstdint>
#include "driver/mcpwm_timer.h"  // mcpwm_timer_handle_t
#include "driver/mcpwm_oper.h"   // mcpwm_oper_handle_t
#include "driver/mcpwm_cmpr.h"   // mcpwm_cmpr_handle_t
#include "driver/mcpwm_gen.h"    // mcpwm_gen_handle_t

// ── PWM frequency & resolution ───────────────────────────────────
static constexpr uint32_t PWM_FREQ_HZ     = 20'000UL;         // 20 kHz — above audible range, good for DRV8833
static constexpr uint32_t PWM_RES_HZ      = 20'000'000UL;    // 20 MHz timer clock
static constexpr uint32_t PWM_PERIOD_TICKS = PWM_RES_HZ / PWM_FREQ_HZ;  // 1 000

// ── Motor count ──────────────────────────────────────────────────
static constexpr uint8_t MAZ_MOTOR_COUNT = 4;

// ================================================================
//  MAZPWM
// ================================================================
class MAZPWM {
public:
    /**
     * @brief Creates and starts timers, operators, and comparators for
     *        all four motors.  Call once from app_main before
     *        attachMotorPin().
     */
    void init();

    /**
     * @brief Creates generators and routes CW/CCW outputs to GPIO pins.
     *        Must be called after init(), once per motor.
     *
     * @param motorIdx  0–3
     * @param cwGpio    GPIO for CW  output (DRV8833 IN1)
     * @param ccwGpio   GPIO for CCW output (DRV8833 IN2)
     */
    void attachMotorPin(uint8_t motorIdx, int cwGpio, int ccwGpio);

    /**
     * @brief Sets duty cycles for both direction pins of one motor.
     *
     *   0 %   → output forced LOW  (idle / one leg of a coast/brake pair)
     *   1–99% → standard up-count PWM
     *   100 % → output forced HIGH (brake leg / maximum speed leg)
     *
     * @param motorIdx  0–3
     * @param cwDuty    CW  duty [0.0 – 100.0]
     * @param ccwDuty   CCW duty [0.0 – 100.0]
     */
    void setMotorDuty(uint8_t motorIdx, float cwDuty, float ccwDuty);

    /** @brief Coast one motor (both outputs LOW). */
    void stopMotor(uint8_t motorIdx);

    /** @brief Coast all four motors. */
    void stopAll();

private:
    mcpwm_timer_handle_t _timer[MAZ_MOTOR_COUNT]    = {};
    mcpwm_oper_handle_t  _oper[MAZ_MOTOR_COUNT]     = {};
    mcpwm_cmpr_handle_t  _cmpr[MAZ_MOTOR_COUNT][2]  = {};  // [motor][0=CW, 1=CCW]
    mcpwm_gen_handle_t   _gen[MAZ_MOTOR_COUNT][2]   = {};  // [motor][0=CW, 1=CCW]
};
