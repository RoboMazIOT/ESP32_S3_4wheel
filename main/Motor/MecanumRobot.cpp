// ================================================================
//  MecanumRobot.cpp
//  RoboMAZ - Mecanum Wheel Robot Controller
//  ESP32-S3 / ESP-IDF  |  C++20
//
//  This file belongs in:  Motor/MecanumRobot.cpp
//  Its PWM dependency is: PWM/MAZPWM  (via MecanumRobot.hpp)
// ================================================================
#include "MecanumRobot.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include <algorithm>
#include <cmath>

static const char* TAG = "MecanumRobot";

// ================================================================
//  Constructor
// ================================================================
/**
 * Stores the pin configuration for all four motors.
 * Hardware is NOT touched here — call begin() from app_main.
 */
MecanumRobot::MecanumRobot(MotorPins frontLeft,
                            MotorPins frontRight,
                            MotorPins rearLeft,
                            MotorPins rearRight)
{
    _pins[MOTOR_FRONT_LEFT]  = frontLeft;
    _pins[MOTOR_FRONT_RIGHT] = frontRight;
    _pins[MOTOR_REAR_LEFT]   = rearLeft;
    _pins[MOTOR_REAR_RIGHT]  = rearRight;
}

// ================================================================
//  begin()
// ================================================================
/**
 * Initialises the bare-metal MAZPWM peripheral, routes every motor's
 * two GPIO pins, then coasts all motors as a safe default.
 *
 * Call order:
 *   1. _pwm.init()             → configures MCPWM0 timers/operators/generators
 *   2. _pwm.attachMotorPin()   → GPIO matrix routes MCPWM outputs to physical pins
 *   3. coast()                 → ensures motors are off on startup
 */
void MecanumRobot::begin(DriveMode mode)
{
    _mode = mode;
    ESP_LOGI(TAG, "Initialising MecanumRobot — mode: %s",
             mode == DriveMode::PWM ? "PWM @ 20 kHz" : "DC direct GPIO");

    // Reset every motor pin to a clean input state before reconfiguring.
    // This clears any leftover IO MUX / GPIO matrix state from a previous boot.
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        gpio_reset_pin(static_cast<gpio_num_t>(_pins[i].cw_gpio));
        gpio_reset_pin(static_cast<gpio_num_t>(_pins[i].ccw_gpio));
    }

    if (mode == DriveMode::PWM) {
        _pwm.init();
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            _pwm.attachMotorPin(
                static_cast<uint8_t>(i),
                _pins[i].cw_gpio,
                _pins[i].ccw_gpio
            );
        }
    } else {
        // DC mode: plain GPIO outputs, no MCPWM involved
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            gpio_set_direction(static_cast<gpio_num_t>(_pins[i].cw_gpio),  GPIO_MODE_OUTPUT);
            gpio_set_direction(static_cast<gpio_num_t>(_pins[i].ccw_gpio), GPIO_MODE_OUTPUT);
        }
    }

    coast();
    ESP_LOGI(TAG, "All motors initialised → COAST");
}

// ================================================================
//  Cardinal movements
// ================================================================
/**
 * Drives all four wheels forward at the requested duty-cycle.
 * All motors spin in the same direction → straight forward motion.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::moveForward(float speed_pct)
{
    _applyMotorVector(+speed_pct, +speed_pct,
                      +speed_pct, +speed_pct);
}

/**
 * Drives all four wheels backward at the requested duty-cycle.
 * All motors spin in reverse → straight backward motion.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::moveBackward(float speed_pct)
{
    _applyMotorVector(-speed_pct, -speed_pct,
                      -speed_pct, -speed_pct);
}

/**
 * Strafes the robot purely to the left without rotating.
 * Mecanum wheel vector: FL=back, FR=fwd, RL=fwd, RR=back
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::strafeLeft(float speed_pct)
{
    _applyMotorVector(-speed_pct, +speed_pct,
                      +speed_pct, -speed_pct);
}

/**
 * Strafes the robot purely to the right without rotating.
 * Mecanum wheel vector: FL=fwd, FR=back, RL=back, RR=fwd
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::strafeRight(float speed_pct)
{
    _applyMotorVector(+speed_pct, -speed_pct,
                      -speed_pct, +speed_pct);
}

// ================================================================
//  Diagonal movements
// ================================================================
/**
 * Moves diagonally toward the front-left corner.
 * FL and RR coast; only FR and RL drive forward.
 * Net force vector points 45° toward front-left.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::moveDiagonalFrontLeft(float speed_pct)
{
    _applyMotorVector(0,           +speed_pct,
                      +speed_pct,  0);
}

/**
 * Moves diagonally toward the front-right corner.
 * FR and RL coast; only FL and RR drive forward.
 * Net force vector points 45° toward front-right.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::moveDiagonalFrontRight(float speed_pct)
{
    _applyMotorVector(+speed_pct,  0,
                      0,           +speed_pct);
}

/**
 * Moves diagonally toward the rear-left corner.
 * FL and RR drive backward; FR and RL coast.
 * Net force vector points 45° toward rear-left.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::moveDiagonalRearLeft(float speed_pct)
{
    _applyMotorVector(-speed_pct,  0,
                      0,           -speed_pct);
}

/**
 * Moves diagonally toward the rear-right corner.
 * FR and RL drive backward; FL and RR coast.
 * Net force vector points 45° toward rear-right.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::moveDiagonalRearRight(float speed_pct)
{
    _applyMotorVector(0,           -speed_pct,
                      -speed_pct,  0);
}

// ================================================================
//  Rotation
// ================================================================
/**
 * Rotates the robot clockwise in place (zero translation).
 * Left wheels spin forward, right wheels spin backward.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::rotateClockwise(float speed_pct)
{
    _applyMotorVector(+speed_pct, -speed_pct,
                      +speed_pct, -speed_pct);
}

/**
 * Rotates the robot counter-clockwise in place (zero translation).
 * Left wheels spin backward, right wheels spin forward.
 *
 * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
 */
void MecanumRobot::rotateCounterClockwise(float speed_pct)
{
    _applyMotorVector(-speed_pct, +speed_pct,
                      -speed_pct, +speed_pct);
}

// ================================================================
//  Stop modes
// ================================================================
/**
 * Applies fast electromagnetic braking to all four motors.
 * Both DRV8833 inputs are driven at 100 % → motor windings shorted.
 * Use when a quick, precise stop is needed.
 */
void MecanumRobot::brake()
{
    for (int i = 0; i < MOTOR_COUNT; ++i)
        _driveMotor(static_cast<MotorID>(i), MotorDirection::BRAKE, 0);
}

/**
 * Cuts power to all motors (both inputs 0 %).
 * Motors decelerate naturally through friction.
 * This is the safe power-on default set by begin().
 */
void MecanumRobot::coast()
{
    for (int i = 0; i < MOTOR_COUNT; ++i)
        _driveMotor(static_cast<MotorID>(i), MotorDirection::COAST, 0);
}

// ================================================================
//  setGlobalSpeed()
// ================================================================
/**
 * Caches a new global speed value (clamped to [0, 100]).
 * Does NOT immediately update any motor — takes effect on the next
 * movement call if the caller passes _currentSpeed.
 *
 * @param speed_pct  Desired speed percentage [0.0 – 100.0]
 */
void MecanumRobot::setGlobalSpeed(float speed_pct)
{
    _currentSpeed = std::clamp(speed_pct, 0.0f, 100.0f);
    ESP_LOGI(TAG, "Global speed set to %.1f%%", _currentSpeed);
}

// ================================================================
//  _applyMotorVector()  (private)
// ================================================================
/**
 * Resolves four signed speed percentages into direction + magnitude
 * and calls _driveMotor() for each wheel.
 *
 * Sign convention:
 *   > 0  → FORWARD  at +val %
 *   < 0  → BACKWARD at -val %
 *   = 0  → COAST
 *
 * @param fl  Signed speed for front-left  motor [-100.0 – +100.0]
 * @param fr  Signed speed for front-right motor
 * @param rl  Signed speed for rear-left   motor
 * @param rr  Signed speed for rear-right  motor
 */
void MecanumRobot::_applyMotorVector(float fl, float fr,
                                      float rl, float rr)
{
    auto resolve = [&](MotorID id, float val) {
        if      (val > 0.0f) _driveMotor(id, MotorDirection::FORWARD,  val);
        else if (val < 0.0f) _driveMotor(id, MotorDirection::BACKWARD, -val);
        else                 _driveMotor(id, MotorDirection::COAST,     0.0f);
    };

    resolve(MOTOR_FRONT_LEFT,  fl);
    resolve(MOTOR_FRONT_RIGHT, fr);
    resolve(MOTOR_REAR_LEFT,   rl);
    resolve(MOTOR_REAR_RIGHT,  rr);
}

// ================================================================
//  _driveMotor()  (private)
// ================================================================
/**
 * Translates a direction + speed request into two MAZPWM duty values
 * and writes them to the motor's MCPWM channel.
 *
 * DRV8833 truth table applied:
 *   FORWARD  → CW = duty %,   CCW = 0 %
 *   BACKWARD → CW = 0 %,      CCW = duty %
 *   BRAKE    → CW = 100 %,    CCW = 100 %
 *   COAST    → CW = 0 %,      CCW = 0 %
 *
 * MAZPWM.setMotorDuty() accepts motor index 0–3 which directly maps
 * to MotorID values, so no channel translation is needed.
 *
 * @param id         Target motor (MOTOR_FRONT_LEFT … MOTOR_REAR_RIGHT)
 * @param dir        Desired drive direction / stop mode
 * @param speed_pct  Speed percentage [0.0 – 100.0]; ignored for BRAKE/COAST
 */
void MecanumRobot::_driveMotor(MotorID id, MotorDirection dir, float speed_pct)
{
    if (_mode == DriveMode::DC) {
        const auto cw  = static_cast<gpio_num_t>(_pins[id].cw_gpio);
        const auto ccw = static_cast<gpio_num_t>(_pins[id].ccw_gpio);
        switch (dir) {
            case MotorDirection::FORWARD:
                gpio_set_level(cw, 1); gpio_set_level(ccw, 0); break;
            case MotorDirection::BACKWARD:
                gpio_set_level(cw, 0); gpio_set_level(ccw, 1); break;
            case MotorDirection::BRAKE:
                gpio_set_level(cw, 1); gpio_set_level(ccw, 1); break;
            case MotorDirection::COAST:
            default:
                gpio_set_level(cw, 0); gpio_set_level(ccw, 0); break;
        }
        return;
    }

    // PWM mode
    const uint8_t ch = static_cast<uint8_t>(id);
    switch (dir) {
        case MotorDirection::FORWARD:
            _pwm.setMotorDuty(ch, speed_pct, 0.0f);   break;
        case MotorDirection::BACKWARD:
            _pwm.setMotorDuty(ch, 0.0f, speed_pct);   break;
        case MotorDirection::BRAKE:
            _pwm.setMotorDuty(ch, 100.0f, 100.0f);    break;
        case MotorDirection::COAST:
        default:
            _pwm.setMotorDuty(ch, 0.0f, 0.0f);        break;
    }
}
