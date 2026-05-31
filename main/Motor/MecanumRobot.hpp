// ================================================================
//  MecanumRobot.hpp
//  RoboMAZ - Mecanum Wheel Robot Controller
//  ESP32-S3 / ESP-IDF  |  C++20
//
//  This file belongs in:  Motor/MecanumRobot.hpp
//  Its PWM dependency is: PWM/MAZPWM.hpp
// ================================================================
#pragma once

#include <cstdint>
#include "../PWM/MAZPWM.hpp"   // bare-metal PWM driver

// ── Drive mode ───────────────────────────────────────────────────
/**
 * @brief Selects how motor outputs are driven.
 *
 *   PWM — MCPWM hardware generates variable-duty PWM (variable speed).
 *   DC  — Simple GPIO high/low (full speed or off; no speed control).
 *         Use DC to verify wiring and DRV8833 hardware before enabling PWM.
 */
enum class DriveMode { PWM, DC };

// ── Pin bundle for one motor ─────────────────────────────────────
/**
 * @brief Holds the two GPIO pin numbers needed to drive a single
 *        DRV8833 H-bridge channel.
 *
 * The DRV8833 uses two logic inputs per motor:
 *   IN1 (cw_gpio)  → drives the motor clockwise when HIGH
 *   IN2 (ccw_gpio) → drives the motor counter-clockwise when HIGH
 *
 * Special combined states:
 *   IN1=0, IN2=0 → Coast  (freewheel)
 *   IN1=1, IN2=1 → Brake  (fast electromagnetic stop)
 */
struct MotorPins {
    int cw_gpio;   // IN1 on DRV8833 → Clockwise
    int ccw_gpio;  // IN2 on DRV8833 → Counter-Clockwise
};

// ── Motor index ──────────────────────────────────────────────────
/**
 * @brief Identifies each of the four mecanum wheels by position.
 *
 * Viewed from above (front of robot pointing up):
 *
 *   FRONT_LEFT  ──────  FRONT_RIGHT
 *        │     (top)        │
 *   REAR_LEFT   ──────  REAR_RIGHT
 *
 * MOTOR_COUNT is a sentinel used for array sizing — not a real motor.
 * The numeric values also serve as MAZPWM channel indices (0–3).
 */
enum MotorID {
    MOTOR_FRONT_LEFT  = 0,
    MOTOR_FRONT_RIGHT = 1,
    MOTOR_REAR_LEFT   = 2,
    MOTOR_REAR_RIGHT  = 3,
    MOTOR_COUNT       = 4
};

// ── Direction for a single motor ─────────────────────────────────
/**
 * @brief The four possible drive states for a single DRV8833 channel.
 *
 *   FORWARD  – CW pin gets PWM duty,  CCW pin = 0 %
 *   BACKWARD – CW pin = 0 %,          CCW pin gets PWM duty
 *   BRAKE    – both pins = 100 %      (fast electromagnetic stop)
 *   COAST    – both pins = 0 %        (motor freewheels)
 */
enum class MotorDirection {
    FORWARD,
    BACKWARD,
    BRAKE,
    COAST
};

// ================================================================
//  MecanumRobot class
// ================================================================
/**
 * @brief High-level controller for a four-wheel mecanum drive robot.
 *
 * Each wheel is driven by one channel of a DRV8833 dual H-bridge.
 * PWM is provided by the bare-metal MAZPWM driver (MCPWM peripheral).
 *
 * Mecanum wheel directions quick reference:
 *
 *   Movement        FL   FR   RL   RR
 *   Forward          +    +    +    +
 *   Backward         -    -    -    -
 *   Strafe Left      -    +    +    -
 *   Strafe Right     +    -    -    +
 *   Rotate CW        +    -    +    -
 *   Rotate CCW       -    +    -    +
 *   Diagonal FL      0    +    +    0
 *   Diagonal FR      +    0    0    +
 *
 * Usage:
 * @code
 *   MecanumRobot robot(pinFL, pinFR, pinRL, pinRR);
 *   robot.begin();           // call once in app_main
 *   robot.moveForward(75);   // drive at 75 %
 *   vTaskDelay(pdMS_TO_TICKS(2000));
 *   robot.brake();
 * @endcode
 */
class MecanumRobot {
public:
    /**
     * @brief Stores the GPIO pin configuration for all four motors.
     *        Does NOT initialise hardware — call begin() afterwards.
     *
     * @param frontLeft   GPIO pins for the front-left  motor (DRV8833 IN1/IN2)
     * @param frontRight  GPIO pins for the front-right motor
     * @param rearLeft    GPIO pins for the rear-left   motor
     * @param rearRight   GPIO pins for the rear-right  motor
     */
    MecanumRobot(MotorPins frontLeft,
                 MotorPins frontRight,
                 MotorPins rearLeft,
                 MotorPins rearRight);

    /**
     * @brief Resets all motor GPIO pins and initialises the chosen drive mode.
     *        Must be called exactly once from app_main before any movement command.
     *
     *        DriveMode::PWM — configures MCPWM hardware (variable speed).
     *        DriveMode::DC  — configures GPIO high/low outputs (full on/off only).
     *                         Use to verify wiring before enabling PWM.
     *
     * @param mode  DriveMode::PWM (default) or DriveMode::DC
     */
    void begin(DriveMode mode = DriveMode::PWM);

    // ── Cardinal movements ───────────────────────────────────────
    /**
     * @brief Drives the robot straight forward.
     *        All four wheels rotate in the forward direction.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void moveForward (float speed_pct);

    /**
     * @brief Drives the robot straight backward.
     *        All four wheels rotate in the reverse direction.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void moveBackward(float speed_pct);

    /**
     * @brief Translates the robot purely sideways to the left
     *        without any rotation (mecanum strafing).
     *        FL=back, FR=fwd, RL=fwd, RR=back
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void strafeLeft  (float speed_pct);

    /**
     * @brief Translates the robot purely sideways to the right
     *        without any rotation (mecanum strafing).
     *        FL=fwd, FR=back, RL=back, RR=fwd
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void strafeRight (float speed_pct);

    // ── Diagonal movements (Mecanum exclusive) ───────────────────
    /**
     * @brief Moves diagonally toward the front-left corner.
     *        FL and RR coast; FR and RL drive forward.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void moveDiagonalFrontLeft (float speed_pct);

    /**
     * @brief Moves diagonally toward the front-right corner.
     *        FR and RL coast; FL and RR drive forward.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void moveDiagonalFrontRight(float speed_pct);

    /**
     * @brief Moves diagonally toward the rear-left corner.
     *        FL and RR drive backward; FR and RL coast.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void moveDiagonalRearLeft  (float speed_pct);

    /**
     * @brief Moves diagonally toward the rear-right corner.
     *        FR and RL drive backward; FL and RR coast.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void moveDiagonalRearRight (float speed_pct);

    // ── Rotation ─────────────────────────────────────────────────
    /**
     * @brief Rotates the robot clockwise in place.
     *        Left wheels forward, right wheels backward.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void rotateClockwise       (float speed_pct);

    /**
     * @brief Rotates the robot counter-clockwise in place.
     *        Left wheels backward, right wheels forward.
     * @param speed_pct  Duty-cycle percentage [0.0 – 100.0]
     */
    void rotateCounterClockwise(float speed_pct);

    // ── Stop modes ───────────────────────────────────────────────
    /**
     * @brief Applies active electromagnetic braking to all motors.
     *        Both DRV8833 inputs are driven at 100 % duty → fast stop.
     */
    void brake();

    /**
     * @brief Cuts power to all motors (both inputs 0 %).
     *        Motors decelerate naturally. Safe power-on default.
     */
    void coast();

    /**
     * @brief Updates the cached global speed without changing direction.
     *        Takes effect on the next movement call.
     * @param speed_pct  Desired speed percentage [0.0 – 100.0] (clamped)
     */
    void setGlobalSpeed(float speed_pct);

private:
    MotorPins _pins[MOTOR_COUNT];   ///< GPIO config for each motor
    MAZPWM    _pwm;                 ///< Bare-metal PWM driver instance
    DriveMode _mode = DriveMode::PWM;

    /**
     * @brief Resolves four signed speed values into direction + magnitude
     *        and calls _driveMotor() for each wheel.
     *
     *        > 0 → FORWARD, < 0 → BACKWARD, 0 → COAST
     *
     * @param fl  Signed speed for front-left  motor [-100.0 – +100.0]
     * @param fr  Signed speed for front-right motor
     * @param rl  Signed speed for rear-left   motor
     * @param rr  Signed speed for rear-right  motor
     */
    void _applyMotorVector(float fl, float fr, float rl, float rr);

    /**
     * @brief Writes PWM duty values directly to MAZPWM for one motor.
     *
     * @param id         Target motor
     * @param dir        Desired direction / stop mode
     * @param speed_pct  Speed percentage [0.0 – 100.0]; ignored for BRAKE/COAST
     */
    void _driveMotor(MotorID id, MotorDirection dir, float speed_pct);

    float _currentSpeed = 50.0f;   ///< Cached global speed (unused internally)
};
