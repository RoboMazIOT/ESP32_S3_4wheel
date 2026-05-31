// ================================================================
//  MAZMPU6050.hpp
//  RoboMAZ - MPU-6050 accelerometer/gyro driver over I2C
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  Wiring:
//    VCC → 3.3 V     GND → GND
//    SDA → shared with LCD and GY-271 (GPIO05)
//    SCL → shared with LCD and GY-271 (GPIO06)
//    AD0 → GND  → I2C address 0x68
//    AD0 → 3.3V → I2C address 0x69
//
//  Angles are computed from the accelerometer only (no drift,
//  but sensitive to vibration). Good enough for static display.
// ================================================================
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

static constexpr uint8_t MPU6050_I2C_ADDR = 0x68;  // AD0 = GND

class MAZMPU6050 {
public:
    /**
     * @brief Adds the MPU-6050 as a device on an existing I2C bus
     *        and wakes it from sleep mode.
     *        Call after lcd.init() so the bus already exists.
     *
     * @param bus   Bus handle from lcd.busHandle()
     * @param addr  0x68 (AD0=GND) or 0x69 (AD0=3.3V)
     */
    esp_err_t init(i2c_master_bus_handle_t bus, uint8_t addr = MPU6050_I2C_ADDR);

    /**
     * @brief Reads accelerometer and computes pitch and roll.
     *        Pitch: rotation around Y axis (nose up/down).
     *        Roll:  rotation around X axis (left/right tilt).
     *        Both in degrees, range ±90°.
     *
     * @param pitch_deg  Output: pitch angle
     * @param roll_deg   Output: roll angle
     */
    esp_err_t readAngles(float& pitch_deg, float& roll_deg);

private:
    i2c_master_dev_handle_t _dev = nullptr;

    esp_err_t _writeReg(uint8_t reg, uint8_t val);
    esp_err_t _readRegs(uint8_t reg, uint8_t* buf, size_t len);
};
