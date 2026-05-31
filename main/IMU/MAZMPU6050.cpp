// ================================================================
//  MAZMPU6050.cpp
//  RoboMAZ - MPU-6050 driver
//  ESP32-S3 / ESP-IDF v6  |  C++20
// ================================================================
#include "MAZMPU6050.hpp"

#include "esp_err.h"
#include <cmath>  // atan2f, sqrtf, M_PI

// MPU-6050 register addresses
static constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // first of 6 accel bytes

// Accelerometer sensitivity at default ±2g range: 16384 LSB/g
static constexpr float ACCEL_SCALE = 16384.0f;

// ── _writeReg() ─────────────────────────────────────────────────
esp_err_t MAZMPU6050::_writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(_dev, buf, sizeof(buf), 100);
}

// ── _readRegs() ─────────────────────────────────────────────────
esp_err_t MAZMPU6050::_readRegs(uint8_t reg, uint8_t* out, size_t len)
{
    return i2c_master_transmit_receive(_dev, &reg, 1, out, len, 100);
}

// ================================================================
//  init()
// ================================================================
esp_err_t MAZMPU6050::init(i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr;
    dc.scl_speed_hz    = 50000;  // match LCD/GY-271 bus speed

    esp_err_t ret = i2c_master_bus_add_device(bus, &dc, &_dev);
    if (ret != ESP_OK) return ret;

    // Wake up: MPU-6050 powers on in sleep mode (PWR_MGMT_1 bit6 = 1)
    return _writeReg(REG_PWR_MGMT_1, 0x00);
}

// ================================================================
//  readAngles()
// ================================================================
esp_err_t MAZMPU6050::readAngles(float& pitch_deg, float& roll_deg)
{
    uint8_t buf[6] = {};
    // Reads ACCEL_XOUT_H through ACCEL_ZOUT_L (6 bytes)
    esp_err_t ret = _readRegs(REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (ret != ESP_OK) return ret;

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    float ax = ax_raw / ACCEL_SCALE;
    float ay = ay_raw / ACCEL_SCALE;
    float az = az_raw / ACCEL_SCALE;

    // Pitch: nose up = positive, nose down = negative
    pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);

    // Roll: right side down = positive, left side down = negative
    roll_deg = atan2f(ay, az) * (180.0f / (float)M_PI);

    return ESP_OK;
}
