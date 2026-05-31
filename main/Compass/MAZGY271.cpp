// ================================================================
//  MAZGY271.cpp
//  RoboMAZ - HMC5883L (GY-271) compass driver
//  ESP32-S3 / ESP-IDF v6  |  C++20
// ================================================================
#include "MAZGY271.hpp"

#include "esp_err.h"
#include <cmath>   // atan2f, M_PI

// HMC5883L register addresses
static constexpr uint8_t REG_CONFIG_A = 0x00;
static constexpr uint8_t REG_CONFIG_B = 0x01;
static constexpr uint8_t REG_MODE     = 0x02;
static constexpr uint8_t REG_DATA_X_H = 0x03;  // first of 6 data bytes

// ── _writeReg() ─────────────────────────────────────────────────
esp_err_t MAZGY271::_writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(_dev, buf, sizeof(buf), 100);
}

// ── _readRegs() ─────────────────────────────────────────────────
esp_err_t MAZGY271::_readRegs(uint8_t reg, uint8_t* out, size_t len)
{
    return i2c_master_transmit_receive(_dev, &reg, 1, out, len, 100);
}

// ================================================================
//  init()
// ================================================================
esp_err_t MAZGY271::init(i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr;
    dc.scl_speed_hz    = 50000;  // match LCD bus speed (internal pull-ups)

    esp_err_t ret = i2c_master_bus_add_device(bus, &dc, &_dev);
    if (ret != ESP_OK) return ret;

    // 8 samples averaged, 15 Hz output rate, normal measurement
    ret = _writeReg(REG_CONFIG_A, 0x70);
    if (ret != ESP_OK) return ret;

    // Gain = 1.3 Ga (±1.3 Ga range, 1090 LSB/Ga)
    ret = _writeReg(REG_CONFIG_B, 0x20);
    if (ret != ESP_OK) return ret;

    // Continuous measurement mode
    return _writeReg(REG_MODE, 0x00);
}

// ================================================================
//  read()
// ================================================================
esp_err_t MAZGY271::read(float& heading_deg)
{
    uint8_t buf[6] = {};
    // HMC5883L data order: X_H, X_L, Z_H, Z_L, Y_H, Y_L  (Z before Y)
    esp_err_t ret = _readRegs(REG_DATA_X_H, buf, sizeof(buf));
    if (ret != ESP_OK) return ret;

    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t z = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t y = (int16_t)((buf[4] << 8) | buf[5]);
    (void)z;  // Z not needed for 2D heading

    // Overflow: HMC5883L outputs -4096 on overflow
    if (x == -4096 || y == -4096) return ESP_ERR_INVALID_RESPONSE;

    float heading = atan2f((float)y, (float)x) * (180.0f / (float)M_PI);
    if (heading < 0.0f) heading += 360.0f;

    heading_deg = heading;
    return ESP_OK;
}
