// ================================================================
//  MAZGY271.hpp
//  RoboMAZ - HMC5883L (GY-271) 3-axis compass driver over I2C
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  Wiring:
//    VCC → 3.3 V     GND → GND
//    SDA → shared with LCD SDA (GPIO05)
//    SCL → shared with LCD SCL (GPIO06)
//
//  I2C address: 0x1E (fixed, no solder jumper on GY-271)
// ================================================================
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

static constexpr uint8_t GY271_I2C_ADDR = 0x1E;

class MAZGY271 {
public:
    /**
     * @brief Adds the HMC5883L as a device on an existing I2C bus and
     *        configures it for continuous measurement at 15 Hz.
     *        Call after lcd.init() so the bus already exists.
     *
     * @param bus   Bus handle from lcd.busHandle()
     * @param addr  HMC5883L address (always 0x1E on GY-271 modules)
     */
    esp_err_t init(i2c_master_bus_handle_t bus, uint8_t addr = GY271_I2C_ADDR);

    /**
     * @brief Reads the latest measurement and returns heading in degrees [0–360).
     *        0° = magnetic North, increases clockwise.
     *
     * @param heading_deg  Output: computed heading
     * @return ESP_OK on success, ESP_ERR_* on I2C failure or overflow
     */
    esp_err_t read(float& heading_deg);

private:
    i2c_master_dev_handle_t _dev = nullptr;

    esp_err_t _writeReg(uint8_t reg, uint8_t val);
    esp_err_t _readRegs(uint8_t reg, uint8_t* buf, size_t len);
};
