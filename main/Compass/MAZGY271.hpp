// ================================================================
//  MAZGY271.hpp
//  RoboMAZ - GY-271 compass driver (auto-detects HMC5883L / QMC5883L)
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  Wiring:
//    VCC → 3.3 V     GND → GND
//    SDA → shared with LCD SDA (GPIO05)
//    SCL → shared with LCD SCL (GPIO06)
//
//  I2C address:
//    HMC5883L → 0x1E (original Honeywell)
//    QMC5883L → 0x0D (common clone on most GY-271 boards sold today)
//
//  init() probes both addresses and configures whichever responds.
// ================================================================
#pragma once

#include <cstdint>
#include "esp_err.h"
#include "driver/i2c_master.h"

static constexpr uint8_t HMC5883L_ADDR = 0x1E;
static constexpr uint8_t QMC5883L_ADDR = 0x0D;

enum class CompassChip : uint8_t { NONE, HMC5883L, QMC5883L };

class MAZGY271 {
public:
    /**
     * @brief Probes the I2C bus for HMC5883L (0x1E) then QMC5883L (0x0D).
     *        Configures whichever responds for continuous measurement.
     *        Call after lcd.init() so the bus already exists.
     *
     * @param bus   Bus handle from lcd.busHandle()
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if neither chip responds
     */
    esp_err_t init(i2c_master_bus_handle_t bus);

    /**
     * @brief Reads the latest measurement and returns heading in degrees [0–360).
     *        0° = magnetic North, increases clockwise.
     *
     * @param heading_deg  Output: computed heading
     * @return ESP_OK on success, ESP_ERR_* on I2C failure or overflow
     */
    esp_err_t read(float& heading_deg);

    /**
     * @brief Returns which chip was detected (NONE before init).
     */
    CompassChip detectedChip() const { return _chip; }

private:
    i2c_master_dev_handle_t _dev  = nullptr;
    CompassChip             _chip = CompassChip::NONE;

    esp_err_t _writeReg(uint8_t reg, uint8_t val);
    esp_err_t _readRegs(uint8_t reg, uint8_t* buf, size_t len);

    esp_err_t _initHMC();
    esp_err_t _initQMC();
    esp_err_t _readHMC(float& heading_deg);
    esp_err_t _readQMC(float& heading_deg);

    esp_err_t _tryProbe(i2c_master_bus_handle_t bus, uint8_t addr);
};
