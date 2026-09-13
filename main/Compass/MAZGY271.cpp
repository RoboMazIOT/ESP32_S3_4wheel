// ================================================================
//  MAZGY271.cpp
//  RoboMAZ - GY-271 compass driver (auto-detects HMC5883L / QMC5883L)
//  ESP32-S3 / ESP-IDF v6  |  C++20
// ================================================================
#include "MAZGY271.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}
#include "esp_log.h"
#include "esp_err.h"
#include <cmath>   // atan2f, M_PI

static const char* TAG = "Compass";

// ── HMC5883L registers ──────────────────────────────────────────
static constexpr uint8_t HMC_REG_CONFIG_A = 0x00;
static constexpr uint8_t HMC_REG_CONFIG_B = 0x01;
static constexpr uint8_t HMC_REG_MODE     = 0x02;
static constexpr uint8_t HMC_REG_DATA_X_H = 0x03;  // X_H X_L Z_H Z_L Y_H Y_L

// ── QMC5883L registers ──────────────────────────────────────────
static constexpr uint8_t QMC_REG_DATA_X_L = 0x00;  // X_L X_H Y_L Y_H Z_L Z_H
static constexpr uint8_t QMC_REG_STATUS   = 0x06;
static constexpr uint8_t QMC_REG_CTRL1    = 0x09;
static constexpr uint8_t QMC_REG_CTRL2    = 0x0A;
static constexpr uint8_t QMC_REG_SET_RST  = 0x0B;

// ── Low-level I2C helpers ───────────────────────────────────────
esp_err_t MAZGY271::_writeReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(_dev, buf, sizeof(buf), 100);
}

esp_err_t MAZGY271::_readRegs(uint8_t reg, uint8_t* out, size_t len)
{
    return i2c_master_transmit_receive(_dev, &reg, 1, out, len, 100);
}

// ── Probe: try adding device + a test read ──────────────────────
esp_err_t MAZGY271::_tryProbe(i2c_master_bus_handle_t bus, uint8_t addr)
{
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr;
    dc.scl_speed_hz    = 50000;  // match LCD bus speed (internal pull-ups)

    esp_err_t ret = i2c_master_bus_add_device(bus, &dc, &_dev);
    if (ret != ESP_OK) return ret;

    // Try a dummy read — if the chip isn't there, this will NACK
    uint8_t dummy = 0;
    ret = _readRegs(0x00, &dummy, 1);
    if (ret != ESP_OK) {
        // Remove the device handle — chip not present at this address
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

// ── HMC5883L configuration ─────────────────────────────────────
esp_err_t MAZGY271::_initHMC()
{
    esp_err_t ret;

    // 8 samples averaged, 15 Hz output rate, normal measurement
    ret = _writeReg(HMC_REG_CONFIG_A, 0x70);
    if (ret != ESP_OK) return ret;

    // Gain = 1.3 Ga (±1.3 Ga range, 1090 LSB/Ga)
    ret = _writeReg(HMC_REG_CONFIG_B, 0x20);
    if (ret != ESP_OK) return ret;

    // Continuous measurement mode
    return _writeReg(HMC_REG_MODE, 0x00);
}

// ── QMC5883L configuration ──────────────────────────────────────
esp_err_t MAZGY271::_initQMC()
{
    esp_err_t ret;

    // Recommended SET/RESET period
    ret = _writeReg(QMC_REG_SET_RST, 0x01);
    if (ret != ESP_OK) return ret;

    // Soft reset first
    ret = _writeReg(QMC_REG_CTRL2, 0x80);
    if (ret != ESP_OK) return ret;

    // Wait for reset
    vTaskDelay(pdMS_TO_TICKS(10));

    // SET/RESET period again after reset
    ret = _writeReg(QMC_REG_SET_RST, 0x01);
    if (ret != ESP_OK) return ret;

    // Control register 1:
    //   Continuous mode (bits 1:0 = 01)
    //   200 Hz ODR      (bits 3:2 = 11)
    //   8G range        (bits 5:4 = 01)  — wider range, less overflow
    //   512 oversample  (bits 7:6 = 00)
    ret = _writeReg(QMC_REG_CTRL1, 0x1D);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

// ================================================================
//  init() — auto-detect
// ================================================================
esp_err_t MAZGY271::init(i2c_master_bus_handle_t bus)
{
    // Try HMC5883L first (0x1E)
    if (_tryProbe(bus, HMC5883L_ADDR) == ESP_OK) {
        esp_err_t ret = _initHMC();
        if (ret == ESP_OK) {
            _chip = CompassChip::HMC5883L;
            ESP_LOGI(TAG, "HMC5883L detected at 0x%02X — configured OK", HMC5883L_ADDR);
            return ESP_OK;
        }
        // Probe succeeded but config failed — remove and try QMC
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
    }

    // Try QMC5883L (0x0D)
    if (_tryProbe(bus, QMC5883L_ADDR) == ESP_OK) {
        esp_err_t ret = _initQMC();
        if (ret == ESP_OK) {
            _chip = CompassChip::QMC5883L;
            ESP_LOGI(TAG, "QMC5883L detected at 0x%02X — configured OK", QMC5883L_ADDR);
            return ESP_OK;
        }
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
    }

    ESP_LOGE(TAG, "No compass found at 0x%02X (HMC) or 0x%02X (QMC)",
             HMC5883L_ADDR, QMC5883L_ADDR);
    return ESP_ERR_NOT_FOUND;
}

// ── HMC5883L read ───────────────────────────────────────────────
esp_err_t MAZGY271::_readHMC(float& heading_deg)
{
    uint8_t buf[6] = {};
    // HMC5883L data order: X_H, X_L, Z_H, Z_L, Y_H, Y_L  (Z before Y!)
    esp_err_t ret = _readRegs(HMC_REG_DATA_X_H, buf, sizeof(buf));
    if (ret != ESP_OK) return ret;

    int16_t x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t y = (int16_t)((buf[4] << 8) | buf[5]);
    // z = buf[2..3] — not needed for 2D heading

    // HMC5883L outputs -4096 on overflow
    if (x == -4096 || y == -4096) return ESP_ERR_INVALID_RESPONSE;

    float heading = atan2f((float)y, (float)x) * (180.0f / (float)M_PI);
    if (heading < 0.0f) heading += 360.0f;
    heading_deg = heading;
    return ESP_OK;
}

// ── QMC5883L read ───────────────────────────────────────────────
esp_err_t MAZGY271::_readQMC(float& heading_deg)
{
    // Check data ready (bit 0 of status register)
    uint8_t status = 0;
    esp_err_t ret = _readRegs(QMC_REG_STATUS, &status, 1);
    if (ret != ESP_OK) return ret;

    if (!(status & 0x01)) return ESP_ERR_NOT_FINISHED;  // data not ready yet

    uint8_t buf[6] = {};
    // QMC5883L data order: X_L, X_H, Y_L, Y_H, Z_L, Z_H  (LSB first!)
    ret = _readRegs(QMC_REG_DATA_X_L, buf, sizeof(buf));
    if (ret != ESP_OK) return ret;

    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    // z = buf[4..5] — not needed for 2D heading

    // QMC5883L outputs -32768 on overflow
    if (x == -32768 || y == -32768) return ESP_ERR_INVALID_RESPONSE;

    float heading = atan2f((float)y, (float)x) * (180.0f / (float)M_PI);
    if (heading < 0.0f) heading += 360.0f;
    heading_deg = heading;
    return ESP_OK;
}

// ================================================================
//  read() — dispatches to correct chip handler
// ================================================================
esp_err_t MAZGY271::read(float& heading_deg)
{
    switch (_chip) {
        case CompassChip::HMC5883L: return _readHMC(heading_deg);
        case CompassChip::QMC5883L: return _readQMC(heading_deg);
        default:                    return ESP_ERR_INVALID_STATE;
    }
}
