// ================================================================
//  MAZLCD.hpp
//  RoboMAZ - HD44780 LCD driver over I2C (PCF8574 expander)
//  ESP32-S3 / ESP-IDF v6  |  C++20
//
//  Wiring (PCF8574 standard bit mapping):
//    P0 → RS   P1 → RW   P2 → EN   P3 → BL
//    P4 → D4   P5 → D5   P6 → D6   P7 → D7
//
//  Common I2C addresses: 0x27 (default) or 0x3F
// ================================================================
#pragma once

#include <cstdint>
#include "driver/i2c_master.h"

// ── Default I2C address ──────────────────────────────────────────
// Change to 0x3F if the display does not respond at 0x27.
static constexpr uint8_t LCD_I2C_ADDR = 0x27;

// ================================================================
//  MAZLCD
// ================================================================
class MAZLCD {
public:
    /**
     * @brief Initialises I2C bus and runs the HD44780 power-on sequence.
     *        Must be called once from app_main before any other method.
     *
     * @param sda_gpio  SDA pin (GPIO05 on your board)
     * @param scl_gpio  SCL pin (GPIO06 on your board)
     * @param addr      PCF8574 I2C address (default 0x27)
     */
    esp_err_t init(gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr = LCD_I2C_ADDR);

    /** @brief Clears the display and moves cursor to (0,0). */
    void clear();

    /**
     * @brief Positions the cursor.
     * @param col  Column 0–15
     * @param row  Row 0–1
     */
    void setCursor(uint8_t col, uint8_t row);

    /** @brief Writes a null-terminated string at the current cursor position. */
    void print(const char* str);

    /** @brief Writes an integer at the current cursor position. */
    void printNum(int num);

    /** @brief Returns the I2C bus handle so other devices can join the same bus. */
    i2c_master_bus_handle_t busHandle() const { return _bus; }

private:
    i2c_master_bus_handle_t _bus = nullptr;
    i2c_master_dev_handle_t _dev = nullptr;
    uint8_t _bl = 0x08;  // backlight bit (PCF8574 P3)

    void _expanderWrite(uint8_t data);
    void _pulseEnable(uint8_t data);
    void _write4bits(uint8_t nibble);
    void _send(uint8_t byte, uint8_t flags);
};
