// ================================================================
//  MAZLCD.cpp
//  RoboMAZ - HD44780 LCD driver over I2C (PCF8574 expander)
//  ESP32-S3 / ESP-IDF v6  |  C++20
// ================================================================
#include "MAZLCD.hpp"

#include "esp_err.h"
#include "esp_rom_sys.h"  // esp_rom_delay_us()
#include <cstdio>

// PCF8574 control bits
#define RS_BIT  0x01   // P0 — Register Select (0=cmd, 1=data)
#define EN_BIT  0x04   // P2 — Enable pulse

// ── _expanderWrite() ────────────────────────────────────────────
// Sends one byte to the PCF8574 via I2C.
// The backlight bit is OR'd in on every write so the BL state is
// preserved regardless of what data is being sent.
void MAZLCD::_expanderWrite(uint8_t data)
{
    uint8_t b = data | _bl;
    i2c_master_transmit(_dev, &b, 1, 100); // 100 ms timeout
}

// ── _pulseEnable() ──────────────────────────────────────────────
// Sets data lines first (EN=0), then strobes EN: high >450 ns, low,
// hold >37 µs. The setup write ensures data is stable before EN rises.
void MAZLCD::_pulseEnable(uint8_t data)
{
    _expanderWrite(data & ~EN_BIT);  // data stable, EN low
    _expanderWrite(data | EN_BIT);   // EN high — LCD latches on falling edge
    esp_rom_delay_us(1);
    _expanderWrite(data & ~EN_BIT);  // EN low — latch happens here
    esp_rom_delay_us(50);            // wait >37 µs for command to execute
}

// ── _write4bits() ───────────────────────────────────────────────
// Used only during the 8→4-bit init handshake.
// Sends a single nibble on D4–D7 (bits 4–7 of the PCF8574 byte).
void MAZLCD::_write4bits(uint8_t nibble)
{
    _pulseEnable((nibble << 4) & 0xF0);
}

// ── _send() ─────────────────────────────────────────────────────
// Sends one full byte to the HD44780 in 4-bit mode:
//   high nibble first, then low nibble.
// flags: RS_BIT for data, 0 for command.
void MAZLCD::_send(uint8_t byte, uint8_t flags)
{
    _pulseEnable((byte & 0xF0) | flags);
    _pulseEnable(((byte << 4) & 0xF0) | flags);
}

// ================================================================
//  init()
// ================================================================
esp_err_t MAZLCD::init(gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t addr)
{
    // ── I2C master bus ───────────────────────────────────────────
    i2c_master_bus_config_t bc = {};
    bc.i2c_port                    = I2C_NUM_0;
    bc.sda_io_num                  = sda_gpio;
    bc.scl_io_num                  = scl_gpio;
    bc.clk_source                  = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt           = 7;
    bc.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &_bus));

    // ── PCF8574 device ───────────────────────────────────────────
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr;
    dc.scl_speed_hz    = 50000;   // 50 kHz — internal pull-ups (~45kΩ) limit rise time
    ESP_ERROR_CHECK(i2c_master_bus_add_device(_bus, &dc, &_dev));

    // ── HD44780 power-on initialisation sequence ─────────────────
    // PCF8574 powers up with all pins HIGH (open-drain + pull-ups).
    // Drive everything LOW (keep backlight) before the LCD is ready
    // so no accidental EN pulse reaches the HD44780 during its reset.
    _expanderWrite(0x08);            // BL=1, EN=0, RS=0, RW=0, D4-D7=0
    esp_rom_delay_us(100000);        // wait >40 ms — use 100 ms for slow clone modules

    // Send 0x3 three times to ensure 8-bit mode regardless of
    // prior state, with required inter-command delays.
    _write4bits(0x03); esp_rom_delay_us(4500);
    _write4bits(0x03); esp_rom_delay_us(4500);
    _write4bits(0x03); esp_rom_delay_us(150);

    // Switch to 4-bit interface
    _write4bits(0x02);
    esp_rom_delay_us(5000);          // slow clone modules need more settling time here

    // Configure in 4-bit mode
    _send(0x28, 0);             // function set: 4-bit, 2 lines, 5×8 dots
    _send(0x0C, 0);             // display on, cursor off, blink off
    _send(0x01, 0);             // clear display
    esp_rom_delay_us(5000);     // clear needs >1.52 ms — use 5 ms for slow clone modules
    _send(0x06, 0);             // entry mode: cursor increments, no display shift

    return ESP_OK;
}

// ================================================================
//  Public API
// ================================================================
void MAZLCD::clear()
{
    _send(0x01, 0);
    esp_rom_delay_us(5000);  // slow clone modules need >1.52 ms — use 5 ms
}

void MAZLCD::setCursor(uint8_t col, uint8_t row)
{
    static constexpr uint8_t row_addr[] = {0x00, 0x40};
    _send(0x80 | (col + row_addr[row & 0x01]), 0);
}

void MAZLCD::print(const char* str)
{
    while (*str)
        _send(static_cast<uint8_t>(*str++), RS_BIT);
}

void MAZLCD::printNum(int num)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", num);
    print(buf);
}
