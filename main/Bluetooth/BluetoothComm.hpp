// ================================================================
//  BluetoothComm.hpp
//  RoboMAZ - BLE Communication Module (NimBLE / Nordic UART Service)
//  ESP32-S3 / ESP-IDF  |  C++20
//
//  This file belongs in:  Bluetooth/BluetoothComm.hpp
//
//  NOTE: ESP32-S3 does NOT support Bluetooth Classic (BR/EDR).
//        This module uses BLE with the Nordic UART Service (NUS)
//        as a serial-port replacement.  PcPanel connects via BLE
//        and exchanges the same CMD:/TEL: text protocol.
//
//  Nordic UART Service UUIDs:
//    Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
//    RX Char : 6E400002-...  (write — PcPanel → Robot)
//    TX Char : 6E400003-...  (notify — Robot → PcPanel)
// ================================================================
#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <atomic>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
}

// ── Line buffer sizing ───────────────────────────────────────────
static constexpr size_t BT_LINE_MAX = 128;

// ── Max queued lines (NimBLE task → main loop) ───────────────────
static constexpr size_t BT_LINE_QUEUE_DEPTH = 8;

// ── Parsed command ───────────────────────────────────────────────
/**
 * @brief Holds a parsed protocol command ready for dispatch.
 *
 * Fields:
 *   type   — "CMD" (always, for received commands)
 *   cmd    — e.g. "FWD", "BRK", "LED", "TEL", "PING", "INFO"
 *   params — up to 4 colon-separated parameter strings
 *   nParams— number of populated params
 *   raw    — the original line (for logging / error messages)
 */
struct ParsedCommand {
    char type[8];         // "CMD"
    char cmd[16];         // "FWD", "BRK", "LED", …
    char params[4][32];   // up to 4 parameters
    uint8_t nParams;      // number of populated params
    char raw[BT_LINE_MAX];// original line for error reporting
};

// ── Command callback signature ───────────────────────────────────
/**
 * @brief Called by BluetoothComm::update() for every valid parsed command.
 *        The callback receives a const reference to the parsed command.
 *        Return true if the command was handled, false for unknown commands.
 */
using CommandCallback = std::function<bool(const ParsedCommand&)>;

// ── Motor state names (for TEL:MOTOR) ────────────────────────────
/**
 * @brief Robot motor state reported in telemetry.
 *        Set by the command handler in main.cpp.
 */
enum class MotorState : uint8_t {
    IDLE, FWD, BWD, SL, SR, RCW, RCCW,
    DFL, DFR, DRL, DRR, BRK, CST
};

const char* motorStateStr(MotorState s);

// ── Queued line item ─────────────────────────────────────────────
struct LineItem {
    char line[BT_LINE_MAX + 1];
    size_t len;
};

// ================================================================
//  BluetoothComm
// ================================================================
/**
 * @brief BLE serial communication module for the RoboMAZ robot.
 *
 * Uses NimBLE's Nordic UART Service to provide a text-based serial
 * channel over BLE.  The protocol is line-based (LF terminated),
 * transport-agnostic, and identical to the one specified for RFCOMM.
 *
 * Thread safety:
 *   - _onReceive() runs on the NimBLE host task; it only buffers
 *     complete lines into a FreeRTOS queue.
 *   - update() runs on the main loop task; it dequeues and dispatches
 *     commands, so all motor/LCD/LED access stays on one task.
 *   - _connected is std::atomic<bool> for safe cross-task reads.
 *
 * Usage:
 * @code
 *   BluetoothComm bt;
 *   bt.begin("RoboMAZ-Explorer");
 *   bt.onCommand([](const ParsedCommand& cmd) -> bool {
 *       // dispatch command
 *       return true;
 *   });
 *   // in loop:
 *   bt.update();
 * @endcode
 */
class BluetoothComm {
public:
    // ── Lifecycle ────────────────────────────────────────────────

    /**
     * @brief Initialises the NimBLE stack, configures the GATT server
     *        with Nordic UART Service, and begins advertising.
     *
     * @param deviceName  The BLE device name visible during scanning
     *                    (default: "RoboMAZ-Explorer")
     */
    void begin(const char* deviceName = "RoboMAZ-Explorer");

    /**
     * @brief Returns true when a BLE central (PcPanel) is connected.
     *        Thread-safe (atomic).
     */
    bool isConnected() const;

    /**
     * @brief Call from the main loop.  Dequeues complete lines received
     *        by the NimBLE task, parses them, and dispatches commands
     *        via the registered callback — all on the main-loop task.
     */
    void update();

    // ── Callback registration ────────────────────────────────────

    /**
     * @brief Registers the command callback.
     * @param cb  Function called for each parsed CMD: message
     */
    void onCommand(CommandCallback cb);

    // ── Sending (Robot → PcPanel) ────────────────────────────────

    /**
     * @brief Sends a formatted line over BLE (appends \n).
     *        Safe to call when disconnected — silently drops.
     *
     * @param fmt  printf-style format string
     */
    void sendLine(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

    /**
     * @brief Sends TEL:ACK:<cmd>
     */
    void sendAck(const char* cmd);

    /**
     * @brief Sends TEL:ERR:<msg>
     */
    void sendError(const char* msg);

    /**
     * @brief Sends TEL:PONG
     */
    void sendPong();

    // ── Connection event hooks (called from NimBLE GAP callbacks) ─

    /// @brief Called internally on BLE connect.  Do not call directly.
    void _onConnect(uint16_t connHandle);

    /// @brief Called internally on BLE disconnect.  Do not call directly.
    void _onDisconnect();

    /// @brief Called internally to (re)start advertising.  Do not call directly.
    void _startAdvertising();

    /// @brief Called internally when RX characteristic is written.
    void _onReceive(const uint8_t* data, size_t len);

    // ── Disconnect callback for main.cpp ─────────────────────────
    using DisconnectCallback = std::function<void()>;
    void onDisconnect(DisconnectCallback cb);

private:
    // ── State ────────────────────────────────────────────────────
    bool              _initialized = false;
    std::atomic<bool> _connected{false};       // thread-safe
    uint16_t          _connHandle  = 0;
    uint16_t          _txAttrHandle = 0;

    // ── Line buffer (NimBLE task context) ────────────────────────
    char   _rxBuf[BT_LINE_MAX + 1] = {};
    size_t _rxLen = 0;

    // ── Line queue (NimBLE task → main loop) ─────────────────────
    QueueHandle_t _lineQueue = nullptr;

    // ── Disconnect flag (NimBLE task → main loop) ────────────────
    std::atomic<bool> _disconnectPending{false};

    // ── Callbacks ────────────────────────────────────────────────
    CommandCallback    _cmdCallback;
    DisconnectCallback _disconnectCallback;

    // ── Internal helpers ─────────────────────────────────────────
    void _processLine(const char* line, size_t len);
    bool _parseLine(const char* line, ParsedCommand& out);
};

// ── Singleton accessor ───────────────────────────────────────────
/**
 * @brief Returns the single BluetoothComm instance.
 *        Used by NimBLE C callbacks to reach the C++ object.
 */
BluetoothComm& btCommInstance();
