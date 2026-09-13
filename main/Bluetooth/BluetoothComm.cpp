// ================================================================
//  BluetoothComm.cpp
//  RoboMAZ - BLE Communication Module (NimBLE / Nordic UART Service)
//  ESP32-S3 / ESP-IDF  |  C++20
//
//  This file belongs in:  Bluetooth/BluetoothComm.cpp
//
//  Implements a BLE GATT server using NimBLE with the Nordic UART
//  Service (NUS).  The line-based CMD:/TEL: protocol is identical
//  to the RFCOMM spec — only the transport layer differs.
//
//  Thread model:
//    _onReceive() runs on the NimBLE host task — it buffers bytes
//    and enqueues complete lines into a FreeRTOS queue.
//    update() runs on the main-loop task — it dequeues lines and
//    dispatches commands, so motor/LCD/LED calls never run on the
//    NimBLE stack.
// ================================================================

#include "BluetoothComm.hpp"

extern "C" {
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
}

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <algorithm>

static const char* TAG = "BT";

// ── Nordic UART Service UUIDs ────────────────────────────────────
// Service : 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// RX Char : 6E400002-...  (PcPanel writes here → Robot receives)
// TX Char : 6E400003-...  (Robot notifies here → PcPanel receives)
static const ble_uuid128_t NUS_SVC_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5,
                     0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5,
                     0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t NUS_RX_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5,
                     0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5,
                     0x02, 0x00, 0x40, 0x6e);

static const ble_uuid128_t NUS_TX_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5,
                     0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5,
                     0x03, 0x00, 0x40, 0x6e);

// ── Singleton ────────────────────────────────────────────────────
BluetoothComm& btCommInstance()
{
    static BluetoothComm instance;
    return instance;
}

// ── Motor state string table ─────────────────────────────────────
const char* motorStateStr(MotorState s)
{
    switch (s) {
        case MotorState::IDLE: return "IDLE";
        case MotorState::FWD:  return "FWD";
        case MotorState::BWD:  return "BWD";
        case MotorState::SL:   return "SL";
        case MotorState::SR:   return "SR";
        case MotorState::RCW:  return "RCW";
        case MotorState::RCCW: return "RCCW";
        case MotorState::DFL:  return "DFL";
        case MotorState::DFR:  return "DFR";
        case MotorState::DRL:  return "DRL";
        case MotorState::DRR:  return "DRR";
        case MotorState::BRK:  return "BRK";
        case MotorState::CST:  return "CST";
        default:               return "IDLE";
    }
}

// ================================================================
//  NimBLE C callbacks (forward to singleton)
// ================================================================

// ── GATT access callback for RX characteristic ───────────────────
static int nus_rx_access_cb(uint16_t conn_handle,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt* ctxt,
                            void* arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len > 0 && om_len <= BT_LINE_MAX) {
            uint8_t buf[BT_LINE_MAX];
            uint16_t copied = 0;
            ble_hs_mbuf_to_flat(ctxt->om, buf, om_len, &copied);
            btCommInstance()._onReceive(buf, copied);
        }
    }
    return 0;
}

// ── GATT access callback for TX characteristic (read, unused) ────
static int nus_tx_access_cb(uint16_t conn_handle,
                            uint16_t attr_handle,
                            struct ble_gatt_access_ctxt* ctxt,
                            void* arg)
{
    // TX is notify-only; reads return empty
    return 0;
}

// ── GATT service definition ──────────────────────────────────────
static uint16_t s_tx_attr_handle;  // filled by NimBLE during registration

// ble_gatt_chr_def field order:
//   uuid, access_cb, arg, descriptors, flags, min_key_size, val_handle, cpfd
// ble_gatt_svc_def field order:
//   type, uuid, includes, characteristics

static const struct ble_gatt_chr_def nus_chars[] = {
    // RX: PcPanel writes → Robot receives
    { &NUS_RX_UUID.u, nus_rx_access_cb, nullptr, nullptr,
      BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
      0, nullptr, nullptr },
    // TX: Robot notifies → PcPanel receives
    { &NUS_TX_UUID.u, nus_tx_access_cb, nullptr, nullptr,
      BLE_GATT_CHR_F_NOTIFY,
      0, &s_tx_attr_handle, nullptr },
    // sentinel
    { nullptr, nullptr, nullptr, nullptr, 0, 0, nullptr, nullptr },
};

static const struct ble_gatt_svc_def gatt_svcs[] = {
    { BLE_GATT_SVC_TYPE_PRIMARY, &NUS_SVC_UUID.u, nullptr, nus_chars },
    { 0, nullptr, nullptr, nullptr },  // sentinel
};

// ── GAP event handler ────────────────────────────────────────────
static int ble_gap_event_cb(struct ble_gap_event* event, void* arg)
{
    auto& bt = btCommInstance();

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "BLE connected (handle %d)", event->connect.conn_handle);
                bt._onConnect(event->connect.conn_handle);
            } else {
                ESP_LOGW(TAG, "BLE connection failed, status=%d", event->connect.status);
                bt._startAdvertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE disconnected (reason=0x%02x)",
                     event->disconnect.reason);
            bt._onDisconnect();
            bt._startAdvertising();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            bt._startAdvertising();
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU updated: conn=%d, mtu=%d",
                     event->mtu.conn_handle, event->mtu.value);
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event: attr=%d, cur_notify=%d",
                     event->subscribe.attr_handle,
                     event->subscribe.cur_notify);
            break;

        default:
            break;
    }
    return 0;
}

// ── NimBLE host task ─────────────────────────────────────────────
static void nimble_host_task(void* param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// ── on_sync callback — called when NimBLE stack is ready ─────────
static void ble_on_sync(void)
{
    int rc;
    uint8_t addr_type;

    // Infer the best address type — second param MUST be a valid pointer
    rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    btCommInstance()._startAdvertising();
    ESP_LOGI(TAG, "NimBLE stack synced — advertising started (addr_type=%d)", addr_type);
}

// ── on_reset callback ────────────────────────────────────────────
static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset, reason=%d", reason);
}

// ================================================================
//  BluetoothComm — public methods
// ================================================================

void BluetoothComm::begin(const char* deviceName)
{
    if (_initialized) return;
    _initialized = true;

    ESP_LOGI(TAG, "Initialising BLE as \"%s\"", deviceName);

    // Create the line queue (NimBLE task → main loop)
    _lineQueue = xQueueCreate(BT_LINE_QUEUE_DEPTH, sizeof(LineItem));
    if (!_lineQueue) {
        ESP_LOGE(TAG, "Failed to create line queue");
        return;
    }

    // ── Init NimBLE ──────────────────────────────────────────────
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init() failed: %s", esp_err_to_name(ret));
        return;
    }

    ble_hs_cfg.sync_cb  = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    ble_svc_gap_device_name_set(deviceName);

    // ── Register GATT services ───────────────────────────────────
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    // ── Start NimBLE host task ───────────────────────────────────
    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "BLE initialised — waiting for connections");
}

bool BluetoothComm::isConnected() const
{
    return _connected.load(std::memory_order_relaxed);
}

void BluetoothComm::onCommand(CommandCallback cb)
{
    _cmdCallback = cb;
}

void BluetoothComm::onDisconnect(DisconnectCallback cb)
{
    _disconnectCallback = cb;
}

// ================================================================
//  update() — call from main loop
// ================================================================
/**
 * Dequeues complete lines posted by the NimBLE task and dispatches
 * commands.  All motor/LCD/LED calls happen here, on the main-loop
 * task — never on the NimBLE stack.
 */
void BluetoothComm::update()
{
    // Handle pending disconnect (raised by NimBLE task)
    if (_disconnectPending.exchange(false, std::memory_order_acq_rel)) {
        if (_disconnectCallback) {
            _disconnectCallback();
        }
    }

    // Drain the line queue
    LineItem item;
    while (xQueueReceive(_lineQueue, &item, 0) == pdTRUE) {
        _processLine(item.line, item.len);
    }
}

// ================================================================
//  Sending
// ================================================================

void BluetoothComm::sendLine(const char* fmt, ...)
{
    if (!_connected.load(std::memory_order_relaxed)) return;

    char buf[BT_LINE_MAX];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
    va_end(args);

    if (len <= 0) return;

    // Append LF
    buf[len] = '\n';
    len++;
    buf[len] = '\0';

    struct os_mbuf* om = ble_hs_mbuf_from_flat(buf, len);
    if (om) {
        int rc = ble_gatts_notify_custom(_connHandle, s_tx_attr_handle, om);
        if (rc != 0) {
            ESP_LOGW(TAG, "Notify failed: %d", rc);
        }
    }
}

void BluetoothComm::sendAck(const char* cmd)
{
    sendLine("TEL:ACK:%s", cmd);
}

void BluetoothComm::sendError(const char* msg)
{
    sendLine("TEL:ERR:%s", msg);
}

void BluetoothComm::sendPong()
{
    sendLine("TEL:PONG");
}

// ================================================================
//  Connection event handlers (called from NimBLE task)
// ================================================================

void BluetoothComm::_onConnect(uint16_t connHandle)
{
    _connHandle   = connHandle;
    _txAttrHandle = s_tx_attr_handle;
    _rxLen = 0;
    _connected.store(true, std::memory_order_release);

    ESP_LOGI(TAG, "Client connected — sending PONG handshake");

    // Small delay to let the central enable notifications
    vTaskDelay(pdMS_TO_TICKS(200));
    sendPong();
}

void BluetoothComm::_onDisconnect()
{
    _connected.store(false, std::memory_order_release);
    _connHandle = 0;
    _rxLen = 0;

    // Signal main loop to run the disconnect callback
    _disconnectPending.store(true, std::memory_order_release);

    ESP_LOGW(TAG, "Client disconnected — safety stop pending");
}

void BluetoothComm::_onReceive(const uint8_t* data, size_t len)
{
    // Buffer incoming bytes.  When a complete line is found,
    // enqueue it for the main-loop task to dispatch.
    for (size_t i = 0; i < len; ++i) {
        char c = static_cast<char>(data[i]);

        if (c == '\n' || c == '\r') {
            if (_rxLen > 0) {
                _rxBuf[_rxLen] = '\0';

                // Enqueue line for main-loop dispatch
                LineItem item;
                memcpy(item.line, _rxBuf, _rxLen + 1);
                item.len = _rxLen;
                if (xQueueSend(_lineQueue, &item, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Line queue full — dropped: %s", _rxBuf);
                }
                _rxLen = 0;
            }
            continue;
        }

        if (_rxLen < BT_LINE_MAX) {
            _rxBuf[_rxLen++] = c;
        } else {
            _rxLen = 0;
            // Can't call sendError here (on NimBLE task) safely,
            // so enqueue a synthetic error line for the main loop
            LineItem item;
            strncpy(item.line, "__ERR_LINE_TOO_LONG", sizeof(item.line));
            item.len = strlen(item.line);
            xQueueSend(_lineQueue, &item, 0);
        }
    }
}

// ================================================================
//  Protocol parser (runs on main-loop task)
// ================================================================

void BluetoothComm::_processLine(const char* line, size_t len)
{
    // Handle synthetic error from _onReceive
    if (strcmp(line, "__ERR_LINE_TOO_LONG") == 0) {
        sendError("LINE_TOO_LONG");
        return;
    }

    ESP_LOGI(TAG, "RX: %s", line);

    ParsedCommand cmd = {};
    if (!_parseLine(line, cmd)) {
        char errMsg[64];
        snprintf(errMsg, sizeof(errMsg), "BAD_FORMAT:%.40s", line);
        sendError(errMsg);
        return;
    }

    if (_cmdCallback) {
        bool handled = _cmdCallback(cmd);
        if (!handled) {
            char errMsg[64];
            snprintf(errMsg, sizeof(errMsg), "UNKNOWN_CMD:%s", cmd.cmd);
            sendError(errMsg);
        }
    }
}

bool BluetoothComm::_parseLine(const char* line, ParsedCommand& out)
{
    // Copy raw line
    strncpy(out.raw, line, BT_LINE_MAX - 1);
    out.raw[BT_LINE_MAX - 1] = '\0';
    out.nParams = 0;

    // Work on a mutable copy
    char temp[BT_LINE_MAX];
    strncpy(temp, line, BT_LINE_MAX - 1);
    temp[BT_LINE_MAX - 1] = '\0';

    // First token: type (must be "CMD")
    char* saveptr = nullptr;
    char* tok = strtok_r(temp, ":", &saveptr);
    if (!tok) return false;

    strncpy(out.type, tok, sizeof(out.type) - 1);
    out.type[sizeof(out.type) - 1] = '\0';

    if (strcmp(out.type, "CMD") != 0) return false;

    // Second token: command name
    tok = strtok_r(nullptr, ":", &saveptr);
    if (!tok) return false;

    strncpy(out.cmd, tok, sizeof(out.cmd) - 1);
    out.cmd[sizeof(out.cmd) - 1] = '\0';

    // Parameters: first 3 are colon-delimited, but the 4th (last)
    // captures the REMAINDER so embedded colons are preserved.
    // This fixes CMD:LCD:0:Hello:World → params[1] = "Hello:World"
    for (int i = 0; i < 4; ++i) {
        if (i < 3) {
            // Normal colon-delimited token
            tok = strtok_r(nullptr, ":", &saveptr);
        } else {
            // Last param: grab everything remaining
            tok = saveptr;
            if (tok && *tok == '\0') tok = nullptr;
        }
        if (!tok) break;
        strncpy(out.params[i], tok, sizeof(out.params[i]) - 1);
        out.params[i][sizeof(out.params[i]) - 1] = '\0';
        out.nParams++;
    }

    return true;
}

// ================================================================
//  Advertising
// ================================================================

void BluetoothComm::_startAdvertising()
{
    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char* name = ble_svc_gap_device_name();
    fields.name             = (uint8_t*)name;
    fields.name_len         = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    // Scan response: include the 128-bit service UUID
    struct ble_hs_adv_fields rsp_fields = {};
    rsp_fields.uuids128             = (ble_uuid128_t*)&NUS_SVC_UUID;
    rsp_fields.num_uuids128         = 1;
    rsp_fields.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp_fields);

    int rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,
        nullptr,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event_cb,
        nullptr
    );

    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    }
}
