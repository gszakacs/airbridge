#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "wifi_setup.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "app_config.h"
#include "crc.h"

#include <NimBLEDevice.h>
#include <Preferences.h>
#include "nvs_optional.h"

#define OXI_TASK_STACK      4096
#define OXI_TASK_PRIO       4
#define SCAN_DURATION_MS    10000
#define RECONNECT_DELAY_MS  3000

static const NimBLEUUID PLX_SERVICE_UUID((uint16_t)0x1822);
static const NimBLEUUID PLX_CONTINUOUS_UUID((uint16_t)0x2A5F);
static const NimBLEUUID PLX_SPOT_UUID((uint16_t)0x2A5E);
static const NimBLEUUID HR_SERVICE_UUID((uint16_t)0x180D);
static const NimBLEUUID HR_MEASUREMENT_UUID((uint16_t)0x2A37);

// Nonin proprietary
static const NimBLEUUID NONIN_OXI_SERVICE_UUID("46A970E0-0D5F-11E2-8B5E-0002A5D5C51B");
static const NimBLEUUID NONIN_CONTINUOUS_UUID("0AAD7EA0-0D60-11E2-8E3C-0002A5D5C51B");
static const NimBLEUUID NONIN_CONTROL_POINT_UUID("1447AF80-0D60-11E2-88B6-0002A5D5C51B");

// Wellue/Viatom proprietary (O2Ring, CheckmeO2, SleepU, O2M)
static const NimBLEUUID VIATOM_SERVICE_UUID("14839AC4-7D7E-415C-9A42-167340CF2339");
static const NimBLEUUID VIATOM_READ_UUID("0734594A-A8E7-4B1A-A6B1-CD5243059A57");
static const NimBLEUUID VIATOM_WRITE_UUID("8B00ACE7-EB0B-49B0-BBE9-9AEE0A26E1A3");

// OxyII proprietary (O2Ring-S and related devices)
static const NimBLEUUID OXYII_SERVICE_UUID("E8FB0001-A14B-98F9-831B-4E2941D01248");
static const NimBLEUUID OXYII_WRITE_UUID("E8FB0002-A14B-98F9-831B-4E2941D01248");
static const NimBLEUUID OXYII_NOTIFY_UUID("E8FB0003-A14B-98F9-831B-4E2941D01248");

// ACCARE WS20A proprietary
static const NimBLEUUID WS20A_NOTIFY_SERVICE_UUID((uint16_t)0xFFE0);
static const NimBLEUUID WS20A_WRITE_SERVICE_UUID((uint16_t)0xFFE5);
static const NimBLEUUID WS20A_NOTIFY_UUID((uint16_t)0xFFE4);
static const NimBLEUUID WS20A_WRITE_UUID((uint16_t)0xFFE9);

static TaskHandle_t oxi_task_handle = nullptr;
static volatile oxi_state_t state = OXI_DISABLED;
static volatile bool state_dirty = false;
static inline void set_state(oxi_state_t s) { state = s; state_dirty = true; }
static oxi_reading_t reading = { -1, -1, false, 0 };  // local copy for callbacks
static volatile bool scan_requested = false;
static volatile bool active_scan_requested = false;
typedef enum { CONN_NONE, CONN_AUTO, CONN_USER } connect_mode_t;
static volatile connect_mode_t connect_mode = CONN_NONE;
static volatile bool disconnect_requested = false;  // drop connection, stay enabled
static volatile bool disable_requested = false;     // drop connection, disable scanning
static volatile bool del_one_requested = false;
static volatile bool del_all_requested = false;
static char del_one_addr[18] = "";
static volatile bool ble_suspended = false;
static volatile bool suspend_enter_requested = false;  // handle entry side-effects in task

#define USER_CONNECT_RETRIES  3
#define USER_RETRY_DELAY_MS   2000
static volatile bool scan_complete = false;
static char target_addr[18] = "";

static NimBLEClient *pClient = nullptr;

static SemaphoreHandle_t scan_mutex = nullptr;
static oxi_scan_result_t scan_results[MAX_SCAN_RESULTS];
static int scan_result_count = 0;
static bool device_needs_encryption = false;  // Nonin needs it, Viatom/O2Ring don't

// Known devices list
// For devices that don't use BLE bonding (O2Ring, CheckMe, etc.)
#define KNOWN_MAX CONFIG_BT_NIMBLE_MAX_BONDS
static char known_addrs[KNOWN_MAX][18] = {};
static int known_count = 0;

static void known_load() {
    Preferences p;
    if (!open_optional_preferences(p, "oxi_known")) return;
    known_count = p.getUChar("count", 0);
    if (known_count > KNOWN_MAX) known_count = KNOWN_MAX;
    for (int i = 0; i < known_count; i++) {
        char key[4];
        snprintf(key, sizeof(key), "a%d", i);
        String a = p.getString(key, "");
        strncpy(known_addrs[i], a.c_str(), 17);
        known_addrs[i][17] = '\0';
    }
    p.end();
    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Loaded %d known devices\n", known_count);
}

static void known_save() {
    Preferences p;
    p.begin("oxi_known", false);
    p.putUChar("count", known_count);
    for (int i = 0; i < KNOWN_MAX; i++) {
        char key[4];
        snprintf(key, sizeof(key), "a%d", i);
        if (i < known_count) p.putString(key, known_addrs[i]);
        else p.remove(key);
    }
    p.end();
}

static bool known_contains(const char *addr) {
    for (int i = 0; i < known_count; i++) {
        if (strcasecmp(known_addrs[i], addr) == 0) return true;
    }
    return false;
}

static bool known_add(const char *addr) {
    if (known_contains(addr)) return true;
    if (known_count >= KNOWN_MAX) return false;
    strncpy(known_addrs[known_count], addr, 17);
    known_addrs[known_count][17] = '\0';
    known_count++;
    known_save();
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Added known device: %s\n", addr);
    return true;
}

static bool known_remove(const char *addr) {
    for (int i = 0; i < known_count; i++) {
        if (strcasecmp(known_addrs[i], addr) == 0) {
            // Shift remaining entries
            for (int j = i; j < known_count - 1; j++)
                memcpy(known_addrs[j], known_addrs[j+1], 18);
            known_count--;
            known_save();
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Removed known device: %s\n", addr);
            return true;
        }
    }
    return false;
}

static void known_clear() {
    known_count = 0;
    known_save();
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Cleared all known devices\n");
}

// Check if a device is "known" (either NimBLE-bonded or in our known list)
static bool is_device_known(const char *addr, uint8_t addr_type) {
    NimBLEAddress ba(std::string(addr), addr_type);
    if (NimBLEDevice::isBonded(ba)) return true;
    return known_contains(addr);
}


static void plx_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len < 5) return;
    uint16_t spo2_raw = data[1] | (data[2] << 8);
    uint16_t pr_raw = data[3] | (data[4] << 8);
    int16_t spo2 = parse_sfloat(spo2_raw);
    int16_t pr = parse_sfloat(pr_raw);
    if (spo2 > 0 && spo2 <= 100 && pr > 0 && pr < 500) {
        OxiArbiter::feed(OXI_SRC_BLE, spo2, pr, true);
    } else {
        OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
    }
}

static void hr_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len < 2) return;
    uint8_t flags = data[0];
    uint16_t hr;
    if (flags & 0x01) {
        if (len < 3) return;
        hr = data[1] | (data[2] << 8);
    } else {
        hr = data[1];
    }
    // HR-only service - feed with current SpO2 from arbiter reading
    const oxi_reading_t &r = OxiArbiter::get_reading();
    if (hr > 0 && hr < 500) {
        OxiArbiter::feed(OXI_SRC_BLE, r.spo2, (int16_t)hr, r.valid);
    }
}

static void nonin_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len >= 5) {
        uint8_t spo2 = data[2];
        uint16_t pr = data[3] | (data[4] << 8);
        if (spo2 > 0 && spo2 <= 100 && pr > 0 && pr < 500) {
            OxiArbiter::feed(OXI_SRC_BLE, (int8_t)spo2, (int16_t)pr, true);
        } else {
            OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
        }
    }
}


// ACCARE WS20A: FE 5A LEN CMD PAYLOAD... SUM
// SUM is the low byte of the sum over LEN, CMD, and payload.
static NimBLERemoteCharacteristic *ws20a_write_chr = nullptr;
#define WS20A_RX_BUF_LEN 256
static uint8_t ws20a_rx_buf[WS20A_RX_BUF_LEN];
static size_t ws20a_rx_len = 0;
static uint32_t ws20a_frame_errors = 0;

static bool ws20a_send_command(uint8_t cmd, const uint8_t *payload, size_t payload_len) {
    uint8_t frame[32];
    if (!ws20a_write_chr || payload_len > sizeof(frame) - 5) return false;

    size_t frame_len = payload_len + 5;
    frame[0] = 0xFE;
    frame[1] = 0x5A;
    frame[2] = (uint8_t)frame_len;
    frame[3] = cmd;
    if (payload_len > 0) memcpy(frame + 4, payload, payload_len);

    uint8_t sum = 0;
    for (size_t i = 2; i < frame_len - 1; i++) sum += frame[i];
    frame[frame_len - 1] = sum;

    bool response = ws20a_write_chr->canWrite();
    if (!response && !ws20a_write_chr->canWriteNoResponse()) return false;
    if (!ws20a_write_chr->writeValue(frame, frame_len, response)) {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] WS20A command 0x%02X write failed\n", cmd);
        return false;
    }

    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] WS20A command 0x%02X sent\n", cmd);
    return true;
}

static void ws20a_process_frame(const uint8_t *frame, size_t frame_len) {
    uint8_t cmd = frame[3];
    const uint8_t *payload = frame + 4;
    size_t payload_len = frame_len - 5;

    if (cmd == 0x12) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] WS20A measurement started\n");
        return;
    }

    if (cmd != 0x10) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] WS20A RX cmd=0x%02X len=%u\n",
                  cmd, (unsigned)payload_len);
        return;
    }

    if (payload_len < 15) {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] WS20A realtime packet too short: %u\n",
                  (unsigned)payload_len);
        return;
    }

    uint16_t pulse = ((uint16_t)payload[0] << 8) | payload[1];
    uint8_t spo2 = payload[2];
    uint16_t pi = ((uint16_t)payload[3] << 8) | payload[4];
    uint8_t battery = payload[5];
    uint8_t seq = payload[14];

    Log::logf(CAT_OXI, LOG_DEBUG,
              "[OXI] WS20A: SpO2=%d HR=%d PI=%d battery=%d seq=%d\n",
              spo2, pulse, pi, battery, seq);

    if (spo2 >= 50 && spo2 <= 100 && pulse >= 25 && pulse <= 250) {
        OxiArbiter::feed(OXI_SRC_BLE, (int8_t)spo2, (int16_t)pulse, true);
    } else {
        OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
    }
}

static void ws20a_process_rx() {
    while (ws20a_rx_len >= 2) {
        if (ws20a_rx_buf[0] != 0xFE || ws20a_rx_buf[1] != 0x5A) {
            memmove(ws20a_rx_buf, ws20a_rx_buf + 1, --ws20a_rx_len);
            continue;
        }

        if (ws20a_rx_len < 3) return;
        size_t frame_len = ws20a_rx_buf[2];
        if (frame_len < 5) {
            ws20a_frame_errors++;
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] WS20A invalid frame length: %u\n",
                      (unsigned)frame_len);
            memmove(ws20a_rx_buf, ws20a_rx_buf + 1, --ws20a_rx_len);
            continue;
        }
        if (ws20a_rx_len < frame_len) return;

        uint8_t sum = 0;
        for (size_t i = 2; i < frame_len - 1; i++) sum += ws20a_rx_buf[i];
        if (sum != ws20a_rx_buf[frame_len - 1]) {
            ws20a_frame_errors++;
            Log::logf(CAT_OXI, LOG_WARN,
                      "[OXI] WS20A checksum mismatch: expected=%02X actual=%02X errors=%lu\n",
                      ws20a_rx_buf[frame_len - 1], sum, (unsigned long)ws20a_frame_errors);
            memmove(ws20a_rx_buf, ws20a_rx_buf + 1, --ws20a_rx_len);
            continue;
        }

        ws20a_process_frame(ws20a_rx_buf, frame_len);
        ws20a_rx_len -= frame_len;
        if (ws20a_rx_len > 0) {
            memmove(ws20a_rx_buf, ws20a_rx_buf + frame_len, ws20a_rx_len);
        }
    }

    if (ws20a_rx_len == 1 && ws20a_rx_buf[0] != 0xFE) ws20a_rx_len = 0;
}

static void ws20a_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len > 0) {
        char hex[64] = {};
        int n = len > 20 ? 20 : (int)len;
        for (int i = 0; i < n; i++) snprintf(hex + i*3, 4, "%02X ", data[i]);
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] WS20A RX len=%d: %s\n", (int)len, hex);
    }

    while (len > 0) {
        size_t available = sizeof(ws20a_rx_buf) - ws20a_rx_len;
        if (available == 0) {
            ws20a_frame_errors++;
            ws20a_rx_len = 0;
            available = sizeof(ws20a_rx_buf);
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] WS20A RX buffer overflow\n");
        }

        size_t chunk_len = len < available ? len : available;
        memcpy(ws20a_rx_buf + ws20a_rx_len, data, chunk_len);
        ws20a_rx_len += chunk_len;
        data += chunk_len;
        len -= chunk_len;
        ws20a_process_rx();
    }
}


// Viatom/Wellue: response packet header is 7 bytes (0x55, cmd, ~cmd, blk_lo, blk_hi, len_lo, len_hi)
// CMD_READ_SENSORS response: payload byte 0 = SpO2, byte 1 = HR
static NimBLERemoteCharacteristic *viatom_write_chr = nullptr;
static uint8_t viatom_invalid_count = 0;
#define VIATOM_MAX_INVALID  15  // disconnect after 15 invalid readings (~30s)
#define VIATOM_WRITE_CHUNK_LEN 20
#define VIATOM_WRITE_CHUNK_DELAY_MS 50

static void viatom_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    // debug
    if (len > 0) {
        char hex[64] = {};
        int n = len > 20 ? 20 : len;
        for (int i = 0; i < n; i++) snprintf(hex + i*3, 4, "%02X ", data[i]);
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Viatom RX len=%d: %s\n", len, hex);
    }

    // Response: 55 CMD ~CMD BLK_LO BLK_HI LEN_LO LEN_HI [payload...]
    // Byte 7 = SpO2, Byte 8 = HR, 0xFF = no finger, 0x00 = no reading
    if (len >= 9 && data[0] == 0x55) {
        uint8_t spo2 = data[7];
        uint8_t hr = data[8];
        bool no_finger = (spo2 == 0 || spo2 == 0xFF || hr == 0 || hr == 0xFF);
        if (!no_finger && spo2 <= 100 && hr < 250) {
            OxiArbiter::feed(OXI_SRC_BLE, (int8_t)spo2, (int16_t)hr, true);
            viatom_invalid_count = 0;
        } else {
            OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
            if (++viatom_invalid_count >= VIATOM_MAX_INVALID) {
                Log::logf(CAT_OXI, LOG_INFO, "[OXI] Viatom: no valid data for %d readings, disconnecting\n",
                          VIATOM_MAX_INVALID);
                disconnect_requested = true;
            }
        }
    }
}


// OxyII: A5 CMD ~CMD 00 SEQ LEN_LO LEN_HI PAYLOAD... CRC8
#define OXYII_CMD_LIVE_SAMPLES       0x04
#define OXYII_CMD_SETUP              0x10
#define OXYII_CMD_SET_TIME           0xC0
#define OXYII_CMD_AUTH               0xFF
#define OXYII_NO_PENDING_CMD         0xFE
#define OXYII_SENSOR_POLL_MS         1000
#define OXYII_RESPONSE_TIMEOUT_MS    1500
#define OXYII_RX_BUF_LEN             640
#define OXYII_TX_BUF_LEN             32

static const uint8_t oxyii_lepucloud_md5[16] = {
    0xC2, 0xA7, 0xCF, 0x50, 0xDA, 0xFE, 0xD8, 0x85,
    0xA8, 0xF8, 0xF7, 0xEA, 0xC4, 0x43, 0x35, 0xF3,
};

static NimBLERemoteCharacteristic *oxyii_write_chr = nullptr;
static uint8_t oxyii_rx_buf[OXYII_RX_BUF_LEN];
static size_t oxyii_rx_len = 0;
static size_t oxyii_rx_want = 0;
static volatile uint8_t oxyii_pending_cmd = OXYII_NO_PENDING_CMD;
static volatile uint32_t oxyii_pending_ms = 0;
static uint32_t oxyii_last_poll_ms = 0;
static uint8_t oxyii_sequence = 0;
static volatile bool oxyii_need_auth = false;
static volatile bool oxyii_need_setup = false;
static volatile bool oxyii_need_time_sync = false;

static const char *oxyii_command_name(uint8_t cmd) {
    switch (cmd) {
        case OXYII_CMD_LIVE_SAMPLES: return "live_samples";
        case OXYII_CMD_SETUP: return "setup";
        case OXYII_CMD_SET_TIME: return "set_time";
        case OXYII_CMD_AUTH: return "auth";
        case OXYII_NO_PENDING_CMD: return "none";
        default: return "unknown";
    }
}

static void oxyii_reset_rx() {
    oxyii_rx_len = 0;
    oxyii_rx_want = 0;
}

static void oxyii_clear_pending() {
    oxyii_pending_cmd = OXYII_NO_PENDING_CMD;
    oxyii_pending_ms = 0;
}

static void oxyii_reset() {
    oxyii_write_chr = nullptr;
    oxyii_reset_rx();
    oxyii_clear_pending();
    oxyii_last_poll_ms = 0;
    oxyii_sequence = 0;
    oxyii_need_auth = false;
    oxyii_need_setup = false;
    oxyii_need_time_sync = false;
}

static void oxyii_process_frame(const uint8_t *frame, size_t frame_len) {
    if (frame_len < 8 || frame[0] != 0xA5) return;

    uint8_t cmd = frame[1];
    if (frame[2] != (uint8_t)~cmd) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII RX invalid command complement\n");
        return;
    }

    size_t payload_len = frame[5] | ((size_t)frame[6] << 8);
    if (payload_len + 8 != frame_len ||
        crc8_ccitt(frame, frame_len - 1) != frame[frame_len - 1]) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII RX decode failed len=%u\n",
                  (unsigned)frame_len);
        return;
    }

    uint8_t pending_cmd = oxyii_pending_cmd;
    oxyii_clear_pending();
    if (pending_cmd == OXYII_CMD_SETUP && cmd == OXYII_CMD_SETUP) {
        oxyii_need_time_sync = true;
        return;
    }
    if (pending_cmd == OXYII_CMD_SET_TIME && cmd == OXYII_CMD_SET_TIME) return;
    if (pending_cmd != OXYII_CMD_LIVE_SAMPLES || cmd != OXYII_CMD_LIVE_SAMPLES) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII RX ignored cmd=%s pending=%s\n",
                  oxyii_command_name(cmd), oxyii_command_name(pending_cmd));
        return;
    }

    if (payload_len < 9) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII live packet too short: %u\n",
                  (unsigned)payload_len);
        return;
    }

    const uint8_t *payload = frame + 7;
    uint8_t spo2 = payload[6];
    uint8_t pulse = payload[8];
    bool valid = spo2 > 0 && spo2 <= 100 && pulse > 0 && pulse != 0xFF && pulse < 250;
    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII: SpO2=%d HR=%d valid=%d\n",
              spo2, pulse, valid);
    if (valid) {
        OxiArbiter::feed(OXI_SRC_BLE, (int8_t)spo2, (int16_t)pulse, true);
    } else {
        OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
    }
}

static void oxyii_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (!data || len == 0) return;

    if (data[0] == 0xA5) oxyii_reset_rx();
    if (oxyii_rx_len == 0 && data[0] != 0xA5) return;
    if (oxyii_rx_len + len > sizeof(oxyii_rx_buf)) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII RX buffer overflow\n");
        oxyii_reset_rx();
        return;
    }

    memcpy(oxyii_rx_buf + oxyii_rx_len, data, len);
    oxyii_rx_len += len;

    if (oxyii_rx_want == 0 && oxyii_rx_len >= 7) {
        size_t payload_len = oxyii_rx_buf[5] | ((size_t)oxyii_rx_buf[6] << 8);
        oxyii_rx_want = payload_len + 8;
        if (oxyii_rx_want > sizeof(oxyii_rx_buf)) {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII RX too large: %u\n",
                      (unsigned)oxyii_rx_want);
            oxyii_reset_rx();
            return;
        }
    }

    if (oxyii_rx_want == 0 || oxyii_rx_len < oxyii_rx_want) return;
    oxyii_process_frame(oxyii_rx_buf, oxyii_rx_want);
    oxyii_reset_rx();
}

static bool oxyii_write_frame(uint8_t cmd, const uint8_t *payload, size_t payload_len) {
    if (!oxyii_write_chr || payload_len > 0xFFFF) return false;

    size_t frame_len = payload_len + 8;
    if (frame_len > OXYII_TX_BUF_LEN) return false;

    uint8_t frame[OXYII_TX_BUF_LEN] = {};
    frame[0] = 0xA5;
    frame[1] = cmd;
    frame[2] = (uint8_t)~cmd;
    frame[4] = oxyii_sequence++;
    frame[5] = payload_len & 0xFF;
    frame[6] = (payload_len >> 8) & 0xFF;
    if (payload && payload_len > 0) memcpy(frame + 7, payload, payload_len);
    frame[frame_len - 1] = crc8_ccitt(frame, frame_len - 1);
    return oxyii_write_chr->writeValue(frame, frame_len, false);
}

static bool oxyii_send_command(uint8_t cmd, const uint8_t *payload, size_t payload_len,
                               uint32_t now_ms, bool expect_reply = true) {
    if (!oxyii_write_frame(cmd, payload, payload_len)) return false;

    if (expect_reply) {
        oxyii_pending_cmd = cmd;
        oxyii_pending_ms = now_ms;
    }
    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII TX cmd=%s payload_len=%u\n",
              oxyii_command_name(cmd), (unsigned)payload_len);
    return true;
}

static bool oxyii_send_auth(uint32_t now_ms) {
    time_t seconds = time(nullptr);
    if (seconds < 1704067200) return false;

    uint8_t session_key[16] = {};
    for (size_t i = 0; i < 8; i++) session_key[i] = oxyii_lepucloud_md5[i * 2];
    session_key[8] = '0';
    session_key[9] = '0';
    session_key[10] = '0';
    session_key[11] = '0';

    uint32_t timestamp = (uint32_t)seconds;
    // The device expects bit shifts 0, 1, 2, and 3 here.
    for (uint8_t i = 0; i < 4; i++) {
        session_key[12 + i] = (timestamp >> i) & 0xFF;
    }

    uint8_t payload[16];
    for (size_t i = 0; i < sizeof(payload); i++) {
        payload[i] = session_key[i] ^ oxyii_lepucloud_md5[i];
    }
    return oxyii_send_command(OXYII_CMD_AUTH, payload, sizeof(payload), now_ms, false);
}

static bool oxyii_sync_datetime(uint32_t now_ms) {
    if (!WiFiSetup::time_synced()) return false;

    time_t now = time(nullptr);
    if (now < 1704067200) return false;

    struct tm t;
    localtime_r(&now, &t);

    uint8_t payload[8] = {};
    uint16_t year = t.tm_year + 1900;
    payload[0] = year & 0xFF;
    payload[1] = (year >> 8) & 0xFF;
    payload[2] = t.tm_mon + 1;
    payload[3] = t.tm_mday;
    payload[4] = t.tm_hour;
    payload[5] = t.tm_min;
    payload[6] = t.tm_sec;
    return oxyii_send_command(OXYII_CMD_SET_TIME, payload, sizeof(payload), now_ms);
}

static void oxyii_poll(uint32_t now_ms) {
    if (!oxyii_write_chr || !pClient || !pClient->isConnected()) return;

    if (oxyii_pending_cmd != OXYII_NO_PENDING_CMD &&
        now_ms - oxyii_pending_ms >= OXYII_RESPONSE_TIMEOUT_MS) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII response timeout cmd=%s\n",
                  oxyii_command_name(oxyii_pending_cmd));
        oxyii_reset_rx();
        oxyii_clear_pending();
    }
    if (oxyii_pending_cmd != OXYII_NO_PENDING_CMD) return;

    if (oxyii_need_auth) {
        if (time(nullptr) < 1704067200) return;
        if (oxyii_send_auth(now_ms)) {
            oxyii_need_auth = false;
            oxyii_need_setup = true;
        } else {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII auth write failed\n");
        }
        return;
    }

    if (oxyii_need_setup) {
        uint8_t payload = 0;
        if (oxyii_send_command(OXYII_CMD_SETUP, &payload, sizeof(payload), now_ms)) {
            oxyii_need_setup = false;
        } else {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII setup write failed\n");
        }
        return;
    }

    if (oxyii_need_time_sync) {
        if (!WiFiSetup::time_synced()) {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Skipping OxyII datetime - NTP not synced\n");
        } else if (!oxyii_sync_datetime(now_ms)) {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII datetime write failed\n");
        }
        oxyii_need_time_sync = false;
        return;
    }

    if (now_ms - oxyii_last_poll_ms < OXYII_SENSOR_POLL_MS) return;
    if (!oxyii_send_command(OXYII_CMD_LIVE_SAMPLES, nullptr, 0, now_ms)) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] OxyII poll write failed\n");
    }
    oxyii_last_poll_ms = now_ms;
}

static bool has_oxyii_manufacturer(const NimBLEAdvertisedDevice *dev) {
    for (uint8_t i = 0; i < dev->getManufacturerDataCount(); i++) {
        std::string data = dev->getManufacturerData(i);
        if (data.size() < 2) continue;

        uint16_t company = (uint8_t)data[0] | ((uint16_t)(uint8_t)data[1] << 8);
        if (company == 0x036F || company == 0xF34E) return true;
    }
    return false;
}

class OxiScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        String name = dev->getName().c_str();
        String addr = dev->getAddress().toString().c_str();
        // Passive advertisements need not contain a name or service UUID.
        bool known = is_device_known(addr.c_str(), dev->getAddress().getType());
        bool is_oxi = has_oxyii_manufacturer(dev) ||
                       known || addr.equalsIgnoreCase(Config::get().oxi_device_addr) ||
                       dev->isAdvertisingService(PLX_SERVICE_UUID) ||
                       dev->isAdvertisingService(NONIN_OXI_SERVICE_UUID) ||
                       dev->isAdvertisingService(HR_SERVICE_UUID) ||
                       dev->isAdvertisingService(VIATOM_SERVICE_UUID) ||
                       dev->isAdvertisingService(OXYII_SERVICE_UUID) ||
                       dev->isAdvertisingService(WS20A_NOTIFY_SERVICE_UUID) ||
                       dev->isAdvertisingService(WS20A_WRITE_SERVICE_UUID) ||
                       name.startsWith("Nonin") ||
                       name.startsWith("O2 ") ||
                       name.startsWith("O2Ring") ||
                       name.startsWith("O2M") ||
                       name.startsWith("S8-AW") ||
                       name.startsWith("T8520_") ||
                       name.startsWith("CheckMe") ||
                       name.startsWith("Checkme") ||
                       name.startsWith("CheckO2") ||
                       name.startsWith("SleepU") ||
                       name.startsWith("SleepO2") ||
                       name.startsWith("WearO2") ||
                       name.startsWith("KidsO2") ||
                       name.startsWith("BabyO2") ||
                       name.startsWith("OxyLink") ||
                       name.startsWith("Oxylink") ||
                       name.startsWith("WS20") ||
                       name.startsWith("ACCARE") ||
                       name.startsWith("Accare");

        if (is_oxi && scan_mutex && xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (scan_result_count < MAX_SCAN_RESULTS) {
                scan_results[scan_result_count].addr = dev->getAddress().toString().c_str();
                scan_results[scan_result_count].name = name;
                scan_results[scan_result_count].rssi = dev->getRSSI();
                scan_results[scan_result_count].addr_type = dev->getAddress().getType();
                scan_result_count++;
            }
            xSemaphoreGive(scan_mutex);
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Found: %s (%s) RSSI=%d\n",
                          name.c_str(), dev->getAddress().toString().c_str(), dev->getRSSI());
        }
    }

    void onScanEnd(const NimBLEScanResults &results, int reason) override {
        Log::logf(CAT_OXI, scan_result_count || reason ? LOG_INFO : LOG_DEBUG,
                  "[OXI] Scan complete, %d oximeters found (reason=%d)\n", scan_result_count, reason);
        scan_complete = true;
    }
};

static OxiScanCB scanCB;

class OxiClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *client) override {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Connected\n");
    }

    void onDisconnect(NimBLEClient *client, int reason) override {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Disconnected (reason=0x%X)\n", reason);
        viatom_write_chr = nullptr;
        oxyii_reset();
        ws20a_write_chr = nullptr;
        ws20a_rx_len = 0;
        OxiArbiter::stop_feed();
        if (state == OXI_STREAMING || state == OXI_BONDING) {
            set_state(OXI_DISCONNECTED);
        }
    }

    bool onConnParamsUpdateRequest(NimBLEClient *client, const ble_gap_upd_params *params) override {
        return true;
    }

    void onPassKeyEntry(NimBLEConnInfo &connInfo) override {
        // "Just Works"
        NimBLEDevice::injectPassKey(connInfo, 0);
    }

    void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pin) override {
        NimBLEDevice::injectConfirmPasskey(connInfo, true);
    }

    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
        if (connInfo.isEncrypted()) {
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Encrypted + bonded\n");
        } else {
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] Auth complete (no encryption)\n");
        }
    }
};

static OxiClientCB clientCB;


static bool subscribe_services(NimBLEClient *cl) {
    bool got_spo2 = false;
    bool got_hr = false;

    NimBLERemoteService *plxSvc = cl->getService(PLX_SERVICE_UUID);
    if (plxSvc) {
        NimBLERemoteCharacteristic *plxCont = plxSvc->getCharacteristic(PLX_CONTINUOUS_UUID);
        if (plxCont && plxCont->canNotify() && plxCont->subscribe(true, plx_notify_cb)) {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed PLX Continuous\n");
            got_spo2 = got_hr = true;
        }
        if (!got_spo2) {
            NimBLERemoteCharacteristic *plxSpot = plxSvc->getCharacteristic(PLX_SPOT_UUID);
            if (plxSpot && plxSpot->canIndicate() && plxSpot->subscribe(false, plx_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed PLX Spot\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *noninSvc = cl->getService(NONIN_OXI_SERVICE_UUID);
        if (noninSvc) {
            NimBLERemoteCharacteristic *noninCont = noninSvc->getCharacteristic(NONIN_CONTINUOUS_UUID);
            if (noninCont && noninCont->canNotify() && noninCont->subscribe(true, nonin_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed Nonin Continuous\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *viatomSvc = cl->getService(VIATOM_SERVICE_UUID);
        if (viatomSvc) {
            NimBLERemoteCharacteristic *viatomRead = viatomSvc->getCharacteristic(VIATOM_READ_UUID);
            if (viatomRead && viatomRead->canNotify() && viatomRead->subscribe(true, viatom_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed Viatom read\n");
                viatom_invalid_count = 0;
                viatom_write_chr = viatomSvc->getCharacteristic(VIATOM_WRITE_UUID);
                if (viatom_write_chr) Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Viatom write chr found\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *oxyiiSvc = cl->getService(OXYII_SERVICE_UUID);
        if (oxyiiSvc) {
            NimBLERemoteCharacteristic *oxyiiNotify = oxyiiSvc->getCharacteristic(OXYII_NOTIFY_UUID);
            NimBLERemoteCharacteristic *oxyiiWrite = oxyiiSvc->getCharacteristic(OXYII_WRITE_UUID);
            if (oxyiiNotify && oxyiiNotify->canNotify() && oxyiiWrite) {
                oxyii_reset();
                oxyii_write_chr = oxyiiWrite;
                oxyii_need_auth = true;
                if (oxyiiNotify->subscribe(true, oxyii_notify_cb)) {
                    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed OxyII notify\n");
                    got_spo2 = got_hr = true;
                } else {
                    Log::logf(CAT_OXI, LOG_WARN, "[OXI] OxyII notification subscribe failed\n");
                    oxyii_reset();
                }
            } else {
                Log::logf(CAT_OXI, LOG_WARN,
                          "[OXI] OxyII characteristics unavailable: notify=%d write=%d\n",
                          oxyiiNotify && oxyiiNotify->canNotify(), oxyiiWrite != nullptr);
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *wsNotifySvc = cl->getService(WS20A_NOTIFY_SERVICE_UUID);
        NimBLERemoteService *wsWriteSvc = cl->getService(WS20A_WRITE_SERVICE_UUID);
        NimBLERemoteCharacteristic *wsNotify = nullptr;
        if (wsNotifySvc) wsNotify = wsNotifySvc->getCharacteristic(WS20A_NOTIFY_UUID);
        if (!wsNotify && wsWriteSvc) wsNotify = wsWriteSvc->getCharacteristic(WS20A_NOTIFY_UUID);

        NimBLERemoteCharacteristic *wsWrite = nullptr;
        if (wsWriteSvc) wsWrite = wsWriteSvc->getCharacteristic(WS20A_WRITE_UUID);

        bool can_subscribe = wsNotify && (wsNotify->canNotify() || wsNotify->canIndicate());
        bool can_write = wsWrite && (wsWrite->canWrite() || wsWrite->canWriteNoResponse());
        if (can_subscribe && can_write) {
            bool notifications = wsNotify->canNotify();
            if (wsNotify->subscribe(notifications, ws20a_notify_cb)) {
                ws20a_write_chr = wsWrite;
                ws20a_rx_len = 0;
                ws20a_frame_errors = 0;
                uint8_t start_payload = 0x00;
                if (ws20a_send_command(0x12, &start_payload, 1)) {
                    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed WS20A realtime data\n");
                    got_spo2 = got_hr = true;
                } else {
                    ws20a_write_chr = nullptr;
                }
            } else {
                Log::logf(CAT_OXI, LOG_WARN, "[OXI] WS20A notification subscribe failed\n");
            }
        } else if (wsNotifySvc || wsWriteSvc) {
            Log::logf(CAT_OXI, LOG_WARN,
                      "[OXI] WS20A characteristics unavailable: notify=%d write=%d\n",
                      can_subscribe, can_write);
        }
    }

    if (!got_hr) {
        NimBLERemoteService *hrSvc = cl->getService(HR_SERVICE_UUID);
        if (hrSvc) {
            NimBLERemoteCharacteristic *hrMeas = hrSvc->getCharacteristic(HR_MEASUREMENT_UUID);
            if (hrMeas && hrMeas->canNotify() && hrMeas->subscribe(true, hr_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed Heart Rate\n");
                got_hr = true;
            }
        }
    }

    return got_spo2 || got_hr;
}

// Set date/time on Nonin devices so stored records have correct timestamps.
static void set_nonin_datetime(NimBLEClient *cl) {
    if (!WiFiSetup::time_synced()) {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Skipping Nonin datetime — NTP not synced\n");
        return;
    }

    NimBLERemoteService *svc = cl->getService(NONIN_OXI_SERVICE_UUID);
    if (!svc) return;

    NimBLERemoteCharacteristic *cp = svc->getCharacteristic(NONIN_CONTROL_POINT_UUID);
    if (!cp || !cp->canWrite()) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Nonin control point not writable\n");
        return;
    }

    struct tm timeinfo;
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);

    char ts[13];
    strftime(ts, sizeof(ts), "%y%m%d%H%M%S", &timeinfo);

    uint8_t cmd[] = {0x60, 0x4E, 0x4D, 0x49, 0x12, 0x44, 0x54, 0x4D, 0x3D,
                     0,0,0,0,0,0,0,0,0,0,0,0, 0x0D, 0x0A};
    memcpy(cmd + 9, ts, 12);

    if (cp->writeValue(cmd, sizeof(cmd), true)) {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Nonin datetime set: %s\n", ts);
    } else {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Nonin datetime write failed\n");
    }
}

static void set_viatom_datetime() {
    if (!WiFiSetup::time_synced() || !viatom_write_chr) return;

    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);

    // json {"SetTIME":"YYYY-MM-DD,HH:MM:SS"}
    char json[48];
    strftime(json, sizeof(json), "{\"SetTIME\":\"%Y-%m-%d,%H:%M:%S\"}", &t);
    int json_len = strlen(json);

    // AA CMD ~CMD BLK_LO BLK_HI LEN_LO LEN_HI [json] CRC8
    int pkt_len = 7 + json_len + 1;
    uint8_t pkt[64];
    pkt[0] = 0xAA;
    pkt[1] = 0x16;  // CMD_CONFIG
    pkt[2] = 0x16 ^ 0xFF;
    pkt[3] = 0x00; pkt[4] = 0x00;  // block
    pkt[5] = json_len & 0xFF;
    pkt[6] = (json_len >> 8) & 0xFF;
    memcpy(pkt + 7, json, json_len);

    uint8_t crc = crc8_ccitt(pkt, 7 + json_len);
    pkt[7 + json_len] = crc;

    bool ok = true;
    for (int off = 0; off < pkt_len; off += VIATOM_WRITE_CHUNK_LEN) {
        int chunk_len = pkt_len - off;
        if (chunk_len > VIATOM_WRITE_CHUNK_LEN) chunk_len = VIATOM_WRITE_CHUNK_LEN;
        if (!viatom_write_chr->writeValue(pkt + off, chunk_len, false)) {
            ok = false;
            break;
        }
        if (off + chunk_len < pkt_len) {
            vTaskDelay(pdMS_TO_TICKS(VIATOM_WRITE_CHUNK_DELAY_MS));
        }
    }

    if (ok) {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Viatom datetime set: %s\n", json);
    } else {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Viatom datetime write failed\n");
    }
}


static bool do_remove_known(const char *addr);
static void do_clear_all_known();

void OxiBle::task(void *param) {
    scan_mutex = xSemaphoreCreateMutex();
    known_load();
    NimBLEDevice::init(Config::get().hostname.c_str());
    NimBLEDevice::setSecurityAuth(true, false, false);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCB);
    pClient->setConnectionParams(12, 12, 0, 400);

    auto &cfg = Config::get();
    if (cfg.oxi_enabled) {
        set_state(OXI_DISCONNECTED);
        scan_requested = true;
    }

    uint32_t last_reconnect = 0;

    while (true) {
        if (disable_requested) {
            disable_requested = false;
            disconnect_requested = false;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disable requested\n");
            if (pClient->isConnected()) pClient->disconnect();
            OxiArbiter::stop_feed();
            set_state(OXI_DISABLED);
        }

        if (disconnect_requested) {
            disconnect_requested = false;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disconnect requested\n");
            if (pClient->isConnected()) pClient->disconnect();
            OxiArbiter::stop_feed();
            set_state(OXI_DISCONNECTED);
        }

        if (suspend_enter_requested) {
            suspend_enter_requested = false;
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Suspend: stopping scan and dropping connection\n");
            NimBLEDevice::getScan()->stop();
            if (pClient->isConnected()) pClient->disconnect();
            OxiArbiter::stop_feed();
            scan_requested = false;
            connect_mode = CONN_NONE;
            if (state != OXI_DISABLED) set_state(OXI_DISCONNECTED);
        }

        if (del_one_requested) {
            del_one_requested = false;
            char addr[18];
            strncpy(addr, del_one_addr, sizeof(addr));
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Remove-known requested: %s\n", addr);
            NimBLEDevice::getScan()->stop();
            if (pClient->isConnected()) pClient->disconnect();
            vTaskDelay(pdMS_TO_TICKS(200));
            bool ok = do_remove_known(addr);
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Remove-known %s: %s\n",
                      addr, ok ? "done" : "not found");
        }

        if (del_all_requested) {
            del_all_requested = false;
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Clear-all-known requested\n");
            NimBLEDevice::getScan()->stop();
            if (pClient->isConnected()) pClient->disconnect();
            vTaskDelay(pdMS_TO_TICKS(200));
            do_clear_all_known();
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] All known devices cleared\n");
        }

        if (scan_complete) {
            scan_complete = false;
            last_reconnect = millis();
            if (state == OXI_SCANNING) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Scan done, %d results\n", scan_result_count);
                set_state(OXI_DISCONNECTED);
                if (scan_result_count > 0 && !active_scan_requested) {
                    String target = cfg.oxi_device_addr;
                    bool found = false;
                    if (target.length() > 0) {
                        for (int i = 0; i < scan_result_count; i++) {
                            if (scan_results[i].addr.equalsIgnoreCase(target)) {
                                found = true;
                                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Target %s found in scan\n", target.c_str());
                                break;
                            }
                        }
                        if (!found) Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Target %s not in scan results\n", target.c_str());
                    } else {
                        for (int i = 0; i < scan_result_count; i++) {
                            bool known = is_device_known(scan_results[i].addr.c_str(),
                                                         scan_results[i].addr_type);
                            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] %s %s known=%d\n",
                                      scan_results[i].name.c_str(), scan_results[i].addr.c_str(), known);
                            if (cfg.oxi_require_known) {
                                if (known) { target = scan_results[i].addr; found = true; break; }
                            } else {
                                if (scan_results[i].name.startsWith("Nonin")) {
                                    if (known) { target = scan_results[i].addr; found = true; break; }
                                } else {
                                    target = scan_results[i].addr;
                                    found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (found) {
                        target.toCharArray(target_addr, sizeof(target_addr));
                        connect_mode = CONN_AUTO;
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Auto-connect triggered\n");
                    }
                }
            }
        }

        if (scan_requested && !ble_suspended) {
            scan_requested = false;
            NimBLEScan *pScan = NimBLEDevice::getScan();
            if (pScan->isScanning()) {
                if (active_scan_requested) {
                    pScan->stop();
                    scan_requested = true;
                } else {
                    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] scan_requested ignored, scan already in progress\n");
                }
            } else {
                if (scan_mutex) xSemaphoreTake(scan_mutex, portMAX_DELAY);
                scan_result_count = 0;
                if (scan_mutex) xSemaphoreGive(scan_mutex);
                scan_complete = false;
                pScan->clearResults();   // defeat NimBLE dedup across scans
                set_state(OXI_SCANNING);
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Starting scan (%dms)\n", SCAN_DURATION_MS);
                pScan->setScanCallbacks(&scanCB);
                bool active = active_scan_requested;
                active_scan_requested = false;
                // AirCANnect observer timing; active discovery is user-requested.
                pScan->setActiveScan(active);
                pScan->setInterval(active ? 100 : 1000);
                pScan->setWindow(active ? 99 : 20);
                if (!pScan->start(SCAN_DURATION_MS)) {
                    last_reconnect = millis();
                    set_state(OXI_DISCONNECTED);
                }
            }
        }

        if (connect_mode != CONN_NONE && !ble_suspended) {
            connect_mode_t mode = connect_mode;
            connect_mode = CONN_NONE;
            NimBLEDevice::getScan()->stop();
            // Wait for scan to actually stop before connecting
            for (int i = 0; i < 20 && NimBLEDevice::getScan()->isScanning(); i++)
                vTaskDelay(pdMS_TO_TICKS(50));
            scan_complete = false;  // discard any pending scan-complete trigger

            String addr = target_addr;
            if (addr.length() == 0 && cfg.oxi_device_addr.length() > 0)
                addr = cfg.oxi_device_addr;
            if (addr.length() == 0 && scan_result_count > 0)
                addr = scan_results[0].addr;

            int max_attempts = (mode == CONN_USER) ? USER_CONNECT_RETRIES : 1;

            if (addr.length() > 0) {
                set_state(OXI_CONNECTING);

                uint8_t atype = 1;
                String dev_name = "";
                for (int i = 0; i < scan_result_count; i++) {
                    if (scan_results[i].addr.equalsIgnoreCase(addr)) {
                        atype = scan_results[i].addr_type;
                        dev_name = scan_results[i].name;
                        break;
                    }
                }

                NimBLEAddress bleAddr(std::string(addr.c_str()), atype);
                // Bonded Nonin may omit its name from passive advertisements.
                device_needs_encryption = dev_name.startsWith("Nonin") || NimBLEDevice::isBonded(bleAddr);
                bool connected = false;

                for (int attempt = 1; attempt <= max_attempts; attempt++) {
                    if (disconnect_requested || disable_requested) break;

                    if (attempt > 1) {
                        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Retry %d/%d after %dms\n",
                                  attempt, max_attempts, USER_RETRY_DELAY_MS);
                        vTaskDelay(pdMS_TO_TICKS(USER_RETRY_DELAY_MS));
                    }

                    // cancel any pending connection and clean up stale state
                    if (pClient->isConnected()) {
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disconnecting stale connection\n");
                        pClient->disconnect();
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    pClient->cancelConnect();
                    vTaskDelay(pdMS_TO_TICKS(500));

                    // connect
                    set_state(OXI_CONNECTING);
                    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Connecting to %s (type=%d, %s, attempt %d/%d)...\n",
                              addr.c_str(), atype, mode == CONN_USER ? "user" : "auto",
                              attempt, max_attempts);

                    bool ok = pClient->connect(bleAddr);

                    if (!ok) {
                        int err = pClient->getLastError();
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] connect() failed (err=%d)\n", err);

                        // EALREADY: previous connect still in flight
                        if (err == 2) {
                            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Waiting for pending connect...\n");
                            for (int i = 0; i < 50 && !pClient->isConnected(); i++)
                                vTaskDelay(pdMS_TO_TICKS(200));
                            ok = pClient->isConnected();
                            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Pending connect %s\n", ok ? "succeeded" : "failed");
                            if (ok) vTaskDelay(pdMS_TO_TICKS(500));
                        }

                        // EDONE: stale bond - delete and retry within this attempt
                        if (!ok && err == 13) {
                            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Removing stale bond and retrying\n");
                            NimBLEDevice::deleteBond(bleAddr);
                            vTaskDelay(pdMS_TO_TICKS(500));
                            ok = pClient->connect(bleAddr);
                            if (!ok) {
                                Log::logf(CAT_OXI, LOG_WARN, "[OXI] Post-bond-delete retry failed (err=%d)\n",
                                          pClient->getLastError());
                            }
                        }
                    }

                    if (!ok) continue;

                    // encrypt for devices that require it 
                    if (device_needs_encryption) {
                        set_state(OXI_BONDING);
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Initiating encryption\n");

                        bool secured = pClient->secureConnection(false);
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Encryption: secure=%d connected=%d err=%d\n",
                                  secured, pClient->isConnected(), pClient->getLastError());

                        if (!pClient->isConnected()) {
                            Log::logf(CAT_OXI, LOG_WARN, "[OXI] Lost connection during encryption\n");
                            continue;
                        }

                        if (!secured) {
                            Log::logf(CAT_OXI, LOG_WARN, "[OXI] Encryption failed, disconnecting to retry\n");
                            pClient->disconnect();
                            vTaskDelay(pdMS_TO_TICKS(500));
                            continue;
                        }
                    } else {
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Skipping encryption (not required)\n");
                    }

                    // subscribe
                    if (!subscribe_services(pClient)) {
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] No suitable services, disconnecting\n");
                        pClient->disconnect();
                        continue;
                    }

                    // Final check - onDisconnect may have fired during subscribe
                    if (!pClient->isConnected()) {
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Connection lost after subscribe\n");
                        continue;
                    }

                    set_nonin_datetime(pClient);
                    set_viatom_datetime();
                    OxiArbiter::set_source_id(pClient->getPeerAddress().toString().c_str());
                    set_state(OXI_STREAMING);
                    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Streaming started\n");

                    if (mode == CONN_USER && !device_needs_encryption) {
                        known_add(addr.c_str());
                    }
                    connected = true;
                    break;
                }

                if (!connected) {
                    Log::logf(CAT_OXI, LOG_WARN, "[OXI] Connect sequence failed after %d attempt(s)\n",
                              max_attempts);
                    if (pClient->isConnected()) pClient->disconnect();
                    set_state(OXI_DISCONNECTED);
                    last_reconnect = millis();
                }
            }
        }

        // Auto-reconnect: scan periodically when disconnected and no other source active
        if (state == OXI_DISCONNECTED && cfg.oxi_enabled && !ble_suspended &&
            OxiArbiter::active_source() == OXI_SRC_NONE &&
            millis() - last_reconnect > RECONNECT_DELAY_MS) {
            last_reconnect = millis();
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Auto-reconnect: starting scan\n");
            scan_requested = true;
        }

        // Viatom: poll sensor readings every 2s while streaming
        if (state == OXI_STREAMING && viatom_write_chr && pClient->isConnected()) {
            static uint32_t last_viatom_poll = 0;
            if (millis() - last_viatom_poll >= 2000) {
                last_viatom_poll = millis();
                // CMD_READ_SENSORS packet: AA 17 E8 00 00 00 00 CRC
                uint8_t cmd[] = {0xAA, 0x17, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00};
                cmd[7] = crc8_ccitt(cmd, 7);
                viatom_write_chr->writeValue(cmd, sizeof(cmd), false);
            }
        }

        if (state == OXI_STREAMING && oxyii_write_chr && pClient->isConnected()) {
            oxyii_poll(millis());
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void OxiBle::init() {
    xTaskCreatePinnedToCore(OxiBle::task, "ble_oxi", OXI_TASK_STACK,
                            nullptr, OXI_TASK_PRIO, &oxi_task_handle, 0);
}

void OxiBle::start_scan()  { active_scan_requested = true; scan_requested = true; }
void OxiBle::stop_scan()   { NimBLEDevice::getScan()->stop(); }

void OxiBle::connect(const char *addr) {
    strncpy(target_addr, addr ? addr : "", sizeof(target_addr) - 1);
    target_addr[sizeof(target_addr) - 1] = '\0';
    connect_mode = CONN_USER;
}

void OxiBle::disconnect()  { disconnect_requested = true; }
void OxiBle::disable()     { disable_requested = true; }
void OxiBle::enable() {
    if (state == OXI_DISABLED) {
        set_state(OXI_DISCONNECTED);
        scan_requested = true;
    }
}
void OxiBle::suspend() {
    if (!ble_suspended) {
        ble_suspended = true;
        suspend_enter_requested = true;
    }
}
void OxiBle::resume() {
    if (ble_suspended) {
        ble_suspended = false;
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Resumed\n");
        // Task's auto-reconnect will pick up from OXI_DISCONNECTED.
    }
}
oxi_state_t OxiBle::get_state()            { return state; }
bool OxiBle::state_changed()               { bool d = state_dirty; state_dirty = false; return d; }

int OxiBle::get_scan_results(oxi_scan_result_t *out, int max) {
    if (!out || max <= 0) return 0;
    int n = 0;
    if (scan_mutex) xSemaphoreTake(scan_mutex, portMAX_DELAY);
    n = (scan_result_count < max) ? scan_result_count : max;
    for (int i = 0; i < n; i++) out[i] = scan_results[i];  // String deep-copy
    if (scan_mutex) xSemaphoreGive(scan_mutex);
    return n;
}

int OxiBle::get_all_known(char addrs[][18], int max) {
    int n = 0;
    // NimBLE bonds
    int nb = NimBLEDevice::getNumBonds();
    for (int i = 0; i < nb && n < max; i++) {
        NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
        strncpy(addrs[n], ba.toString().c_str(), 17);
        addrs[n][17] = '\0';
        n++;
    }
    // Known list (skip duplicates with bonds)
    for (int i = 0; i < known_count && n < max; i++) {
        bool dup = false;
        for (int j = 0; j < n; j++) {
            if (strcasecmp(addrs[j], known_addrs[i]) == 0) { dup = true; break; }
        }
        if (!dup) {
            strncpy(addrs[n], known_addrs[i], 17);
            addrs[n][17] = '\0';
            n++;
        }
    }
    return n;
}

// Internal: must run on the BLE task (touches NimBLE + NVS).
static bool do_remove_known(const char *addr) {
    bool removed = false;
    int nb = NimBLEDevice::getNumBonds();
    for (int i = 0; i < nb; i++) {
        NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
        if (strcasecmp(ba.toString().c_str(), addr) == 0) {
            int rc = ble_gap_unpair(ba.getBase());
            if (rc == 0) removed = true;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Unpair %s rc=%d\n", addr, rc);
            break;
        }
    }
    if (known_remove(addr)) removed = true;
    return removed;
}

static void do_clear_all_known() {
    NimBLEDevice::deleteAllBonds();
    known_clear();
}

void OxiBle::request_remove_known(const char *addr) {
    if (!addr) return;
    strncpy(del_one_addr, addr, sizeof(del_one_addr) - 1);
    del_one_addr[sizeof(del_one_addr) - 1] = '\0';
    del_one_requested = true;
}

void OxiBle::request_clear_all_known() {
    del_all_requested = true;
}
