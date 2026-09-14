#include "resmed_ota.h"
#include "uart_arbiter.h"
#include "qframe.h"
#include "crc.h"
#include "debug_log.h"
#include "app_config.h"
#include "live_stream.h"
#include <esp_partition.h>
#include <esp_ota_ops.h>

#define FLASH_TASK_STACK    8192
#define FLASH_TASK_PRIO     3
#define CHUNK_SIZE          250
#define BID_OFFSET_SX577    0x3F80
#define FULL_IMAGE_SIZE     0x100000
#define BOOTLOADER_CATCH_TEST_ONLY 1

const esp_partition_t* ResmedOta::get_staging_partition() {
    const esp_partition_t *p = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "resmed");
    if (p) return p;
    const esp_partition_t *running = esp_ota_get_running_partition();
    return esp_ota_get_next_update_partition(running);
}

static bool verify_block_crc(const esp_partition_t *part, size_t offset, size_t size) {
    uint8_t buf[128];
    uint16_t crc = 0xFFFF;
    size_t remaining = size;
    size_t pos = 0;
    while (remaining > 0) {
        size_t chunk = (remaining > sizeof(buf)) ? sizeof(buf) : remaining;
        if (esp_partition_read(part, offset + pos, buf, chunk) != ESP_OK) return false;
        crc = crc16_ccitt(buf, chunk, crc);
        pos += chunk;
        remaining -= chunk;
    }
    return crc == 0;
}

static bool check_bytes(const esp_partition_t *part, size_t offset,
                        const uint8_t *expected, size_t len) {
    uint8_t buf[16];
    if (len > sizeof(buf)) return false;
    if (esp_partition_read(part, offset, buf, len) != ESP_OK) return false;
    return memcmp(buf, expected, len) == 0;
}

fw_verify_result_t ResmedOta::verify_image(const esp_partition_t *part, size_t fw_size) {
    fw_verify_result_t r = {};
    const size_t BLX_OFF = 0x00000, BLX_SIZE = 0x04000;
    const size_t CCX_OFF = 0x04000, CCX_SIZE = 0x3C000;
    const size_t CDX_OFF = 0x40000, CDX_SIZE = 0xC0000;

    r.has_blx = (fw_size >= BLX_OFF + BLX_SIZE);
    r.has_ccx = (fw_size >= CCX_OFF + CCX_SIZE);
    r.has_cdx = (fw_size >= CDX_OFF + CDX_SIZE);

    if (r.has_blx) {
        memset(r.bid, 0, sizeof(r.bid));
        esp_partition_read(part, BID_OFFSET_SX577, r.bid, sizeof(r.bid) - 1);
        for (int i = 0; i < (int)sizeof(r.bid) - 1; i++) {
            if (r.bid[i] < 0x20 || r.bid[i] > 0x7E) { r.bid[i] = '\0'; break; }
        }
        r.bid_ok = (strncmp(r.bid, "SX577-0200", 10) == 0);
    }

    if (r.has_blx) r.blx_crc_ok = verify_block_crc(part, BLX_OFF, BLX_SIZE);
    if (r.has_ccx) r.ccx_crc_ok = verify_block_crc(part, CCX_OFF, CCX_SIZE);
    if (r.has_cdx) r.cdx_crc_ok = verify_block_crc(part, CDX_OFF, CDX_SIZE);

    r.blx_patch = BLX_PATCH_NONE;
    if (r.has_blx) {
        const uint8_t patch_a[] = {0xC0, 0x46};
        if (check_bytes(part, 0xF0, patch_a, sizeof(patch_a))) {
            r.blx_patch = BLX_PATCH_A_DANGEROUS;
        }
        const uint8_t patch_b1[] = {0x01, 0x20, 0xC0, 0x46};
        const uint8_t patch_b2[] = {0x00, 0x20, 0xC0, 0x46};
        if (check_bytes(part, 0x310E, patch_b1, sizeof(patch_b1)) &&
            check_bytes(part, 0x313E, patch_b2, sizeof(patch_b2)) &&
            check_bytes(part, 0x3130, patch_b2, sizeof(patch_b2))) {
            r.blx_patch = BLX_PATCH_B_SAFE;
        }
    }
    return r;
}

struct block_info_t {
    const char *name;
    uint32_t base_addr;
    uint32_t max_size;
    uint32_t file_offset;
};

static const block_info_t BLOCKS_SX577[] = {
    {"BLX", 0x08000000, 0x04000,  0x00000},
    {"CCX", 0x08004000, 0x3C000,  0x04000},
    {"CDX", 0x08040000, 0xC0000,  0x40000},
    {"CMX", 0x08004000, 0xFC000,  0x04000},
};
#define BLOCK_COUNT 4

static volatile bool flash_active = false;
static volatile bool flash_cancel = false;
static volatile size_t flash_sent = 0;
static volatile size_t flash_total = 0;
static char flash_phase[32] = "";
static char flash_error[128] = "";
static TaskHandle_t flash_task_handle = nullptr;

struct flash_params_t {
    char block[8];
    size_t fw_size;
    bool flash_blx;
    bool force_blx;
};
static flash_params_t flash_params;

static const block_info_t* find_block(const char *name) {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (strcmp(BLOCKS_SX577[i].name, name) == 0) return &BLOCKS_SX577[i];
    }
    return nullptr;
}

static bool is_full_upload(const flash_params_t *p) {
    return p && p->fw_size == FULL_IMAGE_SIZE;
}

static size_t block_staging_offset(const flash_params_t *p, const block_info_t *block) {
    return (is_full_upload(p) && block) ? block->file_offset : 0;
}

static size_t block_data_size(const flash_params_t *p, const block_info_t *block) {
    return (is_full_upload(p) && block) ? block->max_size : (p ? p->fw_size : 0);
}

static int build_record_03(uint8_t *out, size_t out_size,
                           uint32_t addr, const uint8_t *data, size_t data_len) {
    size_t rec_len = 2 + 4 + data_len + 1;
    if (out_size < rec_len) return -1;
    uint8_t payload_len = 4 + data_len + 1;
    out[0] = 0x03;
    out[1] = payload_len;
    out[2] = (addr >> 24) & 0xFF;
    out[3] = (addr >> 16) & 0xFF;
    out[4] = (addr >> 8) & 0xFF;
    out[5] = addr & 0xFF;
    memcpy(out + 6, data, data_len);
    out[6 + data_len] = 0x00;
    return (int)rec_len;
}

static int build_f_payload(uint8_t *out, size_t out_size,
                           const char *block_name, uint8_t seq,
                           const uint8_t *record, size_t record_len,
                           bool is_completion) {
    size_t name_len = strlen(block_name);
    size_t total = name_len + 1 + 1 + (is_completion ? 0 : record_len);
    if (out_size < total) return -1;
    memcpy(out, block_name, name_len);
    out[name_len] = is_completion ? 'F' : 0x00;
    out[name_len + 1] = seq;
    if (!is_completion && record && record_len > 0) {
        memcpy(out + name_len + 2, record, record_len);
    }
    return is_completion ? (int)(name_len + 2) : (int)(name_len + 2 + record_len);
}

static bool send_raw_cmd(const char *cmd, char *resp, uint16_t resp_size,
                         uint16_t timeout_ms = 2000) {
    uint8_t frame[QFRAME_MAX_RAW];
    int frame_len = qframe_build_cmd(cmd, frame, sizeof(frame));
    if (frame_len < 0) return false;
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(frame, frame_len);

    qframe_t rx;
    if (Arbiter::wait_frame(&rx, timeout_ms)) {
        if (resp && resp_size > 0) {
            uint16_t copy = min((uint16_t)rx.payload_len, (uint16_t)(resp_size - 1));
            memcpy(resp, rx.payload, copy);
            resp[copy] = '\0';
        }
        return (rx.type == QFRAME_TYPE_R);
    }
    return false;
}

static bool send_bootloader_entry_no_wait() {
    uint8_t frame[QFRAME_MAX_RAW];
    int frame_len = qframe_build_cmd("P S #BLL 0001", frame, sizeof(frame));
    if (frame_len < 0) return false;
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(frame, frame_len);
    vTaskDelay(pdMS_TO_TICKS(50));
    Arbiter::clear_rx_frames();
    return true;
}

static bool send_and_check(const char *cmd, char *resp, uint16_t resp_size,
                           uint16_t timeout_ms = 3000) {
    uint16_t len = resp_size;
    return Arbiter::send_cmd(cmd, CMD_SRC_OTA, CMD_PRIO_CRITICAL,
                             resp, &len, timeout_ms);
}

static bool query_hex_var(const char *name, int *value, uint16_t timeout_ms = 500) {
    char cmd[24];
    char resp[64] = {};
    snprintf(cmd, sizeof(cmd), "G S #%s", name);
    if (!send_raw_cmd(cmd, resp, sizeof(resp), timeout_ms)) return false;
    const char *v = qframe_response_value(resp);
    if (!v) return false;
    if (value) *value = (int)strtol(v, nullptr, 16);
    return true;
}

static bool query_device_bid(char *bid, size_t bid_size, uint16_t timeout_ms = 500) {
    for (int attempt = 0; attempt < 3; attempt++) {
        char resp[64] = {};
        Arbiter::clear_rx_frames();
        if (send_raw_cmd("G S #BID", resp, sizeof(resp), timeout_ms)) {
            const char *v = qframe_response_value(resp);
            if (v) {
                strncpy(bid, v, bid_size - 1);
                bid[bid_size - 1] = '\0';
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return false;
}

static bool extract_bls_from_frame(const qframe_t &rx, int *bls) {
    if (rx.type != QFRAME_TYPE_R || rx.payload_len == 0) return false;
    char resp[64] = {};
    uint16_t copy = min((uint16_t)rx.payload_len, (uint16_t)(sizeof(resp) - 1));
    memcpy(resp, rx.payload, copy);
    resp[copy] = '\0';
    if (!strstr(resp, "BLS")) return false;
    const char *v = qframe_response_value(resp);
    if (!v) return false;
    if (bls) *bls = (int)strtol(v, nullptr, 16);
    return true;
}

// Run the timing-critical S10 bootloader catch entirely on the ESP32.
// This intentionally mirrors the old resmed_flash.py S10 flood method, but
// removes PC/TCP/Wi-Fi latency from the critical reboot window.
static bool catch_bootloader_local(int *caught_bls, uint32_t window_ms = 1500) {
    uint8_t bls_frame[64];
    int bls_len = qframe_build_cmd("G S #BLS", bls_frame, sizeof(bls_frame));
    if (bls_len <= 0) return false;

    uint8_t preamble[128];
    memset(preamble, 0x55, sizeof(preamble));
    Arbiter::clear_rx_frames();

    const uint32_t start = millis();
    int probes = 0;
    int last_bls = -1;

    while ((millis() - start) < window_ms && !flash_cancel) {
        Arbiter::write_raw(preamble, sizeof(preamble));
        Arbiter::write_raw(bls_frame, bls_len);
        probes++;

        const uint32_t read_until = millis() + 35;
        while ((int32_t)(read_until - millis()) > 0) {
            qframe_t rx;
            if (!Arbiter::wait_frame(&rx, 5)) continue;
            int bls = -1;
            if (extract_bls_from_frame(rx, &bls)) {
                last_bls = bls;
                Log::logf(CAT_OTA, LOG_INFO,
                          "[OTA] Local BLS response: %d at +%u ms (probe %d)\n",
                          bls, (unsigned)(millis() - start), probes);
                if (bls >= 1) {
                    if (caught_bls) *caught_bls = bls;
                    return true;
                }
            }
        }
    }

    if (caught_bls) *caught_bls = last_bls;
    Log::logf(CAT_OTA, LOG_WARN,
              "[OTA] Local bootloader catch expired after %u ms (%d probes, last BLS=%d)\n",
              (unsigned)(millis() - start), probes, last_bls);
    return false;
}

static bool check_bid(const esp_partition_t *part, size_t blx_partition_offset, bool force) {
    if (force) return true;
    strncpy(flash_phase, "BID check", sizeof(flash_phase));

    char img_bid[32] = {};
    esp_partition_read(part, blx_partition_offset + BID_OFFSET_SX577,
                       img_bid, sizeof(img_bid) - 1);
    char dev_bid[32] = {};
    if (!query_device_bid(dev_bid, sizeof(dev_bid))) {
        snprintf(flash_error, sizeof(flash_error), "Failed to read device BID");
        return false;
    }
    if (strncmp(img_bid, dev_bid, 20) != 0) {
        snprintf(flash_error, sizeof(flash_error),
                 "BID mismatch: image=%.20s device=%.20s", img_bid, dev_bid);
        return false;
    }
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] BID check passed\n");
    return true;
}

static void sync_uart() {
    uint8_t preamble[128];
    memset(preamble, 0x55, sizeof(preamble));
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(preamble, sizeof(preamble));
    vTaskDelay(pdMS_TO_TICKS(50));
    Arbiter::clear_rx_frames();
}

static const char* bdd_code_for(uint32_t baud) {
    switch (baud) {
        case 57600:  return "0000";
        case 115200: return "0001";
        case 460800: return "0002";
        default: return nullptr;
    }
}

static bool switch_baud_verified(uint32_t target, bool quiet = false) {
    uint32_t old = Arbiter::get_baud();
    if (old == target) return true;
    const char *code = bdd_code_for(target);
    if (!code) return false;

    if (!quiet) Log::logf(CAT_OTA, LOG_INFO, "[OTA] Switching baud %u -> %u...\n", old, target);

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "P S #BDD %s", code);
    uint8_t frame[64];
    int frame_len = qframe_build_cmd(cmd, frame, sizeof(frame));
    if (frame_len <= 0) return false;

    Arbiter::clear_rx_frames();
    Arbiter::write_raw(frame, frame_len);
    vTaskDelay(pdMS_TO_TICKS(300));
    qframe_t ack;
    Arbiter::wait_frame(&ack, 500);

    Arbiter::set_baud(target);
    sync_uart();

    char bid[32] = {};
    if (query_device_bid(bid, sizeof(bid), 500)) {
        if (!quiet) Log::logf(CAT_OTA, LOG_INFO, "[OTA] Baud verified at %u (BID=%s)\n", target, bid);
        return true;
    }

    if (!quiet) Log::logf(CAT_OTA, LOG_WARN, "[OTA] No BID response at %u; reverting to %u\n", target, old);
    Arbiter::set_baud(old);
    sync_uart();
    return false;
}

static bool negotiate_best_baud() {
    const uint32_t rates[] = {460800, 115200, 57600};
    for (uint32_t rate : rates) {
        if (Arbiter::get_baud() == rate) return true;
        if (switch_baud_verified(rate)) return true;
    }
    snprintf(flash_error, sizeof(flash_error), "Failed to negotiate a verified UART baud");
    return false;
}

static bool enter_bootloader(bool send_bll = true) {
    strncpy(flash_phase, "Enter bootloader", sizeof(flash_phase));
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Entering bootloader (bll=%s)...\n", send_bll ? "yes" : "no");

    int bls = -1;
    if (query_hex_var("BLS", &bls, 300) && bls >= 1) {
        char bid[32] = {};
        if (query_device_bid(bid, sizeof(bid), 500)) {
            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Already in bootloader (BLS=%d, BID=%s)\n", bls, bid);
            return true;
        }
    }

    if (!send_bll) {
        snprintf(flash_error, sizeof(flash_error), "Bootloader not already active");
        return false;
    }

    for (int attempt = 0; attempt < 3 && !flash_cancel; attempt++) {
        Log::logf(CAT_OTA, LOG_INFO,
                  "[OTA] Local bootloader catch attempt %d/3: sending BLL...\n",
                  attempt + 1);
        if (!send_bootloader_entry_no_wait()) {
            snprintf(flash_error, sizeof(flash_error), "Failed to send BLL command");
            return false;
        }

        int caught_bls = -1;
        if (catch_bootloader_local(&caught_bls, 1500)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            char bid[32] = {};
            if (query_device_bid(bid, sizeof(bid), 500)) {
                Log::logf(CAT_OTA, LOG_INFO,
                          "[OTA] Bootloader caught locally (BLS=%d, BID=%s)\n",
                          caught_bls, bid);
                return true;
            }
            Log::logf(CAT_OTA, LOG_WARN,
                      "[OTA] Local BLS caught bootloader but BID query failed\n");
        }

        if (attempt < 2) vTaskDelay(pdMS_TO_TICKS(300));
    }

    snprintf(flash_error, sizeof(flash_error), "Failed to catch bootloader locally");
    return false;
}

static bool wait_for_erase(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    int p_count = 0;
    while (millis() - t0 < timeout_ms && !flash_cancel) {
        qframe_t rx;
        if (Arbiter::wait_frame(&rx, 500)) {
            if (rx.type == QFRAME_TYPE_P) {
                p_count++;
            } else if (rx.type == QFRAME_TYPE_R) {
                Log::logf(CAT_OTA, LOG_INFO, "[OTA] Erase done (%d ACKs)\n", p_count);
                return true;
            } else if (rx.type == QFRAME_TYPE_E) {
                char err[32] = {};
                memcpy(err, rx.payload, min((int)rx.payload_len, 31));
                snprintf(flash_error, sizeof(flash_error), "Erase error: %s", err);
                return false;
            }
        }
    }
    snprintf(flash_error, sizeof(flash_error), "Erase timeout (%d ACKs received)", p_count);
    return false;
}

static bool check_flash_status() {
    int ble = -1;
    if (!query_hex_var("BLE", &ble, 1000)) {
        snprintf(flash_error, sizeof(flash_error), "No BLE response after completion");
        return false;
    }
    if (ble != 0) {
        snprintf(flash_error, sizeof(flash_error), "Flash completion failed: BLE=%04X", ble);
        return false;
    }
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Completion confirmed: BLE=0000\n");
    return true;
}

static bool validate_flash_input(const esp_partition_t *part, const flash_params_t *p) {
    strncpy(flash_phase, "Validate image", sizeof(flash_phase));

    char dev_bid[32] = {};
    if (!query_device_bid(dev_bid, sizeof(dev_bid), 700)) {
        snprintf(flash_error, sizeof(flash_error), "Unable to read device BID before flash");
        return false;
    }
    if (strncmp(dev_bid, "SX577-0200", 10) != 0) {
        snprintf(flash_error, sizeof(flash_error), "Unsupported device BID: %.20s", dev_bid);
        return false;
    }

    const bool full_upload = is_full_upload(p);
    const bool target_full = strcmp(p->block, "FULL") == 0;

    if (full_upload) {
        char img_bid[32] = {};
        esp_partition_read(part, BID_OFFSET_SX577, img_bid, sizeof(img_bid) - 1);
        if (strncmp(img_bid, dev_bid, 20) != 0) {
            snprintf(flash_error, sizeof(flash_error),
                     "Image/device BID mismatch: %.20s / %.20s", img_bid, dev_bid);
            return false;
        }
    }

    if (target_full) {
        if (!full_upload) {
            snprintf(flash_error, sizeof(flash_error), "FULL target requires exactly %u-byte full image", FULL_IMAGE_SIZE);
            return false;
        }
        if (!verify_block_crc(part, 0x04000, 0x3C000) ||
            !verify_block_crc(part, 0x40000, 0xC0000)) {
            snprintf(flash_error, sizeof(flash_error), "FULL image CCX/CDX CRC validation failed");
            return false;
        }
        if (p->flash_blx && !p->force_blx && !verify_block_crc(part, 0, 0x4000)) {
            snprintf(flash_error, sizeof(flash_error), "BLX CRC validation failed");
            return false;
        }
        return true;
    }

    const block_info_t *block = find_block(p->block);
    if (!block) {
        snprintf(flash_error, sizeof(flash_error), "Unknown block: %s", p->block);
        return false;
    }

    size_t part_offset = block_staging_offset(p, block);
    size_t data_size = block_data_size(p, block);

    if (!full_upload && data_size != block->max_size) {
        snprintf(flash_error, sizeof(flash_error),
                 "%s image must be exactly %u bytes (got %u)",
                 block->name, block->max_size, p->fw_size);
        return false;
    }

    if (strcmp(block->name, "CMX") == 0) {
        const size_t ccx_off = part_offset;
        const size_t cdx_off = part_offset + 0x3C000;
        if (!verify_block_crc(part, ccx_off, 0x3C000) ||
            !verify_block_crc(part, cdx_off, 0xC0000)) {
            snprintf(flash_error, sizeof(flash_error), "CMX CCX/CDX CRC validation failed");
            return false;
        }
    } else if (!(strcmp(block->name, "BLX") == 0 && p->force_blx)) {
        if (!verify_block_crc(part, part_offset, block->max_size)) {
            snprintf(flash_error, sizeof(flash_error), "%s CRC validation failed", block->name);
            return false;
        }
    }

    if (strcmp(block->name, "BLX") == 0 && !p->force_blx) {
        if (!check_bid(part, part_offset, false)) return false;
    }

    Log::logf(CAT_OTA, LOG_INFO,
              "[OTA] Flash plan: source=%s target=%s staging_offset=0x%X size=%u\n",
              full_upload ? "FULL" : "BLOCK", block->name,
              (unsigned)part_offset, (unsigned)data_size);
    return true;
}

static bool flash_one_block(const esp_partition_t *part, size_t part_offset,
                            const block_info_t *block, size_t data_size,
                            bool send_completion = true) {
    size_t trimmed_size = data_size;
    {
        uint8_t tail[256];
        while (trimmed_size > 0) {
            size_t check_len = min(trimmed_size, sizeof(tail));
            size_t check_off = trimmed_size - check_len;
            esp_partition_read(part, part_offset + check_off, tail, check_len);
            bool all_ff = true;
            for (int i = check_len - 1; i >= 0; i--) {
                if (tail[i] != 0xFF) {
                    trimmed_size = check_off + i + 1;
                    all_ff = false;
                    break;
                }
            }
            if (!all_ff) break;
            trimmed_size = check_off;
        }
    }
    trimmed_size = (trimmed_size + 3) & ~3;
    if (trimmed_size == 0) {
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s data is all 0xFF, skipping\n", block->name);
        return true;
    }

    Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s: %u bytes (trimmed from %u)\n",
              block->name, trimmed_size, data_size);

    char erase_cmd[24];
    snprintf(erase_cmd, sizeof(erase_cmd), "P F *%s 0000", block->name);
    snprintf(flash_phase, sizeof(flash_phase), "Erase %s", block->name);

    bool erased = false;
    for (int attempt = 0; attempt < 3 && !flash_cancel; attempt++) {
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] Erasing %s (attempt %d/3)...\n", block->name, attempt + 1);
        uint8_t frame[64];
        int frame_len = qframe_build_cmd(erase_cmd, frame, sizeof(frame));
        if (frame_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "Erase frame build error");
            return false;
        }
        Arbiter::clear_rx_frames();
        Arbiter::write_raw(frame, frame_len);
        if (wait_for_erase(30000)) {
            erased = true;
            break;
        }
        if (attempt < 2) {
            Log::logf(CAT_OTA, LOG_WARN, "[OTA] Erase stalled, retrying in 1s...\n");
            flash_error[0] = '\0';
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    if (!erased) return false;

    vTaskDelay(pdMS_TO_TICKS(300));

    snprintf(flash_phase, sizeof(flash_phase), "Flash %s", block->name);
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Writing %u bytes to %s @ %u baud...\n",
              trimmed_size, block->name, Arbiter::get_baud());

    uint8_t seq = 0;
    size_t offset = 0;
    int frame_count = 0;

    while (offset < trimmed_size && !flash_cancel) {
        size_t chunk_len = min((size_t)CHUNK_SIZE, trimmed_size - offset);
        uint8_t chunk_buf[CHUNK_SIZE];
        esp_err_t err = esp_partition_read(part, part_offset + offset, chunk_buf, chunk_len);
        if (err != ESP_OK) {
            snprintf(flash_error, sizeof(flash_error), "Read error at %u: %s", offset, esp_err_to_name(err));
            return false;
        }

        uint8_t record[CHUNK_SIZE + 8];
        uint32_t addr = block->base_addr + offset;
        int rec_len = build_record_03(record, sizeof(record), addr, chunk_buf, chunk_len);
        if (rec_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "Record build error at %u", offset);
            return false;
        }

        uint8_t f_payload[CHUNK_SIZE + 16];
        int f_len = build_f_payload(f_payload, sizeof(f_payload), block->name, seq, record, rec_len, false);
        if (f_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "F-payload build error");
            return false;
        }

        uint8_t frame[QFRAME_MAX_RAW];
        int frame_len = qframe_build('f', f_payload, f_len, frame, sizeof(frame));
        if (frame_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "F-frame build error");
            return false;
        }

        Arbiter::write_raw(frame, frame_len);
        frame_count++;
        offset += chunk_len;
        flash_sent += chunk_len;
        seq = (seq + 1) & 0xFF;

        if (frame_count % 20 == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            qframe_t rx;
            while (Arbiter::wait_frame(&rx, 10)) {
                if (rx.type == QFRAME_TYPE_E) {
                    char err_str[32] = {};
                    memcpy(err_str, rx.payload, min((int)rx.payload_len, 31));
                    snprintf(flash_error, sizeof(flash_error),
                             "Flash error at %s+0x%X: %s", block->name, offset, err_str);
                    return false;
                }
            }
        }
    }

    if (flash_cancel) return false;

    if (send_completion) {
        snprintf(flash_phase, sizeof(flash_phase), "Complete %s", block->name);
        Log::logf(CAT_OTA, LOG_INFO,
                  "[OTA] Sending completion frame for %s (%d frames sent)...\n",
                  block->name, frame_count);
        uint8_t f_payload[8];
        int f_len = build_f_payload(f_payload, sizeof(f_payload), block->name, seq, nullptr, 0, true);
        uint8_t frame[64];
        int frame_len = qframe_build('f', f_payload, f_len, frame, sizeof(frame));
        if (frame_len <= 0) {
            snprintf(flash_error, sizeof(flash_error), "Completion frame build error");
            return false;
        }
        Arbiter::write_raw(frame, frame_len);
        if (!check_flash_status()) return false;
    } else {
        Log::logf(CAT_OTA, LOG_INFO,
                  "[OTA] %s data sent (%d frames), skipping completion (chaining)\n",
                  block->name, frame_count);
    }

    Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s flash done\n", block->name);
    return true;
}

static bool wait_for_application() {
    uint32_t deadline = millis() + 15000;
    bool extended = false;
    vTaskDelay(pdMS_TO_TICKS(3000));

    while ((int32_t)(deadline - millis()) > 0 && !flash_cancel) {
        int bls = -1;
        if (query_hex_var("BLS", &bls, 300)) {
            if (bls == 0) return true;

            int ble = -1;
            bool have_ble = query_hex_var("BLE", &ble, 500);
            if ((have_ble && ble != 0) || bls >= 2) {
                snprintf(flash_error, sizeof(flash_error),
                         "Application did not start: BLS=%04X%s",
                         bls, have_ble ? ", BLE nonzero" : "");
                return false;
            }
            if (have_ble && ble == 0 && !extended) {
                Log::logf(CAT_OTA, LOG_INFO,
                          "[OTA] Bootloader BLE=0000; allowing 5s more for application startup...\n");
                deadline += 5000;
                extended = true;
            }
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    snprintf(flash_error, sizeof(flash_error), "Device did not return to app after flash");
    return false;
}

static void flash_task(void *param) {
    flash_params_t *p = (flash_params_t*)param;
    bool target_full = (strcmp(p->block, "FULL") == 0);

    flash_active = true;
    flash_cancel = false;
    flash_sent = 0;
    flash_error[0] = '\0';

    if (target_full) {
        const block_info_t *cmx = find_block("CMX");
        flash_total = cmx->max_size;
        if (p->flash_blx) flash_total += find_block("BLX")->max_size;
    } else {
        const block_info_t *block = find_block(p->block);
        flash_total = block ? block_data_size(p, block) : p->fw_size;
    }

    const esp_partition_t *part = ResmedOta::get_staging_partition();
    if (!part) {
        snprintf(flash_error, sizeof(flash_error), "No staging partition found");
        goto done;
    }
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Using partition '%s' (0x%X, %u bytes)\n",
              part->label, part->address, part->size);

    strncpy(flash_phase, "Suspend streams", sizeof(flash_phase));
    if (!LiveStream::suspend()) {
        snprintf(flash_error, sizeof(flash_error), "Failed to stop live streams");
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    Arbiter::clear_rx_frames();

    strncpy(flash_phase, "Claim UART", sizeof(flash_phase));
    if (!Arbiter::wait_idle(3000)) {
        snprintf(flash_error, sizeof(flash_error), "UART busy before OTA");
        goto cleanup;
    }
    Arbiter::set_state(SYS_OTA_AIRSENSE);

    Arbiter::set_baud(57600);
    vTaskDelay(pdMS_TO_TICKS(50));
    Arbiter::clear_rx_frames();

    if (!validate_flash_input(part, p)) goto cleanup;

    if (!enter_bootloader()) goto cleanup;
    if (flash_cancel) goto cleanup;

#if BOOTLOADER_CATCH_TEST_ONLY
    Log::logf(CAT_OTA, LOG_INFO,
              "[OTA] TEST ONLY: bootloader catch succeeded; NO ERASE OR WRITE will be attempted\n");
    strncpy(flash_phase, "BL test passed", sizeof(flash_phase));
    {
        char resp[48] = {};
        send_raw_cmd("P S #RES 0001", resp, sizeof(resp), 2000);
    }
    if (!wait_for_application()) goto cleanup;
    Log::logf(CAT_OTA, LOG_INFO,
              "[OTA] TEST ONLY complete: application responding again\n");
    goto cleanup;
#endif

    strncpy(flash_phase, "Baud negotiate", sizeof(flash_phase));
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Negotiating best baud...\n");
    if (!negotiate_best_baud()) goto cleanup;

    if (target_full) {
        if (p->flash_blx) {
            const block_info_t *blx = find_block("BLX");
            if (!flash_one_block(part, 0, blx, blx->max_size, false)) goto cleanup;

            strncpy(flash_phase, "BLX mode timeout", sizeof(flash_phase));
            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Waiting for mode 5 timeout (~2s)...\n");
            vTaskDelay(pdMS_TO_TICKS(2500));
            Arbiter::set_baud(57600);
            sync_uart();

            if (!enter_bootloader(true)) {
                snprintf(flash_error, sizeof(flash_error), "Lost bootloader after BLX flash");
                goto cleanup;
            }

            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Bootloader confirmed after BLX\n");
            if (!negotiate_best_baud()) goto cleanup;
        }

        const block_info_t *cmx = find_block("CMX");
        if (!flash_one_block(part, cmx->file_offset, cmx, cmx->max_size, true)) goto cleanup;
    } else {
        const block_info_t *block = find_block(p->block);
        if (!block) {
            snprintf(flash_error, sizeof(flash_error), "Unknown block: %s", p->block);
            goto cleanup;
        }
        const size_t part_offset = block_staging_offset(p, block);
        const size_t data_size = block_data_size(p, block);
        if (!flash_one_block(part, part_offset, block, data_size, true)) goto cleanup;
    }

    strncpy(flash_phase, "Reset device", sizeof(flash_phase));
    if (Arbiter::get_baud() != 57600) {
        if (!switch_baud_verified(57600)) {
            Arbiter::set_baud(57600);
            sync_uart();
        }
    }

    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Resetting AirSense...\n");
    {
        char resp[48] = {};
        send_raw_cmd("P S #RES 0001", resp, sizeof(resp), 2000);
    }

    strncpy(flash_phase, "Verifying", sizeof(flash_phase));
    if (!wait_for_application()) goto cleanup;

    strncpy(flash_phase, "Complete", sizeof(flash_phase));
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Flash complete, device running (BLS=0000)\n");
    Config::invalidate_device_info();

cleanup:
    Arbiter::set_baud(57600);
    Arbiter::set_state(SYS_IDLE);
    if (flash_cancel && flash_error[0] == '\0') {
        strncpy(flash_error, "Cancelled by user", sizeof(flash_error));
        strncpy(flash_phase, "Cancelled", sizeof(flash_phase));
    }

done:
    if (flash_error[0] != '\0') {
        strncpy(flash_phase, "Error", sizeof(flash_phase));
        Log::logf(CAT_OTA, LOG_ERROR, "[OTA] Error: %s\n", flash_error);
    }
    flash_active = false;
    flash_task_handle = nullptr;
    vTaskDelete(nullptr);
}

const char* ResmedOta::detect_block(size_t fw_size) {
    if (fw_size == 0x04000)  return "BLX";
    if (fw_size == 0x3C000)  return "CCX";
    if (fw_size == 0xC0000)  return "CDX";
    if (fw_size == 0xFC000)  return "CMX";
    if (fw_size == FULL_IMAGE_SIZE) return "FULL";
    return nullptr;
}

void ResmedOta::start_flash(const char *block, size_t fw_size,
                            bool flash_blx, bool force_blx) {
    if (flash_active) return;

    if (!block || block[0] == '\0') {
        block = detect_block(fw_size);
        if (!block) {
            strncpy(flash_error, "Cannot detect block: file size must exactly match BLX/CCX/CDX/CMX/FULL", sizeof(flash_error));
            return;
        }
    }

    strncpy(flash_params.block, block, sizeof(flash_params.block) - 1);
    flash_params.block[sizeof(flash_params.block) - 1] = '\0';
    flash_params.fw_size = fw_size;
    flash_params.flash_blx = flash_blx;
    flash_params.force_blx = force_blx;

    xTaskCreatePinnedToCore(flash_task, "resmed_ota", FLASH_TASK_STACK,
                            &flash_params, FLASH_TASK_PRIO, &flash_task_handle, AB_IO_TASK_CORE);
}

void ResmedOta::cancel()            { flash_cancel = true; }
bool ResmedOta::is_active()         { return flash_active; }
const char* ResmedOta::get_phase()  { return flash_phase; }
size_t ResmedOta::get_sent()        { return flash_sent; }
size_t ResmedOta::get_total()       { return flash_total; }
const char* ResmedOta::last_error() { return flash_error; }
