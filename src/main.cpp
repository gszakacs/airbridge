#include <Arduino.h>
#include <atomic>
#include "app_config.h"
#include "debug_log.h"
#include "uart_arbiter.h"
#include "tcp_bridge.h"
#include "wifi_setup.h"
#include "oxi_ble.h"
#include "oxi_udp.h"
#include "oxi_arbiter.h"
#include "airbridge_ota.h"
#include "web_ui.h"
#include "qframe.h"
#include "network_hints.h"
#include "live_stream.h"
#include "live_web_consumer.h"

const char *airbridge_version() { return AIRBRIDGE_VERSION; }
const char *airbridge_build_date() { return AIRBRIDGE_BUILD_DATE; }

extern void dispatch_command(const char *line, String &response);

#define SERIAL_LINE_MAX 256
static char serial_line[SERIAL_LINE_MAX];
static int serial_pos = 0;

static void serial_poll() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serial_pos > 0) {
                serial_line[serial_pos] = '\0';

                if (serial_line[0] == '$') {
                    // Internal command
                    String response;
                    dispatch_command(serial_line + 1, response);
                    if (response.length() > 0) Serial.print(response);
                } else {
                    // Q-frame
                    char resp_buf[512] = {};
                    uint16_t resp_len = sizeof(resp_buf);
                    bool ok = Arbiter::send_cmd(serial_line, CMD_SRC_INTERNAL,
                                                CMD_PRIO_NORMAL, resp_buf,
                                                &resp_len, 2000);
                    if (ok) {
                        Serial.println(resp_buf);
                    } else {
                        Serial.println("ERR:TIMEOUT");
                    }
                }
                serial_pos = 0;
            }
        } else if (serial_pos < SERIAL_LINE_MAX - 1) {
            serial_line[serial_pos++] = c;
        }
    }
}

// Health monitoring:
//   ROP every 10s to detect therapy state
//   MHR every 30 min and once on therapy stop
#define HEALTH_POLL_INTERVAL_MS     10000
#define HEALTH_TIMEOUT_MS           500
#define MHR_POLL_INTERVAL_MS        (30UL * 60 * 1000)

static uint32_t last_health_poll = 0;
static uint32_t last_mhr_poll = 0;
static uint32_t consecutive_timeouts = 0;
static bool airsense_present = false;
static uint32_t airsense_seen_ms = 0;
static std::atomic<bool> clock_sync_pending{true};
static uint32_t clock_sync_attempt_ms = 0;
static std::atomic<bool> clock_sync_attempted{false};

static void poll_mhr() {
    char mhr_resp[32] = {};
    uint16_t mhr_len = sizeof(mhr_resp);
    if (!Arbiter::send_cmd("G S #MHR", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                           mhr_resp, &mhr_len)) {
        // UART unhappy; leave cache alone and retry next opportunity.
        return;
    }
    const char *mv = qframe_response_value(mhr_resp);
    int new_mhr = mv ? (int)strtol(mv, nullptr, 16) : -1;
    int prev_mhr = Arbiter::get_cached_mhr();
    Arbiter::set_cached_mhr(new_mhr);
    last_mhr_poll = millis();
    if (new_mhr != prev_mhr) {
        WebUI::push_status_event();
    }
}

static bool mhr_poll_due() {
    if (Arbiter::get_cached_mhr() < 0) return true;
    return millis() - last_mhr_poll >= MHR_POLL_INTERVAL_MS;
}

static void poll_therapy_state() {
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);

    uint32_t t0 = millis();
    Log::logf(CAT_HEALTH, LOG_DEBUG, "[HEALTH] ROP poll start t=%lu\n", t0);
    bool ok = Arbiter::send_cmd("G S #ROP", CMD_SRC_INTERNAL, CMD_PRIO_HIGH,
                                resp, &resp_len, HEALTH_TIMEOUT_MS);
    if (ok) {
        consecutive_timeouts = 0;

        const char *rv = qframe_response_value(resp);
        airsense_present = rv && (strcmp(rv, "0000") == 0 || strcmp(rv, "0001") == 0);
        if (airsense_present) {
            airsense_seen_ms = millis();
            int new_rop = (int)strtoul(rv, nullptr, 16);
            int prev_rop = Arbiter::get_cached_rop();
            Arbiter::set_cached_rop(new_rop);

            system_state_t current = Arbiter::get_state();
            if (new_rop == 1 && current == SYS_IDLE) {
                Arbiter::set_state(SYS_THERAPY);
                Log::logf(CAT_HEALTH, LOG_INFO, "[HEALTH] Therapy started\n");
            } else if (new_rop == 0 && current == SYS_THERAPY) {
                Arbiter::set_state(SYS_IDLE);
                Log::logf(CAT_HEALTH, LOG_INFO, "[HEALTH] Therapy ended\n");
                poll_mhr();
            }

            if (new_rop != prev_rop) {
                WebUI::push_status_event();
            }
        }
    } else {
        airsense_present = false;
        consecutive_timeouts++;
        Log::logf(CAT_HEALTH, consecutive_timeouts >= 2 ? LOG_WARN : LOG_DEBUG,
                  "[HEALTH] ROP poll timeout (%d consecutive) t=%lu dt=%lu\n",
                  consecutive_timeouts, millis(), millis() - t0);

        if (consecutive_timeouts >= 3) {
            system_state_t current = Arbiter::get_state();
            if (current != SYS_ERROR && current != SYS_TRANSPARENT &&
                current != SYS_OTA_AIRSENSE && current != SYS_OTA_ESP) {
                Arbiter::set_state(SYS_ERROR);
                Log::logf(CAT_HEALTH, LOG_ERROR, "[HEALTH] UART unresponsive, entering ERROR state\n");
            }
        }
    }
}

static void attempt_recovery() {
    if (Arbiter::get_state() != SYS_ERROR) return;

    char resp[32] = {};
    uint16_t resp_len = sizeof(resp);

    bool ok = Arbiter::send_cmd("G S #BLS", CMD_SRC_INTERNAL, CMD_PRIO_HIGH,
                                resp, &resp_len);
    if (ok) {
        Log::logf(CAT_HEALTH, LOG_INFO, "[HEALTH] Device responded, clearing error\n");
        consecutive_timeouts = 0;
        Arbiter::set_state(SYS_IDLE);
        Config::invalidate_device_info();
        // AirSense may have rebooted; force re-subscribe regardless of
        // the broker's stale subscribed flags.
        LiveStream::reattach();
    }
}

bool pull_time_from_resmed(bool force = false);

void setup() {
    Serial.begin(115200);
    delay(500);
    while (Serial.available()) Serial.read();  // flush boot garbage
    Log::init();

    Log::printf("\n=== AirBridge " AIRBRIDGE_VERSION " ===\n");
    Log::printf("Chip: %s, Heap: %d bytes\n", ESP.getChipModel(), ESP.getFreeHeap());

    Config::init();
    // NetworkHints must come up before Config::load runs the wnet migration,
    // because that step calls NetworkHints::upsert with legacy hint values.
    NetworkHints::init();
    Config::load();
    Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] Config loaded\n");

    Arbiter::init(Serial1, PIN_AS10_RX, PIN_AS10_TX, Config::get().uart_baud);
    Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] UART arbiter started\n");

    bool wifi_ok = WiFiSetup::init();

    TcpBridge::init();
    Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] TCP bridge started\n");

    if (wifi_ok) {
        auto &cfg = Config::get();
        if (cfg.debug_port > 0 && cfg.debug_port != cfg.tcp_port)
            TcpBridge::init_debug_server(cfg.debug_port);

        if (cfg.http_port > 0 && cfg.http_port != cfg.tcp_port)
            WebUI::init(cfg.http_port);

        OtaManager::init();
    }

    // If NTP didn't sync, fall back to resmed device clock
    if (!WiFiSetup::time_synced()) pull_time_from_resmed();

    OxiArbiter::init();
    OxiBle::init();
    OxiUdp::init();
    Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] BLE oximetry started\n");

    LiveStream::init();
    LiveWebConsumer::init();

    Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] All systems go\n");
}

void reset_resmed_time_sync() {
    clock_sync_pending = true;
    clock_sync_attempted = false;
}

bool push_time_to_resmed() {
    if (!WiFiSetup::time_synced()) return false;

    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);

    char dac_cmd[32], tic_cmd[32];
    snprintf(dac_cmd, sizeof(dac_cmd), "P S #DAC %02d%02d%04d",
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
    snprintf(tic_cmd, sizeof(tic_cmd), "P S #TIC %02d%02d%02d",
             t.tm_hour, t.tm_min, t.tm_sec);

    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);
    bool ok_dac = Arbiter::send_cmd(dac_cmd, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &resp_len);
    if (!ok_dac) {
        // A device error is terminal until TIMESYNC; a timeout can be retried.
        if (resp[0]) clock_sync_pending = false;
        Log::logf(CAT_GENERAL, LOG_WARN, "[INIT] ResMed date %s: %s\n",
                  resp[0] ? "rejected (use TIMESYNC to retry)" : "timeout", resp);
        return false;
    }
    resp[0] = '\0';
    resp_len = sizeof(resp);
    bool ok_tic = Arbiter::send_cmd(tic_cmd, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &resp_len);
    if (!ok_tic && resp[0]) {
        clock_sync_pending = false;
        Log::logf(CAT_GENERAL, LOG_WARN, "[INIT] ResMed time rejected (use TIMESYNC to retry): %s\n", resp);
        return false;
    }

    if (ok_dac && ok_tic) {
        Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] ResMed clock set: %02d%02d%04d %02d%02d%02d\n",
                  t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                  t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        Log::logf(CAT_GENERAL, LOG_WARN, "[INIT] ResMed clock set failed (dac=%d tic=%d)\n",
                  ok_dac, ok_tic);
    }
    return ok_dac && ok_tic;
}

bool pull_time_from_resmed(bool force) {
    char dac_resp[32] = {}, tic_resp[32] = {};
    uint16_t dac_len = sizeof(dac_resp), tic_len = sizeof(tic_resp);
    Arbiter::send_cmd("G S #DAC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, dac_resp, &dac_len);
    Arbiter::send_cmd("G S #TIC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, tic_resp, &tic_len);
    const char *dv = qframe_response_value(dac_resp);
    const char *tv = qframe_response_value(tic_resp);
    if (dv && tv && strlen(dv) >= 8 && strlen(tv) >= 6) {
        int dd, mm, yyyy, hh, mn, ss;
        if (sscanf(dv, "%2d%2d%4d", &dd, &mm, &yyyy) == 3 &&
            sscanf(tv, "%2d%2d%2d", &hh, &mn, &ss) == 3) {
            if (!WiFiSetup::set_fallback_time(yyyy, mm, dd, hh, mn, ss, force)) return false;
            Log::logf(CAT_GENERAL, LOG_INFO, "[INIT] Time from ResMed: %04d-%02d-%02d %02d:%02d:%02d\n",
                      yyyy, mm, dd, hh, mn, ss);
            return true;
        }
    }
    return false;
}

static void sync_resmed_clock() {
    if (!clock_sync_pending || !airsense_present || !WiFiSetup::time_synced()) return;
    if (millis() - airsense_seen_ms > HEALTH_POLL_INTERVAL_MS) return;
    system_state_t st = Arbiter::get_state();
    if (st != SYS_IDLE && st != SYS_THERAPY) return;
    if (clock_sync_attempted && millis() - clock_sync_attempt_ms < 30000) return;
    clock_sync_attempt_ms = millis();
    clock_sync_attempted = true;
    if (push_time_to_resmed()) clock_sync_pending = false;
}

void loop() {
    serial_poll();

    OtaManager::handle();
    WiFiSetup::check();

    // Suspend WiFi scanning during therapy/streaming/oximetry/OTA
    system_state_t sys_st = Arbiter::get_state();
    bool oxi_active = OxiArbiter::is_feeding();
    bool ota_active = (sys_st == SYS_OTA_AIRSENSE || sys_st == SYS_OTA_ESP);

    if (sys_st == SYS_THERAPY || sys_st == SYS_TRANSPARENT ||
        ota_active || oxi_active) {
        WiFiSetup::suspend_roaming();
    } else {
        WiFiSetup::resume_roaming();
    }

    static bool prev_ota_active = false;
    if (ota_active) {
        OxiBle::suspend();
        if (!prev_ota_active && sys_st == SYS_OTA_ESP) LiveStream::suspend();
    } else {
        OxiBle::resume();
        if (prev_ota_active) LiveStream::resume();
    }
    prev_ota_active = ota_active;

    sync_resmed_clock();

    LiveWebConsumer::tick();

    OxiArbiter::poll();

    // health monitoring
    if (millis() - last_health_poll >= HEALTH_POLL_INTERVAL_MS) {
        last_health_poll = millis();

        system_state_t st = Arbiter::get_state();
        if (st == SYS_IDLE || st == SYS_THERAPY) {
            poll_therapy_state();
            // Catch-up resync if AirSense rebooted out from under us, or
            // any consumer's initial subscribe attempt failed.
            LiveStream::resync();
            // MHR refreshed 30 min cadence and at therapy-stop transition
            if (mhr_poll_due()) poll_mhr();
        } else if (st == SYS_ERROR) {
            attempt_recovery();
        }
    }

    delay(10);
}
