#include <Arduino.h>
#include <WiFi.h>
#include "app_config.h"
#include "wifi_setup.h"

#ifndef AB_WIFI_DEBUG
#define AB_WIFI_DEBUG 0
#endif

// Keep antenna routing active in normal builds. Only the verbose USB Wi-Fi
// diagnostics are controlled by AB_WIFI_DEBUG.
#if defined(AB_BOARD_XIAO_ESP32C6)
#ifndef AB_EXTERNAL_ANTENNA
#define AB_EXTERNAL_ANTENNA 0
#endif

static constexpr uint8_t XIAO_C6_RF_SWITCH_ENABLE_GPIO = 3;
static constexpr uint8_t XIAO_C6_RF_SWITCH_SELECT_GPIO = 14;

static void configure_xiao_c6_antenna() {
    pinMode(XIAO_C6_RF_SWITCH_ENABLE_GPIO, OUTPUT);
    digitalWrite(XIAO_C6_RF_SWITCH_ENABLE_GPIO, LOW);
    delay(10);

    pinMode(XIAO_C6_RF_SWITCH_SELECT_GPIO, OUTPUT);
#if AB_EXTERNAL_ANTENNA
    digitalWrite(XIAO_C6_RF_SWITCH_SELECT_GPIO, HIGH);
#else
    digitalWrite(XIAO_C6_RF_SWITCH_SELECT_GPIO, LOW);
#endif
}

static const char *antenna_name() {
#if AB_EXTERNAL_ANTENNA
    return "external-UFL";
#else
    return "onboard";
#endif
}
#else
static void configure_xiao_c6_antenna() {}
static const char *antenna_name() { return "board-default"; }
#endif

#if AB_WIFI_DEBUG

static const uint32_t WIFI_MONITOR_INTERVAL_MS = 5000;
static const uint32_t WIFI_MONITOR_START_DELAY_MS = 3000;
static uint32_t wifi_monitor_started_ms = 0;
static uint32_t wifi_monitor_last_ms = 0;

static volatile uint8_t wifi_last_disconnect_reason = 0;
static volatile uint32_t wifi_disconnect_count = 0;
static volatile bool wifi_disconnect_pending = false;

static const char *disconnect_reason_name(uint8_t reason) {
    switch (reason) {
        case 2:   return "AUTH_EXPIRE";
        case 3:   return "AUTH_LEAVE";
        case 4:   return "ASSOC_EXPIRE";
        case 5:   return "ASSOC_TOOMANY";
        case 6:   return "NOT_AUTHED";
        case 7:   return "NOT_ASSOCED";
        case 8:   return "ASSOC_LEAVE";
        case 15:  return "4WAY_HANDSHAKE_TIMEOUT";
        case 23:  return "802_1X_AUTH_FAILED";
        case 36:  return "STA_LEAVING";
        case 200: return "BEACON_TIMEOUT";
        case 201: return "NO_AP_FOUND";
        case 202: return "AUTH_FAIL";
        case 203: return "ASSOC_FAIL";
        case 204: return "HANDSHAKE_TIMEOUT";
        case 205: return "CONNECTION_FAIL";
        case 206: return "AP_TSF_RESET";
        case 208: return "ASSOC_COMEBACK_TIME_TOO_LONG";
        default:  return "UNKNOWN";
    }
}

static const char *auth_mode_name(wifi_auth_mode_t auth) {
    switch (auth) {
        case WIFI_AUTH_OPEN:          return "OPEN";
        case WIFI_AUTH_WEP:           return "WEP";
        case WIFI_AUTH_WPA_PSK:       return "WPA-PSK";
        case WIFI_AUTH_WPA2_PSK:      return "WPA2-PSK";
        case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/WPA2-PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE:return "WPA2-ENT";
        default:                      return "OTHER";
    }
}

static void wifi_monitor_event_cb(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        wifi_last_disconnect_reason = info.wifi_sta_disconnected.reason;
        wifi_disconnect_count++;
        wifi_disconnect_pending = true;
    }
}

static const char *mode_name(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_NULL:  return "NULL";
        case WIFI_MODE_STA:   return "STA";
        case WIFI_MODE_AP:    return "AP";
        case WIFI_MODE_APSTA: return "AP+STA";
        default:              return "?";
    }
}

static const char *wl_status_name(wl_status_t status) {
    switch (status) {
        case WL_IDLE_STATUS:     return "idle";
        case WL_NO_SSID_AVAIL:   return "no_ssid";
        case WL_SCAN_COMPLETED:  return "scan_done";
        case WL_CONNECTED:       return "connected";
        case WL_CONNECT_FAILED:  return "connect_failed";
        case WL_CONNECTION_LOST: return "connection_lost";
        case WL_DISCONNECTED:    return "disconnected";
        default:                 return "?";
    }
}

static const char *cfg_mode_name(uint8_t mode) {
    switch (mode) {
        case WIFI_MODE_AUTO:     return "AUTO";
        case WIFI_MODE_AP_ONLY:  return "AP_ONLY";
        case WIFI_MODE_OFF:      return "OFF";
        case WIFI_MODE_STA_ONLY: return "STA_ONLY";
        case WIFI_MODE_STA_AP:   return "STA_AP";
        default:                 return "?";
    }
}

static bool configured_ssid(const String &ssid, uint8_t *idx_out = nullptr) {
    auto &cfg = Config::get();
    for (uint8_t i = 0; i < cfg.wifi_net_count && i < WIFI_MAX_NETWORKS; ++i) {
        if (cfg.wifi_nets[i].enabled && cfg.wifi_nets[i].ssid.equalsIgnoreCase(ssid)) {
            if (idx_out) *idx_out = i;
            return true;
        }
    }
    return false;
}

static void print_known_networks() {
    auto &cfg = Config::get();
    Serial.printf("[WIFI-MON] configured networks: %u\n", cfg.wifi_net_count);
    for (uint8_t i = 0; i < cfg.wifi_net_count && i < WIFI_MAX_NETWORKS; ++i) {
        const WiFiNetwork &net = cfg.wifi_nets[i];
        int8_t rssi = WiFiSetup::net_rssi(i);
        if (rssi != 0) {
            Serial.printf("[WIFI-MON]   [%u] %s enabled=%s last_scan_rssi=%d dBm\n",
                          i, net.ssid.c_str(), net.enabled ? "yes" : "no", rssi);
        } else {
            Serial.printf("[WIFI-MON]   [%u] %s enabled=%s last_scan_rssi=unknown\n",
                          i, net.ssid.c_str(), net.enabled ? "yes" : "no");
        }
    }
}

static void print_completed_scan_if_available() {
    int16_t n = WiFi.scanComplete();
    if (n < 0) return;

    Serial.printf("[WIFI-SCAN] completed: %d visible AP(s)\n", n);
    uint8_t known_count = 0;
    for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        uint8_t cfg_idx = 0xFF;
        bool known = configured_ssid(ssid, &cfg_idx);
        if (known) known_count++;

        String bssid = WiFi.BSSIDstr(i);
        wifi_auth_mode_t auth = WiFi.encryptionType(i);
        Serial.printf("[WIFI-SCAN] %c %2d: %-24s RSSI=%4d dBm ch=%2d auth=%s(%d) BSSID=%s",
                      known ? '*' : ' ', i + 1, ssid.c_str(), WiFi.RSSI(i),
                      WiFi.channel(i), auth_mode_name(auth), (int)auth, bssid.c_str());
        if (known) Serial.printf(" configured_idx=%u", cfg_idx);
        Serial.println();
    }
    Serial.printf("[WIFI-SCAN] configured SSID matches in THIS scan: %u\n", known_count);
}

static void print_disconnect_event() {
    uint8_t reason = wifi_last_disconnect_reason;
    uint32_t count = wifi_disconnect_count;
    wifi_disconnect_pending = false;
    Serial.printf("[WIFI-MON] STA disconnect reason=%u (%s) count=%lu\n",
                  reason, disconnect_reason_name(reason), (unsigned long)count);
}

static void print_status() {
    auto &cfg = Config::get();
    wifi_mode_t hw_mode = WiFi.getMode();
    wl_status_t wl = WiFi.status();
    bool connected = (wl == WL_CONNECTED);

    Serial.printf("[WIFI-MON] cfg=%s hw=%s state=%s wl=%s antenna=%s",
                  cfg_mode_name(cfg.wifi_mode), mode_name(hw_mode),
                  WiFiSetup::state_name(), wl_status_name(wl), antenna_name());

    if (wifi_disconnect_count > 0) {
        uint8_t reason = wifi_last_disconnect_reason;
        Serial.printf(" last_disc=%u(%s) disc_count=%lu", reason,
                      disconnect_reason_name(reason), (unsigned long)wifi_disconnect_count);
    }

    if (connected) {
        Serial.printf(" ssid='%s' rssi=%d dBm ch=%d ip=%s gw=%s",
                      WiFi.SSID().c_str(), WiFi.RSSI(), WiFi.channel(),
                      WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str());
    }

    bool ap_enabled = (hw_mode == WIFI_MODE_AP || hw_mode == WIFI_MODE_APSTA);
    if (ap_enabled) {
        Serial.printf(" ap_ip=%s ap_clients=%u", WiFi.softAPIP().toString().c_str(),
                      WiFi.softAPgetStationNum());
    }

    Serial.println();
    print_known_networks();
}

#endif  // AB_WIFI_DEBUG

void wifi_usb_monitor_init() {
    configure_xiao_c6_antenna();
#if AB_WIFI_DEBUG
    WiFi.onEvent(wifi_monitor_event_cb);
    wifi_monitor_started_ms = millis();
    wifi_monitor_last_ms = 0;
#endif
}

void wifi_usb_monitor_tick() {
#if AB_WIFI_DEBUG
    uint32_t now = millis();

    if (wifi_disconnect_pending) print_disconnect_event();

    // The main loop calls this before WiFiSetup::check(), so debug builds can
    // inspect a completed scan before the normal state machine deletes it.
    print_completed_scan_if_available();

    if (now - wifi_monitor_started_ms < WIFI_MONITOR_START_DELAY_MS) return;
    if (wifi_monitor_last_ms != 0 && now - wifi_monitor_last_ms < WIFI_MONITOR_INTERVAL_MS) return;

    wifi_monitor_last_ms = now;
    print_status();
#endif
}
