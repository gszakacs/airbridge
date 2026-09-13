#include <Arduino.h>
#include <WiFi.h>
#include "app_config.h"
#include "wifi_setup.h"

// USB serial Wi-Fi diagnostics for bench/debug use.
// No extra task is created. The main loop calls wifi_usb_monitor_tick(),
// which reports status every 5 seconds and never starts its own scan.

static const uint32_t WIFI_MONITOR_INTERVAL_MS = 5000;
static const uint32_t WIFI_MONITOR_START_DELAY_MS = 3000;
static uint32_t wifi_monitor_started_ms = 0;
static uint32_t wifi_monitor_last_ms = 0;

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

static const char *mode_name(wifi_mode_t mode) {
    switch (mode) {
        case WIFI_MODE_NULL:   return "NULL";
        case WIFI_MODE_STA:    return "STA";
        case WIFI_MODE_AP:     return "AP";
        case WIFI_MODE_APSTA:  return "AP+STA";
        default:               return "?";
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

static void print_known_networks() {
    auto &cfg = Config::get();
    Serial.printf("[WIFI-MON] configured networks: %u\n", cfg.wifi_net_count);

    for (uint8_t i = 0; i < cfg.wifi_net_count && i < WIFI_MAX_NETWORKS; ++i) {
        const WiFiNetwork &net = cfg.wifi_nets[i];
        int8_t rssi = WiFiSetup::net_rssi(i);
        if (rssi != 0) {
            Serial.printf("[WIFI-MON]   [%u] %s enabled=%s last_rssi=%d dBm\n",
                          i, net.ssid.c_str(), net.enabled ? "yes" : "no", rssi);
        } else {
            Serial.printf("[WIFI-MON]   [%u] %s enabled=%s last_rssi=unknown\n",
                          i, net.ssid.c_str(), net.enabled ? "yes" : "no");
        }
    }
}

static void print_scan_if_available() {
    int16_t n = WiFi.scanComplete();
    if (n < 0) return;

    Serial.printf("[WIFI-MON] scan cache: %d visible AP(s)\n", n);
    for (int i = 0; i < n; ++i) {
        String bssid = WiFi.BSSIDstr(i);
        Serial.printf("[WIFI-MON]   %2d: %-24s RSSI=%4d dBm ch=%2d BSSID=%s\n",
                      i + 1,
                      WiFi.SSID(i).c_str(),
                      WiFi.RSSI(i),
                      WiFi.channel(i),
                      bssid.c_str());
    }
}

static void print_status() {
    auto &cfg = Config::get();
    wifi_mode_t hw_mode = WiFi.getMode();
    wl_status_t wl = WiFi.status();
    bool connected = (wl == WL_CONNECTED);

    Serial.printf("[WIFI-MON] cfg=%s hw=%s state=%s wl=%s antenna=%s",
                  cfg_mode_name(cfg.wifi_mode),
                  mode_name(hw_mode),
                  WiFiSetup::state_name(),
                  wl_status_name(wl),
                  antenna_name());

    if (connected) {
        Serial.printf(" ssid='%s' rssi=%d dBm ch=%d ip=%s gw=%s",
                      WiFi.SSID().c_str(),
                      WiFi.RSSI(),
                      WiFi.channel(),
                      WiFi.localIP().toString().c_str(),
                      WiFi.gatewayIP().toString().c_str());
    }

    bool ap_enabled = (hw_mode == WIFI_MODE_AP || hw_mode == WIFI_MODE_APSTA);
    if (ap_enabled) {
        Serial.printf(" ap_ip=%s ap_clients=%u",
                      WiFi.softAPIP().toString().c_str(),
                      WiFi.softAPgetStationNum());
    }

    Serial.println();
    print_known_networks();
    print_scan_if_available();
}

void wifi_usb_monitor_init() {
    configure_xiao_c6_antenna();
    wifi_monitor_started_ms = millis();
    wifi_monitor_last_ms = 0;
}

void wifi_usb_monitor_tick() {
    uint32_t now = millis();
    if (now - wifi_monitor_started_ms < WIFI_MONITOR_START_DELAY_MS) return;
    if (wifi_monitor_last_ms != 0 && now - wifi_monitor_last_ms < WIFI_MONITOR_INTERVAL_MS) return;

    wifi_monitor_last_ms = now;
    print_status();
}
