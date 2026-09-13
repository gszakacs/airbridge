#include "web_ui.h"
#include "web_ui_html.h"
#include "settings_defs.h"
#include "uart_arbiter.h"
#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "resmed_ota.h"
#include "debug_log.h"
#include "app_config.h"
#include "wifi_setup.h"
#include "network_hints.h"
#include "live_web_consumer.h"
#include "crc.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <stdarg.h>
#include <time.h>

extern const char *airbridge_version();
extern const char *airbridge_build_date();

static AsyncWebServer *http = nullptr;
static AsyncEventSource *events = nullptr;

static bool checkAuth(AsyncWebServerRequest *request) {
    auto &cfg = Config::get();
    if (cfg.http_user.isEmpty() && cfg.http_pass.isEmpty()) return true;
    if (!request->authenticate(cfg.http_user.c_str(), cfg.http_pass.c_str())) {
        request->requestAuthentication();
        return false;
    }
    return true;
}

// iterate json string key-value pairs
// calls fn(key, val) for each pair. Only handles string values.
typedef void (*json_kv_fn)(const String &key, const String &val, void *ctx);
static void json_foreach_kv(const String &body, json_kv_fn fn, void *ctx) {
    int pos = 0;
    while (pos < (int)body.length()) {
        int ks = body.indexOf('"', pos);
        if (ks < 0) break;
        int ke = body.indexOf('"', ks + 1);
        if (ke < 0) break;
        String key = body.substring(ks + 1, ke);
        int vs = body.indexOf('"', ke + 1);
        if (vs < 0) break;
        int ve = body.indexOf('"', vs + 1);
        if (ve < 0) break;
        String val = body.substring(vs + 1, ve);
        pos = ve + 1;
        fn(key, val, ctx);
    }
}

static int parseResponseValue(const char *resp) {
    const char *v = qframe_response_value(resp);
    return v ? (int)strtol(v, nullptr, 16) : -1;
}

static bool readSetting(const char *cmd, int &value) {
    char req[16];
    snprintf(req, sizeof(req), "G S #%s", cmd);
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);
    bool ok = Arbiter::send_cmd(req, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                 resp, &resp_len);
    if (!ok) return false;
    value = parseResponseValue(resp);
    return value >= 0;
}

static bool writeSetting(const char *cmd, int value) {
    char req[32];
    snprintf(req, sizeof(req), "P S #%s %04X", cmd, (uint16_t)value);
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);
    return Arbiter::send_cmd(req, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                              resp, &resp_len);
}

static void jsonAddString(String &out, const char *key, const char *val, bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":\"";
    while (*val) {
        if (*val == '"') out += "\\\"";
        else if (*val == '\\') out += "\\\\";
        else if (*val == '\n') out += "\\n";
        else if (*val == '\r') out += "\\r";
        else if (*val == '\t') out += "\\t";
        else if ((uint8_t)*val < 0x20) {} // skip other control chars
        else out += *val;
        val++;
    }
    out += '"';
}

static void jsonAddInt(String &out, const char *key, int val, bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":";
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", val);
    out += buf;
}


static String pending_body;
static bool pending_body_ready = false;

static void handleJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    if (index == 0) {
        pending_body = "";
        pending_body.reserve(total);
        pending_body_ready = false;
    }
    pending_body.concat((char *)data, len);
    if (index + len == total) {
        pending_body_ready = true;
    }
}

static String getBody(AsyncWebServerRequest *request) {
    if (pending_body_ready) {
        pending_body_ready = false;
        String result = pending_body;
        pending_body = "";
        return result;
    }
    return "";
}

static void handleRoot(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", HTML_PAGE_GZ, HTML_PAGE_GZ_SIZE);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
}


struct FixedJson {
    char *buf;
    size_t cap;
    size_t len;
    bool overflow;
};

static void fixedJsonPut(FixedJson &json, char c) {
    if (json.len + 1 >= json.cap) {
        json.overflow = true;
        return;
    }
    json.buf[json.len++] = c;
    json.buf[json.len] = '\0';
}

static void fixedJsonAppend(FixedJson &json, const char *s) {
    while (*s) fixedJsonPut(json, *s++);
}

static void fixedJsonPrintf(FixedJson &json, const char *fmt, ...) {
    if (json.len >= json.cap) {
        json.overflow = true;
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(json.buf + json.len, json.cap - json.len, fmt, ap);
    va_end(ap);

    if (n < 0 || (size_t)n >= json.cap - json.len) {
        json.len = json.cap ? json.cap - 1 : 0;
        if (json.cap) json.buf[json.len] = '\0';
        json.overflow = true;
        return;
    }
    json.len += (size_t)n;
}

static void fixedJsonAddString(FixedJson &json, const char *key, const char *val, bool comma = true) {
    if (comma) fixedJsonPut(json, ',');
    fixedJsonPut(json, '"');
    fixedJsonAppend(json, key);
    fixedJsonAppend(json, "\":\"");

    if (!val) val = "";
    while (*val) {
        if (*val == '"') fixedJsonAppend(json, "\\\"");
        else if (*val == '\\') fixedJsonAppend(json, "\\\\");
        else if (*val == '\n') fixedJsonAppend(json, "\\n");
        else if (*val == '\r') fixedJsonAppend(json, "\\r");
        else if (*val == '\t') fixedJsonAppend(json, "\\t");
        else if ((uint8_t)*val >= 0x20) fixedJsonPut(json, *val);
        val++;
    }

    fixedJsonPut(json, '"');
}

static void fixedJsonAddInt(FixedJson &json, const char *key, long val, bool comma = true) {
    if (comma) fixedJsonPut(json, ',');
    fixedJsonPut(json, '"');
    fixedJsonAppend(json, key);
    fixedJsonAppend(json, "\":");
    fixedJsonPrintf(json, "%ld", val);
}

static size_t buildStatusJson(char *out, size_t cap) {
    if (!out || cap == 0) return 0;
    out[0] = '\0';

    oxi_state_t oxi = OxiBle::get_state();
    const oxi_reading_t &r = OxiArbiter::get_reading();

    int rop = Arbiter::get_cached_rop();
    int mhr = Arbiter::get_cached_mhr();
    system_state_t sys = Arbiter::get_state();

    char dac_resp[32] = {}, tic_resp[32] = {};
    uint16_t dac_len = sizeof(dac_resp), tic_len = sizeof(tic_resp);
    bool got_dac = Arbiter::send_cmd("G S #DAC", CMD_SRC_TCP, CMD_PRIO_NORMAL, dac_resp, &dac_len);
    bool got_tic = Arbiter::send_cmd("G S #TIC", CMD_SRC_TCP, CMD_PRIO_NORMAL, tic_resp, &tic_len);

    Config::refresh_device_info();
    auto &cfg = Config::get();

    char esp_time[20] = "--";
    time_t now = time(nullptr);
    if (now > 1700000000) {
        struct tm t;
        localtime_r(&now, &t);
        snprintf(esp_time, sizeof(esp_time), "%04d-%02d-%02d %02d:%02d",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min);
    }

    char resmed_time[20] = "--";
    if (got_dac && got_tic) {
        const char *dv = qframe_response_value(dac_resp);
        const char *tv = qframe_response_value(tic_resp);
        if (dv && tv && strlen(dv) >= 8 && strlen(tv) >= 6) {
            int dd, mm, yyyy, hh, mn, ss;
            if (sscanf(dv, "%2d%2d%4d", &dd, &mm, &yyyy) == 3 &&
                sscanf(tv, "%2d%2d%2d", &hh, &mn, &ss) == 3) {
                snprintf(resmed_time, sizeof(resmed_time), "%04d-%02d-%02d %02d:%02d",
                         yyyy, mm, dd, hh, mn);
            }
        }
    }

    FixedJson json = {out, cap, 0, false};
    fixedJsonPut(json, '{');
    fixedJsonAddString(json, "version", airbridge_version(), false);
    fixedJsonAddString(json, "built", airbridge_build_date());
    fixedJsonAddString(json, "system", system_state_name(sys));
    fixedJsonAddInt(json, "rop", rop);
    fixedJsonAddString(json, "pna", cfg.device_pna.c_str());
    fixedJsonAddString(json, "srn", cfg.device_srn.c_str());
    fixedJsonAddString(json, "esp_time", esp_time);
    fixedJsonAddString(json, "resmed_time", resmed_time);
    fixedJsonAddString(json, "oxi", oxi_state_name(oxi));
    fixedJsonAddString(json, "feeding", OxiArbiter::is_feeding() ? "yes" : "no");
    fixedJsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    fixedJsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    fixedJsonAddInt(json, "heap", ESP.getFreeHeap());
    fixedJsonAddInt(json, "rssi", WiFi.RSSI());
    fixedJsonAddInt(json, "mhr", mhr);
    fixedJsonAddInt(json, "uptime", millis() / 1000);
    fixedJsonPut(json, '}');

    return json.overflow ? 0 : json.len;
}

static const uint32_t STATUS_CACHE_TTL_MS = 500;
static const size_t STATUS_JSON_MAX = 512;
static String status_cache;
static uint32_t status_cache_built_at = 0;

static bool refreshStatusCacheIfNeeded() {
    uint32_t now = millis();
    if (status_cache.length() > 0 && now - status_cache_built_at < STATUS_CACHE_TTL_MS)
        return true;

    char body[STATUS_JSON_MAX];
    size_t len = buildStatusJson(body, sizeof(body));
    if (len == 0) return false;

    status_cache = body;
    status_cache_built_at = millis();
    return true;
}

static void handleStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    if (!refreshStatusCacheIfNeeded()) {
        request->send(503, "application/json", "{\"ok\":false,\"error\":\"status_unavailable\"}");
        return;
    }
    request->send(200, "application/json", status_cache);
}


static void emitVar(String &json, const var_def_t *v, const char *group, int raw, bool ok, bool &first) {
    if (!first) json += ',';
    first = false;

    json += '{';
    jsonAddString(json, "cmd", v->cmd, false);
    jsonAddString(json, "label", v->label);
    jsonAddString(json, "group", group);
    jsonAddInt(json, "value", ok ? raw : -1);

    if (v->type == SET_ENUM) {
        jsonAddString(json, "type", "enum");
        if (v->enum_options) {
            json += ",\"options\":[";
            String opts = v->enum_options;
            int start = 0;
            bool firstOpt = true;
            for (int j = 0; j <= (int)opts.length(); j++) {
                if (j == (int)opts.length() || opts[j] == ',') {
                    if (!firstOpt) json += ',';
                    json += '"';
                    json += opts.substring(start, j);
                    json += '"';
                    firstOpt = false;
                    start = j + 1;
                }
            }
            json += ']';
        }
    } else if (v->type == SET_SCALED && v->scale_div > 1) {
        jsonAddString(json, "type", "scaled");
        jsonAddInt(json, "scale_div", v->scale_div);
        jsonAddInt(json, "decimals", v->decimals);
        if (ok) {
            float disp = (float)raw / (float)v->scale_div;
            char buf[16];
            snprintf(buf, sizeof(buf), "%.*f", v->decimals, disp);
            jsonAddString(json, "display", buf);
            float step = 1.0f;
            for (int d = 0; d < v->decimals; d++) step /= 10.0f;
            snprintf(buf, sizeof(buf), "%g", step);
            jsonAddString(json, "step", buf);
        }
    } else {
        jsonAddString(json, "type", "int");
    }
    json += '}';
}


static void emitVarList(String &json, const char * const *cmds, const char *group, bool &first) {
    for (int i = 0; cmds[i]; i++) {
        const var_def_t *v = var_lookup(cmds[i]);
        if (!v) continue;
        int raw = 0;
        bool ok = readSetting(v->cmd, raw);
        emitVar(json, v, group, raw, ok, first);
    }
}

static void handleGetSettings(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    int mop = 0;
    readSetting("MOP", mop);
    if (mop < 0 || mop >= MODE_COUNT) mop = 0;

    String json = "[";
    bool first = true;

    emitVarList(json, PATIENT_VARS, "patient", first);

    // Mode selector
    {
        const var_def_t *v = var_lookup("MOP");
        if (v) {
            int raw = mop;
            emitVar(json, v, "clinical", raw, true, first);
        }
    }

    // Mode settings
    emitVarList(json, MODE_LAYOUT[mop], "mode", first);
    {
        static const char * const mask_var[] = {"MSK", NULL};
        emitVarList(json, mask_var, "mode", first);
    }

    emitVarList(json, COMFORT_LAYOUT[mop], "comfort", first);
    emitVarList(json, EPR_VARS, "comfort", first);

    emitVarList(json, CLIMATE_VARS, "climate", first);

    emitVarList(json, SYSTEM_VARS, "system", first);

    emitVarList(json, ALARM_VARS, "alarm", first);

    json += ']';
    request->send(200, "application/json", json);
}


static void handlePostSettings(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    struct { int count; String errors; } ctx = {0, ""};
    json_foreach_kv(body, [](const String &key, const String &val, void *p) {
        auto *c = (decltype(ctx)*)p;
        const var_def_t *def = var_lookup(key.c_str());
        if (!def) { c->errors += key + ":unknown,"; return; }
        if (writeSetting(key.c_str(), atoi(val.c_str()))) c->count++;
        else c->errors += key + ":fail,";
    }, &ctx);
    int count = ctx.count;
    String errors = ctx.errors;

    String json = "{";
    jsonAddInt(json, "saved", count, false);
    if (errors.length() > 0) {
        errors.remove(errors.length() - 1);  // trailing comma
        json += ",\"errors\":[\"" + errors + "\"]";
    }
    json += '}';
    request->send(200, "application/json", json);
}


static void handleGetConfig(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    struct { String *json; bool first; } ctx = {&json, true};
    Config::foreach_kv([](const char *key, const String &val, void *p) {
        auto *c = (decltype(ctx)*)p;
        if (!c->first) *c->json += ',';
        c->first = false;
        jsonAddString(*c->json, key, val.c_str(), false);
    }, &ctx);
    json += '}';
    request->send(200, "application/json", json);
}


static void handlePostConfig(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    int count = 0;
    json_foreach_kv(body, [](const String &key, const String &val, void *p) {
        if (Config::set_value(key.c_str(), val.c_str())) (*(int*)p)++;
    }, &count);

    if (count > 0) Config::save();

    String json = "{\"ok\":true,\"saved\":";
    json += String(count);
    json += '}';
    request->send(200, "application/json", json);
}


static const esp_partition_t *resmed_part = nullptr;
size_t uploadSize = 0;
static bool uploadOk = false;
static uint16_t uploadCrc = 0xFFFF;

typedef enum {
    UPLOAD_NONE,
    UPLOAD_RESMED,
    UPLOAD_ESP,
} upload_kind_t;

static upload_kind_t uploadKind = UPLOAD_NONE;

static bool hasValidResmedUpload() {
    return uploadKind == UPLOAD_RESMED && uploadOk && uploadSize > 0 && resmed_part;
}

static void handleUploadChunk(AsyncWebServerRequest *request, const String& filename,
                               size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        Log::logf(CAT_WEB, LOG_INFO, "[WEB] Upload start: %s\n", filename.c_str());
        uploadKind = UPLOAD_RESMED;
        uploadSize = 0;
        uploadOk = false;
        uploadCrc = 0xFFFF;

        resmed_part = ResmedOta::get_staging_partition();
        if (!resmed_part) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] No staging partition found\n");
            uploadKind = UPLOAD_NONE;
            return;
        }
        Log::logf(CAT_WEB, LOG_INFO, "[WEB] Staging to '%s' (0x%X, %u bytes)\n",
                     resmed_part->label, resmed_part->address, resmed_part->size);

        esp_err_t err = esp_partition_erase_range(resmed_part, 0, resmed_part->size);
        if (err != ESP_OK) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] Erase failed: %s\n", esp_err_to_name(err));
            resmed_part = nullptr;
            uploadKind = UPLOAD_NONE;
            return;
        }
        uploadOk = true;
    }

    if (uploadKind == UPLOAD_RESMED && resmed_part && uploadOk && len > 0) {
        // reject ESP32 binaries uploaded to resmed slot
        if (uploadSize == 0 && len > 0 && data[0] == 0xE9) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] Rejected: ESP32 binary uploaded to ResMed slot\n");
            uploadOk = false;
            return;
        }
        if (uploadSize + len > resmed_part->size) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] File too large for partition!\n");
            uploadOk = false;
            return;
        }
        esp_err_t err = esp_partition_write(resmed_part, uploadSize, data, len);
        if (err != ESP_OK) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] Write failed at offset %u: %s\n",
                         uploadSize, esp_err_to_name(err));
            uploadOk = false;
            return;
        }
        uploadCrc = crc16_ccitt(data, len, uploadCrc);
        uploadSize += len;
    }

    if (final) {
        if (uploadOk) {
            Log::logf(CAT_WEB, LOG_INFO, "[WEB] Upload complete: %u bytes\n", uploadSize);
        }
    }
}

static void handleUploadDone(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    bool validUpload = hasValidResmedUpload();
    jsonAddString(json, "ok", validUpload ? "true" : "false", false);
    jsonAddInt(json, "size", uploadSize);

    if (validUpload) {
        char hexcrc[8];
        snprintf(hexcrc, sizeof(hexcrc), "%04X", uploadCrc);
        jsonAddString(json, "crc", hexcrc);

        // Firmware verification
        fw_verify_result_t v = ResmedOta::verify_image(resmed_part, uploadSize);
        if (v.has_blx) {
            jsonAddString(json, "bid", v.bid);
            jsonAddString(json, "bid_ok", v.bid_ok ? "true" : "false");
            jsonAddString(json, "blx_crc", v.blx_crc_ok ? "ok" : "fail");
            const char *patch = "none";
            if (v.blx_patch == BLX_PATCH_A_DANGEROUS) patch = "method_a";
            else if (v.blx_patch == BLX_PATCH_B_SAFE) patch = "method_b";
            jsonAddString(json, "blx_patch", patch);
        }
        if (v.has_ccx) jsonAddString(json, "ccx_crc", v.ccx_crc_ok ? "ok" : "fail");
        if (v.has_cdx) jsonAddString(json, "cdx_crc", v.cdx_crc_ok ? "ok" : "fail");
    }

    json += '}';
    request->send(200, "application/json", json);
}


#define LIVE_BATCH_MAX  128

static void handleLive(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    uint16_t since = 0;
    if (request->hasArg("since"))
        since = (uint16_t)request->arg("since").toInt();

    LivePmd::Sample samples[LIVE_BATCH_MAX];
    uint16_t cur_seq = 0;
    int n = LiveWebConsumer::get_samples(samples, LIVE_BATCH_MAX, since, &cur_seq);

    const oxi_reading_t &r = OxiArbiter::get_reading();

    String json;
    json.reserve(64 + n * 24);
    json = "{";
    jsonAddInt(json, "seq", cur_seq, false);
    jsonAddInt(json, "rate", 25);
    jsonAddString(json, "active", LiveWebConsumer::is_active() ? "yes" : "no");
    jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    json += ",\"samples\":[";
    for (int i = 0; i < n; i++) {
        if (i > 0) json += ',';
        json += '[';
        json += String(samples[i].mkp);
        json += ',';
        json += String(samples[i].rfl);
        json += ',';
        json += String(samples[i].lyk);
        json += ']';
    }
    json += "]}";
    request->send(200, "application/json", json);
}


static void handleBleStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    oxi_state_t st = OxiBle::get_state();
    const oxi_reading_t &r = OxiArbiter::get_reading();
    auto &cfg = Config::get();

    String json = "{";
    jsonAddString(json, "state", oxi_state_name(st), false);
    jsonAddString(json, "feeding", OxiArbiter::is_feeding() ? "yes" : "no");
    jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    jsonAddString(json, "configured_addr", cfg.oxi_device_addr.c_str());
    jsonAddString(json, "auto_start", cfg.oxi_auto_start ? "yes" : "no");


    oxi_scan_result_t devs[MAX_SCAN_RESULTS];
    int scan_count = OxiBle::get_scan_results(devs, MAX_SCAN_RESULTS);
    json += ",\"devices\":[";
    for (int i = 0; i < scan_count; i++) {
        if (i > 0) json += ',';
        json += "{";
        jsonAddString(json, "addr", devs[i].addr.c_str(), false);
        jsonAddString(json, "name", devs[i].name.c_str());
        jsonAddInt(json, "rssi", devs[i].rssi);
        json += "}";
    }
    json += "],\"bonds\":[";
    char known_addrs[6][18];
    int nk = OxiBle::get_all_known(known_addrs, 6);
    for (int i = 0; i < nk; i++) {
        if (i > 0) json += ',';
        json += '"';
        json += known_addrs[i];
        json += '"';
    }
    json += "]}";
    request->send(200, "application/json", json);
}


static void handleBleAction(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    String action, addr;
    struct { String *action; String *addr; } ctx = {&action, &addr};
    json_foreach_kv(body, [](const String &key, const String &val, void *p) {
        auto *c = (decltype(ctx)*)p;
        if (key == "action") *c->action = val;
        else if (key == "addr") *c->addr = val;
    }, &ctx);

    String result = "unknown action";
    bool ok = false;

    if (action == "scan") {
        OxiBle::start_scan();
        result = "scan started";
        ok = true;
    } else if (action == "stop_scan") {
        OxiBle::stop_scan();
        result = "scan stopped";
        ok = true;
    } else if (action == "connect") {
        if (addr.length() > 0) {
            OxiBle::connect(addr.c_str());
        } else {
            OxiBle::connect(nullptr);
        }
        result = "connecting";
        ok = true;
    } else if (action == "disconnect") {
        OxiBle::disconnect();
        result = "disconnected";
        ok = true;
    } else if (action == "enable") {
        OxiBle::enable();
        result = "oximetry enabled";
        ok = true;
    } else if (action == "disable") {
        OxiBle::disable();
        result = "oximetry disabled";
        ok = true;
    } else if (action == "start_feed") {
        OxiArbiter::start_feed();
        result = "feeding started";
        ok = true;
    } else if (action == "stop_feed") {
        OxiArbiter::stop_feed();
        result = "feeding stopped";
        ok = true;
    } else if (action == "delete_bond") {
        if (addr.length() > 0) {
            // Clear the configured device addr synchronously if it matches.
            if (strcasecmp(addr.c_str(), Config::get().oxi_device_addr.c_str()) == 0) {
                Config::set_value("oxi_device_addr", "");
                Config::save();
            }
            OxiBle::request_remove_known(addr.c_str());
            result = "queued";
            ok = true;
        } else {
            result = "no address specified";
            ok = true;
        }
    } else if (action == "delete_all_bonds") {
        if (Config::get().oxi_device_addr.length() > 0) {
            Config::set_value("oxi_device_addr", "");
            Config::save();
        }
        OxiBle::request_clear_all_known();
        result = "queued";
        ok = true;
    }

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}


extern void dispatch_command(const char *line, String &response);

static void handleCmd(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    int valStart = body.indexOf("\"cmd\"");
    if (valStart < 0) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
        return;
    }
    int qs = body.indexOf('"', valStart + 5);
    int qe = body.indexOf('"', qs + 1);
    if (qs < 0 || qe < 0) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
        return;
    }
    String cmd = body.substring(qs + 1, qe);

    String json = "{";

    if (cmd.startsWith("$")) {
        // Internal command
        String response;
        dispatch_command(cmd.c_str() + 1, response);
        response.trim();
        jsonAddString(json, "ok", "true", false);
        jsonAddString(json, "response", response.c_str());
    } else {
        // Q-frame
        char resp[128] = {};
        uint16_t resp_len = sizeof(resp);
        bool ok = Arbiter::send_cmd(cmd.c_str(), CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                     resp, &resp_len);
        jsonAddString(json, "ok", ok ? "true" : "false", false);
        if (ok && resp_len > 0) {
            jsonAddString(json, "response", resp);
        }
        if (ok && cmd.startsWith("P S #ROP ")) {
            int new_rop = (int)strtoul(cmd.c_str() + 9, nullptr, 16);
            Arbiter::set_cached_rop(new_rop);
            WebUI::push_status_event();
        }
    }

    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    jsonAddString(json, "active", ResmedOta::is_active() ? "true" : "false", false);
    jsonAddString(json, "phase", ResmedOta::get_phase());
    jsonAddInt(json, "sent", ResmedOta::get_sent());
    jsonAddInt(json, "total", ResmedOta::get_total());
    const char *err = ResmedOta::last_error();
    if (err && err[0]) {
        jsonAddString(json, "error", err);
    }

    if (hasValidResmedUpload()) {
        const char *detected = ResmedOta::detect_block(uploadSize);
        jsonAddString(json, "detected_block", detected ? detected : "unknown");
        jsonAddInt(json, "fw_size", uploadSize);
    }
    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashStart(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    if (ResmedOta::is_active()) {
        request->send(409, "application/json", "{\"ok\":false,\"error\":\"flash already active\"}");
        return;
    }

    if (!hasValidResmedUpload()) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"no valid ResMed firmware uploaded\"}");
        return;
    }

    String body = getBody(request);
    String block = "";
    bool flash_blx = false;
    bool force_blx = false;

    int pos = 0;
    while (pos < (int)body.length()) {
        int keyStart = body.indexOf('"', pos);
        if (keyStart < 0) break;
        int keyEnd = body.indexOf('"', keyStart + 1);
        if (keyEnd < 0) break;
        String key = body.substring(keyStart + 1, keyEnd);

        int valStart = body.indexOf(':', keyEnd);
        if (valStart < 0) break;
        valStart++;
        while (valStart < (int)body.length() && body[valStart] == ' ') valStart++;

        if (key == "force_blx") {
            force_blx = (body.substring(valStart, valStart + 4) == "true");
            pos = valStart + 5;
        } else if (key == "flash_blx") {
            flash_blx = (body.substring(valStart, valStart + 4) == "true");
            pos = valStart + 5;
        } else {
            int qs = body.indexOf('"', valStart);
            int qe = body.indexOf('"', qs + 1);
            if (qs < 0 || qe < 0) break;
            if (key == "block") block = body.substring(qs + 1, qe);
            pos = qe + 1;
        }
    }

    ResmedOta::start_flash(
        block.length() > 0 ? block.c_str() : nullptr,
        uploadSize,
        flash_blx,
        force_blx
    );

    String json = "{";
    jsonAddString(json, "ok", "true", false);
    jsonAddString(json, "block", block.length() > 0 ? block.c_str() :
                  (ResmedOta::detect_block(uploadSize) ? ResmedOta::detect_block(uploadSize) : "auto"));
    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashCancel(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    ResmedOta::cancel();
    request->send(200, "application/json", "{\"ok\":true}");
}


struct report_var_t {
    const char *cmd;
    const char *label;
    int scale_div;      // 0 = raw int, -60 = raw minutes -> hours
    int decimals;
    const char *unit;
    const char *section;  // "session" or "summary"
};

#define RPT_SESSION "session"
#define RPT_SUMMARY "summary"

static const report_var_t REPORT_VARS[] = {
    // LAST SESSION
    {"UQD", "Usage", -61, 0, "", RPT_SESSION},              {"OND", "Mask On Duration", -61, 0, "", RPT_SESSION},
    {"AQD", "Events/hr", 10, 1, "/hr", RPT_SESSION},        {"MSP", "Median Pressure", 50, 1, "cmH2O", RPT_SESSION},
    {"AIS", "AI (All)", 10, 1, "/hr", RPT_SESSION},          {"PM9", "Pressure P95", 50, 1, "cmH2O", RPT_SESSION},
    {"OPI", "Obstructive AI", 10, 1, "/hr", RPT_SESSION},    {"PMA", "Max Pressure", 50, 1, "cmH2O", RPT_SESSION},
    {"CLI", "Central AI", 10, 1, "/hr", RPT_SESSION},        {"AEP", "Avg EPR Pressure", 50, 1, "cmH2O", RPT_SESSION},
    {"HIS", "Hypopnea Index", 10, 1, "/hr", RPT_SESSION},    {"LKM", "Leak Median", 50, 2, "L/s", RPT_SESSION},
    {"UAI", "Unknown AI", 10, 1, "/hr", RPT_SESSION},        {"LK9", "Leak P95", 50, 2, "L/s", RPT_SESSION},
    {"RIN", "RERA Index", 10, 1, "/hr", RPT_SESSION},

    // SUMMARY
    {"PHM", "Total Used Hrs", 0, 0, "hrs", RPT_SUMMARY},    {"LRD", "Leak", 10, 0, "L/min", RPT_SUMMARY},
    {"DRD", "Days Used", -3, 0, "", RPT_SUMMARY},           {"ZAV", "Vt", 0, 0, "ml", RPT_SUMMARY},
    {"VRD", "Days 4hrs+", -3, 0, "", RPT_SUMMARY},          {"ZAR", "RR", 5, 0, "bpm", RPT_SUMMARY},
    {"WRD", "Avg. Usage", -60, 1, "hrs", RPT_SUMMARY},      {"ZAM", "MV", 8, 1, "L/min", RPT_SUMMARY},
    {"XRD", "Used Hrs", -60, 1, "hrs", RPT_SUMMARY},        {"ZA2", "TgMV", 8, 1, "L/min", RPT_SUMMARY},
    {"ZAI", "Pressure", 50, 1, "cmH2O", RPT_SUMMARY},      {"ZA3", "Va", 8, 1, "L/min", RPT_SUMMARY},
    {"ZAE", "Exp. Pressure", 50, 1, "cmH2O", RPT_SUMMARY}, {"ZAZ", "Ti", 50, 2, "s", RPT_SUMMARY},
    {"ARD", "AHI", 10, 1, "/hr", RPT_SUMMARY},              {"ZA1", "I:E", -1, 0, "", RPT_SUMMARY},
    {"TRD", "Total AI", 10, 1, "/hr", RPT_SUMMARY},         {"ZAS", "Spont Trig", 2, 1, "%", RPT_SUMMARY},
    {"CRD", "Central AI", 10, 1, "/hr", RPT_SUMMARY},       {"ZAY", "Spont Cyc", 2, 1, "%", RPT_SUMMARY},

    {NULL, NULL, 0, 0, NULL, NULL}
};

static const char *period_label(int raw, char *buf, size_t len) {
    switch (raw) {
        case 1:   return "1 Day";
        case 7:   return "1 Week";
        case 30:  return "1 Month";
        case 90:  return "3 Months";
        case 180: return "6 Months";
        case 365: return "1 Year";
        default:
            snprintf(buf, len, "%d days", raw);
            return buf;
    }
}

static void handleReport(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    int period = 0;
    readSetting("URD", period);

    String json = "[";
    bool first = true;

    for (const report_var_t *v = REPORT_VARS; v->cmd; v++) {
        int raw = 0;
        bool ok = readSetting(v->cmd, raw);

        if (!first) json += ',';
        first = false;

        json += '{';
        jsonAddString(json, "cmd", v->cmd, false);
        jsonAddString(json, "label", v->label);
        jsonAddInt(json, "raw", ok ? raw : -1);

        char buf[32];
        if (ok && v->scale_div == -61) {
            // Raw minutes -> H:MM
            snprintf(buf, sizeof(buf), "%d:%02d", raw / 60, raw % 60);
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div == -60) {
            // Raw minutes -> decimal hours (round properly)
            double hrs = (double)raw / 60.0;
            snprintf(buf, sizeof(buf), "%.*f", v->decimals, hrs);
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div == -3) {
            // N/period format (e.g. "6/7")
            snprintf(buf, sizeof(buf), "%d/%d", raw, period);
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div == -2) {
            // Period enum
            jsonAddString(json, "value", period_label(raw, buf, sizeof(buf)));
        } else if (ok && v->scale_div == -1) {
            // I:E ratio: raw/100
            if (raw > 0) {
                float ratio = (float)raw / 100.0f;
                if (ratio >= 1.0f) {
                    snprintf(buf, sizeof(buf), "%.1f:1", ratio);
                } else {
                    snprintf(buf, sizeof(buf), "1:%.1f", 1.0f / ratio);
                }
            } else {
                snprintf(buf, sizeof(buf), "--");
            }
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div > 0) {
            double disp = (double)raw / (double)v->scale_div;
            snprintf(buf, sizeof(buf), "%.*f", v->decimals, disp);
            jsonAddString(json, "value", buf);
        } else {
            if (ok) {
                snprintf(buf, sizeof(buf), "%d", raw);
                jsonAddString(json, "value", buf);
            } else {
                jsonAddString(json, "value", "--");
            }
        }
        jsonAddString(json, "unit", v->unit);
        jsonAddString(json, "section", v->section);
        json += '}';
    }

    json += ']';
    request->send(200, "application/json", json);
}

// ESP32 OTA
static esp_ota_handle_t esp_ota_handle = 0;
static const esp_partition_t *esp_ota_part = nullptr;

static void abortEspOtaUpload() {
    if (esp_ota_handle) {
        esp_ota_abort(esp_ota_handle);
        esp_ota_handle = 0;
    }
    esp_ota_part = nullptr;
    uploadKind = UPLOAD_NONE;
    Arbiter::set_state(SYS_IDLE);
}

static void handleEspOtaChunk(AsyncWebServerRequest *request, const String& filename,
                               size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        if (ResmedOta::is_active()) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] ESP OTA rejected: ResMed flash active\n");
            uploadOk = false;
            return;
        }
        Log::logf(CAT_WEB, LOG_INFO, "[WEB] ESP OTA start: %s\n", filename.c_str());
        uploadKind = UPLOAD_ESP;
        resmed_part = nullptr;
        uploadSize = 0;
        uploadOk = false;
        uploadCrc = 0xFFFF;

        esp_ota_part = esp_ota_get_next_update_partition(NULL);
        if (!esp_ota_part) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] No OTA partition found\n");
            uploadKind = UPLOAD_NONE;
            return;
        }
        Log::logf(CAT_WEB, LOG_INFO, "[WEB] OTA target: '%s' (0x%X, %u bytes)\n",
                  esp_ota_part->label, esp_ota_part->address, esp_ota_part->size);

        esp_err_t err = esp_ota_begin(esp_ota_part, OTA_SIZE_UNKNOWN, &esp_ota_handle);
        if (err != ESP_OK) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] esp_ota_begin failed: %s\n", esp_err_to_name(err));
            esp_ota_part = nullptr;
            uploadKind = UPLOAD_NONE;
            Arbiter::set_state(SYS_IDLE);
            return;
        }
        uploadOk = true;
    }

    if (esp_ota_part && uploadOk && len > 0) {
        // validate esp binary magic on first data
        if (uploadSize == 0 && len > 0 && data[0] != 0xE9) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] Not an ESP32 binary (magic=0x%02X)\n", data[0]);
            uploadOk = false;
            abortEspOtaUpload();
            return;
        }
        if (uploadSize == 0) Arbiter::set_state(SYS_OTA_ESP);
        esp_err_t err = esp_ota_write(esp_ota_handle, data, len);
        if (err != ESP_OK) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] esp_ota_write failed at %u: %s\n",
                      uploadSize, esp_err_to_name(err));
            uploadOk = false;
            abortEspOtaUpload();
            return;
        }
        uploadCrc = crc16_ccitt(data, len, uploadCrc);
        uploadSize += len;
    }

    if (final && uploadOk) {
        Log::logf(CAT_WEB, LOG_INFO, "[WEB] ESP OTA upload complete: %u bytes\n", uploadSize);
    }
}

static void handleEspOtaDone(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    bool ok = false;

    if (uploadKind == UPLOAD_ESP && uploadOk && uploadSize > 0 && esp_ota_part) {
        esp_err_t err = esp_ota_end(esp_ota_handle);
        if (err == ESP_OK) {
            err = esp_ota_set_boot_partition(esp_ota_part);
            if (err == ESP_OK) {
                ok = true;
                Log::logf(CAT_WEB, LOG_INFO, "[WEB] ESP OTA OK, boot set to '%s'\n",
                          esp_ota_part->label);
            } else {
                Log::logf(CAT_WEB, LOG_ERROR, "[WEB] set_boot_partition failed: %s\n",
                          esp_err_to_name(err));
            }
        } else {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] esp_ota_end failed: %s\n", esp_err_to_name(err));
            jsonAddString(json, "error", esp_err_to_name(err));
        }
    }

    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddInt(json, "size", uploadSize);
    char hexcrc[8];
    snprintf(hexcrc, sizeof(hexcrc), "%04X", uploadCrc);
    jsonAddString(json, "crc", hexcrc);
    if (esp_ota_part)
        jsonAddString(json, "partition", esp_ota_part->label);

    esp_ota_handle = 0;
    esp_ota_part = nullptr;
    uploadKind = UPLOAD_NONE;
    Arbiter::set_state(SYS_IDLE);

    json += '}';
    request->send(200, "application/json", json);
}

// WiFi management

static void handleWifiGet(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    auto &cfg = Config::get();

    String json = "{";
    jsonAddString(json, "state", WiFiSetup::state_name(), false);
    jsonAddString(json, "ssid", WiFiSetup::connected_ssid());
    jsonAddInt(json, "rssi", WiFiSetup::current_rssi());
    jsonAddInt(json, "net_idx", WiFiSetup::connected_net_idx());
    jsonAddString(json, "roam", cfg.wifi_roam ? "1" : "0");

    json += ",\"networks\":[";
    for (int i = 0; i < cfg.wifi_net_count; i++) {
        if (i > 0) json += ',';
        json += "{";
        jsonAddString(json, "ssid", cfg.wifi_nets[i].ssid.c_str(), false);
        jsonAddString(json, "enabled", cfg.wifi_nets[i].enabled ? "1" : "0");
        // Hint info comes from NetworkHints now (one slot may have multiple
        // BSSID hints with multi-AP roaming; pick the most recent here).
        const NetworkHint *h = NetworkHints::find_best(cfg.wifi_nets[i].ssid.c_str());
        jsonAddString(json, "hint", h ? "1" : "0");
        jsonAddInt(json, "channel", h ? h->channel : 0);
        jsonAddInt(json, "rssi", WiFiSetup::net_rssi(i));
        json += "}";
    }
    json += "]}";
    request->send(200, "application/json", json);
}

static void handleWifiPost(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String body = getBody(request);
    String action, ssid, pass;
    int idx = -1;

    struct wifi_kv_ctx { String *action; String *ssid; String *pass; int *idx; };
    wifi_kv_ctx wctx = {&action, &ssid, &pass, &idx};
    json_foreach_kv(body, [](const String &key, const String &val, void *p) {
        wifi_kv_ctx *c = (wifi_kv_ctx *)p;
        if (key == "action") *c->action = val;
        else if (key == "ssid") *c->ssid = val;
        else if (key == "pass") *c->pass = val;
        else if (key == "idx") *c->idx = val.toInt();
    }, &wctx);

    String result = "unknown action";
    bool ok = false;

    if (action == "add") {
        if (ssid.length() > 0) {
            ok = Config::add_network(ssid.c_str(), pass.c_str());
            result = ok ? "network added" : "list full (max 4)";
        } else {
            result = "ssid required";
        }
    } else if (action == "remove") {
        if (idx >= 0) {
            ok = Config::remove_network((uint8_t)idx);
            result = ok ? "network removed" : "invalid index";
        } else {
            result = "idx required";
        }
    } else if (action == "update") {
        auto &cfg = Config::get();
        if (idx >= 0 && idx < cfg.wifi_net_count) {
            if (ssid.length() > 0) cfg.wifi_nets[idx].ssid = ssid;
            if (pass.length() > 0) cfg.wifi_nets[idx].pass = pass;
            Config::save_wifi_nets();
            ok = true;
            result = "network updated";
        } else {
            result = "invalid index";
        }
    }

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}

static void handleReboot(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    request->send(200, "application/json", "{\"ok\":true}");
    delay(100);
    ESP.restart();
}


void WebUI::push_event(const char *event, const char *json) {
    if (events) events->send(json, event, millis());
}

void WebUI::push_event(const char *event, const String &json) {
    if (events) events->send(json.c_str(), event, millis());
}

extern bool push_time_to_resmed();
extern bool pull_time_from_resmed(bool force);

static void handleTimeAction(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String body = getBody(request);
    String action;
    json_foreach_kv(body, [](const String &key, const String &val, void *p) {
        if (key == "action") *(String*)p = val;
    }, &action);

    String result = "unknown action";
    bool ok = false;

    if (action == "ntp_sync") {
        WiFiSetup::force_ntp_sync();
        result = "NTP resync triggered";
        ok = true;
    } else if (action == "sync_to_resmed") {
        ok = push_time_to_resmed();
        result = ok ? "ResMed clock updated" : "Failed (NTP not synced or device error)";
    } else if (action == "sync_from_resmed") {
        ok = pull_time_from_resmed(true);
        result = ok ? "ESP32 clock set from ResMed" : "Failed to read ResMed clock";
    }

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}

void WebUI::init(uint16_t port) {
    if (port == 0) return;

    http = new AsyncWebServer(port);
    events = new AsyncEventSource("/events");
    // Tie LiveWebConsumer PMD subscription to SSE client presence 
    events->onConnect([](AsyncEventSourceClient *) {
        LiveWebConsumer::acquire();
    });
    events->onDisconnect([](AsyncEventSourceClient *) {
        LiveWebConsumer::release();
    });
    http->addHandler(events);

    http->on("/", HTTP_GET, handleRoot);
    http->on("/api/status", HTTP_GET, handleStatus);
    http->on("/api/settings", HTTP_GET, handleGetSettings);
    http->on("/api/settings", HTTP_POST, handlePostSettings, NULL, handleJsonBody);
    http->on("/api/config", HTTP_GET, handleGetConfig);
    http->on("/api/config", HTTP_POST, handlePostConfig, NULL, handleJsonBody);
    http->on("/api/live", HTTP_GET, handleLive);
    http->on("/api/upload", HTTP_POST, handleUploadDone, handleUploadChunk);
    http->on("/api/ble", HTTP_GET, handleBleStatus);
    http->on("/api/ble", HTTP_POST, handleBleAction, NULL, handleJsonBody);
    http->on("/api/cmd", HTTP_POST, handleCmd, NULL, handleJsonBody);
    http->on("/api/flash", HTTP_GET, handleFlashStatus);
    http->on("/api/flash", HTTP_POST, handleFlashStart, NULL, handleJsonBody);
    http->on("/api/flash/cancel", HTTP_POST, handleFlashCancel);
    http->on("/api/time", HTTP_POST, handleTimeAction, NULL, handleJsonBody);
    http->on("/api/report", HTTP_GET, handleReport);
    http->on("/api/wifi", HTTP_GET, handleWifiGet);
    http->on("/api/wifi", HTTP_POST, handleWifiPost, NULL, handleJsonBody);
    http->on("/api/esp32/upload", HTTP_POST, handleEspOtaDone, handleEspOtaChunk);
    http->on("/api/reboot", HTTP_POST, handleReboot);

    DefaultHeaders::Instance().addHeader("Cache-Control", "no-store");

    http->begin();
    Log::logf(CAT_WEB, LOG_INFO, "[WEB] HTTP server on port %d\n", port);
}

static String build_status_payload() {
    system_state_t sys = Arbiter::get_state();
    oxi_state_t oxi = OxiBle::get_state();
    const oxi_reading_t &r = OxiArbiter::get_reading();

    String sj;
    sj.reserve(256);
    sj = "{";
    jsonAddString(sj, "system", system_state_name(sys), false);
    jsonAddInt(sj, "rop", Arbiter::get_cached_rop());
    jsonAddInt(sj, "mhr", Arbiter::get_cached_mhr());
    jsonAddString(sj, "oxi", oxi_state_name(oxi));
    jsonAddString(sj, "oxi_addr", OxiArbiter::get_source_id());
    jsonAddString(sj, "feeding", OxiArbiter::is_feeding() ? "yes" : "no");
    jsonAddInt(sj, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(sj, "pulse", r.valid ? r.pulse_bpm : -1);
    jsonAddInt(sj, "heap", ESP.getFreeHeap());
    jsonAddInt(sj, "rssi", WiFi.RSSI());
    jsonAddInt(sj, "uptime", millis() / 1000);
    sj += '}';
    return sj;
}

struct PublishedStatusSnapshot {
    int             rop      = INT_MIN;
    int             mhr      = INT_MIN;
    system_state_t  sys      = (system_state_t)0xFF;
    oxi_state_t     oxi      = (oxi_state_t)0xFF;
    bool            feeding  = false;
    int             spo2     = INT_MIN;
    int             pulse    = INT_MIN;
};
static PublishedStatusSnapshot last_published;

static PublishedStatusSnapshot current_snapshot() {
    PublishedStatusSnapshot s;
    s.rop     = Arbiter::get_cached_rop();
    s.mhr     = Arbiter::get_cached_mhr();
    s.sys     = Arbiter::get_state();
    s.oxi     = OxiBle::get_state();
    s.feeding = OxiArbiter::is_feeding();
    const oxi_reading_t &r = OxiArbiter::get_reading();
    s.spo2    = r.valid ? r.spo2 : -1;
    s.pulse   = r.valid ? r.pulse_bpm : -1;
    return s;
}

static bool snapshot_differs(const PublishedStatusSnapshot &a,
                             const PublishedStatusSnapshot &b) {
    return a.rop     != b.rop     ||
           a.mhr     != b.mhr     ||
           a.sys     != b.sys     ||
           a.oxi     != b.oxi     ||
           a.feeding != b.feeding ||
           a.spo2    != b.spo2    ||
           a.pulse   != b.pulse;
}

static uint32_t last_status_push = 0;

void WebUI::push_status_event() {
    if (!events || events->count() == 0) return;
    last_published = current_snapshot();
    last_status_push = millis();
    String sj = build_status_payload();
    events->send(sj.c_str(), "status", millis());
}

void WebUI::handle() {
    if (!events || events->count() == 0) return;
    if (snapshot_differs(current_snapshot(), last_published) ||
        millis() - last_status_push >= 10000) {
        WebUI::push_status_event();
    }
}
