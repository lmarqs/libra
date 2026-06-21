#include "web.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_http_server.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"

namespace {

// httpd handlers run in their own task, so the control loop (loop()) and the web
// server are two execution contexts. This mux guards the small shared state
// below; critical sections only copy a few scalars, so they never stall control.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

struct Shared {
  // Pending command (web -> control loop). Consumed and cleared by poll().
  Pid::Gains cmd_gains{config::kKp, config::kKi, config::kKd};
  float cmd_setpoint = config::kSetpointDeg;
  bool cmd_pending = false;

  // Latest telemetry (control loop -> web), for the /telemetry endpoint.
  float angle = 0.0f;
  bool enabled = false;
  bool tripped = false;
  Pid::Gains gains{config::kKp, config::kKi, config::kKd};
  float setpoint = config::kSetpointDeg;
};
Shared g;

httpd_handle_t server = nullptr;

constexpr float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Single-page UI: read-only telemetry line + sliders for setpoint and PID gains.
// Served from a string literal (no asset pipeline). Arming is intentionally not
// exposed here — it stays serial-only.
constexpr char kIndexHtml[] = R"HTML(<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Libra</title>
<style>
body{font-family:system-ui,sans-serif;margin:0;background:#111;color:#eee}
.wrap{max-width:520px;margin:0 auto;padding:16px}
h1{font-size:1.2rem;letter-spacing:.15em}
#tel{font-family:ui-monospace,monospace;background:#1c1c1c;border-radius:8px;padding:12px;margin-bottom:8px}
#tel b{color:#7fd}
.row{display:flex;align-items:center;gap:10px;margin:14px 0}
.row label{width:3em;font-family:ui-monospace,monospace}
.row input{flex:1}
.row output{width:5.5em;text-align:right;font-family:ui-monospace,monospace}
.warn{color:#f96;font-size:.8rem;margin-top:18px}
.tripped{color:#f55;font-weight:bold}
</style>
<div class=wrap>
<h1>&#9878; LIBRA</h1>
<div id=tel>connecting&hellip;</div>
<div class=row><label>sp&deg;</label><input type=range id=sp min=-20 max=20 step=0.5><output id=spv></output></div>
<div class=row><label>kp</label><input type=range id=kp min=0 max=0.05 step=0.001><output id=kpv></output></div>
<div class=row><label>ki</label><input type=range id=ki min=0 max=0.01 step=0.0001><output id=kiv></output></div>
<div class=row><label>kd</label><input type=range id=kd min=0 max=0.005 step=0.0001><output id=kdv></output></div>
<div class=warn>Arm/disarm is serial-only (send <b>e</b> / <b>d</b>). The web UI sets target tilt and gains only.</div>
</div>
<script>
const $=id=>document.getElementById(id),K=['sp','kp','ki','kd'];
let drag=null;
K.forEach(k=>{const el=$(k),o=$(k+'v');
  el.addEventListener('input',()=>{o.value=el.value;drag=k;});
  el.addEventListener('change',()=>{fetch('/set?'+k+'='+el.value);drag=null;});});
async function poll(){
  try{
    const t=await (await fetch('/telemetry')).json();
    const st=t.tripped?'<span class=tripped>TRIPPED</span>':(t.enabled?'<b>ARMED</b>':'disarmed');
    $('tel').innerHTML='angle <b>'+t.angle.toFixed(1)+'&deg;</b> &nbsp; '+st;
    K.forEach(k=>{if(drag!==k){$(k).value=t[k];$(k+'v').value=(+t[k]).toFixed(k=='sp'?1:4);}});
  }catch(e){$('tel').textContent='disconnected';}
}
setInterval(poll,500);poll();
</script>
)HTML";

esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t telemetryHandler(httpd_req_t* req) {
  taskENTER_CRITICAL(&mux);
  const Shared s = g;
  taskEXIT_CRITICAL(&mux);

  char buf[256];
  const int n = snprintf(buf, sizeof(buf),
                         "{\"angle\":%.2f,\"enabled\":%s,\"tripped\":%s,"
                         "\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f,\"sp\":%.2f}",
                         s.angle, s.enabled ? "true" : "false", s.tripped ? "true" : "false", s.gains.kp, s.gains.ki,
                         s.gains.kd, s.setpoint);
  // snprintf returns the length it *would* have written; guard against passing a
  // truncated/oversized length to httpd_resp_send (which would over-read buf).
  if (n < 0 || n >= static_cast<int>(sizeof(buf))) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

// GET /set?kp=..&ki=..&kd=..&sp=..  — any subset. Stages a pending command for
// the control loop to apply on its next step. No arm/disarm here (serial-only).
esp_err_t setHandler(httpd_req_t* req) {
  char query[128];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char v[32];
    // Parse outside the lock — string/float parsing must not run with interrupts
    // disabled on the WiFi core. (v is reused; each lookup rewrites it before atof.)
    const bool has_kp = httpd_query_key_value(query, "kp", v, sizeof(v)) == ESP_OK;
    const float kp = has_kp ? atof(v) : 0.0f;
    const bool has_ki = httpd_query_key_value(query, "ki", v, sizeof(v)) == ESP_OK;
    const float ki = has_ki ? atof(v) : 0.0f;
    const bool has_kd = httpd_query_key_value(query, "kd", v, sizeof(v)) == ESP_OK;
    const float kd = has_kd ? atof(v) : 0.0f;
    const bool has_sp = httpd_query_key_value(query, "sp", v, sizeof(v)) == ESP_OK;
    const float sp = has_sp ? atof(v) : 0.0f;

    taskENTER_CRITICAL(&mux);
    if (has_kp) g.cmd_gains.kp = kp;
    if (has_ki) g.cmd_gains.ki = ki;
    if (has_kd) g.cmd_gains.kd = kd;
    if (has_sp) g.cmd_setpoint = sp;
    g.cmd_pending = true;
    taskEXIT_CRITICAL(&mux);
  }
  return httpd_resp_sendstr(req, "ok");
}

// AP association diagnostics. Prints unconditionally (not gated on log level) so a
// connect attempt is always visible. A STACONNECTED with no matching STADISCONNECTED
// means the client is on; a STADISCONNECTED reason code tells us why a join dropped.
void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    const uint8_t* m = info.wifi_ap_staconnected.mac;
    Serial.printf("libra: AP client joined %02x:%02x:%02x:%02x:%02x:%02x (aid=%d)\n", m[0], m[1], m[2], m[3], m[4],
                  m[5], info.wifi_ap_staconnected.aid);
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    const auto& d = info.wifi_ap_stadisconnected;
    Serial.printf("libra: AP client left  %02x:%02x:%02x:%02x:%02x:%02x (aid=%d)\n", d.mac[0], d.mac[1], d.mac[2],
                  d.mac[3], d.mac[4], d.mac[5], d.aid);
  }
}

void registerUri(const char* path, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t uri = {};
  uri.uri = path;
  uri.method = HTTP_GET;
  uri.handler = handler;
  httpd_register_uri_handler(server, &uri);
}

}  // namespace

namespace web {

bool begin() {
  WiFi.onEvent(onWifiEvent);  // log AP client join/leave (+ disconnect reason)
  WiFi.mode(WIFI_AP);
  // Open AP (no WPA2): the tuning UI exposes setpoint + gains only and can never
  // arm, so there is no thrust to protect here — and an open network avoids the
  // WPA2 4-way-handshake failures that blocked clients from associating.
  if (!WiFi.softAP(config::kApSsid)) {
    Serial.println("libra: WiFi SoftAP failed");
    return false;
  }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.core_id = 0;  // single-core C3; pin to the only core
  cfg.lru_purge_enable = true;
  if (httpd_start(&server, &cfg) != ESP_OK) {
    Serial.println("libra: HTTP server failed to start");
    return false;
  }
  registerUri("/", indexHandler);
  registerUri("/telemetry", telemetryHandler);
  registerUri("/set", setHandler);

  Serial.printf("libra: web UI on open AP '%s' -> http://%s/\n", config::kApSsid, WiFi.softAPIP().toString().c_str());
  return true;
}

void publish(float angle, bool enabled, bool tripped, const Pid::Gains& gains, float setpoint) {
  taskENTER_CRITICAL(&mux);
  g.angle = angle;
  g.enabled = enabled;
  g.tripped = tripped;
  g.gains = gains;
  g.setpoint = setpoint;
  taskEXIT_CRITICAL(&mux);
}

bool poll(Pid::Gains& gains, float& setpoint) {
  taskENTER_CRITICAL(&mux);
  const bool pending = g.cmd_pending;
  Pid::Gains cmd_gains = g.cmd_gains;
  float cmd_setpoint = g.cmd_setpoint;
  g.cmd_pending = false;
  taskEXIT_CRITICAL(&mux);

  if (!pending) return false;
  gains = cmd_gains;
  // Never let a remote client target a tilt past the failsafe.
  setpoint = clampf(cmd_setpoint, -config::kTiltLimitDeg, config::kTiltLimitDeg);
  return true;
}

}  // namespace web
