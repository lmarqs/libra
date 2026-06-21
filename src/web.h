#pragma once

#include <Pid.h>  // Pid::Gains

// Optional WiFi web UI for live tuning. Brings up a WPA2 SoftAP and a small HTTP
// server (ESP-IDF httpd, runs in its own task) serving a single page with
// sliders for the setpoint and PID gains.
//
// Scope is deliberately narrow for safety: the web UI may change the target tilt
// and the gains, but NOT arm/disarm and NOT any limit. Arming stays serial-only,
// so a client on the AP can never start the props. The applied setpoint is
// clamped to the tilt failsafe range.
//
// This is the one place with cross-context shared state (the httpd task vs. the
// control loop); access goes through publish()/poll(), guarded internally.
namespace web {

// Bring up the SoftAP and start the HTTP server. Returns false on failure.
bool begin();

// Control loop -> web: push the latest values for the /telemetry endpoint and to
// keep the browser sliders in sync with changes made over serial.
void publish(float angle, bool enabled, bool tripped, const Pid::Gains& gains, float setpoint);

// Web -> control loop: if a client changed gains/setpoint since the last call,
// overwrite the caller's values (setpoint clamped to the tilt limit) and return
// true. Otherwise leave them untouched and return false.
bool poll(Pid::Gains& gains, float& setpoint);

}  // namespace web
