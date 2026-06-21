#pragma once

#include <Pid.h>  // Pid::Gains

// Optional WiFi web UI for live tuning. Brings up a SoftAP (open by default, or WPA2
// if LIBRA_AP_PASS is set) and a small HTTP server (ESP-IDF httpd, runs in its own
// task) serving a single page with sliders for the setpoint and PID gains.
//
// Scope is deliberately narrow for safety: the web UI may change the target tilt
// and the gains, but NOT any limit and it has no arming control at all — arming is
// a hardware switch on the ESC supply, so a client on the AP can never start the
// props. The applied setpoint is clamped to the tilt failsafe range.
//
// This is the one place with cross-context shared state (the httpd task vs. the
// control loop); access goes through publish()/poll(), guarded internally.
namespace web {

// Bring up the SoftAP and start the HTTP server. Returns false on failure.
bool begin();

// Control loop -> web: push the latest values for the /telemetry endpoint and to
// keep the browser sliders in sync with changes made over serial.
void publish(float angle, bool tripped, bool bench, const Pid::Gains& gains, float setpoint);

// Web -> control loop: if a client changed gains/setpoint since the last call,
// overwrite the caller's values (setpoint clamped to the tilt limit) and return
// true. Otherwise leave them untouched and return false.
bool poll(Pid::Gains& gains, float& setpoint);

}  // namespace web
