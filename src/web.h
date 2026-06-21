#pragma once

#include <Pid.h>  // Pid::Gains

// Optional WiFi web UI for live tuning. Brings up a SoftAP (open by default, or WPA2
// if LIBRA_AP_PASS is set) and a small HTTP server (ESP-IDF httpd, runs in its own
// task) serving a single page with sliders for the setpoint and PID gains.
//
// Scope: the web UI changes the target tilt, the gains, and arm/disarm (the software
// master-enable). It cannot change any limit, and a web-set setpoint is clamped to the
// tilt failsafe range. The firmware boots DISARMED; arming here only enables the control
// loop — the ESC supply is still gated by a separate HARDWARE switch, and the tilt
// failsafe still trips. Because arming is exposed, set LIBRA_AP_PASS (WPA2) if anyone
// could be in range — on an open AP any client can arm.
//
// This is the one place with cross-context shared state (the httpd task vs. the
// control loop); access goes through publish()/poll()/pollArm(), guarded internally.
namespace web {

// Bring up the SoftAP and start the HTTP server. Returns false on failure.
bool begin();

// Control loop -> web: push the latest values for the /telemetry endpoint and to
// keep the browser in sync with changes made elsewhere.
void publish(float angle, bool enabled, bool tripped, bool bench, const Pid::Gains& gains, float setpoint);

// Web -> control loop: if a client changed gains/setpoint since the last call,
// overwrite the caller's values (setpoint clamped to the tilt limit) and return
// true. Otherwise leave them untouched and return false.
bool poll(Pid::Gains& gains, float& setpoint);

// Web -> control loop: pending arm/disarm request from the UI. Returns 1 (arm),
// 0 (disarm), or -1 (none); clears the request. The control loop owns `enabled`.
int pollArm();

}  // namespace web
