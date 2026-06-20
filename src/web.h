#pragma once

// Starts the web UI: two HTTP servers pinned to core 0 (away from the control
// loop on core 1).
//   :80  — the page (/), telemetry JSON (/telemetry), and gain/enable (/set)
//   :81  — the MJPEG camera stream (/stream), kept on its own server so the
//          long-lived stream handler can't block the control endpoints.
// Connect to the board's WiFi AP first (set up in main). Returns false if a
// server fails to start.
bool webStart();
