#pragma once

// Initializes the ESP32-CAM's OV2640 for JPEG capture (used by the MJPEG stream
// in web.cpp). Returns false if the camera fails to initialize. Call this
// BEFORE arming the ESCs so the camera claims LEDC timer 0 for its XCLK before
// ESP32Servo grabs the others (see EscPair).
bool cameraInit();
