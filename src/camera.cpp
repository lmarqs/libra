#include "camera.h"

#include <Arduino.h>
#include <esp_camera.h>

// AI-Thinker ESP32-CAM pin map (fixed by the module wiring). Note the camera's
// SCCB bus (SIOD/SIOC on 26/27) is separate from the MPU6050's I2C bus, and
// none of these collide with the SD-card pins we use for I2C/ESC in config.h.
namespace {
constexpr int kPwdn = 32;
constexpr int kReset = -1;
constexpr int kXclk = 0;
constexpr int kSiod = 26;
constexpr int kSioc = 27;
constexpr int kY9 = 35, kY8 = 34, kY7 = 39, kY6 = 36, kY5 = 21, kY4 = 19, kY3 = 18, kY2 = 5;
constexpr int kVsync = 25, kHref = 23, kPclk = 22;
}  // namespace

bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;  // see EscPair: servos avoid this timer
  c.pin_d0 = kY2;
  c.pin_d1 = kY3;
  c.pin_d2 = kY4;
  c.pin_d3 = kY5;
  c.pin_d4 = kY6;
  c.pin_d5 = kY7;
  c.pin_d6 = kY8;
  c.pin_d7 = kY9;
  c.pin_xclk = kXclk;
  c.pin_pclk = kPclk;
  c.pin_vsync = kVsync;
  c.pin_href = kHref;
  c.pin_sccb_sda = kSiod;
  c.pin_sccb_scl = kSioc;
  c.pin_pwdn = kPwdn;
  c.pin_reset = kReset;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.grab_mode = CAMERA_GRAB_LATEST;  // always stream the freshest frame

  if (psramFound()) {
    c.frame_size = FRAMESIZE_VGA;  // 640x480
    c.jpeg_quality = 12;           // lower = better quality, more bytes
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size = FRAMESIZE_QVGA;  // 320x240, fits in internal RAM
    c.jpeg_quality = 15;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }

  return esp_camera_init(&c) == ESP_OK;
}
