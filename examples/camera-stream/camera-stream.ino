// ==============================================================
// Pardalote — Camera example (ESP32 only)
//
// Streams MJPEG video and serves JPEG snapshots over HTTP from
// an ESP32 camera module. PSRAM is required.
//
// IMPORTANT: define your board model BEFORE including the camera
// header. Pick one from the list below. Names match Espressif's
// CameraWebServer example.
//
// Browser side:
//   arduino.add('cam', new Camera());
//   arduino.on('ready', () => arduino.cam.attach(82));
// ==============================================================

// Pick ONE camera board model:
#define CAMERA_MODEL_WROVER_KIT
// #define CAMERA_MODEL_ESP_EYE
// #define CAMERA_MODEL_M5STACK_PSRAM
// #define CAMERA_MODEL_M5STACK_V2_PSRAM
// #define CAMERA_MODEL_M5STACK_WIDE
// #define CAMERA_MODEL_M5STACK_ESP32CAM
// #define CAMERA_MODEL_M5STACK_UNITCAM
// #define CAMERA_MODEL_M5STACK_CAMS3_UNIT
// #define CAMERA_MODEL_AI_THINKER
// #define CAMERA_MODEL_TTGO_T_JOURNAL
// #define CAMERA_MODEL_XIAO_ESP32S3
// #define CAMERA_MODEL_ESP32_CAM_BOARD
// #define CAMERA_MODEL_ESP32S3_CAM_LCD
// #define CAMERA_MODEL_ESP32S2_CAM_BOARD
// #define CAMERA_MODEL_ESP32S3_EYE
// #define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3
// #define CAMERA_MODEL_DFRobot_Romeo_ESP32S3

#include <Pardalote.h>
#include <PardaloteCamera.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
