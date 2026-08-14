// ==============================================================
// internal/platform.h
// Platform detection — sets PLATFORM_UNO_R4 / PLATFORM_ESP32 and
// PARDALOTE_BOARD, and pulls in the right WiFi header.
//
// To override the board name (custom board or a specific ESP32
// variant), set it before including <Pardalote.h>:
//   #define PARDALOTE_BOARD "My Custom Board"
// ==============================================================

#pragma once

#if defined(ARDUINO_UNOR4_WIFI)
  #include "WiFiS3.h"
  #include "ArduinoGraphics.h"
  #include "Arduino_LED_Matrix.h"
  #include "TextAnimation.h"
  #define PLATFORM_UNO_R4
#elif defined(ARDUINO_UNOR4_MINIMA)
  // Same RA4M1 core as the R4 WiFi, but no radio and no LED matrix —
  // serial transport only. PARDALOTE_NO_WIFI compiles out the whole
  // WiFi + WebSocket path; every begin() form starts the serial
  // transport (the only one this hardware can have).
  #define PLATFORM_UNO_R4_MINIMA
  #define PARDALOTE_NO_WIFI
#elif defined(ESP32)
  #include <WiFi.h>
  #define PLATFORM_ESP32
#else
  #error "Unsupported platform — only UNO R4 WiFi/Minima and ESP32 are supported"
#endif

#ifndef PARDALOTE_BOARD
  #if defined(ARDUINO_UNOR4_WIFI)
    #define PARDALOTE_BOARD "UNO R4 WiFi"
  #elif defined(ARDUINO_UNOR4_MINIMA)
    #define PARDALOTE_BOARD "UNO R4 Minima"
  #elif defined(ARDUINO_DFROBOT_FIREBEETLE_2_ESP32C5)
    #define PARDALOTE_BOARD "FireBeetle 2 ESP32-C5"
  #elif defined(ARDUINO_ESP32_WROVER_KIT) || defined(ARDUINO_UPESY_WROVER) || defined(ARDUINO_ESP32_DEV)
    #define PARDALOTE_BOARD "ESP32-WROVER-DEV"
  #elif defined(ARDUINO_XIAO_ESP32C3)
    #define PARDALOTE_BOARD "XIAO ESP32-C3"
  #elif defined(ARDUINO_XIAO_ESP32S3) || defined(ARDUINO_XIAO_ESP32S3_PLUS)
    #define PARDALOTE_BOARD "XIAO ESP32-S3"
  #else
    #define PARDALOTE_BOARD "unknown"
    #warning "PARDALOTE_BOARD not recognised — add '#define PARDALOTE_BOARD \"Your Board\"' at the top of your sketch"
  #endif
#endif
