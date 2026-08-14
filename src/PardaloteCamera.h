// ==============================================================
// PardaloteCamera.h
// Pardalote Camera Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0 License
//
// Streams MJPEG video and serves JPEG snapshots over HTTP from
// an ESP32 camera module. The browser accesses the stream
// directly via HTTP — no video data flows over the WebSocket.
//
// Usage in your sketch:
//   #define CAMERA_MODEL_WROVER_KIT   // select your board first
//   #include <PardaloteCamera.h>      // self-registers, no other changes needed
//
// Supported board defines (same names as ESP CameraWebServer example):
//   CAMERA_MODEL_WROVER_KIT
//   CAMERA_MODEL_ESP_EYE
//   CAMERA_MODEL_M5STACK_PSRAM
//   CAMERA_MODEL_M5STACK_V2_PSRAM
//   CAMERA_MODEL_M5STACK_WIDE
//   CAMERA_MODEL_M5STACK_ESP32CAM
//   CAMERA_MODEL_M5STACK_UNITCAM
//   CAMERA_MODEL_M5STACK_CAMS3_UNIT
//   CAMERA_MODEL_AI_THINKER
//   CAMERA_MODEL_TTGO_T_JOURNAL
//   CAMERA_MODEL_XIAO_ESP32S3
//   CAMERA_MODEL_ESP32_CAM_BOARD
//   CAMERA_MODEL_ESP32S3_CAM_LCD
//   CAMERA_MODEL_ESP32S2_CAM_BOARD
//   CAMERA_MODEL_ESP32S3_EYE
//   CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3
//   CAMERA_MODEL_DFRobot_Romeo_ESP32S3
//
// HTTP endpoints (started when JS calls camera.attach(port)):
//   http://<ip>:<port>/stream    — MJPEG stream
//   http://<ip>:<port>/snapshot  — single JPEG (CORS enabled)
// ==============================================================

#ifndef PARDALOTE_CAMERA_H
#define PARDALOTE_CAMERA_H

#if !defined(ESP32)
  #error "PardaloteCamera requires an ESP32 board"
#endif

#include "esp_camera.h"
#include "esp_http_server.h"
#include "Pardalote.h"

// -------------------------------------------------------------------
// Camera pin configurations — define ONE before including this file.
// Pin names match the ESP CameraWebServer example (camera_pins.h).
// -------------------------------------------------------------------

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  21
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    19
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM     5
#define Y2_GPIO_NUM     4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

#elif defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   4
#define SIOD_GPIO_NUM  18
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    36
#define Y8_GPIO_NUM    37
#define Y7_GPIO_NUM    38
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM    35
#define Y4_GPIO_NUM    14
#define Y3_GPIO_NUM    13
#define Y2_GPIO_NUM    34
#define VSYNC_GPIO_NUM  5
#define HREF_GPIO_NUM  27
#define PCLK_GPIO_NUM  25
#define LED_GPIO_NUM   22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM     5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_V2_PSRAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  22
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM     5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_WIDE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  22
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM     5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21
#define LED_GPIO_NUM    2

#elif defined(CAMERA_MODEL_M5STACK_ESP32CAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM     5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_UNITCAM)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM     5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_M5STACK_CAMS3_UNIT)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM 21
#define XCLK_GPIO_NUM  11
#define SIOD_GPIO_NUM  17
#define SIOC_GPIO_NUM  41
#define Y9_GPIO_NUM    13
#define Y8_GPIO_NUM     4
#define Y7_GPIO_NUM    10
#define Y6_GPIO_NUM     5
#define Y5_GPIO_NUM     7
#define Y4_GPIO_NUM    16
#define Y3_GPIO_NUM    15
#define Y2_GPIO_NUM     6
#define VSYNC_GPIO_NUM 42
#define HREF_GPIO_NUM  18
#define PCLK_GPIO_NUM  12
#define LED_GPIO_NUM   14

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define LED_GPIO_NUM    4  // 4 for flash LED, 33 for normal LED

#elif defined(CAMERA_MODEL_TTGO_T_JOURNAL)
#define PWDN_GPIO_NUM   0
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM  27
#define SIOD_GPIO_NUM  25
#define SIOC_GPIO_NUM  23
#define Y9_GPIO_NUM    19
#define Y8_GPIO_NUM    36
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    39
#define Y5_GPIO_NUM     5
#define Y4_GPIO_NUM    34
#define Y3_GPIO_NUM    35
#define Y2_GPIO_NUM    17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM  26
#define PCLK_GPIO_NUM  21

#elif defined(CAMERA_MODEL_XIAO_ESP32S3)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#elif defined(CAMERA_MODEL_ESP32_CAM_BOARD)
// The 18-pin header on the board has Y5 and Y3 swapped vs the flex connector.
// Set USE_BOARD_HEADER 1 if wiring via the board header, 0 for the flex connector.
#define USE_BOARD_HEADER   0
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    33
#define XCLK_GPIO_NUM      4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23
#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       19
#define Y7_GPIO_NUM       21
#define Y6_GPIO_NUM       39
#if USE_BOARD_HEADER
  #define Y5_GPIO_NUM     13
#else
  #define Y5_GPIO_NUM     35
#endif
#define Y4_GPIO_NUM       14
#if USE_BOARD_HEADER
  #define Y3_GPIO_NUM     35
#else
  #define Y3_GPIO_NUM     13
#endif
#define Y2_GPIO_NUM       34
#define VSYNC_GPIO_NUM     5
#define HREF_GPIO_NUM     27
#define PCLK_GPIO_NUM     25

#elif defined(CAMERA_MODEL_ESP32S3_CAM_LCD)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  40
#define SIOD_GPIO_NUM  17
#define SIOC_GPIO_NUM  18
#define Y9_GPIO_NUM    39
#define Y8_GPIO_NUM    41
#define Y7_GPIO_NUM    42
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM     3
#define Y4_GPIO_NUM    14
#define Y3_GPIO_NUM    47
#define Y2_GPIO_NUM    13
#define VSYNC_GPIO_NUM 21
#define HREF_GPIO_NUM  38
#define PCLK_GPIO_NUM  11

#elif defined(CAMERA_MODEL_ESP32S2_CAM_BOARD)
// The 18-pin header on the board has Y5 and Y3 swapped vs the flex connector.
#define USE_BOARD_HEADER   0
#define PWDN_GPIO_NUM      1
#define RESET_GPIO_NUM     2
#define XCLK_GPIO_NUM     42
#define SIOD_GPIO_NUM     41
#define SIOC_GPIO_NUM     18
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       39
#define Y7_GPIO_NUM       40
#define Y6_GPIO_NUM       15
#if USE_BOARD_HEADER
  #define Y5_GPIO_NUM     12
#else
  #define Y5_GPIO_NUM     13
#endif
#define Y4_GPIO_NUM        5
#if USE_BOARD_HEADER
  #define Y3_GPIO_NUM     13
#else
  #define Y3_GPIO_NUM     12
#endif
#define Y2_GPIO_NUM       14
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM      4
#define PCLK_GPIO_NUM      3

#elif defined(CAMERA_MODEL_ESP32S3_EYE)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

#elif defined(CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3) || defined(CAMERA_MODEL_DFRobot_Romeo_ESP32S3)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  45
#define SIOD_GPIO_NUM   1
#define SIOC_GPIO_NUM   2
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    46
#define Y7_GPIO_NUM     8
#define Y6_GPIO_NUM     7
#define Y5_GPIO_NUM     4
#define Y4_GPIO_NUM    41
#define Y3_GPIO_NUM    40
#define Y2_GPIO_NUM    39
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM  42
#define PCLK_GPIO_NUM   5

#else
  #error "PardaloteCamera: no camera model defined. Define one of the CAMERA_MODEL_* names before including <PardaloteCamera.h>"
#endif

// -------------------------------------------------------------------
// Idle shutdown — stop the HTTP server and deinit the camera this
// many milliseconds after the last WebSocket client disconnects.
// A reconnect within the window cancels the timer.
// -------------------------------------------------------------------
#define CAMERA_IDLE_TIMEOUT_MS 30000UL

// -------------------------------------------------------------------
// MJPEG stream boundary string
// -------------------------------------------------------------------
#define _CAM_BOUNDARY     "pardalote_boundary"
#define _CAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" _CAM_BOUNDARY

// -------------------------------------------------------------------
// CameraExt
// -------------------------------------------------------------------
class CameraExt {
private:
    inline static bool           _cameraReady     = false;
    inline static bool           _serverRunning   = false;
    inline static httpd_handle_t _server          = nullptr;
    inline static uint16_t       _port            = 82;
    inline static uint8_t        _quality         = 12;
    inline static framesize_t    _framesize       = FRAMESIZE_QVGA;
    inline static bool           _dramFallback    = false;  // true when no PSRAM: locked to QQVGA in DRAM
    inline static uint8_t        _clientCount     = 0;     // WebSocket clients currently connected
    inline static bool           _shutdownPending = false;
    inline static uint32_t       _shutdownStart   = 0;     // millis() when last client left

    // ----------------------------------------------------------------
    // MJPEG stream handler — runs in its own FreeRTOS task per client.
    // Loops until the client disconnects or a frame capture fails.
    // ----------------------------------------------------------------
    static esp_err_t _streamHandler(httpd_req_t* req) {
        httpd_resp_set_type(req, _CAM_CONTENT_TYPE);
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
        httpd_resp_set_hdr(req, "Pragma", "no-cache");

        const char* boundary = "\r\n--" _CAM_BOUNDARY "\r\n";
        char        partHdr[128];
        esp_err_t   res = ESP_OK;
        uint8_t     fails = 0;   // consecutive failed captures

        while (res == ESP_OK) {
            delay(1);  // yield to main loop so WebSocket events are processed between frames

            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) {
                // A single dropped frame — e.g. a transient FB-OVF at a
                // too-high framesize like HD on the OV2640 — shouldn't kill
                // the whole stream (that surfaces in the browser as
                // ERR_INCOMPLETE_CHUNKED_ENCODING). Skip it and retry; bail
                // only if captures keep failing (the sensor is truly stuck).
                if (++fails >= 5) {
                    Serial.println(F("[Camera] Frame capture failed repeatedly — closing stream"));
                    return ESP_FAIL;
                }
                continue;
            }
            fails = 0;

            size_t hlen = snprintf(partHdr, sizeof(partHdr),
                "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                (unsigned)fb->len);

            res = httpd_resp_send_chunk(req, boundary, strlen(boundary));
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, partHdr, hlen);
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

            esp_camera_fb_return(fb);
        }
        return res;
    }

    // ----------------------------------------------------------------
    // Single JPEG snapshot handler.
    // ----------------------------------------------------------------
    static esp_err_t _snapshotHandler(httpd_req_t* req) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) return httpd_resp_send_500(req);

        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
        httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");

        esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        return res;
    }

    // ----------------------------------------------------------------
    // Initialise the camera hardware. Called once on first attach().
    // ----------------------------------------------------------------
    static bool _initCamera() {
        if (_cameraReady) return true;

        camera_config_t cfg = {};
        cfg.ledc_channel = LEDC_CHANNEL_0;
        cfg.ledc_timer   = LEDC_TIMER_0;
        cfg.pin_d0       = Y2_GPIO_NUM;
        cfg.pin_d1       = Y3_GPIO_NUM;
        cfg.pin_d2       = Y4_GPIO_NUM;
        cfg.pin_d3       = Y5_GPIO_NUM;
        cfg.pin_d4       = Y6_GPIO_NUM;
        cfg.pin_d5       = Y7_GPIO_NUM;
        cfg.pin_d6       = Y8_GPIO_NUM;
        cfg.pin_d7       = Y9_GPIO_NUM;
        cfg.pin_xclk     = XCLK_GPIO_NUM;
        cfg.pin_pclk     = PCLK_GPIO_NUM;
        cfg.pin_vsync    = VSYNC_GPIO_NUM;
        cfg.pin_href     = HREF_GPIO_NUM;
        cfg.pin_sccb_sda = SIOD_GPIO_NUM;
        cfg.pin_sccb_scl = SIOC_GPIO_NUM;
        cfg.pin_pwdn     = PWDN_GPIO_NUM;
        cfg.pin_reset    = RESET_GPIO_NUM;
        cfg.xclk_freq_hz = 20000000;
        cfg.pixel_format = PIXFORMAT_JPEG;
        cfg.frame_size   = _framesize;
        cfg.jpeg_quality = _quality;
        if (psramFound()) {
            cfg.fb_count    = 2;                  // double-buffer for smooth streaming
            cfg.fb_location = CAMERA_FB_IN_PSRAM;
            _dramFallback   = false;
        } else {
            cfg.fb_count    = 1;
            cfg.fb_location = CAMERA_FB_IN_DRAM;
            cfg.frame_size  = FRAMESIZE_QQVGA;    // 160×120 — only size that fits in DRAM
            _framesize      = FRAMESIZE_QQVGA;    // keep our cached size in step with the hardware
            _dramFallback   = true;               // lock out resolution changes — a larger DMA
                                                  // buffer than QQVGA can't fit, and would FB-OVF
            Serial.println(F("[Camera] No PSRAM — using DRAM, forced to QQVGA"));
        }
        cfg.grab_mode    = CAMERA_GRAB_LATEST;

        esp_err_t err = esp_camera_init(&cfg);
        if (err != ESP_OK) {
            Serial.print(F("[Camera] Init failed: 0x"));
            Serial.println(err, HEX);
            return false;
        }

        _cameraReady = true;
        Serial.println(F("[Camera] Hardware ready"));
        return true;
    }

    // ----------------------------------------------------------------
    // Start the HTTP server on the requested port.
    // ----------------------------------------------------------------
    static bool _startServer(uint16_t port) {
        if (_serverRunning) return true;

        httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
        cfg.server_port       = port;
        cfg.max_open_sockets  = 4;                        // stream + snapshot + headroom
        cfg.uri_match_fn      = httpd_uri_match_wildcard; // strip query strings before routing
        cfg.send_wait_timeout = 15;                       // seconds — absorbs occasional WiFi
        cfg.recv_wait_timeout = 15;                       // backpressure spikes on slower radios

        if (httpd_start(&_server, &cfg) != ESP_OK) {
            Serial.println(F("[Camera] HTTP server start failed"));
            return false;
        }

        // Trailing * absorbs the cache-buster query string appended by camera.js
        // (/stream?_t=<timestamp>) so every reconnect gets a fresh TCP connection.
        httpd_uri_t streamUri = {
            .uri      = "/stream*",
            .method   = HTTP_GET,
            .handler  = _streamHandler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(_server, &streamUri);

        httpd_uri_t snapshotUri = {
            .uri      = "/snapshot*",
            .method   = HTTP_GET,
            .handler  = _snapshotHandler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(_server, &snapshotUri);

        _serverRunning = true;
        _port          = port;
        Serial.print(F("[Camera] HTTP server on port "));
        Serial.println(port);
        return true;
    }

    // ----------------------------------------------------------------
    // Stop the HTTP server immediately — frees WiFi bandwidth so
    // WebSocket clients can reconnect without competing with MJPEG.
    // ----------------------------------------------------------------
    static void _stopHttpServer() {
        if (!_serverRunning) return;
        httpd_stop(_server);
        _server        = nullptr;
        _serverRunning = false;
        Serial.println(F("[Camera] HTTP server stopped"));
    }

    // ----------------------------------------------------------------
    // Deinit the camera hardware — saves power after a longer idle.
    // ----------------------------------------------------------------
    static void _stopCamera() {
        if (!_cameraReady) return;
        esp_camera_deinit();
        _cameraReady = false;
        Serial.println(F("[Camera] Camera deinit"));
    }

public:
    // ----------------------------------------------------------------
    // Main dispatch — called by the extension registry for every frame
    // whose TARGET == DEVICE_CAMERA.
    // ----------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t /*typeMask*/,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* /*payload*/, uint16_t /*payloadLen*/) {
        switch (cmd) {

            case CMD_CAMERA_INIT: {
                if (nparams < 2) return;
                int id   = (int)paramInt(params, 0);
                int port = (int)paramInt(params, 1);

                if (!_initCamera())              return;
                if (!_startServer((uint16_t)port)) return;

                // Echo confirmed port back to all clients so every browser
                // connecting after the first also learns the stream URL.
                FrameBuilder fb;
                fb.begin(CMD_CAMERA_INIT, DEVICE_CAMERA);
                fb.addInt(id);
                fb.addInt((int)_port);
                Pardalote.broadcastFrame(fb);
                break;
            }

            case CMD_CAMERA_SET_RES: {
                if (nparams < 2) return;
                // No-PSRAM boards run a single QQVGA frame buffer in DRAM. A larger
                // framesize won't fit that buffer and the driver spews cam_hal: FB-OVF
                // with a broken stream, so refuse the change and stay at QQVGA.
                if (_dramFallback) {
                    Serial.println(F("[Camera] No PSRAM — resolution locked to QQVGA, ignoring set-res"));
                    break;
                }
                _framesize = (framesize_t)(int)paramInt(params, 1);
                if (_cameraReady) {
                    sensor_t* s = esp_camera_sensor_get();
                    if (s) s->set_framesize(s, _framesize);
                }
                break;
            }

            case CMD_CAMERA_SET_QUALITY: {
                if (nparams < 2) return;
                _quality = (uint8_t)constrain((int)paramInt(params, 1), 0, 63);
                if (_cameraReady) {
                    sensor_t* s = esp_camera_sensor_get();
                    if (s) s->set_quality(s, _quality);
                }
                break;
            }

            default:
                Serial.print(F("[Camera] Unknown cmd 0x"));
                Serial.println(cmd, HEX);
                break;
        }
    }

    // ----------------------------------------------------------------
    // Called on every new client connection.
    // Announces the extension. We do not re-send stream state here:
    // the ESP32 camera hardware can only serve one MJPEG client at a
    // time (esp_camera_fb_get() is single-consumer), so each client
    // must call attach() itself if it wants a stream URL. When it
    // does, the Arduino broadcasts CMD_CAMERA_INIT and the requesting
    // client builds its URL via handleMessage().
    // ----------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        _clientCount++;
        _shutdownPending = false;  // cancel idle shutdown if a new client arrived

        FrameBuilder fa;
        fa.begin(CMD_ANNOUNCE, DEVICE_CAMERA);
        fa.addInt(PROTOCOL_VERSION_MAJOR);
        fa.addInt(1);   // max instances
        Pardalote.sendFrame(clientNum, fa);
    }

    // ----------------------------------------------------------------
    // Called by the extension registry when any WebSocket client drops.
    // HTTP server stops immediately so WiFi is free for reconnection.
    // Camera hardware stays warm for CAMERA_IDLE_TIMEOUT_MS in case
    // a client reconnects quickly — only then is it fully deinited.
    // ----------------------------------------------------------------
    static void disconnect(uint8_t /*clientNum*/) {
        if (_clientCount > 0) _clientCount--;
        if (_clientCount == 0) {
            _stopHttpServer();
            if (_cameraReady) {
                _shutdownPending = true;
                _shutdownStart   = millis();
                Serial.print(F("[Camera] Camera warm for "));
                Serial.print(CAMERA_IDLE_TIMEOUT_MS / 1000);
                Serial.println(F("s"));
            }
        }
    }

    // ----------------------------------------------------------------
    // Called every loop() iteration. Deinits the camera hardware once
    // the timeout has elapsed with no clients reconnecting.
    // ----------------------------------------------------------------
    static void loop() {
        if (!_shutdownPending) return;
        if (millis() - _shutdownStart < CAMERA_IDLE_TIMEOUT_MS) return;
        _shutdownPending = false;
        _stopCamera();
    }
};

INSTALL_EXTENSION(DEVICE_CAMERA, CameraExt::handle, CameraExt::announce,
                  CameraExt::disconnect, CameraExt::loop)

#endif
