// ==============================================================
// internal/wifi_config.h
// WiFi credential manager — declarations.
//
// Two credential sources are supported:
//   1. Compile-time secrets.h in the sketch folder. Pardalote.h
//      detects this via __has_include and a static-init binder
//      populates _pardaloteSecrets from the user's TU.
//   2. EEPROM-stored networks, managed via the Serial config menu.
//
// Implementations live in wifi_config.cpp.
// ==============================================================

#pragma once

#include <Arduino.h>

#define WIFI_MAX_NETS    5
#define SSID_LEN         33   // 32 chars + null
#define PASS_LEN         64   // 63 chars + null

struct WifiEntry {
    bool valid;
    char ssid[SSID_LEN];
    char pass[PASS_LEN];
};

struct WifiStore {
    uint32_t  magic;
    WifiEntry nets[WIFI_MAX_NETS];
};

// Compile-time credentials are funnelled through this struct so they
// can be read by the library at runtime. Pardalote.h has a static-init
// binder that copies SECRET_SSID / SECRET_PASS into it from the user's
// translation unit, which is the only TU that can see secrets.h.
struct PardaloteSecrets {
    const char* ssid;
    const char* pass;
};
extern PardaloteSecrets _pardaloteSecrets;

// Boot-watch probe. During the "press 'w' to configure" window, wifiConfigInit
// feeds each received byte to this callback (supplied by the core, which owns
// the USB envelope decoder). Return value:
//   0 = nothing yet — keep waiting
//   1 = a loose 'w' config keystroke — enter the config menu
//   2 = a USB takeover probe completed — stop the window; the core skips WiFi
// nullptr disables the USB watch (only 'w' is honoured).
typedef int (*PardaloteBootProbe)(uint8_t b);

// Call at the top of setup(), before WiFi.begin().
// Loads stored networks from EEPROM, optionally enters the Serial
// config menu (forced if no networks at all are available). During the
// config window it also watches USB for a takeover via `probe` (see above).
void wifiConfigInit(WifiStore& s, PardaloteBootProbe probe = nullptr);

// Tries each available network in order: secrets.h first (if bound), then
// EEPROM entries. Returns true once connected. If all networks fail it drops
// into the Serial config menu, then retries. While waiting for a connection it
// also drains USB via `probe` (see PardaloteBootProbe): if a takeover arrives
// it stops trying WiFi and returns FALSE, so the caller starts serial instead —
// the fallback when WiFi is slow/unreachable but a browser wants USB.
bool wifiConfigConnect(WifiStore& s, PardaloteBootProbe probe = nullptr);
