// ==============================================================
// internal/wifi_config.cpp
// WiFi credential manager — implementation.
// ==============================================================

#include "wifi_config.h"
#include "platform.h"

// Owns the compile-time-secrets struct. Pardalote.h's binder writes
// into this from the user's TU at static-init time, before begin().
PardaloteSecrets _pardaloteSecrets = { nullptr, nullptr };

// On no-WiFi boards (UNO R4 Minima) the whole credential manager
// compiles out — the serial transport needs none of it.
#ifndef PARDALOTE_NO_WIFI

#include <EEPROM.h>

// -------------------------------------------------------------------
// EEPROM layout
//   Offset  Bytes  Content
//   0       4      Magic number
//   4+n*98  98     Network n: valid(1) + ssid(33) + pass(64)
//   Total for 5 networks: 494 bytes
// -------------------------------------------------------------------
static constexpr uint32_t WIFI_MAGIC = 0xAB12CD34UL;

// -------------------------------------------------------------------
// Internal helpers
// -------------------------------------------------------------------
static void _wifiSave(const WifiStore& s) {
    EEPROM.put(0, s);
#ifdef PLATFORM_ESP32
    EEPROM.commit();
#endif
}

static int _wifiCount(const WifiStore& s) {
    int n = 0;
    for (int i = 0; i < WIFI_MAX_NETS; i++) if (s.nets[i].valid) n++;
    return n;
}

static void _wifiShow(const WifiStore& s) {
    bool any = false;
    for (int i = 0; i < WIFI_MAX_NETS; i++) {
        if (!s.nets[i].valid) continue;
        if (!any) { Serial.println(F("Stored networks:")); any = true; }
        Serial.print(F("  "));
        Serial.print(i + 1);
        Serial.print(F(". "));
        Serial.println(s.nets[i].ssid);
    }
    if (!any) Serial.println(F("No networks stored."));
}

// Read a line from Serial with echo. Password mode echoes '*'.
// Supports backspace. Returns false on timeout.
static bool _wifiReadLine(char* buf, int maxLen,
                          bool masked = false,
                          unsigned long timeoutMs = 30000) {
    int pos = 0;
    buf[0] = '\0';
    unsigned long deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (!Serial.available()) continue;
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (pos > 0) { buf[pos] = '\0'; Serial.println(); return true; }
            continue;
        }
        if (c == '\b' || c == 127) {
            if (pos > 0) { pos--; Serial.print(F("\b \b")); }
            continue;
        }
        if (pos < maxLen - 1) {
            buf[pos++] = c;
            Serial.print(masked ? '*' : c);
        }
    }
    Serial.println(F("\nTimeout."));
    return false;
}

static char _wifiReadChar() {
    char c;
    do {
        while (!Serial.available());
        c = Serial.read();
    } while (c == '\r' || c == '\n' || c == ' ');
    Serial.println(c);
    return c;
}

// Read the main menu keystroke, watching USB for a takeover so the board stays
// reachable over USB while idling in network management. Returns the keystroke,
// or sets *tookOver and returns 0 if a USB takeover probe arrived. (Sub-prompts —
// SSID / password / y-n — are active-typing states and are not watched.)
static char _wifiReadMenuChar(PardaloteBootProbe probe, bool* tookOver) {
    *tookOver = false;
    for (;;) {
        while (Serial.available()) {
            uint8_t b = (uint8_t)Serial.read();
            if (probe) {
                int ev = probe(b);
                if (ev == 2) { *tookOver = true; return 0; }   // USB takeover
                if (ev != 1 && ev != 3) continue;              // delimiter / envelope byte — not menu input
            }
            if (b != '\r' && b != '\n' && b != ' ') { Serial.println((char)b); return (char)b; }
        }
        delay(5);
    }
}

// -------------------------------------------------------------------
// Configuration menu. Returns true on normal exit, false if a USB takeover
// arrived at the menu prompt (the caller then starts serial).
// -------------------------------------------------------------------
static bool _wifiEnterConfig(WifiStore& s, PardaloteBootProbe probe = nullptr) {
    Serial.println(F("\n=== WiFi Configuration ==="));

    for (;;) {
        Serial.println(F("\n[a]dd  [d]elete  [c]lear all  [s]how  [x] exit"));
        Serial.print(F("> "));
        bool tookOver = false;
        char cmd = _wifiReadMenuChar(probe, &tookOver);
        if (tookOver) return false;   // USB takeover — abort config, go serial

        // ── Add ──
        if (cmd == 'a') {
            int slot = -1;
            if (_wifiCount(s) >= WIFI_MAX_NETS) {
                Serial.println(F("All 5 slots full."));
                _wifiShow(s);
                Serial.print(F("Replace which? (1-5, 0 = cancel): "));
                int n = _wifiReadChar() - '0';
                if (n < 1 || n > WIFI_MAX_NETS) { Serial.println(F("Cancelled.")); continue; }
                slot = n - 1;
            } else {
                for (int i = 0; i < WIFI_MAX_NETS; i++) {
                    if (!s.nets[i].valid) { slot = i; break; }
                }
            }
            char ssid[SSID_LEN], pass[PASS_LEN];
            Serial.print(F("SSID: "));
            if (!_wifiReadLine(ssid, sizeof(ssid)))         continue;
            Serial.print(F("Password: "));
            if (!_wifiReadLine(pass, sizeof(pass), true))   continue;
            s.nets[slot].valid = true;
            strncpy(s.nets[slot].ssid, ssid, SSID_LEN - 1);
            s.nets[slot].ssid[SSID_LEN - 1] = '\0';
            strncpy(s.nets[slot].pass, pass, PASS_LEN - 1);
            s.nets[slot].pass[PASS_LEN - 1] = '\0';
            _wifiSave(s);
            Serial.print(F("Saved: "));
            Serial.println(ssid);

        // ── Delete one ──
        } else if (cmd == 'd') {
            if (_wifiCount(s) == 0) { Serial.println(F("No networks stored.")); continue; }
            _wifiShow(s);
            Serial.print(F("Delete which? (0 = cancel): "));
            int n = _wifiReadChar() - '0';
            if (n < 1 || n > WIFI_MAX_NETS || !s.nets[n - 1].valid) {
                Serial.println(F("Cancelled.")); continue;
            }
            Serial.print(F("Delete '"));
            Serial.print(s.nets[n - 1].ssid);
            Serial.print(F("'? (y/n): "));
            if (_wifiReadChar() == 'y') {
                s.nets[n - 1].valid = false;
                _wifiSave(s);
                Serial.println(F("Deleted."));
            } else {
                Serial.println(F("Cancelled."));
            }

        // ── Clear all ──
        } else if (cmd == 'c') {
            Serial.print(F("Clear all networks? (y/n): "));
            if (_wifiReadChar() == 'y') {
                for (int i = 0; i < WIFI_MAX_NETS; i++) s.nets[i].valid = false;
                _wifiSave(s);
                Serial.println(F("Cleared."));
            } else {
                Serial.println(F("Cancelled."));
            }

        // ── Show ──
        } else if (cmd == 's') {
            _wifiShow(s);

        // ── Exit ──
        } else if (cmd == 'x') {
            return true;
        }
    }
}

// -------------------------------------------------------------------
// wifiConfigInit
// -------------------------------------------------------------------
void wifiConfigInit(WifiStore& s, PardaloteBootProbe probe) {
#ifdef PLATFORM_ESP32
    EEPROM.begin(sizeof(WifiStore));
#endif
    EEPROM.get(0, s);
    if (s.magic != WIFI_MAGIC) {
        s.magic = WIFI_MAGIC;
        for (int i = 0; i < WIFI_MAX_NETS; i++) s.nets[i].valid = false;
        _wifiSave(s);
    }

    // On UNO R4, native USB Serial needs the host to open the port.
    // Wait briefly so startup messages aren't lost.
    unsigned long t = millis();
    while (!Serial && millis() - t < 2000);

    Serial.println(F("\n=== Pardalote ==="));

    if (_pardaloteSecrets.ssid) {
        Serial.print(F("Network (secrets.h): "));
        Serial.println(_pardaloteSecrets.ssid);
    }

    // Loop until at least one network is available. If secrets.h
    // credentials are bound, we always have at least one option,
    // so skip the forced config loop even when EEPROM is empty.
    bool cameFromConfig = false;
    if (_pardaloteSecrets.ssid) {
        if (_wifiCount(s) == 0) {
            Serial.println(F("No EEPROM networks stored."));
        }
    } else {
        while (_wifiCount(s) == 0) {
            Serial.println(F("No WiFi networks stored."));
            // A takeover here sets _bootTakeover via `probe`; return so _beginWifi
            // sees it and starts serial instead of looping (still 0 networks).
            if (!_wifiEnterConfig(s, probe)) return;
            cameFromConfig = true;
        }
    }

    _wifiShow(s);

    // Skip the config window if we just came from it.
    if (!cameFromConfig) {
        Serial.println(F("Press 'w' within 5 seconds to configure WiFi (or connect over USB)..."));
        t = millis();
        bool done = false;
        while (!done && millis() - t < 5000) {
            // Drain everything available each pass (not one byte per delay) so a
            // USB takeover envelope isn't lost to a full FIFO.
            while (Serial.available()) {
                uint8_t b = (uint8_t)Serial.read();
                if (probe) {
                    int ev = probe(b);
                    if (ev == 2) { done = true; break; }   // USB takeover — the core skips WiFi
                    // ev == 1 is a 'w' (enter config); ev == 3 is other loose text
                    // (ignored here). A takeover inside config sets _bootTakeover,
                    // which _beginWifi checks after this returns.
                    if (ev == 1) { _wifiEnterConfig(s, probe); done = true; break; }
                } else if (b == 'w') {
                    _wifiEnterConfig(s); done = true; break;
                }
            }
            if (!done) delay(5);
        }
    }
}

// Wait up to `ms` for WiFi to connect, draining USB for a takeover along the
// way (so a blocking connect can be interrupted). Returns:
//   1 = WiFi connected, 2 = USB takeover, 0 = timed out (not connected).
static int _waitConnectOrTakeover(unsigned long ms, PardaloteBootProbe probe) {
    unsigned long t = millis() + ms;
    while (millis() < t) {
        if (WiFi.status() == WL_CONNECTED) return 1;
        if (probe) {
            // Drain fully each pass so a takeover envelope isn't lost to the FIFO.
            while (Serial.available()) {
                if (probe((uint8_t)Serial.read()) == 2) return 2;
            }
        }
        delay(20);
    }
    return (WiFi.status() == WL_CONNECTED) ? 1 : 0;
}

// -------------------------------------------------------------------
// wifiConfigConnect — tries secrets.h first, then EEPROM entries.
// -------------------------------------------------------------------
bool wifiConfigConnect(WifiStore& s, PardaloteBootProbe probe) {
#ifdef PLATFORM_UNO_R4
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println(F("WiFi module not found"));
        while (true) delay(1000);
    }
#endif

    for (;;) {

        // Try compile-time credentials first (if bound)
        if (_pardaloteSecrets.ssid) {
            Serial.print(F("Trying (secrets.h): "));
            Serial.println(_pardaloteSecrets.ssid);
            if (_pardaloteSecrets.pass) {
                WiFi.begin(_pardaloteSecrets.ssid, _pardaloteSecrets.pass);
            } else {
                WiFi.begin(_pardaloteSecrets.ssid);
            }
            int r = _waitConnectOrTakeover(10000, probe);
            if (r == 1) return true;
            if (r == 2) return false;   // USB takeover — caller starts serial
            Serial.println(F("Failed."));
            WiFi.disconnect();
            delay(200);
        }

        // Try EEPROM-stored networks in order
        for (int i = 0; i < WIFI_MAX_NETS; i++) {
            if (!s.nets[i].valid) continue;
            Serial.print(F("Trying: "));
            Serial.println(s.nets[i].ssid);
            WiFi.begin(s.nets[i].ssid, s.nets[i].pass);
            int r = _waitConnectOrTakeover(10000, probe);
            if (r == 1) return true;
            if (r == 2) return false;   // USB takeover — caller starts serial
            Serial.println(F("Failed."));
            WiFi.disconnect();
            delay(200);
        }

        Serial.println(F("Could not connect to any network."));
        if (!_wifiEnterConfig(s, probe)) return false;   // USB takeover in config → serial
    }
}

#endif   // PARDALOTE_NO_WIFI
