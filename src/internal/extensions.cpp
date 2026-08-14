// ==============================================================
// internal/extensions.cpp
// Single source of truth for the extension registry.
// ==============================================================

#include "extensions.h"
#include <Wire.h>

static ExtEntry _extRegistry[MAX_EXTENSIONS];
static uint8_t  _numExtensions   = 0;
static bool     _wireInitialised = false;

void registerExtension(uint16_t        deviceId,
                       ExtHandler      handle,
                       ExtAnnouncer    announce,
                       ExtDisconnecter disconnect,
                       ExtLooper       loop) {
    if (_numExtensions < MAX_EXTENSIONS) {
        _extRegistry[_numExtensions++] =
            { deviceId, handle, announce, disconnect, loop };
    }
    // Cannot use Serial here — static-init runs before Serial.begin().
}

void dispatchExtension(uint8_t clientNum, uint16_t deviceId,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {
    for (uint8_t i = 0; i < _numExtensions; i++) {
        if (_extRegistry[i].deviceId == deviceId) {
            _extRegistry[i].handle(clientNum, cmd, typeMask,
                                   params, nparams, payload, payloadLen);
            return;
        }
    }
    Serial.print(F("Unknown extension deviceId: "));
    Serial.println(deviceId);
}

void announceAll(uint8_t clientNum) {
    for (uint8_t i = 0; i < _numExtensions; i++) {
        if (_extRegistry[i].announce) {
            _extRegistry[i].announce(clientNum);
        }
    }
}

void disconnectAll(uint8_t clientNum) {
    for (uint8_t i = 0; i < _numExtensions; i++) {
        if (_extRegistry[i].disconnect) {
            _extRegistry[i].disconnect(clientNum);
        }
    }
}

void loopAll() {
    for (uint8_t i = 0; i < _numExtensions; i++) {
        if (_extRegistry[i].loop) {
            _extRegistry[i].loop();
        }
    }
}

// On ESP32, the default I2C timeout doesn't reliably stop Wire.requestFrom()
// from blocking indefinitely under heavy load (the peripheral can stall on a
// clock-stretch / marginal ACK and never recover). We set an explicit hard
// timeout so reads fail cleanly with 0 bytes instead of hanging the loop.
static void _wireSetSafeTimeout() {
#if defined(ESP32)
    Wire.setTimeOut(50);   // ms — failed reads return 0, caller handles it
#endif
}

void ensureWire() {
    if (!_wireInitialised) {
        Wire.begin();
        _wireSetSafeTimeout();
        _wireInitialised = true;
    }
}

bool ensureWire(int sda, int scl) {
    if (_wireInitialised) return false;
#if defined(ESP32)
    Wire.begin(sda, scl);
#else
    // UNO R4 (and other non-ESP32 Wire impls) don't support custom pins.
    // Fall back to default begin(); IMU's caller is already #ifdef-guarded
    // to ESP32 so this branch is effectively unreachable in real use.
    (void)sda; (void)scl;
    Wire.begin();
#endif
    _wireSetSafeTimeout();
    _wireInitialised = true;
    return true;
}
