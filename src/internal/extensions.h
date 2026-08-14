// ==============================================================
// internal/extensions.h
// Pardalote Extension Registry — declarations only.
//
// Storage and function bodies live in extensions.cpp so the
// registry is shared across translation units. The user's .ino
// TU (which #includes extension headers) and the library's
// Pardalote.cpp TU (which calls dispatchExtension) both see the
// same _extRegistry.
//
// Extensions self-register by placing INSTALL_EXTENSION(...) at
// the bottom of their header.
// ==============================================================

#ifndef PARDALOTE_INTERNAL_EXTENSIONS_H
#define PARDALOTE_INTERNAL_EXTENSIONS_H

#include <Arduino.h>
#include "defs.h"
#include "protocol.h"

#define MAX_EXTENSIONS 8

// -------------------------------------------------------------------
// Extension handler signature.
// clientNum  — WebSocket client to reply to
// cmd        — the CMD byte from the received frame
// typeMask   — which params are float32
// params     — raw param bytes (use paramInt / paramFloat to read)
// nparams    — number of params
// payload    — optional blob (strings, bitmaps); may be nullptr
// payloadLen — byte length of payload
// -------------------------------------------------------------------
typedef void (*ExtHandler)(uint8_t clientNum,
                           uint8_t cmd, uint16_t typeMask,
                           uint8_t* params, uint8_t nparams,
                           uint8_t* payload, uint16_t payloadLen);

// Called on each new client connection so the extension can announce
// itself and re-send any persistent state.
typedef void (*ExtAnnouncer)(uint8_t clientNum);

// Called when a WebSocket client disconnects.
typedef void (*ExtDisconnecter)(uint8_t clientNum);

// Called every Arduino loop() iteration for time-based housekeeping.
typedef void (*ExtLooper)();

struct ExtEntry {
    uint16_t        deviceId;
    ExtHandler      handle;
    ExtAnnouncer    announce;
    ExtDisconnecter disconnect;  // nullptr if not needed
    ExtLooper       loop;        // nullptr if not needed
};

// -------------------------------------------------------------------
// Registry API — implementations in extensions.cpp.
// -------------------------------------------------------------------
void registerExtension(uint16_t        deviceId,
                       ExtHandler      handle,
                       ExtAnnouncer    announce,
                       ExtDisconnecter disconnect = nullptr,
                       ExtLooper       loop       = nullptr);

void dispatchExtension(uint8_t clientNum, uint16_t deviceId,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen);

void announceAll(uint8_t clientNum);
void disconnectAll(uint8_t clientNum);
void loopAll();

// -------------------------------------------------------------------
// ExtReadPoll — per-instance periodic-read registration with per-client
// gating, shared by every extension that supports read(interval,
// threshold). Mirrors the core pin Action table:
//
//   - one physical measurement per pollInterval (the fastest rate any
//     client registered);
//   - each client then gets its own send gate: at least interval[c] ms
//     since its last send AND at least threshold[c] change since the
//     value it last saw. threshold 0 = send every interval tick.
//
// The extension owns a static table of these (one per instance slot),
// registers clients from its READ handler, calls due()/gate() from its
// loop hook, and drops clients in its disconnect hook. Registered
// clients are always live: disconnect hooks remove them, so gate()
// needs no connectivity check.
// -------------------------------------------------------------------
struct ExtReadPoll {
    int8_t        instance = -1;   // extension instance id; -1 = empty slot
    uint8_t       unit     = 0;    // free byte for extension use (ultrasonic unit)
    unsigned long lastPoll = 0;
    unsigned long pollInterval = (unsigned long)-1;
    uint8_t       clientMask = 0;
    uint8_t       seededMask = 0;  // client has received at least one value
    uint16_t      interval[PARDALOTE_MAX_CLIENTS]  = {};
    uint16_t      threshold[PARDALOTE_MAX_CLIENTS] = {};
    int32_t       lastSent[PARDALOTE_MAX_CLIENTS]  = {};
    unsigned long lastSendTime[PARDALOTE_MAX_CLIENTS] = {};

    void reset(int8_t inst) {
        instance = inst; unit = 0; lastPoll = 0;
        pollInterval = (unsigned long)-1;
        clientMask = seededMask = 0;
    }

    void recompute() {
        unsigned long m = (unsigned long)-1;
        for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
            if ((clientMask & (1 << c)) && interval[c] < m) m = interval[c];
        pollInterval = m;
    }

    void setClient(uint8_t c, uint16_t ms, uint16_t thr) {
        clientMask  |= (1 << c);
        interval[c]  = ms;
        threshold[c] = thr;
        recompute();
    }

    // Record a value just sent to client c (registration seed).
    void seed(uint8_t c, int32_t val, unsigned long now) {
        lastSent[c] = val; lastSendTime[c] = now;
        seededMask |= (1 << c);
    }

    // Returns true when the slot is now empty (caller may free it).
    bool removeClient(uint8_t c) {
        clientMask &= ~(1 << c);
        seededMask &= ~(1 << c);
        if (!clientMask) { instance = -1; return true; }
        recompute();
        return false;
    }

    // One physical read due? Commits the poll time on true.
    bool due(unsigned long now) {
        if (instance == -1 || !clientMask) return false;
        if (now - lastPoll < pollInterval)  return false;
        lastPoll = now;
        return true;
    }

    // Should client c receive `val` now? Commits on true.
    bool gate(uint8_t c, int32_t val, unsigned long now) {
        const uint8_t bit = 1 << c;
        if (!(clientMask & bit)) return false;
        if (seededMask & bit) {
            if (now - lastSendTime[c] < interval[c]) return false;
            int32_t d = val - lastSent[c];
            if (d < 0) d = -d;
            if ((uint32_t)d < threshold[c]) return false;
        }
        lastSent[c] = val; lastSendTime[c] = now;
        seededMask |= bit;
        return true;
    }
};

// Find the poll slot for an instance; nullptr if none.
inline ExtReadPoll* extPollFind(ExtReadPoll* table, int n, int inst) {
    for (int i = 0; i < n; i++)
        if (table[i].instance == inst) return &table[i];
    return nullptr;
}

// Find-or-allocate the poll slot for an instance; nullptr if full.
inline ExtReadPoll* extPollGet(ExtReadPoll* table, int n, int inst) {
    ExtReadPoll* p = extPollFind(table, n, inst);
    if (p) return p;
    for (int i = 0; i < n; i++)
        if (table[i].instance == -1) { table[i].reset((int8_t)inst); return &table[i]; }
    return nullptr;
}

// Drop a departing client from every slot in a table.
inline void extPollDropClient(ExtReadPoll* table, int n, uint8_t c) {
    for (int i = 0; i < n; i++)
        if (table[i].instance != -1) table[i].removeClient(c);
}

// Call before Wire.begin() in any extension. Idempotent.
void ensureWire();

// Initialise Wire with custom SDA/SCL pins (ESP32 only). Returns true if
// it actually called Wire.begin(sda, scl), false if Wire was already
// initialised by an earlier ensureWire() / ensureWire(sda, scl) call —
// in which case the existing pins are kept and the caller should fall
// back to whatever the existing pins are.
bool ensureWire(int sda, int scl);

// -------------------------------------------------------------------
// INSTALL_EXTENSION(deviceId, handlerFn, announcerFn,
//                   [disconnectFn], [loopFn])
//
// Place at the bottom of an extension header.
// The static bool triggers registerExtension() during static
// initialisation — before setup() runs.
// -------------------------------------------------------------------
#define INSTALL_EXTENSION(deviceId, handlerFn, announcerFn, ...)        \
    static bool _ext_reg_##deviceId =                                   \
        (registerExtension(deviceId, handlerFn, announcerFn,            \
                           ##__VA_ARGS__), true);

#endif
