// ==============================================================
// PardaloteUltrasonic.h
// Pardalote Ultrasonic Sensor Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0 License
//
// Supports 3-wire (trig+echo on one pin) and 4-wire (HC-SR04 style)
// sensors. Add #include <PardaloteUltrasonic.h> to your sketch.
//
// Distance is measured on demand — CMD_ULTRASONIC_READ triggers a
// blocking pulseIn() on the Arduino and sends the result back.
// Periodic polling is driven by a JS-side timer.
//
// NOTE: pulseIn() blocks the loop for up to timeoutMs milliseconds.
// Keep the JS poll interval at least 3-4× longer than the timeout,
// e.g. setTimeout(30) with read(150) or setTimeout(40) with read(200).
// Longer timeouts reduce the maximum WebSocket responsiveness.
//
// Response value is distance in tenths of the requested unit
// (e.g. 235 = 23.5 cm, or 23.5 inches). -1 = timeout / no echo.
// ==============================================================

#ifndef PARDALOTE_ULTRASONIC_H
#define PARDALOTE_ULTRASONIC_H

#include "Pardalote.h"

#define MAX_ULTRASONIC 4

class UltrasonicExt {
private:
    inline static int16_t  _trigPins[MAX_ULTRASONIC]  = { -1,-1,-1,-1 };
    inline static int16_t  _echoPins[MAX_ULTRASONIC]  = { -1,-1,-1,-1 };  // -1 = 3-wire
    inline static uint16_t _timeoutMs[MAX_ULTRASONIC] = { 30,30,30,30 };
    inline static bool     _attached[MAX_ULTRASONIC]  = {};

    // Sketch-created sensors (PardaloteUltrasonic.attach("name", trig, echo)).
    // The name is what the browser binds (arduino.<name>); announce() replays a
    // CMD_SHARE frame for these so every connecting browser materialises the
    // object. Browser-created sensors have _sketchOwned = false. Mirrors the
    // actuator extensions' sketch-attach path.
    inline static bool     _sketchOwned[MAX_ULTRASONIC] = {};
    inline static char     _names[MAX_ULTRASONIC][MAX_SHARE_NAME + 1] = {};

    // Periodic reads — per-client registration + gating.
    // Default threshold 3 tenths (≈3 mm): the HC-SR04's noise floor.
    inline static ExtReadPoll _polls[MAX_ULTRASONIC] = {};
    static constexpr uint16_t DEFAULT_THRESHOLD = 3;

    static bool validId(int id) { return id >= 0 && id < MAX_ULTRASONIC; }

    static void sendReadTo(uint8_t clientNum, int id, int32_t dist) {
        FrameBuilder fb;
        fb.begin(CMD_ULTRASONIC_READ, DEVICE_ULTRASONIC);
        fb.addInt(id);
        fb.addInt(dist);
        Pardalote.sendFrame(clientNum, fb);
    }

    // Returns distance in tenths of the requested unit, or -1 on timeout.
    static int32_t measure(int id, uint8_t unit) {
        int trig = _trigPins[id];
        int echo = (_echoPins[id] == -1) ? trig : (int)_echoPins[id];
        unsigned long timeout_us = _timeoutMs[id] * 1000UL;

        // Trigger pulse
        pinMode(trig, OUTPUT);
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        // For 3-wire sensors, switch the shared pin to input for echo
        if (_echoPins[id] == -1) pinMode(trig, INPUT);
        else                     pinMode(echo, INPUT);

        unsigned long duration = pulseIn(echo, HIGH, timeout_us);
        if (duration == 0) return -1;

        // Speed of sound: 343 m/s = 0.0343 cm/µs. Round trip ÷ 2, × 10 for tenths:
        //   tenths_cm   = duration × 0.0343 / 2 × 10 = duration × 343  / 2000
        //   tenths_inch = duration × 0.01350 / 2 × 10 = duration × 135 / 2000
        if (unit == UNIT_INCH)
            return (int32_t)(duration * 135L / 2000L);
        else
            return (int32_t)(duration * 343L / 2000L);
    }

public:
    // -------------------------------------------------------------------
    // Sketch-facing accessors (used by the PardaloteUltrasonic object).
    // -------------------------------------------------------------------
    static bool attachedId(int id) { return validId(id) && _attached[id]; }
    static int  listAttached(int* out, int max) {
        int n = 0;
        for (int i = 0; i < MAX_ULTRASONIC && n < max; i++) if (_attached[i]) out[n++] = i;
        return n;
    }
    // Blocking measurement for the sketch — returns tenths of `unit` (like the
    // wire), or -1 on timeout. The Access object divides to decimal units.
    static int32_t measureId(int id, uint8_t unit) {
        return (validId(id) && _attached[id]) ? measure(id, unit) : -1;
    }

    // -------------------------------------------------------------------
    // Sketch-created sensors — PardaloteUltrasonic.attach("name", trig, echo).
    //
    // Creation and browser visibility are one act (see the servo extension for
    // the rationale): the sensor is attached through the same handler a browser
    // attach uses, and a CMD_SHARE frame (+ attach state) is broadcast so
    // connected browsers materialise arduino.<name>. announce() replays it for
    // later connects. Logical ids allocated TOP-DOWN (browser add() grows from
    // 0 up). Idempotent per name. echo = -1 → 3-wire. Returns the logical id,
    // or -1 if no slot is free.
    // -------------------------------------------------------------------
    static int sketchAttach(const char* name, int trig, int echo) {
        if (name == nullptr || name[0] == '\0') return -1;

        int id = -1;
        for (int i = 0; i < MAX_ULTRASONIC; i++)
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        if (id < 0)
            for (int i = MAX_ULTRASONIC - 1; i >= 0; i--)
                if (!_attached[i] && !_sketchOwned[i]) { id = i; break; }
        if (id < 0) {
            Serial.print(F("Ultrasonic: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        // Attach through the same code path a browser attach takes.
        Pardalote.command(DEVICE_ULTRASONIC, CMD_ULTRASONIC_ATTACH, id, trig, echo);

        // Tell any connected browsers now (no-op when none are connected;
        // announce() covers future connects): name → attach. Echo pin only for
        // 4-wire, matching the browser's own attach frame.
        broadcastShare(id);
        FrameBuilder fa;
        fa.begin(CMD_ULTRASONIC_ATTACH, DEVICE_ULTRASONIC);
        fa.addInt(id); fa.addInt(_trigPins[id]);
        if (_echoPins[id] != -1) fa.addInt(_echoPins[id]);
        Pardalote.broadcastFrame(fa);

        return id;
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_ULTRASONIC);
        fb.addInt(id);
        fb.addString(_names[id]);
        Pardalote.broadcastFrame(fb);
    }

    // -------------------------------------------------------------------
    // Main dispatch
    // -------------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {
        if (nparams < 1) return;
        int id = (int)paramInt(params, 0);
        if (!validId(id)) {
            Serial.print(F("Ultrasonic: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            case CMD_ULTRASONIC_ATTACH: {
                if (nparams < 2) return;
                int trig = (int)paramInt(params, 1);
                int echo = (nparams > 2) ? (int)paramInt(params, 2) : -1;

                // Skip if already in the requested state — avoids duplicate
                // Serial output on JS reconnect / 'ready' re-attach.
                bool stateChanged = !_attached[id]
                                    || _trigPins[id] != trig
                                    || _echoPins[id] != echo;
                if (!stateChanged) break;

                _trigPins[id]  = (int16_t)trig;
                _echoPins[id]  = (int16_t)echo;
                _timeoutMs[id] = 30;
                _attached[id]  = true;

                Serial.print(F("Ultrasonic ")); Serial.print(id);
                Serial.print(F(" attached: trig=")); Serial.print(trig);
                if (echo != -1) { Serial.print(F(" echo=")); Serial.println(echo); }
                else              Serial.println(F(" (3-wire)"));
                break;
            }

            case CMD_ULTRASONIC_DETACH: {
                _attached[id] = false;
                _trigPins[id] = -1;
                _echoPins[id] = -1;
                ExtReadPoll* p = extPollFind(_polls, MAX_ULTRASONIC, id);
                if (p) p->instance = -1;   // stop any periodic read
                Serial.print(F("Ultrasonic ")); Serial.print(id);
                Serial.println(F(" detached"));
                break;
            }

            // READ — params [id, unit?, interval?, threshold?].
            // Always answers the requester immediately. interval > 0
            // registers a board-side per-client periodic read; interval < 0
            // (JS END) removes this client's registration; absent/0 = one-shot.
            case CMD_ULTRASONIC_READ: {
                if (!_attached[id]) return;
                uint8_t unit = (nparams > 1) ? (uint8_t)paramInt(params, 1) : UNIT_CM;
                long ms  = (nparams > 2) ? paramInt(params, 2) : 0;
                long thr = (nparams > 3) ? paramInt(params, 3) : 0;

                if (ms < 0) {   // END — unregister this client
                    ExtReadPoll* p = extPollFind(_polls, MAX_ULTRASONIC, id);
                    if (p) p->removeClient(clientNum);
                    break;
                }

                int32_t dist = measure(id, unit);
                sendReadTo(clientNum, id, dist);

                if (ms > 0) {
                    ExtReadPoll* p = extPollGet(_polls, MAX_ULTRASONIC, id);
                    if (!p) break;
                    p->unit = unit;   // last registration's unit wins
                    p->setClient(clientNum, (uint16_t)constrain(ms, 1, 65535),
                                 thr > 0 ? (uint16_t)thr : DEFAULT_THRESHOLD);
                    p->seed(clientNum, dist, millis());
                }
                break;
            }

            case CMD_ULTRASONIC_SET_TIMEOUT: {
                if (!_attached[id] || nparams < 2) return;
                uint16_t t = (uint16_t)constrain((int)paramInt(params, 1), 1, 1000);
                _timeoutMs[id] = t;
                Serial.print(F("Ultrasonic ")); Serial.print(id);
                Serial.print(F(" timeout=")); Serial.println(t);
                break;
            }

            default:
                Serial.print(F("Ultrasonic: unknown cmd 0x"));
                Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Called on each new client connection
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_ULTRASONIC);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_ULTRASONIC);
        Pardalote.sendFrame(clientNum, fb);

        // Send full attach state for any currently attached sensors:
        // pins, and echo timeout.
        for (int i = 0; i < MAX_ULTRASONIC; i++) {
            if (!_attached[i]) continue;

            // Sketch-created sensor: send its SHARE frame FIRST so the browser
            // materialises arduino.<name> before the state frames below.
            if (_sketchOwned[i]) {
                FrameBuilder fsh;
                fsh.begin(CMD_SHARE, DEVICE_ULTRASONIC);
                fsh.addInt(i);
                fsh.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fsh);
            }

            FrameBuilder fa;
            fa.begin(CMD_ULTRASONIC_ATTACH, DEVICE_ULTRASONIC);
            fa.addInt(i);
            fa.addInt(_trigPins[i]);
            if (_echoPins[i] != -1) fa.addInt(_echoPins[i]);
            Pardalote.sendFrame(clientNum, fa);

            FrameBuilder ft;
            ft.begin(CMD_ULTRASONIC_SET_TIMEOUT, DEVICE_ULTRASONIC);
            ft.addInt(i);
            ft.addInt(_timeoutMs[i]);
            Pardalote.sendFrame(clientNum, ft);
        }
    }

    // -------------------------------------------------------------------
    // Loop hook — board-side periodic reads. One blocking measure per due
    // registration (at the fastest requested rate), then per-client gating.
    // -------------------------------------------------------------------
    static void loop() {
        const unsigned long now = millis();
        for (int i = 0; i < MAX_ULTRASONIC; i++) {
            ExtReadPoll& p = _polls[i];
            if (!p.due(now)) continue;
            if (!validId(p.instance) || !_attached[p.instance]) { p.instance = -1; continue; }
            const int32_t dist = measure(p.instance, p.unit);
            for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
                if (p.gate(c, dist, now)) sendReadTo(c, p.instance, dist);
        }
    }

    // Client disconnect — drop its read registrations.
    static void disconnect(uint8_t clientNum) {
        extPollDropClient(_polls, MAX_ULTRASONIC, clientNum);
    }
};

// -------------------------------------------------------------------
// PardaloteUltrasonic — sketch-facing collection of distance sensors.
//
// Create a sensor from the sketch (the browser sees it automatically as
// arduino.<name> — a full Ultrasonic instance, identical to a browser-created
// one):
//   int front = PardaloteUltrasonic.attach("front", 7, 8);   // name, trig, echo
//   float cm  = PardaloteUltrasonic.read(front);             // blocking measure
//
// 3-wire sensors share one pin: attach("front", 7). Like the PWM servo, the
// inside/outside boundary is per sensor (each owns its pins). read() blocks in
// pulseIn() for up to the timeout — don't call it every loop; the browser
// polls on its own timer. A private sensor shouldn't be here — read it with
// pulseIn() directly.
// -------------------------------------------------------------------
class PardaloteUltrasonicAccess {
public:
    // attach(name, trigPin, echoPin?) — create a sensor and make it visible to
    // browsers as arduino.<name>. echoPin omitted → 3-wire (shared pin).
    // Returns the logical id for read()/etc., or -1 if no slot is free.
    // Names >MAX_SHARE_NAME (15) are truncated. Idempotent per name.
    int attach(const char* name, int trigPin, int echoPin = -1) const {
        return UltrasonicExt::sketchAttach(name, trigPin, echoPin);
    }

    int  scan(int* out, int max) const { return UltrasonicExt::listAttached(out, max); }
    bool attached(int id)        const { return UltrasonicExt::attachedId(id); }

    // read(id) — blocking measurement in centimetres (decimal), -1 on timeout.
    // readInches(id) — same in inches. Matches arduino.<name>.distance on JS.
    float read(int id)       const { int32_t t = UltrasonicExt::measureId(id, UNIT_CM);   return (t < 0) ? -1.0f : t / 10.0f; }
    float readInches(int id) const { int32_t t = UltrasonicExt::measureId(id, UNIT_INCH); return (t < 0) ? -1.0f : t / 10.0f; }

    // setTimeout(id, ms) — cap on pulseIn() per read (1–1000 ms). Longer =
    // greater max range but a longer loop stall on a miss.
    void setTimeout(int id, int ms) const {
        Pardalote.command(DEVICE_ULTRASONIC, CMD_ULTRASONIC_SET_TIMEOUT, id, ms);
    }
};
inline PardaloteUltrasonicAccess PardaloteUltrasonic;

INSTALL_EXTENSION(DEVICE_ULTRASONIC, UltrasonicExt::handle, UltrasonicExt::announce,
                  UltrasonicExt::disconnect, UltrasonicExt::loop)

#endif
