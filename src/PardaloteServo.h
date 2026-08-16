// ==============================================================
// PardaloteServo.h
// Pardalote Servo Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
//
// Add #include <PardaloteServo.h> to your sketch — the extension
// self-registers, no further setup is required.
//
// Supports up to MAX_SERVOS simultaneously attached servos.
// Each servo is addressed by a logical instance ID, assigned by
// the JS side when calling arduino.add('name', new Servo()) (ids
// grow from 0 up) or by the board when the sketch calls
// PardaloteServo.attach("name", pin) (ids grow from the top down).
// ==============================================================

#ifndef PARDALOTE_SERVO_H
#define PARDALOTE_SERVO_H

#if defined(ESP32)
  #include <ESP32Servo.h>
#else
  #include <Servo.h>
#endif

#include "Pardalote.h"

#define MAX_SERVOS 8

class ServoExt {
private:
    inline static Servo   _servos[MAX_SERVOS];
    inline static int16_t _pins[MAX_SERVOS]     = { -1,-1,-1,-1,-1,-1,-1,-1 };
    inline static int16_t _angles[MAX_SERVOS]   = { 90,90,90,90,90,90,90,90 };
    inline static int16_t _minPulse[MAX_SERVOS] = { 544,544,544,544,544,544,544,544 };
    inline static int16_t _maxPulse[MAX_SERVOS] = { 2400,2400,2400,2400,2400,2400,2400,2400 };
    inline static bool    _attached[MAX_SERVOS] = {};

    // Sketch-created servos (PardaloteServo.attach("name", pin)). The name
    // is what the browser binds (arduino.<name>); announce() replays a
    // CMD_SHARE frame for these so every connecting browser materialises
    // the object. Browser-created servos have _sketchOwned = false.
    inline static bool    _sketchOwned[MAX_SERVOS] = {};
    inline static char    _names[MAX_SERVOS][MAX_SHARE_NAME + 1] = {};

    // Soft angle limits (safety) — every commanded angle is clamped on the
    // board, so neither the browser nor a sketch can push past them.
    inline static bool    _limitSet[MAX_SERVOS] = {};
    inline static int16_t _limitMin[MAX_SERVOS] = {};
    inline static int16_t _limitMax[MAX_SERVOS] = {};

    // On-board timed-move interpolation state (see loop()).
    inline static bool     _moving[MAX_SERVOS]     = {};
    inline static int16_t  _fromAngle[MAX_SERVOS]  = {};
    inline static int16_t  _toAngle[MAX_SERVOS]    = {};
    inline static uint32_t _startMs[MAX_SERVOS]    = {};
    inline static uint32_t _durMs[MAX_SERVOS]      = {};
    inline static uint32_t _lastStepMs[MAX_SERVOS] = {};

    // Min ms between interpolation writes — servos refresh PWM at ~50 Hz, so
    // writing faster is wasted loop time. Angle is always from true elapsed
    // time (millis()), so loop jitter never desynchronises a group move.
    static const uint32_t STEP_MS = 20;

    // Gesture segment schedule (CMD_SERVO_GESTURE). A gesture generalises the
    // single-segment timed move above: the interpolation state (_fromAngle …
    // _durMs, _curveNow) always describes the CURRENT segment, and loop()
    // advances _segIndex through _segs on each boundary. A plain writeTimed()
    // is just the degenerate _segCount == 0 case. ~8 B/segment × 16 × 8 servos
    // ≈ 1 KB RAM — fine on ESP32 / UNO R4; MAX_SERVO_SEGMENTS caps it.
    static const uint8_t MAX_SERVO_SEGMENTS = 16;
    struct Seg { uint8_t curve; uint16_t dur; int32_t value; };
    inline static Seg     _segs[MAX_SERVOS][MAX_SERVO_SEGMENTS] = {};
    inline static uint8_t _segCount[MAX_SERVOS] = {};   // 0 = no gesture (plain timed move)
    inline static uint8_t _segIndex[MAX_SERVOS] = {};   // current segment
    inline static uint8_t _segFlags[MAX_SERVOS] = {};   // GESTURE_FLAG_* (reference frame, loop)
    inline static uint8_t _curveNow[MAX_SERVOS] = {};   // easing id of the current segment

    static bool validId(int id) { return id >= 0 && id < MAX_SERVOS; }

    // Easing shared with every other extension — see pardaloteEase() in
    // defs.h (matches curveShape() in pardalote.js). CURVE_BACK returns >1
    // mid-flight (the overshoot), which loadSegment's write re-clamps.

    // Load segment `idx` as the current interpolation move. `from` is the
    // servo's live angle (dynamic capture); the target is a delta off it
    // (relative) or the segment value itself (absolute), clamped to limits.
    static void loadSegment(int id, uint8_t idx, uint32_t startMs) {
        const Seg& s = _segs[id][idx];
        int32_t base   = _angles[id];
        int32_t target = (_segFlags[id] & GESTURE_FLAG_ABSOLUTE) ? s.value : base + s.value;
        _fromAngle[id]  = (int16_t)base;
        _toAngle[id]    = (int16_t)clampAngle(id, target);
        _curveNow[id]   = s.curve;
        _startMs[id]    = startMs;
        _durMs[id]      = s.dur ? s.dur : 1;   // guard /0
        _segIndex[id]   = idx;
        _lastStepMs[id] = 0;                   // force a write on the first tick
        _moving[id]     = true;
    }

    // End a running gesture (or single timed move): land, drop the schedule,
    // tell the browser once via the existing DONE frame.
    static void finishGesture(int id) {
        _moving[id]   = false;
        _segCount[id] = 0;
        broadcastDone(id, _angles[id]);
    }

    // Periodic reads — per-client registration + gating.
    // Default threshold 1 degree.
    inline static ExtReadPoll _polls[MAX_SERVOS] = {};
    static constexpr uint16_t DEFAULT_THRESHOLD = 1;

    // Poll the current angle (delegating to the Servo library) and cache it.
    // (readAngle() below is the sketch accessor for the cached value.)
    static int32_t pollAngle(int id) {
        int angle = _attached[id] ? _servos[id].read() : -1;
        if (_attached[id]) _angles[id] = (int16_t)angle;
        return angle;
    }

    static void sendReadTo(uint8_t clientNum, int id, int32_t angle) {
        FrameBuilder fb;
        fb.begin(CMD_SERVO_READ, DEVICE_SERVO);
        fb.addInt(id);
        fb.addInt(angle);
        Pardalote.sendFrame(clientNum, fb);
    }

    // Clamp an angle to 0–180 and, if set, the soft limits.
    static int clampAngle(int id, int angle) {
        angle = constrain(angle, 0, 180);
        if (_limitSet[id]) angle = constrain(angle, _limitMin[id], _limitMax[id]);
        return angle;
    }

    // Begin (or immediately apply, if dur == 0) a timed move to `angle`.
    static void startTimed(int id, int angle, uint32_t dur, uint32_t now) {
        angle = clampAngle(id, angle);
        if (dur == 0) {
            _servos[id].write(angle);
            _angles[id] = (int16_t)angle;
            _moving[id] = false;
            broadcastDone(id, angle);
            return;
        }
        _fromAngle[id]  = _angles[id];
        _toAngle[id]    = (int16_t)angle;
        _startMs[id]    = now;
        _durMs[id]      = dur;
        _lastStepMs[id] = 0;          // force a write on the first tick
        _moving[id]     = true;
        _segCount[id]   = 0;          // a plain timed move is a 1-segment, non-gesture case
        _curveNow[id]   = CURVE_LINEAR;
    }

    static void broadcastDone(int id, int angle) {
        FrameBuilder fb;
        fb.begin(CMD_SERVO_DONE, DEVICE_SERVO);
        fb.addInt(id);
        fb.addInt(angle);
        Pardalote.broadcastFrame(fb);
    }

public:
    // -------------------------------------------------------------------
    // Sketch-facing read accessors (used by the PardaloteServo object).
    // -------------------------------------------------------------------
    static int  readAngle(int id)  { return (validId(id) && _attached[id]) ? _angles[id] : -1; }
    static bool isMovingId(int id) { return validId(id) && _attached[id] && _moving[id]; }
    static bool attachedId(int id) { return validId(id) && _attached[id]; }
    static int  listAttached(int* out, int max) {
        int n = 0;
        for (int i = 0; i < MAX_SERVOS && n < max; i++) if (_attached[i]) out[n++] = i;
        return n;
    }
    // Echo a sketch-issued write to the browser so it sets its cached angle
    // exactly as if the browser had written it (CMD_SERVO_WRITE — silent sync).
    static void echoAngle(int id, int angle) {
        if (!validId(id) || !_attached[id]) return;
        FrameBuilder fb;
        fb.begin(CMD_SERVO_WRITE, DEVICE_SERVO);
        fb.addInt(id);
        fb.addInt(clampAngle(id, angle));   // echo what was actually applied
        Pardalote.broadcastFrame(fb);
    }

    // -------------------------------------------------------------------
    // Sketch-created servos — PardaloteServo.attach("name", pin).
    //
    // Creation and browser visibility are one act: the servo is attached
    // through the same handler a browser attach uses, and a CMD_SHARE
    // frame (+ attach/angle state) is broadcast so connected browsers
    // materialise arduino.<name> immediately. announce() replays the same
    // sequence for browsers that connect later.
    //
    // Logical ids are allocated from the TOP of the range downward —
    // browser-assigned ids grow from 0 upward, so the two sides can't
    // collide until all MAX_SERVOS slots are in use.
    //
    // Idempotent on the name: calling attach again with a name that is
    // already sketch-owned reuses its id (and the shared handler skips
    // the detach/attach cycle when the params are unchanged too).
    // -------------------------------------------------------------------
    static int sketchAttach(const char* name, int pin, int minP, int maxP) {
        if (name == nullptr || name[0] == '\0') return -1;

        int id = -1;
        for (int i = 0; i < MAX_SERVOS; i++) {
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        }
        if (id < 0) {
            for (int i = MAX_SERVOS - 1; i >= 0; i--) {
                if (!_attached[i] && !_sketchOwned[i]) { id = i; break; }
            }
        }
        if (id < 0) {
            Serial.print(F("Servo: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        // Attach through the same code path a browser attach takes.
        Pardalote.command(DEVICE_SERVO, CMD_SERVO_ATTACH, id, pin, minP, maxP);

        // Tell any connected browsers now (no-op when none are connected;
        // announce() covers future connects): name → attach → angle, the
        // same order announce() replays.
        broadcastShare(id);
        FrameBuilder fa;
        fa.begin(CMD_SERVO_ATTACH, DEVICE_SERVO);
        fa.addInt(id); fa.addInt(_pins[id]);
        fa.addInt(_minPulse[id]); fa.addInt(_maxPulse[id]);
        Pardalote.broadcastFrame(fa);
        FrameBuilder fw;
        fw.begin(CMD_SERVO_WRITE, DEVICE_SERVO);
        fw.addInt(id); fw.addInt(_angles[id]);
        Pardalote.broadcastFrame(fw);

        return id;
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_SERVO);
        fb.addInt(id);
        fb.addString(_names[id]);
        Pardalote.broadcastFrame(fb);
    }

    // -------------------------------------------------------------------
    // Main dispatch — called by the extension registry for every frame
    // whose TARGET == DEVICE_SERVO.
    // -------------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {

        // Global (multi-servo) command — handled before per-instance id read.
        if (cmd == CMD_SERVO_SYNC_TIMED) {
            uint32_t dur = (nparams >= 1) ? (uint32_t)paramInt(params, 0) : 0;
            uint32_t now = millis();
            int count = payloadLen / 2;                 // { logicalId u8, angle u8 }
            for (int i = 0; i < count; i++) {
                int sid = payload[i * 2];
                int ang = payload[i * 2 + 1];
                if (validId(sid) && _attached[sid]) startTimed(sid, ang, dur, now);
            }
            return;
        }

        // Global (multi-servo) gesture — one or more channel blocks, each a
        // segment schedule the board then plays locally (see defs.h layout).
        if (cmd == CMD_SERVO_GESTURE) {
            uint32_t now = millis();
            uint16_t off = 0;
            while (off + 3 <= payloadLen) {
                int     sid   = payload[off];
                uint8_t flags = payload[off + 1];
                uint8_t count = payload[off + 2];
                off += 3;
                if ((uint32_t)off + (uint32_t)count * 7 > payloadLen) break;   // malformed — stop
                if (validId(sid) && _attached[sid] && count > 0) {
                    uint8_t n = count > MAX_SERVO_SEGMENTS ? MAX_SERVO_SEGMENTS : count;
                    for (uint8_t i = 0; i < n; i++) {
                        const uint8_t* r = payload + off + i * 7;
                        _segs[sid][i].curve = r[0];
                        _segs[sid][i].dur   = (uint16_t)(((uint16_t)r[1] << 8) | r[2]);
                        _segs[sid][i].value = (int32_t)(((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) |
                                                        ((uint32_t)r[5] <<  8) |  (uint32_t)r[6]);
                    }
                    _segCount[sid] = n;
                    _segFlags[sid] = flags;
                    loadSegment(sid, 0, now);
                }
                off += (uint16_t)count * 7;             // skip the whole declared block, even if capped
            }
            return;
        }

        if (nparams < 1) return;
        int id = (int)paramInt(params, 0);
        if (!validId(id)) {
            Serial.print(F("Servo: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            case CMD_SERVO_ATTACH: {
                if (nparams < 2) return;
                int pin  = (int)paramInt(params, 1);
                int minP = (nparams > 2) ? (int)paramInt(params, 2) : 544;
                int maxP = (nparams > 3) ? (int)paramInt(params, 3) : 2400;

                // Skip the detach/attach cycle (and the Serial print) when the
                // state is already what was requested. JS's reconnect logic and
                // 'ready' handlers can both call attach() with identical params;
                // treating those as no-ops avoids redundant PWM glitches and
                // duplicate Serial output.
                bool stateChanged = !_attached[id]
                                    || _pins[id]     != pin
                                    || _minPulse[id] != minP
                                    || _maxPulse[id] != maxP;
                if (!stateChanged) break;

                if (_attached[id]) _servos[id].detach();
                _servos[id].attach(pin, minP, maxP);
                _pins[id]     = (int16_t)pin;
                _minPulse[id] = (int16_t)minP;
                _maxPulse[id] = (int16_t)maxP;
                _attached[id] = true;
                _angles[id]   = 90;

                Serial.print(F("Servo ")); Serial.print(id);
                Serial.print(F(" attached to pin ")); Serial.println(pin);
                break;
            }

            case CMD_SERVO_DETACH:
                if (_attached[id]) {
                    _servos[id].detach();
                    _attached[id] = false;
                    _pins[id]     = -1;
                    _limitSet[id] = false;
                    ExtReadPoll* p = extPollFind(_polls, MAX_SERVOS, id);
                    if (p) p->instance = -1;   // stop any periodic read
                    Serial.print(F("Servo ")); Serial.print(id);
                    Serial.println(F(" detached"));
                }
                break;

            case CMD_SERVO_WRITE: {
                if (!_attached[id] || nparams < 2) return;
                _moving[id] = false; _segCount[id] = 0;   // an immediate write cancels a timed move / gesture
                int angle = clampAngle(id, (int)paramInt(params, 1));
                _servos[id].write(angle);
                _angles[id] = (int16_t)angle;
                break;
            }

            case CMD_SERVO_WRITE_MICROSECONDS: {
                if (!_attached[id] || nparams < 2) return;
                _moving[id] = false; _segCount[id] = 0;   // an immediate write cancels a timed move / gesture
                int us = constrain((int)paramInt(params, 1), 544, 2400);
                if (_limitSet[id]) {
                    // Translate the angle limits into the pulse domain.
                    long span  = _maxPulse[id] - _minPulse[id];
                    long usMin = _minPulse[id] + (long)_limitMin[id] * span / 180;
                    long usMax = _minPulse[id] + (long)_limitMax[id] * span / 180;
                    us = constrain(us, (int)usMin, (int)usMax);
                }
                _servos[id].writeMicroseconds(us);
                break;
            }

            case CMD_SERVO_WRITE_TIMED: {
                if (!_attached[id] || nparams < 3) return;
                int angle    = (int)paramInt(params, 1);
                uint32_t dur = (uint32_t)paramInt(params, 2);
                startTimed(id, angle, dur, millis());
                break;
            }

            case CMD_SERVO_STOP:
                if (_attached[id]) { _moving[id] = false; _segCount[id] = 0; }   // hold current angle, drop any gesture
                break;

            case CMD_SERVO_SET_LIMITS: {
                if (!_attached[id] || nparams < 4) return;
                int lo = constrain((int)paramInt(params, 1), 0, 180);
                int hi = constrain((int)paramInt(params, 2), 0, 180);
                _limitMin[id] = (int16_t)min(lo, hi);
                _limitMax[id] = (int16_t)max(lo, hi);
                _limitSet[id] = paramInt(params, 3) != 0;
                break;
            }

            // READ — params [id, interval?, threshold?]. Delegates to
            // the underlying Servo library's read():
            //   - Arduino Servo (UNO R4): returns the last written angle
            //   - ESP32Servo: reads back from the LEDC PWM duty register
            // Always answers the requester immediately. interval > 0
            // registers a board-side per-client periodic read; interval < 0
            // (JS END) removes this client's registration; absent/0 = one-shot.
            case CMD_SERVO_READ: {
                long ms  = (nparams > 1) ? paramInt(params, 1) : 0;
                long thr = (nparams > 2) ? paramInt(params, 2) : 0;

                if (ms < 0) {   // END — unregister this client
                    ExtReadPoll* p = extPollFind(_polls, MAX_SERVOS, id);
                    if (p) p->removeClient(clientNum);
                    break;
                }

                int32_t angle = pollAngle(id);
                sendReadTo(clientNum, id, angle);

                if (ms > 0 && _attached[id]) {
                    ExtReadPoll* p = extPollGet(_polls, MAX_SERVOS, id);
                    if (!p) break;
                    p->setClient(clientNum, (uint16_t)constrain(ms, 1, 65535),
                                 thr > 0 ? (uint16_t)thr : DEFAULT_THRESHOLD);
                    p->seed(clientNum, angle, millis());
                }
                break;
            }

            // Query — answers the REQUESTER only (JS attached() resolves a
            // promise with this; another browser's question isn't our answer).
            case CMD_SERVO_ATTACHED: {
                bool isAttached = _attached[id] && _servos[id].attached();

                FrameBuilder fb;
                fb.begin(CMD_SERVO_ATTACHED, DEVICE_SERVO);
                fb.addInt(id);
                fb.addInt(isAttached ? 1 : 0);
                Pardalote.sendFrame(clientNum, fb);
                break;
            }

            default:
                Serial.print(F("Servo: unknown cmd 0x"));
                Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Called on every new client connection.
    // Sends an ANNOUNCE frame so JS knows this extension is present,
    // then re-sends current attach state so reconnected clients stay
    // in sync.
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_SERVO);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_SERVOS);
        Pardalote.sendFrame(clientNum, fb);

        // Send full attach state for any currently attached servos:
        // pin, pulse range, and last known angle.
        for (int i = 0; i < MAX_SERVOS; i++) {
            if (!_attached[i]) continue;

            // Sketch-created servo: send its SHARE frame FIRST so the
            // browser materialises arduino.<name> before the attach/state
            // frames below arrive to sync it.
            if (_sketchOwned[i]) {
                FrameBuilder fs;
                fs.begin(CMD_SHARE, DEVICE_SERVO);
                fs.addInt(i);
                fs.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fs);
            }

            FrameBuilder fa;
            fa.begin(CMD_SERVO_ATTACH, DEVICE_SERVO);
            fa.addInt(i);
            fa.addInt(_pins[i]);
            fa.addInt(_minPulse[i]);
            fa.addInt(_maxPulse[i]);
            Pardalote.sendFrame(clientNum, fa);

            FrameBuilder fw;
            fw.begin(CMD_SERVO_WRITE, DEVICE_SERVO);
            fw.addInt(i);
            fw.addInt(_angles[i]);
            Pardalote.sendFrame(clientNum, fw);

            // Replay soft limits if set.
            if (_limitSet[i]) {
                FrameBuilder fl;
                fl.begin(CMD_SERVO_SET_LIMITS, DEVICE_SERVO);
                fl.addInt(i); fl.addInt(_limitMin[i]); fl.addInt(_limitMax[i]); fl.addInt(1);
                Pardalote.sendFrame(clientNum, fl);
            }
        }
    }

    // -------------------------------------------------------------------
    // Loop hook — runs every Pardalote.run(). Advances any in-progress
    // timed moves. The angle is computed from true elapsed time (millis()),
    // so a group of servos sharing one duration stays in phase regardless of
    // loop-rate jitter, and they all finish on the same tick.
    // -------------------------------------------------------------------
    static void loop() {
        uint32_t now = millis();
        for (int i = 0; i < MAX_SERVOS; i++) {
            if (!_attached[i] || !_moving[i]) continue;
            uint32_t elapsed = now - _startMs[i];
            if (elapsed >= _durMs[i]) {
                // Land exactly on this segment's end.
                _servos[i].write(_toAngle[i]);
                _angles[i] = _toAngle[i];
                if (_segCount[i] > 0 && _segIndex[i] + 1 < _segCount[i]) {
                    // Chain to the next segment. Its start is the previous
                    // start+dur (not `now`), so the timeline never drifts.
                    loadSegment(i, _segIndex[i] + 1, _startMs[i] + _durMs[i]);
                } else {
                    finishGesture(i);                               // gesture (or plain move) complete
                }
            } else if (now - _lastStepMs[i] >= STEP_MS) {
                float t   = (float)elapsed / (float)_durMs[i];       // 0..1
                float sh  = pardaloteEase(_curveNow[i], t);          // eased position along the segment
                long  d   = (long)_toAngle[i] - (long)_fromAngle[i];
                int   ang = clampAngle(i, _fromAngle[i] + (int)lroundf(d * sh)); // re-clamp: BACK can overshoot
                _servos[i].write(ang);
                _angles[i]     = (int16_t)ang;
                _lastStepMs[i] = now;
            }
        }

        // Board-side periodic reads — per-client gating.
        for (int i = 0; i < MAX_SERVOS; i++) {
            ExtReadPoll& p = _polls[i];
            if (!p.due(now)) continue;
            if (!validId(p.instance) || !_attached[p.instance]) { p.instance = -1; continue; }
            const int32_t angle = pollAngle(p.instance);
            for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
                if (p.gate(c, angle, now)) sendReadTo(c, p.instance, angle);
        }
    }

    // Client disconnect — drop its read registrations.
    static void disconnect(uint8_t clientNum) {
        extPollDropClient(_polls, MAX_SERVOS, clientNum);
    }
};

// -------------------------------------------------------------------
// PardaloteServo — sketch-facing "bus" of PWM servos.
//
// Create a servo from the sketch (the browser sees it automatically as
// arduino.<name> — a full Servo instance, identical to one the browser
// created itself):
//   int pan = PardaloteServo.attach("pan", 9);   // name, pin → logical id
//   PardaloteServo.write(pan, 90);
//
// Or read/drive actuators the browser configured, addressed by logical id:
//   int ids[8];
//   int n = PardaloteServo.scan(ids, 8);      // attached servos → their ids
//   int angle = PardaloteServo.read(ids[0]);  // current angle (0–180), −1 if none
//
// Either way, every write goes through the same board-side handler the
// browser uses — soft limits are enforced and the browser's cached angle
// stays in sync (auto-echo). A servo that should stay private to the
// sketch shouldn't be here at all: use the plain Servo/ESP32Servo
// library directly, the way unshared pins just use pinMode().
// -------------------------------------------------------------------
class PardaloteServoAccess {
public:
    // attach(name, pin, minPulse?, maxPulse?) — create a servo and make it
    // visible to browsers as arduino.<name>. Returns the logical id for
    // write()/read()/etc., or -1 if no slot is free. Names longer than
    // MAX_SHARE_NAME (15) chars are truncated. Idempotent per name.
    int attach(const char* name, int pin,
               int minPulse = 544, int maxPulse = 2400) const {
        return ServoExt::sketchAttach(name, pin, minPulse, maxPulse);
    }

    int  scan(int* out, int max) const { return ServoExt::listAttached(out, max); }
    int  read(int id)            const { return ServoExt::readAngle(id); }
    bool isMoving(int id)        const { return ServoExt::isMovingId(id); }
    bool attached(int id)        const { return ServoExt::attachedId(id); }

    // Writes — routed through the same handler the browser uses.
    // Writes — command the servo, then echo the angle to the browser so its
    // record matches, exactly as if the browser had issued the write.
    void write(int id, int angle)              const { Pardalote.command(DEVICE_SERVO, CMD_SERVO_WRITE, id, angle);            ServoExt::echoAngle(id, angle); }
    void writeTimed(int id, int angle, int ms) const { Pardalote.command(DEVICE_SERVO, CMD_SERVO_WRITE_TIMED, id, angle, ms); ServoExt::echoAngle(id, angle); }
    void stop(int id)                          const { Pardalote.command(DEVICE_SERVO, CMD_SERVO_STOP, id); }
};
inline PardaloteServoAccess PardaloteServo;

// Self-register — runs before setup(). Passes nullptr for the disconnect
// hook and ServoExt::loop for the per-iteration interpolation.
INSTALL_EXTENSION(DEVICE_SERVO, ServoExt::handle, ServoExt::announce,
                  ServoExt::disconnect, ServoExt::loop)

#endif
