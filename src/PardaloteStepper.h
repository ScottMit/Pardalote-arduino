// ==============================================================
// PardaloteStepper.h
// Pardalote Stepper Motor Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
//
// Add #include <PardaloteStepper.h> to your sketch — the extension
// self-registers, no further setup is required.
//
// Requires the AccelStepper library (by Mike McCauley):
//   Arduino IDE → Tools → Manage Libraries → search "AccelStepper".
//
// Supports up to MAX_STEPPERS simultaneously attached steppers driven
// by STEP/DIR drivers (TMC2208/2209, A4988, EasyDriver, ...) or by
// 4-wire coil drivers (28BYJ-48 via ULN2003, bipolar via H-bridge).
//
// Motion runs ON THE BOARD: JS sends targets and motion profiles, and
// AccelStepper::run() / runSpeed() generate the step pulses inside the
// extension loop hook. This is why we mirror AccelStepper (non-blocking)
// rather than the built-in Stepper library (whose step() blocks and
// would stall Pardalote.run(), dropping WebSocket frames mid-move).
//
// Each stepper is addressed by a logical instance ID, assigned by the JS
// side when calling arduino.add('name', new Stepper()) (ids grow from 0
// up) or by the board when the sketch calls PardaloteStepper.attach("name",
// stepPin, dirPin) (ids grow from the top down).
// ==============================================================

#ifndef PARDALOTE_STEPPER_H
#define PARDALOTE_STEPPER_H

#include <AccelStepper.h>
#include "Pardalote.h"

#define MAX_STEPPERS 6

class StepperExt {
private:
    // Motion mode per instance.
    //   POSITION — accel-limited moveTo (run()).
    //   VELOCITY — continuous constant speed (runSpeed()).
    //   TIMED    — constant-speed move to a target sized to arrive in a set
    //              duration (runSpeedToPosition()); used for group arrive-together.
    //   STOPPING — decelerating a constant-speed (VELOCITY/TIMED) motion to a
    //              clean halt by ramping setSpeed() toward 0; then → POSITION.
    //   EASED    — playing a gesture SEGMENT SCHEDULE: each tick follows the
    //              segment's eased position at a feed-forward speed derived
    //              from the same curve (see loop() + loadStepperSegment()).
    enum Mode : uint8_t { MODE_POSITION = 0, MODE_VELOCITY = 1, MODE_TIMED = 2, MODE_STOPPING = 3, MODE_EASED = 4 };

    inline static AccelStepper* _steppers[MAX_STEPPERS] = {};
    inline static bool          _attached[MAX_STEPPERS] = {};
    inline static Mode          _mode[MAX_STEPPERS]     = {};
    inline static bool          _wasRunning[MAX_STEPPERS] = {};
    inline static uint32_t      _stopPrevMs[MAX_STEPPERS] = {};   // MODE_STOPPING ramp timebase

    // Stored attach params so announce() can replay them to a fresh client.
    inline static int16_t _interface[MAX_STEPPERS] = {};
    inline static int16_t _pins[MAX_STEPPERS][4]   = {};
    inline static int16_t _enPin[MAX_STEPPERS]     = { -1,-1,-1,-1,-1,-1 };
    inline static int16_t _invert[MAX_STEPPERS]    = {};

    // Sketch-created steppers (PardaloteStepper.attach("name", …)). The name
    // is what the browser binds (arduino.<name>); announce() replays a
    // CMD_SHARE frame for these so every connecting browser materialises the
    // object. Browser-created steppers have _sketchOwned = false. Mirrors the
    // servo extension's sketch-attach path.
    inline static bool    _sketchOwned[MAX_STEPPERS] = {};
    inline static char    _names[MAX_STEPPERS][MAX_SHARE_NAME + 1] = {};

    // Motion profile (kept so we can replay it on announce).
    inline static float _maxSpeed[MAX_STEPPERS] = { 1000,1000,1000,1000,1000,1000 };
    inline static float _accel[MAX_STEPPERS]    = { 500,500,500,500,500,500 };

    // Soft position limits (safety).
    inline static bool    _limitEnabled[MAX_STEPPERS] = {};
    inline static int32_t _limitMin[MAX_STEPPERS]     = {};
    inline static int32_t _limitMax[MAX_STEPPERS]     = {};

    // Hardware limit switches — one optional switch per end ([LIMIT_MIN],
    // [LIMIT_MAX]); pin -1 = none. The trip is direction-aware and hard-stops
    // ON THE BOARD (no JS round-trip); the browser is told via
    // CMD_STEPPER_LIMIT, and the normal DONE edge follows.
    inline static int16_t  _swPin[MAX_STEPPERS][2] =
        { {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1} };
    inline static uint8_t  _swTrig[MAX_STEPPERS][2]       = {};   // 0=LOW, 1=HIGH
    inline static bool     _swLatched[MAX_STEPPERS][2]    = {};   // pressed-latch
    inline static uint32_t _swReleasedAt[MAX_STEPPERS][2] = {};   // release-debounce t0

    // Coordinate each switch physically sits at, INDEPENDENT of the soft
    // limits ([LIMIT_MIN], [LIMIT_MAX]). Default 0 = the switch is the origin
    // (the historical behaviour). Homing adopts this value as the step counter
    // when the switch trips, so home (the origin, 0) can sit anywhere relative
    // to the switch — e.g. switch at -500, home at 0.
    inline static int32_t  _swPosition[MAX_STEPPERS][2] = {};

    // Homing routine state (CMD_STEPPER_HOME). While homing, the generic
    // switch/DONE logic in loop() is bypassed — homingStep() owns the motor.
    //   SEEK    — constant speed toward the switch
    //   BACKOFF — reverse until the switch releases
    //   TRAVEL  — normal accel move to home (the origin, 0)
    // Home is always the origin (0); the switch's coordinate (_swPosition)
    // is what varies, so there is no separate stored home target.
    enum HomeState : uint8_t { HOME_IDLE = 0, HOME_SEEK, HOME_BACKOFF, HOME_TRAVEL };
    inline static HomeState _homing[MAX_STEPPERS]       = {};
    inline static uint8_t   _homeEnd[MAX_STEPPERS]      = {};   // which switch (LIMIT_MIN/LIMIT_MAX)
    inline static float     _homeSpeed[MAX_STEPPERS]    = {};   // seek speed (steps/sec, positive)
    inline static uint32_t  _homeDeadline[MAX_STEPPERS] = {};   // millis() cap for SEEK+BACKOFF
    static const uint32_t HOME_MAX_MS = 30000;   // default cap — unplugged switch can't spin forever

    // Gesture segment schedule (CMD_STEPPER_GESTURE, MODE_EASED). Unlike the
    // servo's sampled interpolation, a stepper can't teleport, so each tick
    // follows the segment's eased POSITION at a feed-forward SPEED (magnitude
    // from the curve slope; runSpeedToPosition() supplies direction and the
    // exact landing). `from` is re-captured from currentPosition() at each
    // segment start (dynamic capture), so step-quantisation error never
    // accumulates across a gesture and relative bounces need no homing.
    static const uint8_t MAX_STEPPER_SEGMENTS = 16;
    struct Seg { uint8_t curve; uint16_t dur; int32_t value; };
    inline static Seg      _segs[MAX_STEPPERS][MAX_STEPPER_SEGMENTS] = {};
    inline static uint8_t  _segCount[MAX_STEPPERS]   = {};   // 0 = no gesture running
    inline static uint8_t  _segIndex[MAX_STEPPERS]   = {};   // current segment
    inline static uint8_t  _segFlags[MAX_STEPPERS]   = {};   // GESTURE_FLAG_* (reference frame, loop)
    inline static uint8_t  _curveNow[MAX_STEPPERS]   = {};   // easing id of the current segment
    inline static int32_t  _segFromPos[MAX_STEPPERS] = {};   // captured position at segment start
    inline static int32_t  _segTarget[MAX_STEPPERS]  = {};   // segment end position (clamped)
    inline static uint32_t _segStartMs[MAX_STEPPERS] = {};   // segment timebase (start+dur, drift-free)
    inline static uint32_t _segDurMs[MAX_STEPPERS]   = {};

    static bool validId(int id) { return id >= 0 && id < MAX_STEPPERS; }

    // Periodic reads — per-client registration + gating.
    // Gates on position; default threshold 1 step (positions are exact
    // counters — every step is a real change).
    inline static ExtReadPoll _polls[MAX_STEPPERS] = {};
    static constexpr uint16_t DEFAULT_THRESHOLD = 1;

    // Read a numeric param as float regardless of how JS encoded it.
    // encodeFrame() sends whole numbers as int32 and only non-integers as
    // float32, so speed/accel can arrive as either type — decode per mask.
    static float paramNum(uint8_t* params, uint16_t typeMask, int i) {
        return paramIsFloat(typeMask, i) ? paramFloat(params, i)
                                         : (float)paramInt(params, i);
    }

    // Clamp an absolute target to the soft limits if they're enabled.
    static int32_t clampTarget(int id, int32_t target) {
        if (_limitEnabled[id]) {
            if (target < _limitMin[id]) target = _limitMin[id];
            if (target > _limitMax[id]) target = _limitMax[id];
        }
        return target;
    }

    static bool isRunning(int id) {
        AccelStepper* s = _steppers[id];
        if (!s) return false;
        return (_mode[id] == MODE_VELOCITY || _mode[id] == MODE_STOPPING)
                   ? (s->speed() != 0.0f)
                   : (s->distanceToGo() != 0);
    }

    // ---- Limit-switch helpers ----
    static bool swPressed(int id, int end) {
        int pin = _swPin[id][end];
        if (pin < 0) return false;
        return digitalRead(pin) == (_swTrig[id][end] ? HIGH : LOW);
    }

    // Moving toward this end? Speed sign is authoritative; fall back to
    // distanceToGo for a position move that hasn't taken its first step yet.
    static bool movingToward(int id, int end) {
        AccelStepper* s = _steppers[id];
        float spd = s->speed();
        long  dtg = s->distanceToGo();
        return (end == LIMIT_MIN) ? (spd < 0 || (spd == 0 && dtg < 0))
                                  : (spd > 0 || (spd == 0 && dtg > 0));
    }

    // Instant halt — setCurrentPosition(current) zeroes speed AND
    // distanceToGo in one call (no decel ramp: momentum past a hard limit
    // is exactly what the switch protects against).
    static void hardStop(int id) {
        AccelStepper* s = _steppers[id];
        s->setCurrentPosition(s->currentPosition());
        _mode[id] = MODE_POSITION;
    }

    // Any explicit motion command cancels an in-progress homing routine.
    static void cancelHoming(int id) { _homing[id] = HOME_IDLE; }

    // One iteration of the homing routine. Owns the motor entirely while
    // active — the generic switch/DONE logic in loop() is skipped.
    static void homingStep(int id) {
        AccelStepper* s = _steppers[id];
        int end = _homeEnd[id];

        // Safety cap on SEEK + BACKOFF: if the switch never trips (unplugged,
        // wrong pin) or never releases (jammed), give up — hard-stop, tell
        // the browser (CMD_STEPPER_HOME → 'homeFail'), then DONE so
        // whenDone() still settles. TRAVEL is a normal accel move and
        // terminates on its own, so it isn't capped.
        if (_homing[id] != HOME_TRAVEL && millis() > _homeDeadline[id]) {
            hardStop(id);
            _homing[id]     = HOME_IDLE;
            _wasRunning[id] = false;
            FrameBuilder ff;
            ff.begin(CMD_STEPPER_HOME, DEVICE_STEPPER);
            ff.addInt(id);
            ff.addInt(s->currentPosition());
            Pardalote.broadcastFrame(ff);
            FrameBuilder fd;
            fd.begin(CMD_STEPPER_DONE, DEVICE_STEPPER);
            fd.addInt(id);
            fd.addInt(s->currentPosition());
            Pardalote.broadcastFrame(fd);
            Serial.print(F("Stepper ")); Serial.print(id);
            Serial.println(F(" homing timed out — switch never responded"));
            return;
        }

        switch (_homing[id]) {

            case HOME_SEEK:
                if (swPressed(id, end)) {
                    // The switch sits at its declared coordinate (default 0);
                    // adopt it as the counter. Home stays the origin, so with a
                    // switch at -500 the motor then travels +500 to reach home.
                    int32_t swPos = _swPosition[id][end];
                    s->setCurrentPosition(swPos);             // kills motion too
                    _swLatched[id][end] = true;               // generic latch stays consistent
                    FrameBuilder fp;                          // silent JS position sync
                    fp.begin(CMD_STEPPER_SET_POSITION, DEVICE_STEPPER);
                    fp.addInt(id); fp.addInt(swPos);
                    Pardalote.broadcastFrame(fp);
                    // Back off (away from the switch) until it releases.
                    s->setSpeed(end == LIMIT_MIN ? _homeSpeed[id] : -_homeSpeed[id]);
                    _homing[id] = HOME_BACKOFF;
                } else {
                    s->runSpeed();
                }
                break;

            case HOME_BACKOFF:
                if (!swPressed(id, end)) {
                    s->setCurrentPosition(s->currentPosition());   // stop, keep counter
                    s->setMaxSpeed(_maxSpeed[id]);                 // restore profile
                    s->moveTo(clampTarget(id, 0));                 // travel home (the origin)
                    _homing[id] = HOME_TRAVEL;
                } else {
                    s->runSpeed();
                }
                break;

            case HOME_TRAVEL:
                s->run();
                if (s->distanceToGo() == 0) {
                    _homing[id]     = HOME_IDLE;
                    _mode[id]       = MODE_POSITION;
                    _wasRunning[id] = false;   // suppress a duplicate edge DONE
                    FrameBuilder fb;
                    fb.begin(CMD_STEPPER_DONE, DEVICE_STEPPER);
                    fb.addInt(id);
                    fb.addInt(s->currentPosition());
                    Pardalote.broadcastFrame(fb);
                }
                break;

            default: break;
        }
    }

    // Start a constant-speed move to `target` sized to finish in ~durationMs.
    // The board knows its own exact position, so it computes the matched speed
    // itself — a group sharing one duration arrives together. AccelStepper's
    // setSpeed() is clamped to maxSpeed(), so raise the cap for this move.
    static void startTimedMove(int id, int32_t target, uint32_t durationMs) {
        AccelStepper* s = _steppers[id];
        if (!s) return;
        cancelEased(id);           // a timed move supersedes any running gesture
        _homing[id] = HOME_IDLE;   // explicit move cancels homing
        target = clampTarget(id, target);
        long  distance = (long)target - s->currentPosition();
        float durSec   = (durationMs > 0) ? (durationMs / 1000.0f) : 0.001f;
        float speed    = fabs((float)distance) / durSec;
        if (speed < 1.0f) speed = 1.0f;                  // never 0 (would never step)
        if (s->maxSpeed() < speed) s->setMaxSpeed(speed);
        s->moveTo(target);
        s->setSpeed(distance >= 0 ? speed : -speed);
        _mode[id] = MODE_TIMED;
    }

    // Peak/average speed ratio for a curve — an eased move's velocity peaks
    // above its distance/duration average, so we raise the live maxSpeed cap
    // to render the curve within the authored duration (restored on finish).
    static float curveSlopeMax(uint8_t curve) {
        switch (curve) {
            case CURVE_EASE_IN:     return 2.0f;   // slope 2t → 2 at t=1
            case CURVE_EASE_OUT:    return 2.0f;   // slope 2(1-t) → 2 at t=0
            case CURVE_EASE_IN_OUT: return 1.5f;   // smoothstep peak 1.5 at t=0.5
            case CURVE_BACK:        return 3.6f;   // easeOutBack peak ≈ 3.6
            default:                return 1.0f;   // linear
        }
    }

    // Load segment `idx` as the current eased move. from = live position
    // (dynamic capture); target = from+delta (relative) or the value itself
    // (absolute), clamped to soft limits. Raises the live speed cap to fit the
    // curve's velocity peak inside `dur`. _maxSpeed[id] keeps the USER value.
    static void loadStepperSegment(int id, uint8_t idx, uint32_t startMs) {
        AccelStepper* s = _steppers[id];
        const Seg& seg  = _segs[id][idx];
        int32_t from    = s->currentPosition();
        int32_t target  = (_segFlags[id] & GESTURE_FLAG_ABSOLUTE) ? seg.value : from + seg.value;
        target          = clampTarget(id, target);
        uint32_t dur    = seg.dur ? seg.dur : 1;

        _segFromPos[id] = from;
        _segTarget[id]  = target;
        _curveNow[id]   = seg.curve;
        _segStartMs[id] = startMs;
        _segDurMs[id]   = dur;
        _segIndex[id]   = idx;

        float durSec = dur / 1000.0f;
        float peak   = fabsf((float)(target - from)) / durSec * curveSlopeMax(seg.curve);
        if (peak < 1.0f) peak = 1.0f;
        if (s->maxSpeed() < peak) s->setMaxSpeed(peak);   // raised for the move; restored on finish
        _mode[id] = MODE_EASED;
    }

    // End a gesture: stop, restore the user's speed cap, drop the schedule,
    // and announce completion via the normal DONE frame (whenDone()).
    static void finishStepperGesture(int id) {
        AccelStepper* s = _steppers[id];
        s->setSpeed(0);
        s->moveTo(s->currentPosition());      // no residual target
        s->setMaxSpeed(_maxSpeed[id]);        // restore the user-configured cap
        _mode[id]       = MODE_POSITION;
        _segCount[id]   = 0;
        _wasRunning[id] = false;              // suppress the generic DONE edge — we send it here
        FrameBuilder fb;
        fb.begin(CMD_STEPPER_DONE, DEVICE_STEPPER);
        fb.addInt(id);
        fb.addInt(s->currentPosition());
        Pardalote.broadcastFrame(fb);
    }

    // Abandon a running gesture WITHOUT a DONE frame (a new explicit motion
    // command supersedes it). Restores the speed cap so the next move honours
    // the user's maxSpeed. No-op when no gesture is active.
    static void cancelEased(int id) {
        if (_mode[id] != MODE_EASED && _segCount[id] == 0) return;
        if (_steppers[id]) _steppers[id]->setMaxSpeed(_maxSpeed[id]);
        _segCount[id] = 0;
    }

public:
    // -------------------------------------------------------------------
    // Sketch-facing read accessors (used by the PardaloteStepper object).
    // -------------------------------------------------------------------
    static long positionId(int id)     { return (validId(id) && _steppers[id]) ? _steppers[id]->currentPosition() : 0; }
    static long distanceToGoId(int id) { return (validId(id) && _steppers[id]) ? _steppers[id]->distanceToGo() : 0; }
    static bool runningId(int id)      { return validId(id) && _attached[id] && isRunning(id); }
    static bool attachedId(int id)     { return validId(id) && _attached[id]; }
    static int  listAttached(int* out, int max) {
        int n = 0;
        for (int i = 0; i < MAX_STEPPERS && n < max; i++) if (_attached[i]) out[n++] = i;
        return n;
    }
    // Echo the board's current target to the browser so it sets its cached
    // target exactly as if the browser had issued the move.
    static void echoTarget(int id) {
        if (!validId(id) || !_steppers[id]) return;
        FrameBuilder fb;
        fb.begin(CMD_STEPPER_MOVE_TO, DEVICE_STEPPER);
        fb.addInt(id);
        fb.addInt(_steppers[id]->targetPosition());
        Pardalote.broadcastFrame(fb);
    }

    // -------------------------------------------------------------------
    // Sketch-created steppers — PardaloteStepper.attach("name", …).
    //
    // Creation and browser visibility are one act (see the servo extension
    // for the full rationale): the stepper is attached through the same
    // handler a browser attach uses, and a CMD_SHARE frame (+ the attach and
    // motion-profile state) is broadcast so connected browsers materialise
    // arduino.<name> immediately. announce() replays the same sequence for
    // browsers that connect later.
    //
    // Logical ids are allocated from the TOP of the range downward —
    // browser-assigned ids grow from 0 upward, so the two sides can't collide
    // until every slot is in use. Idempotent on the name: attaching again with
    // a name that is already sketch-owned reuses its id.
    //
    // `iface` is STEPPER_DRIVER or STEPPER_FULL4WIRE; p1..p4 follow the
    // CMD_STEPPER_ATTACH layout (DRIVER: p1=STEP, p2=DIR, p3=enPin, p4=invert;
    // FULL4WIRE: p1..p4 = coil pins). Returns the logical id, or -1 if no slot.
    // -------------------------------------------------------------------
    static int sketchAttach(const char* name, int iface,
                            int p1, int p2, int p3, int p4) {
        if (name == nullptr || name[0] == '\0') return -1;

        int id = -1;
        for (int i = 0; i < MAX_STEPPERS; i++) {
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        }
        if (id < 0) {
            for (int i = MAX_STEPPERS - 1; i >= 0; i--) {
                if (!_attached[i] && !_sketchOwned[i]) { id = i; break; }
            }
        }
        if (id < 0) {
            Serial.print(F("Stepper: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        // Attach through the same code path a browser attach takes.
        Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_ATTACH, id, iface, p1, p2, p3, p4);

        // Tell any connected browsers now (no-op when none are connected;
        // announce() covers future connects): name → attach → profile, the
        // same order announce() replays.
        broadcastShare(id);
        sendAttachState(id, false, 0);

        return id;
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_STEPPER);
        fb.addInt(id);
        fb.addString(_names[id]);
        Pardalote.broadcastFrame(fb);
    }

    // Emit this stepper's attach frame (interface + pins) followed by its
    // motion profile (max speed, accel). Shared by sketchAttach (broadcast to
    // everyone) and announce (unicast to one joining client) so the two stay
    // in lockstep. When `unicast` is false, `clientNum` is ignored.
    static void sendAttachState(int id, bool unicast, uint8_t clientNum) {
        FrameBuilder fa;
        fa.begin(CMD_STEPPER_ATTACH, DEVICE_STEPPER);
        fa.addInt(id);
        fa.addInt(_interface[id]);
        if (_interface[id] == STEPPER_FULL4WIRE) {
            fa.addInt(_pins[id][0]); fa.addInt(_pins[id][1]);
            fa.addInt(_pins[id][2]); fa.addInt(_pins[id][3]);
        } else {
            fa.addInt(_pins[id][0]); fa.addInt(_pins[id][1]);
            fa.addInt(_enPin[id]);   fa.addInt(_invert[id]);
        }
        FrameBuilder fs; fs.begin(CMD_STEPPER_SET_MAX_SPEED, DEVICE_STEPPER);
        fs.addInt(id); fs.addFloat(_maxSpeed[id]);
        FrameBuilder fac; fac.begin(CMD_STEPPER_SET_ACCEL, DEVICE_STEPPER);
        fac.addInt(id); fac.addFloat(_accel[id]);

        if (unicast) {
            Pardalote.sendFrame(clientNum, fa);
            Pardalote.sendFrame(clientNum, fs);
            Pardalote.sendFrame(clientNum, fac);
        } else {
            Pardalote.broadcastFrame(fa);
            Pardalote.broadcastFrame(fs);
            Pardalote.broadcastFrame(fac);
        }
    }

    // -------------------------------------------------------------------
    // Main dispatch — called by the extension registry for every frame
    // whose TARGET == DEVICE_STEPPER.
    // -------------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {

        // Global (multi-stepper) command — handled before per-instance id read.
        if (cmd == CMD_STEPPER_SYNC_MOVE) {
            uint32_t dur = (nparams >= 1) ? (uint32_t)paramInt(params, 0) : 0;
            const int REC = 5;                          // { logicalId u8, target i32 }
            int count = payloadLen / REC;
            for (int i = 0; i < count; i++) {
                uint8_t* r = payload + i * REC;
                int sid = r[0];
                int32_t target = (int32_t)(((uint32_t)r[1] << 24) | ((uint32_t)r[2] << 16) |
                                           ((uint32_t)r[3] <<  8) |  (uint32_t)r[4]);
                if (validId(sid) && _attached[sid] && _steppers[sid]) startTimedMove(sid, target, dur);
            }
            return;
        }

        // Global (multi-stepper) gesture — one or more channel blocks, each a
        // segment schedule the board plays locally (see defs.h layout).
        if (cmd == CMD_STEPPER_GESTURE) {
            uint32_t now = millis();
            uint16_t off = 0;
            while (off + 3 <= payloadLen) {
                int     sid   = payload[off];
                uint8_t flags = payload[off + 1];
                uint8_t count = payload[off + 2];
                off += 3;
                if ((uint32_t)off + (uint32_t)count * 7 > payloadLen) break;   // malformed — stop
                if (validId(sid) && _attached[sid] && _steppers[sid] && count > 0) {
                    _homing[sid] = HOME_IDLE;              // a gesture supersedes homing
                    uint8_t n = count > MAX_STEPPER_SEGMENTS ? MAX_STEPPER_SEGMENTS : count;
                    for (uint8_t i = 0; i < n; i++) {
                        const uint8_t* r = payload + off + i * 7;
                        _segs[sid][i].curve = r[0];
                        _segs[sid][i].dur   = (uint16_t)(((uint16_t)r[1] << 8) | r[2]);
                        _segs[sid][i].value = (int32_t)(((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) |
                                                        ((uint32_t)r[5] <<  8) |  (uint32_t)r[6]);
                    }
                    _segCount[sid] = n;
                    _segFlags[sid] = flags;
                    loadStepperSegment(sid, 0, now);
                }
                off += (uint16_t)count * 7;               // skip the whole declared block, even if capped
            }
            return;
        }

        if (nparams < 1) return;
        int id = (int)paramInt(params, 0);
        if (!validId(id)) {
            Serial.print(F("Stepper: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            case CMD_STEPPER_ATTACH: {
                if (nparams < 4) return;
                int iface = (int)paramInt(params, 1);

                // Tear down any previous instance on this id.
                if (_steppers[id]) { delete _steppers[id]; _steppers[id] = nullptr; }

                int p1 = (int)paramInt(params, 2);
                int p2 = (int)paramInt(params, 3);
                int p3 = (nparams > 4) ? (int)paramInt(params, 4) : -1;
                int p4 = (nparams > 5) ? (int)paramInt(params, 5) : -1;

                if (iface == STEPPER_FULL4WIRE) {
                    _steppers[id] = new AccelStepper(AccelStepper::FULL4WIRE, p1, p2, p3, p4);
                    _enPin[id]  = -1;
                    _invert[id] = 0;
                    _pins[id][0] = p1; _pins[id][1] = p2; _pins[id][2] = p3; _pins[id][3] = p4;
                } else {
                    // STEPPER_DRIVER (default): p1=STEP, p2=DIR.
                    _steppers[id] = new AccelStepper(AccelStepper::DRIVER, p1, p2);
                    int enPin  = (nparams > 4) ? (int)paramInt(params, 4) : -1;
                    int invert = (nparams > 5) ? (int)paramInt(params, 5) : 0x04; // enable active-LOW
                    _enPin[id]  = (int16_t)enPin;
                    _invert[id] = (int16_t)invert;
                    _pins[id][0] = p1; _pins[id][1] = p2; _pins[id][2] = -1; _pins[id][3] = -1;

                    bool invDir = invert & 0x01, invStep = invert & 0x02, invEn = invert & 0x04;
                    _steppers[id]->setPinsInverted(invDir, invStep, invEn);
                    if (enPin >= 0) {
                        _steppers[id]->setEnablePin(enPin);
                        _steppers[id]->enableOutputs();
                    }
                }

                _interface[id] = (int16_t)iface;
                _steppers[id]->setMaxSpeed(_maxSpeed[id]);
                _steppers[id]->setAcceleration(_accel[id]);
                _mode[id]       = MODE_POSITION;
                _wasRunning[id] = false;
                _segCount[id]   = 0;    // no stale gesture from a previous instance on this id
                _attached[id]   = true;

                Serial.print(F("Stepper ")); Serial.print(id);
                Serial.print(F(" attached (iface ")); Serial.print(iface);
                Serial.println(F(")"));
                break;
            }

            case CMD_STEPPER_DETACH:
                if (_attached[id]) {
                    if (_steppers[id]) {
                        _steppers[id]->disableOutputs();
                        delete _steppers[id];
                        _steppers[id] = nullptr;
                    }
                    ExtReadPoll* p = extPollFind(_polls, MAX_STEPPERS, id);
                    if (p) p->instance = -1;   // stop any periodic read
                    _attached[id]   = false;
                    _limitEnabled[id] = false;
                    _swPin[id][0] = _swPin[id][1] = -1;
                    _swLatched[id][0] = _swLatched[id][1] = false;
                    _swPosition[id][0] = _swPosition[id][1] = 0;
                    _homing[id]  = HOME_IDLE;
                    Serial.print(F("Stepper ")); Serial.print(id);
                    Serial.println(F(" detached"));
                }
                break;

            case CMD_STEPPER_MOVE_TO: {
                if (!_attached[id] || nparams < 2) return;
                cancelHoming(id);
                cancelEased(id);
                int32_t target = clampTarget(id, (int32_t)paramInt(params, 1));
                _mode[id] = MODE_POSITION;
                _steppers[id]->setMaxSpeed(_maxSpeed[id]);   // restore cap (a timed move may have raised it)
                _steppers[id]->moveTo(target);
                break;
            }

            case CMD_STEPPER_MOVE: {
                if (!_attached[id] || nparams < 2) return;
                cancelHoming(id);
                cancelEased(id);
                int32_t rel    = (int32_t)paramInt(params, 1);
                int32_t target = clampTarget(id, _steppers[id]->currentPosition() + rel);
                _mode[id] = MODE_POSITION;
                _steppers[id]->setMaxSpeed(_maxSpeed[id]);   // restore cap (a timed move may have raised it)
                _steppers[id]->moveTo(target);
                break;
            }

            case CMD_STEPPER_MOVE_TIMED: {
                if (!_attached[id] || nparams < 3) return;
                int32_t target = (int32_t)paramInt(params, 1);
                uint32_t dur   = (uint32_t)paramInt(params, 2);
                startTimedMove(id, target, dur);
                break;
            }

            case CMD_STEPPER_SET_MAX_SPEED: {
                if (!_attached[id] || nparams < 2) return;
                float v = paramNum(params, typeMask, 1);
                _maxSpeed[id] = v;
                _steppers[id]->setMaxSpeed(v);
                break;
            }

            case CMD_STEPPER_SET_ACCEL: {
                if (!_attached[id] || nparams < 2) return;
                float a = paramNum(params, typeMask, 1);
                _accel[id] = a;
                _steppers[id]->setAcceleration(a);
                break;
            }

            case CMD_STEPPER_RUN_SPEED: {
                if (!_attached[id] || nparams < 2) return;
                cancelHoming(id);
                cancelEased(id);
                float v = paramNum(params, typeMask, 1);
                _mode[id] = MODE_VELOCITY;
                _steppers[id]->setSpeed(v);
                break;
            }

            case CMD_STEPPER_STOP:
                if (!_attached[id]) return;
                cancelHoming(id);
                cancelEased(id);   // stop() ends a gesture (its own DONE is not sent — mirrors servo.stop())
                if (_mode[id] == MODE_VELOCITY || _mode[id] == MODE_TIMED) {
                    // Constant-speed motion (runSpeed/timed): decelerate via a
                    // speed ramp in the loop hook. AccelStepper::stop()/run()
                    // would RE-ACCELERATE here — its accel-ramp state is stale
                    // after setSpeed(), so run() plans a fresh move from rest.
                    _stopPrevMs[id] = millis();
                    _mode[id]       = MODE_STOPPING;
                } else {
                    // Accel-limited (position) move: AccelStepper::stop() ramps
                    // down correctly from valid accel state.
                    _mode[id] = MODE_POSITION;
                    _steppers[id]->stop();
                }
                break;

            case CMD_STEPPER_HARD_STOP:
                if (!_attached[id]) return;
                cancelHoming(id);
                cancelEased(id);
                hardStop(id);   // instant: zero speed + distanceToGo, keep position
                break;

            case CMD_STEPPER_SET_POSITION: {
                if (!_attached[id] || nparams < 2) return;
                cancelHoming(id);
                cancelEased(id);
                _steppers[id]->setCurrentPosition((int32_t)paramInt(params, 1));
                _wasRunning[id] = false;
                break;
            }

            case CMD_STEPPER_ENABLE: {
                if (!_attached[id] || nparams < 2) return;
                bool en = paramInt(params, 1) != 0;
                if (en) _steppers[id]->enableOutputs();
                else    _steppers[id]->disableOutputs();
                break;
            }

            case CMD_STEPPER_SET_LIMITS: {
                if (!_attached[id] || nparams < 4) return;
                _limitMin[id]     = (int32_t)paramInt(params, 1);
                _limitMax[id]     = (int32_t)paramInt(params, 2);
                _limitEnabled[id] = paramInt(params, 3) != 0;
                break;
            }

            case CMD_STEPPER_SET_SWITCH: {
                if (!_attached[id] || nparams < 4) return;
                int end  = (int)paramInt(params, 1);
                int pin  = (int)paramInt(params, 2);
                int trig = (int)paramInt(params, 3);
                if (end != LIMIT_MIN && end != LIMIT_MAX) return;
                _swPin[id][end]        = (int16_t)pin;
                _swTrig[id][end]       = (uint8_t)(trig != 0);
                _swLatched[id][end]    = false;
                _swReleasedAt[id][end] = 0;
                if (pin >= 0) {
                    // Active-LOW: internal pull-up, switch to GND (the easy
                    // default). Active-HIGH: pull-down where the core has
                    // one; else plain INPUT (external pull-down required).
#ifdef INPUT_PULLDOWN
                    pinMode(pin, trig ? INPUT_PULLDOWN : INPUT_PULLUP);
#else
                    pinMode(pin, trig ? INPUT : INPUT_PULLUP);
#endif
                }
                break;
            }

            case CMD_STEPPER_SET_SWITCH_POS: {
                if (!_attached[id] || nparams < 3) return;
                int end = (int)paramInt(params, 1);
                if (end != LIMIT_MIN && end != LIMIT_MAX) return;
                _swPosition[id][end] = (int32_t)paramInt(params, 2);
                // Echo so JS (and other clients) sync.
                FrameBuilder fb;
                fb.begin(CMD_STEPPER_SET_SWITCH_POS, DEVICE_STEPPER);
                fb.addInt(id); fb.addInt(end); fb.addInt(_swPosition[id][end]);
                Pardalote.broadcastFrame(fb);
                break;
            }

            case CMD_STEPPER_SET_HOME: {
                if (!_attached[id]) return;
                AccelStepper* s = _steppers[id];
                // Re-zero the coordinate frame. The current physical position
                // BECOMES `value` (default 0 = the origin/home). Everything
                // that carries a coordinate — the soft limits and the switch
                // positions — shifts by the same offset, so it keeps pointing
                // at the same physical spot. home() still returns to 0.
                //   e.g. counter at 500, SET_HOME → offset -500: pos→0, a min
                //   switch at 0 → -500, limitMax → limitMax-500.
                int32_t value  = (nparams > 1) ? (int32_t)paramInt(params, 1) : 0;
                int32_t offset = value - (int32_t)s->currentPosition();
                s->setCurrentPosition(value);   // also zeroes speed/distanceToGo
                _wasRunning[id] = false;

                // Position echo.
                FrameBuilder fp; fp.begin(CMD_STEPPER_SET_POSITION, DEVICE_STEPPER);
                fp.addInt(id); fp.addInt(value);
                Pardalote.broadcastFrame(fp);

                // Shift + echo the soft limits (only when in use).
                if (_limitEnabled[id]) {
                    _limitMin[id] += offset;
                    _limitMax[id] += offset;
                    FrameBuilder fl; fl.begin(CMD_STEPPER_SET_LIMITS, DEVICE_STEPPER);
                    fl.addInt(id); fl.addInt(_limitMin[id]); fl.addInt(_limitMax[id]); fl.addInt(1);
                    Pardalote.broadcastFrame(fl);
                }

                // Shift + echo each switch coordinate that's in use (configured
                // pin, or a coordinate that was explicitly set).
                for (int e = 0; e < 2; e++) {
                    if (_swPin[id][e] < 0 && _swPosition[id][e] == 0) continue;
                    _swPosition[id][e] += offset;
                    FrameBuilder fs; fs.begin(CMD_STEPPER_SET_SWITCH_POS, DEVICE_STEPPER);
                    fs.addInt(id); fs.addInt(e); fs.addInt(_swPosition[id][e]);
                    Pardalote.broadcastFrame(fs);
                }
                break;
            }

            case CMD_STEPPER_HOME: {
                if (!_attached[id]) return;
                cancelEased(id);   // homing supersedes any running gesture
                AccelStepper* s = _steppers[id];
                // Pick the switch: MIN if configured, else MAX.
                int end = (_swPin[id][LIMIT_MIN] >= 0) ? LIMIT_MIN
                        : (_swPin[id][LIMIT_MAX] >= 0) ? LIMIT_MAX : -1;
                if (end < 0) {
                    // No switch — counter trusted as-is: plain accel move to
                    // home (the origin, 0).
                    _homing[id] = HOME_IDLE;
                    _mode[id]   = MODE_POSITION;
                    s->setMaxSpeed(_maxSpeed[id]);
                    s->moveTo(clampTarget(id, 0));
                    break;
                }
                float speed = (nparams > 1) ? paramNum(params, typeMask, 1) : 0.0f;
                if (speed <= 0.0f) speed = _maxSpeed[id] / 4.0f;   // conservative default
                if (speed < 1.0f)  speed = 1.0f;
                if (s->maxSpeed() < speed) s->setMaxSpeed(speed);  // setSpeed is capped by maxSpeed
                uint32_t cap = (nparams > 2) ? (uint32_t)paramInt(params, 2) : 0;
                if (cap == 0) cap = HOME_MAX_MS;
                _homeDeadline[id] = millis() + cap;
                _homeEnd[id]   = (uint8_t)end;
                _homeSpeed[id] = speed;
                s->setSpeed(end == LIMIT_MIN ? -speed : speed);    // seek toward the switch
                _homing[id]     = HOME_SEEK;
                _wasRunning[id] = false;
                break;
            }

            // READ — params [id, interval?, threshold?]. Always
            // answers the requester immediately. interval > 0 registers a
            // board-side per-client periodic read (gated on position);
            // interval < 0 (JS END) removes this client's registration;
            // absent/0 = one-shot.
            case CMD_STEPPER_READ: {
                long ms  = (nparams > 1) ? paramInt(params, 1) : 0;
                long thr = (nparams > 2) ? paramInt(params, 2) : 0;

                if (ms < 0) {   // END — unregister this client
                    ExtReadPoll* p = extPollFind(_polls, MAX_STEPPERS, id);
                    if (p) p->removeClient(clientNum);
                    break;
                }

                if (!_attached[id]) return;
                sendReadTo(clientNum, id);

                if (ms > 0) {
                    ExtReadPoll* p = extPollGet(_polls, MAX_STEPPERS, id);
                    if (!p) break;
                    p->setClient(clientNum, (uint16_t)constrain(ms, 1, 65535),
                                 thr > 0 ? (uint16_t)thr : DEFAULT_THRESHOLD);
                    p->seed(clientNum, _steppers[id]->currentPosition(), millis());
                }
                break;
            }

            default:
                Serial.print(F("Stepper: unknown cmd 0x"));
                Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Loop hook — runs every Pardalote.run() iteration. This is where the
    // step pulses are actually generated. Also detects position-mode
    // target completion and broadcasts CMD_STEPPER_DONE once per move.
    // -------------------------------------------------------------------
    static void loop() {
        for (int id = 0; id < MAX_STEPPERS; id++) {
            if (!_attached[id] || !_steppers[id]) continue;
            AccelStepper* s = _steppers[id];

            // Homing routine owns the motor while active — the generic
            // switch/DONE logic below is bypassed (it would emit spurious
            // LIMIT/DONE frames as the routine deliberately hits the switch).
            if (_homing[id] != HOME_IDLE) {
                homingStep(id);
                _wasRunning[id] = false;
                continue;
            }

            // Hardware limit switches. Trip INSTANTLY on the first pressed
            // read when moving toward the switch (no confirmation delay —
            // this is safety); re-arm only after the switch reads released
            // for 20 ms (mechanical bounce can't spam LIMIT frames). Moving
            // AWAY from a pressed switch is always allowed (back-off), and
            // any commanded move back toward it is re-stopped each loop.
            for (int end = 0; end < 2; end++) {
                if (_swPin[id][end] < 0) continue;
                if (swPressed(id, end)) {
                    _swReleasedAt[id][end] = 0;
                    if (movingToward(id, end)) {
                        hardStop(id);
                        if (!_swLatched[id][end]) {
                            _swLatched[id][end] = true;
                            FrameBuilder fb;
                            fb.begin(CMD_STEPPER_LIMIT, DEVICE_STEPPER);
                            fb.addInt(id);
                            fb.addInt(end);
                            fb.addInt(s->currentPosition());
                            Pardalote.broadcastFrame(fb);
                            // Force the _wasRunning edge below to send
                            // CMD_STEPPER_DONE even if the move tripped
                            // before its first step (move commanded into an
                            // already-pressed switch) — whenDone() must not
                            // hang on a move that never really started.
                            _wasRunning[id] = true;
                        }
                    }
                } else if (_swLatched[id][end]) {
                    uint32_t now = millis();
                    if (_swReleasedAt[id][end] == 0)          _swReleasedAt[id][end] = now;
                    else if (now - _swReleasedAt[id][end] >= 20) {
                        _swLatched[id][end]    = false;   // debounced release — re-arm
                        _swReleasedAt[id][end] = 0;
                    }
                }
            }

            if (_mode[id] == MODE_VELOCITY) {
                // Enforce soft limits by stopping at the boundary. On the tick
                // we hit the limit, switch to position-hold and DON'T take a
                // further runSpeed() step — otherwise we'd overshoot by a step
                // and the hold would drag it back (visible jitter at the stop).
                bool clamped = false;
                if (_limitEnabled[id]) {
                    int32_t pos = s->currentPosition();
                    float   spd = s->speed();
                    if ((spd > 0 && pos >= _limitMax[id]) ||
                        (spd < 0 && pos <= _limitMin[id])) {
                        _mode[id] = MODE_POSITION;
                        s->moveTo(pos);   // hold here
                        clamped = true;
                    }
                }
                if (!clamped) s->runSpeed();
            } else if (_mode[id] == MODE_STOPPING) {
                // Decelerate a constant-speed spin to a clean halt: shed speed
                // at the configured acceleration each tick, then stop. (Can't
                // hand a runSpeed()-driven motor to run()/stop() — see the
                // CMD_STEPPER_STOP handler.)
                uint32_t now = millis();
                float dt  = (now - _stopPrevMs[id]) / 1000.0f;
                _stopPrevMs[id] = now;
                float spd  = s->speed();
                float aspd = spd < 0 ? -spd : spd;
                float dv   = _accel[id] * dt;              // steps/sec shed this tick
                if (dv > 0.0f && aspd <= dv) {
                    s->setCurrentPosition(s->currentPosition());  // clean halt, keep coordinate
                    _mode[id] = MODE_POSITION;                    // DONE edge fires below
                } else {
                    if (dv > 0.0f) s->setSpeed(spd > 0 ? spd - dv : spd + dv);
                    s->runSpeed();
                }
            } else if (_mode[id] == MODE_TIMED) {
                s->runSpeedToPosition();   // constant speed to target (arrive-together)
            } else if (_mode[id] == MODE_EASED) {
                // Follow the segment's eased position at a feed-forward speed.
                uint32_t now      = millis();
                uint32_t elapsed  = now - _segStartMs[id];
                float    durSec   = _segDurMs[id] / 1000.0f;
                long     from     = _segFromPos[id];
                long     target   = _segTarget[id];
                if (elapsed < _segDurMs[id]) {
                    float t  = (float)elapsed / (float)_segDurMs[id];
                    // Aim at the scheduled position (may pass `target` for BACK,
                    // giving a real overshoot); runSpeedToPosition() lands on it.
                    float p  = (float)from + (float)(target - from) * pardaloteEase(_curveNow[id], t);
                    // Feed-forward speed magnitude = curve slope × distance / dur,
                    // via a central difference of the SAME curve (no derivative table).
                    float h  = 0.02f;
                    float tl = t - h < 0.0f ? 0.0f : t - h;
                    float th = t + h > 1.0f ? 1.0f : t + h;
                    float vmag = fabsf((float)(target - from) *
                                       (pardaloteEase(_curveNow[id], th) - pardaloteEase(_curveNow[id], tl)) /
                                       ((th - tl) * durSec));
                    if (vmag < 1.0f) vmag = 1.0f;
                    s->moveTo(lroundf(p));
                    s->setSpeed(vmag);            // magnitude; runSpeedToPosition() derives direction
                    s->runSpeedToPosition();
                } else {
                    // Segment time elapsed — land exactly on target, then advance.
                    s->moveTo(target);
                    if (s->distanceToGo() != 0) {
                        float vmag = fabsf((float)(target - from)) / durSec;
                        if (vmag < 1.0f) vmag = 1.0f;
                        s->setSpeed(vmag);
                        s->runSpeedToPosition();
                    } else if (_segCount[id] > 0 && _segIndex[id] + 1 < _segCount[id]) {
                        loadStepperSegment(id, _segIndex[id] + 1, _segStartMs[id] + _segDurMs[id]);
                    } else {
                        finishStepperGesture(id);
                    }
                }
            } else {
                s->run();
            }

            bool running = isRunning(id);
            if (_wasRunning[id] && !running &&
                (_mode[id] == MODE_POSITION || _mode[id] == MODE_TIMED)) {
                // Target reached — tell the browser once.
                FrameBuilder fb;
                fb.begin(CMD_STEPPER_DONE, DEVICE_STEPPER);
                fb.addInt(id);
                fb.addInt(s->currentPosition());
                Pardalote.broadcastFrame(fb);
            }
            _wasRunning[id] = running;
        }

        // Board-side periodic reads — per-client gating on position
        //.
        const unsigned long now = millis();
        for (int i = 0; i < MAX_STEPPERS; i++) {
            ExtReadPoll& p = _polls[i];
            if (!p.due(now)) continue;
            if (!validId(p.instance) || !_attached[p.instance] || !_steppers[p.instance]) {
                p.instance = -1;
                continue;
            }
            const int32_t pos = _steppers[p.instance]->currentPosition();
            for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
                if (p.gate(c, pos, now)) sendReadTo(c, p.instance);
        }
    }

    // Client disconnect — drop its read registrations.
    static void disconnect(uint8_t clientNum) {
        extPollDropClient(_polls, MAX_STEPPERS, clientNum);
    }

    // -------------------------------------------------------------------
    // Poll response — current position, remaining distance, speed, state.
    // -------------------------------------------------------------------
    static void buildRead(FrameBuilder& fb, int id) {
        AccelStepper* s = _steppers[id];
        fb.begin(CMD_STEPPER_READ, DEVICE_STEPPER);
        fb.addInt(id);
        fb.addInt(s->currentPosition());
        fb.addInt(s->distanceToGo());
        fb.addFloat(s->speed());
        fb.addInt(isRunning(id) ? 1 : 0);
    }

    static void sendReadTo(uint8_t clientNum, int id) {
        FrameBuilder fb;
        buildRead(fb, id);
        Pardalote.sendFrame(clientNum, fb);
    }

    // -------------------------------------------------------------------
    // Called on every new client connection. Announces the extension,
    // then replays each attached stepper's full state so a browser joining
    // a running system sees true configuration and live position.
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_STEPPER);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_STEPPERS);
        Pardalote.sendFrame(clientNum, fb);

        for (int i = 0; i < MAX_STEPPERS; i++) {
            if (!_attached[i] || !_steppers[i]) continue;

            // Sketch-created stepper: send its SHARE frame FIRST so the browser
            // materialises arduino.<name> before the attach/state frames below
            // arrive to sync it.
            if (_sketchOwned[i]) {
                FrameBuilder fsh;
                fsh.begin(CMD_SHARE, DEVICE_STEPPER);
                fsh.addInt(i);
                fsh.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fsh);
            }

            // Replay attach (interface + pins + enable + invert) and profile.
            sendAttachState(i, true, clientNum);

            // Replay soft limits if set.
            if (_limitEnabled[i]) {
                FrameBuilder fl; fl.begin(CMD_STEPPER_SET_LIMITS, DEVICE_STEPPER);
                fl.addInt(i); fl.addInt(_limitMin[i]); fl.addInt(_limitMax[i]); fl.addInt(1);
                Pardalote.sendFrame(clientNum, fl);
            }

            // Replay limit-switch config (one frame per configured switch).
            for (int e = 0; e < 2; e++) {
                if (_swPin[i][e] < 0) continue;
                FrameBuilder fw; fw.begin(CMD_STEPPER_SET_SWITCH, DEVICE_STEPPER);
                fw.addInt(i); fw.addInt(e); fw.addInt(_swPin[i][e]); fw.addInt(_swTrig[i][e]);
                Pardalote.sendFrame(clientNum, fw);
            }

            // Replay each switch coordinate that's in use (absolute value —
            // NOT the SET_HOME re-zero, which is a one-shot shift). Home itself
            // is always the origin (0), so there's nothing else to replay.
            for (int e = 0; e < 2; e++) {
                if (_swPin[i][e] < 0 && _swPosition[i][e] == 0) continue;
                FrameBuilder fsp; fsp.begin(CMD_STEPPER_SET_SWITCH_POS, DEVICE_STEPPER);
                fsp.addInt(i); fsp.addInt(e); fsp.addInt(_swPosition[i][e]);
                Pardalote.sendFrame(clientNum, fsp);
            }

            // Sync live position.
            FrameBuilder fp; fp.begin(CMD_STEPPER_SET_POSITION, DEVICE_STEPPER);
            fp.addInt(i); fp.addInt(_steppers[i]->currentPosition());
            Pardalote.sendFrame(clientNum, fp);

            // If mid-move, replay the current target so the client knows the goal.
            if (_mode[i] == MODE_POSITION && _steppers[i]->distanceToGo() != 0) {
                FrameBuilder ft; ft.begin(CMD_STEPPER_MOVE_TO, DEVICE_STEPPER);
                ft.addInt(i); ft.addInt(_steppers[i]->targetPosition());
                Pardalote.sendFrame(clientNum, ft);
            }
        }
    }
};

// -------------------------------------------------------------------
// PardaloteStepper — sketch-facing collection of steppers, addressed by
// logical id.
//
// Create a stepper from the sketch (the browser sees it automatically as
// arduino.<name> — a full Stepper instance, identical to a browser-created
// one):
//   int base = PardaloteStepper.attach("base", 2, 3);   // name, STEP, DIR
//   PardaloteStepper.moveTo(base, 2000);
//
// Or drive steppers the browser configured, addressed by logical id:
//   int ids[6];
//   int n = PardaloteStepper.scan(ids, 6);      // attached steppers → their ids
//   long pos = PardaloteStepper.read(ids[0]);   // current step position
//
// Like the PWM servo, the inside/outside boundary is per stepper (each owns
// its own pins — there's no shared bus). A stepper you want to keep private
// to the sketch shouldn't be here at all: drive it with the plain AccelStepper
// library directly, the way an unshared pin just uses pinMode(). Everything
// attached through PardaloteStepper is browser-visible and limit-clamped, by
// design.
// -------------------------------------------------------------------
class PardaloteStepperAccess {
public:
    // attach(name, stepPin, dirPin, enPin?, invertMask?) — create a STEP/DIR
    // stepper and make it visible to browsers as arduino.<name>. Returns the
    // logical id for moveTo()/read()/etc., or -1 if no slot is free. invertMask
    // matches the browser default (0x04 = enable active-LOW; bit0=DIR, bit1=STEP,
    // bit2=ENABLE). Names longer than MAX_SHARE_NAME (15) are truncated.
    // Idempotent per name.
    int attach(const char* name, int stepPin, int dirPin,
               int enPin = -1, int invertMask = 0x04) const {
        return StepperExt::sketchAttach(name, STEPPER_DRIVER, stepPin, dirPin, enPin, invertMask);
    }

    // attach4wire(name, p1, p2, p3, p4) — create a 4-wire coil stepper
    // (28BYJ-48 via ULN2003, bare bipolar via H-bridge) visible as
    // arduino.<name>. Returns the logical id, or -1 if no slot is free.
    int attach4wire(const char* name, int p1, int p2, int p3, int p4) const {
        return StepperExt::sketchAttach(name, STEPPER_FULL4WIRE, p1, p2, p3, p4);
    }

    int  scan(int* out, int max) const { return StepperExt::listAttached(out, max); }
    long read(int id)            const { return StepperExt::positionId(id); }     // steps
    long position(int id)        const { return StepperExt::positionId(id); }     // alias
    long distanceToGo(int id)    const { return StepperExt::distanceToGoId(id); }
    bool isRunning(int id)       const { return StepperExt::runningId(id); }
    bool attached(int id)        const { return StepperExt::attachedId(id); }

    // Writes — routed through the same handler the browser uses.
    // Writes — command the move, then echo the resulting target to the browser
    // so its record matches, exactly as if the browser had issued the move.
    void moveTo(int id, long target) const { Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_MOVE_TO, id, (int32_t)target); StepperExt::echoTarget(id); }
    void move(int id, long rel)      const { Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_MOVE, id, (int32_t)rel);       StepperExt::echoTarget(id); }
    void stop(int id)                const { Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_STOP, id); }
    void hardStop(int id)            const { Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_HARD_STOP, id); }

    // Configure a hardware limit switch (end: LIMIT_MIN / LIMIT_MAX;
    // trigger: LOW (default, internal pull-up, switch to GND) or HIGH).
    // Pin -1 clears the switch. Echoed to browsers so their record syncs.
    void setLimitSwitch(int id, int end, int pin, int trigger = LOW) const {
        Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_SET_SWITCH, id, end, pin, trigger);
        FrameBuilder fb;
        fb.begin(CMD_STEPPER_SET_SWITCH, DEVICE_STEPPER);
        fb.addInt(id); fb.addInt(end); fb.addInt(pin); fb.addInt(trigger ? 1 : 0);
        Pardalote.broadcastFrame(fb);
    }
    void clearLimitSwitch(int id, int end) const { setLimitSwitch(id, end, -1); }

    // setSwitchPosition(id, end, coord) — declare the coordinate a limit switch
    // physically sits at (independent of the soft limits). Homing adopts this
    // coordinate when the switch trips; default 0 = the switch is the origin.
    // The SET_SWITCH_POS handler broadcasts the echo itself.
    void setSwitchPosition(int id, int end, long coord) const {
        Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_SET_SWITCH_POS, id, end, (int32_t)coord);
    }

    // Homing. setHome(id) re-zeros the frame: the current position becomes 0
    // (the origin/home) and the soft limits + switch positions shift with it
    // (the SET_HOME handler broadcasts the shifted state itself). home(id,
    // speed?) runs the routine — seek the switch, adopt its coordinate, back
    // off, travel to the origin.
    void setHome(int id, long value = 0) const { Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_SET_HOME, id, (int32_t)value); }
    void home(int id, float speed = 0, long timeoutMs = 0) const {
        Pardalote.command(DEVICE_STEPPER, CMD_STEPPER_HOME, id, (int32_t)speed, (int32_t)timeoutMs);
    }
};
inline PardaloteStepperAccess PardaloteStepper;

// Self-register — runs before setup(). Passes nullptr for the disconnect
// hook and StepperExt::loop for the per-iteration loop hook.
INSTALL_EXTENSION(DEVICE_STEPPER, StepperExt::handle, StepperExt::announce,
                  StepperExt::disconnect, StepperExt::loop)

#endif
