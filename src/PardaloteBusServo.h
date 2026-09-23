// ==============================================================
// PardaloteBusServo.h
// Pardalote Serial Bus Servo Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
//
// Add #include <PardaloteBusServo.h> to your sketch — the extension
// self-registers, no further setup is required.
//
// Serial-bus smart servos: Feetech ST / SMS series (0–4095 counts, e.g.
// STS3215 — the servo used in the LeRobot SO-100/SO-101 arms) and SC / SCS
// series (0–1023 counts). Driven over a single half-duplex UART, typically
// via a Waveshare Serial Bus Servo Driver board.
//
// Requires the Feetech / Waveshare SCServo library, which provides the
// SMS_STS and SCSCL classes. It handles the packet protocol, half-duplex
// direction switching, and — importantly — the sign-magnitude encoding of
// the offset and wheel-speed registers, a classic source of bugs if you
// hand-roll the protocol.
//
// -------------------------------------------------------------------
// THE BUS IS THE UNIT OF OWNERSHIP. Every bus servo shares ONE UART, so
// inside/outside can't be decided per servo the way it is per pin for PWM
// servos — it's decided per BUS. A serial bus is either a Pardalote bus or
// it isn't, never shared: every servo on a Pardalote bus is Pardalote
// hardware. A servo you want to keep private lives on a SEPARATE UART driven
// with the raw SCServo library; Pardalote never configures, scans, or drives
// that bus. The Pardalote bus is a single shared resource configured once
// (CMD_BUSSERVO_BUS_CONFIG, or lazily on first attach at 1 Mbps on Serial1).
//
// Addressing: a servo's hardware id (1–253) is its ADDRESS ON THE BUS — used
// to attach it and in scan/SyncWrite. Once attached it's controlled by its
// LOGICAL id (a dense instance slot), exactly like the PWM servo and stepper
// extensions, so groups and the browser treat all three uniformly. The
// logical→hardware map is internal; hardware ids are sparse (1, 5, 200…) and
// can't double as dense array slots, which is why the two coexist.
// -------------------------------------------------------------------
//
// NOTE: this targets the standard Feetech/Waveshare SCServo API. Method
// names are stable across the common library versions, but bring this up
// on hardware and confirm your board's wiring and library build before
// relying on it in a studio. The ST series is the primary tested path.
// ==============================================================

#ifndef PARDALOTE_BUSSERVO_H
#define PARDALOTE_BUSSERVO_H

#include <SCServo.h>       // Feetech / Waveshare: provides SMS_STS and SCSCL
#include "Pardalote.h"

#define MAX_BUS_SERVOS      16
#define BUSSERVO_DEF_BAUD   1000000UL
// While a servo isn't answering, retry its poll read at most this often (ms)
// rather than every due interval — keeps a bus-wide dropout from blocking loop()
// on the IOTimeOut every pass (which drops the UNO R4's transport). Lower =
// snappier LOST→found recovery, more blocking during an outage.
#define BUSSERVO_LOST_RETRY_MS  500UL

// STS/SMS EEPROM register addresses (for ops the high-level API doesn't wrap).
#define BUSSERVO_ADDR_MODE      33
#define BUSSERVO_ADDR_MIN_LIMIT  9
#define BUSSERVO_ADDR_MAX_LIMIT 11

// One servo's live feedback (returned by PardaloteBusServo.feedback()).
struct PardaloteBusServoReading {
    int  position;      // counts
    int  velocity;
    int  load;
    int  voltage;       // decivolts (e.g. 74 = 7.4 V)
    int  temperature;   // °C
    int  current;       // raw units (ST only)
    bool ok;            // false if the servo didn't answer
};

class BusServoExt {
private:
    // ---- shared bus ----
    inline static SMS_STS _st;      // ST / SMS series
    inline static SCSCL   _sc;      // SC / SCS series
    inline static bool    _busConfigured = false;
    inline static int     _serialIndex   = 1;
    inline static uint32_t _baud         = BUSSERVO_DEF_BAUD;
    inline static int     _rxPin         = -1;
    inline static int     _txPin         = -1;

    // ---- per-instance state (indexed by logical id) ----
    inline static uint8_t _servoId[MAX_BUS_SERVOS]  = {};
    inline static uint8_t _series[MAX_BUS_SERVOS]   = {};
    inline static bool    _attached[MAX_BUS_SERVOS] = {};
    inline static uint8_t _mode[MAX_BUS_SERVOS]     = {};   // 0=position, 1=wheel
    inline static bool    _torque[MAX_BUS_SERVOS]   = {};
    inline static bool    _limitSet[MAX_BUS_SERVOS] = {};
    inline static int16_t _minPos[MAX_BUS_SERVOS]   = {};
    inline static int16_t _maxPos[MAX_BUS_SERVOS]   = {};

    // The servo's OWN firmware angle-limit registers (EEPROM), read once at
    // attach and cached (announce() replays the cache — no re-read per connect).
    // Raw counts; -1 = unknown / servo didn't answer. min==max==0 is Feetech's
    // "limits disabled / multi-turn" encoding — JS derives that, the board just
    // caches the two raw values. Distinct from _minPos/_maxPos (board-RAM soft
    // limits): these are read-only, never written back to the servo.
    inline static int16_t _fwLimitMin[MAX_BUS_SERVOS] = {};
    inline static int16_t _fwLimitMax[MAX_BUS_SERVOS] = {};

    // Did the servo answer the attach-time ping? 1 = found, 0 = no response,
    // -1 = unknown/detached. Cached so announce() can replay it to late clients
    // without re-pinging (a blocking bus read). Mirrors the _fwLimit* cache.
    inline static int8_t  _found[MAX_BUS_SERVOS] = {};
    // Per-servo back-off: while a servo is not answering, its next allowed poll
    // read (millis). Keeps a dead servo from blocking loop() every pass. See the
    // periodic-read loop and BUSSERVO_LOST_RETRY_MS.
    inline static uint32_t _lostRetryAt[MAX_BUS_SERVOS] = {};

    // Sketch-created bus servos (PardaloteBusServo.attach("name", servoId)).
    // The name is what the browser binds (arduino.<name>); announce() replays a
    // CMD_SHARE frame for these so every connecting browser materialises the
    // object. Browser-created bindings have _sketchOwned = false. Mirrors the
    // servo/stepper sketch-attach path.
    inline static bool    _sketchOwned[MAX_BUS_SERVOS] = {};
    inline static char    _names[MAX_BUS_SERVOS][MAX_SHARE_NAME + 1] = {};

    // Arrival tracking — the bus can't push a "done", so after a position write
    // the board polls the servo's Moving flag in loop() and emits CMD_BUSSERVO_DONE
    // when it settles. Makes bus servos look like steppers/servos to the browser.
    inline static bool     _awaitDone[MAX_BUS_SERVOS]      = {};
    inline static uint32_t _awaitStartMs[MAX_BUS_SERVOS]   = {};
    inline static uint32_t _lastMovePollMs[MAX_BUS_SERVOS] = {};
    inline static uint32_t _lastRespMs[MAX_BUS_SERVOS]     = {};
    static const uint32_t MOVE_POLL_MS    = 33;     // ~30 Hz, like LeRobot
    static const uint32_t MOVE_STARTUP_MS = 40;     // let it start moving before first poll
    static const uint32_t MOVE_NO_RESP_MS = 1000;   // give up if the servo stops answering
    static const uint32_t MOVE_MAX_MS     = 30000;  // absolute ceiling on one move

    // Periodic reads — per-client registration + gating.
    // Gates on present position; default threshold 2 counts (magnetic
    // encoder noise floor).
    inline static ExtReadPoll _polls[MAX_BUS_SERVOS] = {};
    static constexpr uint16_t DEFAULT_THRESHOLD = 2;

    // Gesture segment schedule (CMD_BUSSERVO_GESTURE), rendered by a board-side player
    // that STREAMS setpoints continuously but PACES the schedule to the real servo.
    // Unlike a PWM Servo (whose controller takes a raw angle), a bus servo takes
    // (position, speed, accel) and runs its own move — so to render an authored easing
    // curve the board samples it on a fixed clock (BUS_STEP_MS) and commands the endpoint
    // at the curve's instantaneous speed (batched into ONE SyncWrite so lanes step in
    // phase). A pace-gate reads the real position now and then and freezes the schedule
    // whenever the servo lags, so the stream can't outrun the hardware (which was the
    // creep). This renders ease/overshoot (CURVE_BACK) on real hardware and lands honestly
    // on time. `from` is captured live at gesture start, then chained from each segment's
    // commanded end. See serviceGesture().
    static const uint8_t MAX_BUS_SERVO_SEGMENTS = 12;
    static const int      BUS_SEG_MAX_SPEED     = 4095;   // hardware/lib ceiling (counts/sec)
    // Continuous-stream tick: how often we push a fresh setpoint while a gesture plays.
    // Smaller = smoother curve rendering, more writes; larger = coarser, fewer writes.
    // Tracking is handled separately by the pace-gate (see below), so this is now purely a
    // smoothness/bus-load knob — no longer the thing that causes creep. ~25 Hz is a good start.
    static const uint32_t BUS_STEP_MS           = 40;     // ~25 Hz stream tick (bench knob)
    // Acceleration passed on every streamed WritePosEx. In streaming, WE compute the
    // smooth speed profile tick-by-tick, so the servo should add NO ramp of its own —
    // 0 = no acceleration limit (the SCServo default): the servo runs at exactly the
    // speed we command and stops when it reaches the target. A non-zero ramp makes the
    // servo decelerate to a stop at the target on ITS schedule, which (aimed at the far
    // endpoint) is a long gentle tail — the easeOut end-creep. Bench knob: raise it only
    // if you want the servo to soften our profile. (Plain one-shot writes still use 50.)
    static const uint8_t  BUS_STREAM_ACC        = 0;      // 0 = track our profile exactly (bench knob)
    struct BSeg { uint8_t curve; uint16_t dur; int32_t value; };
    inline static BSeg     _bsegs[MAX_BUS_SERVOS][MAX_BUS_SERVO_SEGMENTS] = {};
    inline static uint8_t  _bsegCount[MAX_BUS_SERVOS]  = {};   // 0 = no gesture running
    inline static bool     _gestureActive[MAX_BUS_SERVOS] = {};   // last-broadcast gesturing state
    inline static uint8_t  _bsegIndex[MAX_BUS_SERVOS]  = {};
    inline static uint8_t  _bsegFlags[MAX_BUS_SERVOS]  = {};   // GESTURE_FLAG_*
    inline static int32_t  _bsegFrom[MAX_BUS_SERVOS]   = {};   // start value of the current segment
    inline static int32_t  _bsegTarget[MAX_BUS_SERVOS] = {};   // end value of the current segment (chained)
    inline static uint16_t _bsegDurMs[MAX_BUS_SERVOS]  = {};   // current segment duration (ms)
    inline static uint32_t _bsegStartMs[MAX_BUS_SERVOS]= {};   // board-clock start of the current segment
    inline static uint32_t _lastBusStepMs              = 0;    // last interpolation tick (shared clock)
    inline static PardaloteGestureDone _onDone[MAX_BUS_SERVOS] = {};   // sketch whenDone() callback

    // ---- Pace-gate (continuous streaming, but paced to the real servo) ----
    // The stream is CONTINUOUS (the servo cruises toward the endpoint and never stops),
    // but the SCHEDULE clock (which drives segment-advance and the final DONE) is paced
    // to the hardware: we sample the servo's real position occasionally and, whenever it
    // has fallen behind the scheduled position, we FREEZE the schedule clock until it
    // catches up. So the stream can't outrun the servo — lag can't accumulate and nothing
    // spills past the end (the creep) — while the motion stays smooth. Reads are rare
    // (BUS_GATE_MS) and skip a non-responding servo, so the bus stays clear for WiFi/USB.
    static const uint32_t BUS_GATE_MS      = 120;  // how often to check the servo's real position (bench knob)
    static const long     BUS_GATE_LAG_HI  = 60;   // freeze the schedule once the servo is this far behind (counts)
    static const long     BUS_GATE_LAG_LO  = 20;   // resume once it has caught back within this (hysteresis)
    // Speed calibration. The counts/sec we compute from the curve doesn't map 1:1 to the
    // servo's speed units. With the look-ahead carrot (below) the servo tracks the paced
    // curve position when the commanded speed matches the curve's LOCAL speed, so ~1.0 is
    // the natural value; the pace-gate mops up any residual lag. Bench knob: raise a touch
    // if a move finishes LATE, lower if it finishes EARLY (arrives then waits). ~1.0 = no
    // scaling. pushSetpoint() still caps at BUS_SEG_MAX_SPEED.
    static constexpr float BUS_SPEED_GAIN  = 1.0f;
    static const uint8_t  BUS_LEAD         = 5;    // look-ahead in stream ticks: every curve aims at a point BUS_LEAD
                                                   // ticks ahead ON the curve and runs at the curve's local speed, so
                                                   // it chases a "carrot" it never catches — cruising continuously
                                                   // (no stop-and-go) while tracing the curve's actual shape, instead
                                                   // of sprinting to a fixed endpoint and arriving early. Larger =
                                                   // smoother but trails slightly at the very end; smaller = tighter
                                                   // timing but risks steppiness. (CURVE_BACK's carrot goes PAST the
                                                   // endpoint, ease > 1, so its overshoot is traced the same way.)
                                                   // Bench knob: higher = smoother BACK, but the overshoot leads more.
    // Short segments play LINEAR regardless of authored curve. A curve only renders cleanly when
    // the move spans several look-ahead windows (BUS_LEAD*BUS_STEP_MS): below that, more and more
    // of it clamps to the endpoint (endpoint-commanding), where a front-loaded curve like easeOut
    // phantom-lags and the pace-gate freezes it into a crawl. 3 windows is the floor for a clean
    // curve; shorter than that, linear lands cleanly and the shape difference is imperceptible.
    // Tied to the tick params so it auto-scales if BUS_LEAD / BUS_STEP_MS are retuned.
    static const uint16_t BUS_MIN_CURVE_MS = 3 * BUS_LEAD * BUS_STEP_MS;   // 600 ms at the defaults
    inline static uint32_t _pausedMs[MAX_BUS_SERVOS]     = {};   // total time this lane's clock has been frozen
    inline static bool     _paused[MAX_BUS_SERVOS]       = {};   // schedule frozen right now (this lane's cohort is waiting)
    inline static bool     _behind[MAX_BUS_SERVOS]       = {};   // THIS lane's own gate verdict — is the servo lagging? (hysteresis)
    inline static uint32_t _gestureStartMs[MAX_BUS_SERVOS] = {}; // cohort key: the gesture's original start; lanes that share it wait for each other
    inline static int32_t  _schedPos[MAX_BUS_SERVOS]     = {};   // scheduled position at the paced phase (gate reference)
    inline static uint32_t _lastGateMs[MAX_BUS_SERVOS]   = {};   // last pace-gate position read
    inline static uint32_t _lastGestureNow               = 0;    // last serviceGesture() time (for dt)

    static bool validId(int id) { return id >= 0 && id < MAX_BUS_SERVOS; }
    static bool isSC(int id)    { return _series[id] == BUSSERVO_SERIES_SC; }

    static HardwareSerial* busSerial() {
#if defined(PLATFORM_ESP32)
        return (_serialIndex == 2) ? &Serial2 : &Serial1;
#else
        return &Serial1;   // UNO R4 WiFi: Serial1 on D0/D1
#endif
    }

    // Bring up the shared UART and point both servo classes at it. Idempotent.
    static void ensureBus() {
        if (_busConfigured) return;
        HardwareSerial* s = busSerial();
#if defined(PLATFORM_ESP32)
        if (_rxPin >= 0 && _txPin >= 0) s->begin(_baud, SERIAL_8N1, _rxPin, _txPin);
        else                            s->begin(_baud);
#else
        s->begin(_baud);
#endif
        _st.pSerial = s;
        _sc.pSerial = s;
        // Keep bus reads short. SCServo's default 100 ms read timeout, taken
        // once per polled servo when the bus goes silent (servo powered down,
        // wire pulled), stalls the main loop long enough to starve _ws.loop()
        // — the WebSocket drops (and WiFiS3 on the R4 destabilises). A live
        // servo answers in well under 1 ms, so 5 ms is ample headroom while
        // capping a fully-silent 6-servo poll at ~30 ms instead of ~600 ms.
        _st.IOTimeOut = 5;
        _sc.IOTimeOut = 5;
        _busConfigured = true;
        Serial.print(F("BusServo: bus up on Serial")); Serial.print(_serialIndex);
        Serial.print(F(" @ ")); Serial.print(_baud); Serial.println(F(" baud"));
    }

    // ---- series-routed primitives ----
    static void writePos(int id, int pos, int speed, int acc) {
        uint8_t sid = _servoId[id];
        // Last gate before the wire: never let an out-of-range count reach the
        // servo, which would wrap it mod-resolution and lurch the wrong way.
        // Covers every caller — browser write, sketch write, gesture.
        pos = constrain(pos, 0, isSC(id) ? 1023 : 4095);
        if (isSC(id)) _sc.WritePos(sid, pos, 0, speed);
        else          _st.WritePosEx(sid, pos, speed, acc);
    }

    // Clamp a target to this id's soft limits (if set) else the series' physical
    // range. Shared by segment loading and the streaming tick's SyncWrite batch
    // (which bypasses writePos()'s own clamp), so an out-of-range value never
    // reaches the wire and mod-wraps the servo. CURVE_BACK overshoot lands here.
    static int32_t clampToRange(int id, int32_t v) {
        if (_limitSet[id]) return constrain(v, (int32_t)_minPos[id], (int32_t)_maxPos[id]);
        return constrain(v, (int32_t)0, (int32_t)(isSC(id) ? 1023 : 4095));
    }

    static void writeSpeed(int id, int speed, int acc) {
        uint8_t sid = _servoId[id];
        // SC/SCS (SCSCL) has no constant-speed WriteSpe — continuous drive is
        // PWMMode/WritePWM instead (different registers/units). Deferred until an
        // SC servo is on the bench; STS/SMS is the supported continuous path.
        if (isSC(id)) { Serial.println(F("BusServo: SC-series continuous mode not supported yet")); return; }
        _st.WriteSpe(sid, speed, acc);
    }

    static void setTorque(int id, bool en) {
        uint8_t sid = _servoId[id];
        if (isSC(id)) _sc.EnableTorque(sid, en ? 1 : 0);
        else          _st.EnableTorque(sid, en ? 1 : 0);
        _torque[id] = en;
    }

    static void setMode(int id, uint8_t mode) {
        uint8_t sid = _servoId[id];
        if (mode == BUSSERVO_MODE_WHEEL) {
            // SC/SCS (SCSCL) has no WheelMode — see writeSpeed(). Deferred.
            if (isSC(id)) { Serial.println(F("BusServo: SC-series continuous mode not supported yet")); return; }
            _st.WheelMode(sid);
        } else {
            // Return to position mode: mode register lives in EEPROM, so
            // unlock → write → lock. (WheelMode() set it to 1 for us.)
            if (isSC(id)) {
                _sc.unLockEprom(sid); _sc.writeByte(sid, BUSSERVO_ADDR_MODE, 0); _sc.LockEprom(sid);
            } else {
                _st.unLockEprom(sid); _st.writeByte(sid, BUSSERVO_ADDR_MODE, 0); _st.LockEprom(sid);
            }
        }
        _mode[id] = mode;
    }

    // Arm the done-poller for a move on this logical id: loop() then polls the
    // servo's Moving flag (~30 Hz) and broadcasts CMD_BUSSERVO_DONE when it
    // settles. Private — every write path routes through the handler (the
    // WRITE / SYNC_WRITE cases), which arms it; nothing outside BusServoExt
    // writes to the bus directly.
    static void beginAwaitDone(int id) {
        if (!validId(id) || !_attached[id]) return;
        uint32_t now        = millis();
        _awaitDone[id]      = true;
        _awaitStartMs[id]   = now;
        _lastRespMs[id]     = now;
        _lastMovePollMs[id] = 0;
    }

    // Reset the pace-gate for a freshly (re)started gesture on this lane: clear any
    // freeze, seed the schedule reference at the live start position, and arm the
    // read timers. Called by both gesture-start paths after loadBusSegment(id, 0, …),
    // so _bsegStartMs[id] already holds this gesture's original start — the cohort key
    // that binds a group's lanes together for the shared pace barrier.
    static void resetPace(int id, int32_t from) {
        uint32_t now        = millis();
        _pausedMs[id]       = 0;
        _paused[id]         = false;
        _behind[id]         = false;
        _gestureStartMs[id] = _bsegStartMs[id];
        _lastGateMs[id]     = now;
        _schedPos[id]       = from;
    }

    // Arm segment `idx` for the streaming interpolator: resolve its absolute
    // target (clamped) and record from / target / duration / start-time. The
    // curve is NOT rendered here — serviceGesture() samples it each stream tick
    // and commands setpoints. `from` is _bsegFrom[id] (the live position for
    // idx 0, then chained from each commanded end). `startMs` anchors the segment
    // on the shared board clock, so grouped lanes stay phase-locked and the
    // timeline never drifts (each next segment starts at prev start + prev dur).
    static void loadBusSegment(int id, uint8_t idx, uint32_t startMs) {
        const BSeg& seg  = _bsegs[id][idx];
        int32_t target   = (_bsegFlags[id] & GESTURE_FLAG_ABSOLUTE) ? seg.value
                                                                     : _bsegFrom[id] + seg.value;
        _bsegIndex[id]   = idx;
        _bsegTarget[id]  = clampToRange(id, target);
        _bsegDurMs[id]   = seg.dur ? seg.dur : 1;
        _bsegStartMs[id] = startMs;
    }

    // Land + retire a finished gesture: report DONE (commanded landing — no bus
    // read on the hot path; the periodic reads correct the marker), fire the
    // sketch whenDone(), and clear the lane. The gesture-active edge scan in
    // loop() broadcasts the inactive state off _bsegCount hitting 0.
    static void finishGesture(int id) {
        int32_t landed = _bsegTarget[id];
        _bsegCount[id] = 0;
        _paused[id] = false; _behind[id] = false; _pausedMs[id] = 0;   // clear the pace-gate for the next gesture
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_DONE, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(landed);
        Pardalote.broadcastFrame(fb);
        if (_onDone[id]) _onDone[id](id);   // sketch whenDone() hook
    }

    // Collect one streamed setpoint. ST lanes batch into the SyncWrite arrays
    // (one phase-locked frame per tick); SC/SCS lanes (no SyncWritePosEx) are
    // written individually here. `pos` is pre-clamped by the caller.
    static void pushSetpoint(int id, int32_t pos, int speed,
                             uint8_t* ids, int16_t* positions, uint16_t* speeds,
                             uint8_t* accs, int& n) {
        if (speed < 1)                 speed = 1;
        if (speed > BUS_SEG_MAX_SPEED) speed = BUS_SEG_MAX_SPEED;
        if (isSC(id)) {
            _sc.WritePos(_servoId[id], (int)pos, 0, speed);   // SC: no acc / no SyncWrite
        } else {
            ids[n]       = _servoId[id];
            positions[n] = (int16_t)pos;
            speeds[n]    = (uint16_t)speed;
            accs[n]      = BUS_STREAM_ACC;
            n++;
        }
    }

    // Gesture player — CONTINUOUS streaming, PACED to the real servo, GROUP-SCOPED.
    //
    // Each gesture lane runs on a paced "phase clock" (real time minus the time it has
    // spent frozen). We sample the eased curve at that phase and, on the stream tick,
    // command a look-ahead "carrot" on the curve at its local speed, so the servo cruises
    // continuously while tracing the curve's shape (see the command block for the detail).
    //
    // The pace-gate keeps the stream from outrunning the hardware (the cause of the creep):
    // every BUS_GATE_MS we read the servo's real position and set its own _behind[] verdict
    // when it has fallen more than BUS_GATE_LAG_HI counts behind the scheduled position
    // (clearing within BUS_GATE_LAG_LO — hysteresis). A frozen phase clock stops advancing,
    // so the segment schedule and the final DONE hold while the servo closes the gap.
    //
    // GROUP-SCOPED barrier: the freeze is decided per COHORT, not per lane. All lanes that
    // share a gesture start (_gestureStartMs — one group dispatched together) freeze as a
    // unit whenever ANY of them is _behind, so the fast lanes wait for the slowest servo and
    // the group stays phase-locked instead of tearing the pose apart. Cohort members share
    // the same accumulated pausedMs (they freeze/resume together from a common start), so
    // their phase clocks stay identical. There is NO give-up ceiling by design: a big move
    // simply slows the whole gesture to the pace it can sustain, and a genuinely stuck lane
    // holds the whole gesture (no DONE) until it recovers — an honest halt rather than a
    // confusing partial/limp-forward. (A merely-slow servo keeps closing the gap during a
    // freeze, so it always resumes; only a truly stuck one stays frozen.) Reads stay rare
    // (skip a LOST servo) so the bus stays clear for WiFi/USB. ST lanes batch into one
    // SyncWrite. Runs beside the plain-write done poller (mutually exclusive per lane).
    static void serviceGesture(uint32_t now) {
        uint32_t dt = now - _lastGestureNow;
        _lastGestureNow = now;
        if (dt > 250) dt = 0;   // first call / long stall — don't jump the phase

        bool tick = (now - _lastBusStepMs >= BUS_STEP_MS);
        if (tick) _lastBusStepMs = now;

        // Group-scoped freeze decision + paced-clock accumulation. A lane freezes when any
        // lane in its cohort (same _gestureStartMs) is behind — so a group waits as one, for
        // as long as it takes (no ceiling: a stuck lane holds the whole gesture until it recovers).
        for (int id = 0; id < MAX_BUS_SERVOS; id++) {
            if (_bsegCount[id] == 0 || !_attached[id]) continue;
            bool cohortBehind = false;
            for (int j = 0; j < MAX_BUS_SERVOS; j++) {
                if (_bsegCount[j] && _attached[j] && _gestureStartMs[j] == _gestureStartMs[id] && _behind[j]) {
                    cohortBehind = true; break;
                }
            }
            _paused[id] = cohortBehind;
            if (_paused[id]) _pausedMs[id] += dt;                        // frozen clock stops advancing
        }

        static uint8_t  ids[MAX_BUS_SERVOS];
        static int16_t  positions[MAX_BUS_SERVOS];
        static uint16_t speeds[MAX_BUS_SERVOS];
        static uint8_t  accs[MAX_BUS_SERVOS];
        int n = 0;   // ST SyncWrite batch size

        for (int id = 0; id < MAX_BUS_SERVOS; id++) {
            if (_bsegCount[id] == 0) continue;              // no gesture on this lane
            if (!_attached[id]) { _bsegCount[id] = 0; continue; }

            uint32_t pnow = now - _pausedMs[id];            // paced phase (frozen while the cohort waits)

            // Advance through any segments the paced clock has passed. Each next segment
            // is anchored at prev start + prev dur (no drift). A lane whose final segment
            // has elapsed (paced — i.e. the servo has kept up to here) lands and finishes.
            bool finished = false;
            while (pnow - _bsegStartMs[id] >= _bsegDurMs[id]) {
                if (_bsegIndex[id] + 1 < _bsegCount[id]) {
                    _bsegFrom[id] = _bsegTarget[id];        // chain from the commanded end
                    loadBusSegment(id, _bsegIndex[id] + 1, _bsegStartMs[id] + _bsegDurMs[id]);
                } else {
                    finishGesture(id);
                    finished = true;
                    break;
                }
            }
            if (finished || _bsegCount[id] == 0) continue;

            // Scheduled position at the paced phase (also the pace-gate's reference), plus
            // the position one stream-tick ahead (for the feed-forward speed). The look-
            // ahead is a FIXED step in the curve, so the speed stays right even while frozen.
            uint16_t dur   = _bsegDurMs[id];
            int32_t  from  = _bsegFrom[id];
            int32_t  d     = _bsegTarget[id] - from;
            // Short segments fall back to linear — too few ticks to render a curve (see BUS_MIN_CURVE_MS).
            uint8_t  curve = (dur < BUS_MIN_CURVE_MS) ? CURVE_LINEAR : _bsegs[id][_bsegIndex[id]].curve;
            uint32_t el    = pnow - _bsegStartMs[id];
            float fracNow  = (float)el / (float)dur;
            int32_t posNow  = clampToRange(id, from + (int32_t)lroundf((float)d * pardaloteEase(curve, fracNow)));
            _schedPos[id]   = posNow;

            // Pace-gate: sample the real position now and then and set THIS lane's own
            // _behind verdict (hysteresis). The freeze itself is decided cohort-wide up top
            // from every lane's verdict, so this only reports "am I lagging?" — a group's
            // clock freezes when any member says yes. Signed by travel direction, so a servo
            // that's caught up (or ahead) clears; only a BEHIND servo raises the flag.
            if (now - _lastGateMs[id] >= BUS_GATE_MS) {
                _lastGateMs[id] = now;
                if (_found[id] != 0) {                       // don't block the loop on a dead servo
                    int actual = readPos(_servoId[id]);
                    if (actual >= 0) {
                        long dir = (d >= 0) ? 1 : -1;
                        long lag = ((long)posNow - (long)actual) * dir;   // >0 = servo is behind schedule
                        if (!_behind[id] && lag > BUS_GATE_LAG_HI)      _behind[id] = true;
                        else if (_behind[id] && lag < BUS_GATE_LAG_LO)  _behind[id] = false;
                    }
                }
            }

            // Command on the stream tick. We aim a few ticks AHEAD on the curve (BUS_LEAD)
            // and move at the curve's LOCAL speed, so the servo chases a point it never
            // quite catches: it cruises continuously (no stop-and-go = no steppiness) and,
            // because the aim point rides the curve itself, it follows the curve's SHAPE.
            // That is what keeps timing honest — a servo aimed at the far endpoint would
            // sprint there on a curve's fast portion and arrive early (worst on easeOut);
            // aiming only BUS_LEAD ticks ahead, it can't shortcut. CURVE_BACK's aim point
            // runs PAST the endpoint (ease > 1) so its overshoot is traced the same way.
            // A pure hold (d==0) needs no command.
            if (tick && d != 0) {
                float fracLead = fracNow + (float)(BUS_LEAD * BUS_STEP_MS) / (float)dur;
                if (fracLead > 1.0f) fracLead = 1.0f;
                int32_t posCmd = clampToRange(id, from + (int32_t)lroundf((float)d * pardaloteEase(curve, fracLead)));
                // Speed = distance to that aim point over the time until it (not the 1-tick
                // curve step). Sizing it to the carrot never collapses to ~0 at a reversal
                // (CURVE_BACK's overshoot peak) the way the instantaneous step does — that
                // stall was the chop.
                long  reach = labs((long)posCmd - (long)posNow);
                // Time until that carrot: the lead window normally, but the REAL time left
                // (fracLead*dur - el) once fracLead clamps at the endpoint — which is the whole
                // of a move shorter than the lead window, and the tail of every move. Using the
                // fixed window there divides by too much and the servo crawls. Floor at one tick.
                float leadMs = fracLead * (float)dur - (float)el;
                if (leadMs < (float)BUS_STEP_MS) leadMs = (float)BUS_STEP_MS;
                int   speed = (int)lroundf((float)reach / (leadMs / 1000.0f) * BUS_SPEED_GAIN);
                pushSetpoint(id, posCmd, speed, ids, positions, speeds, accs, n);
            }
        }

        if (n > 0) _st.SyncWritePosEx(ids, n, positions, speeds, accs);
    }

    // Drop any running gesture (a direct write / mode change / detach
    // supersedes it). No-op when none is active. Does NOT emit DONE — the
    // superseding command owns completion.
    static void cancelBusGesture(int id) { if (validId(id)) { _bsegCount[id] = 0; _paused[id] = false; _behind[id] = false; _pausedMs[id] = 0; } }

    // Gesture-active state (Ar→JS, existence only): broadcast on the _bsegCount
    // 0<->positive edge so browsers reflect "gesturing" for JS- OR sketch-
    // authored gestures, and however they end (see loop()).
    static void sendGestureState(uint8_t clientNum, int id, uint8_t active) {
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_GESTURE_STATE, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(active);
        if (clientNum == 0xFF) Pardalote.broadcastFrame(fb);
        else                   Pardalote.sendFrame(clientNum, fb);
    }
    static void updateGestureState(int id) {
        bool active = _bsegCount[id] > 0;
        if (active != _gestureActive[id]) {
            _gestureActive[id] = active;
            sendGestureState(0xFF, id, active ? 1 : 0);
        }
    }

public:
    // Board-side bus config — the sketch-side twin of the browser's
    // configureBus(). Sets which UART + (ESP32) pins the bus uses, then re-begins.
    // Same effect as the CMD_BUSSERVO_BUS_CONFIG wire path. Call before attach().
    static void configureBus(int rxPin, int txPin, int serialIndex, uint32_t baud) {
        _serialIndex = serialIndex;
        if (baud) _baud = baud;
        _rxPin = rxPin;
        _txPin = txPin;
        _busConfigured = false;   // force re-begin with the new settings
        ensureBus();
    }

    // Compose-and-play a segment schedule from the sketch — the board-side
    // gesture() (see internal/gesture.h). The same code the CMD_BUSSERVO_GESTURE
    // wire path runs, minus the byte unpack. Registered as the DEVICE_BUSSERVO
    // starter for the coordinated PardaloteGesture builder. The bus servo is now
    // time-clocked (the streaming interpolator samples the curve off the shared
    // startMs), so grouped lanes are phase-locked; padToMs appends a trailing
    // hold so short lanes arrive with the longest.
    static void startGesture(int id, const PardaloteSeg* segs, uint8_t count,
                             uint8_t flags, uint32_t startMs, uint32_t padToMs = 0,
                             const PardaloteGestureMod& mod = PardaloteGestureMod()) {
        if (!validId(id) || !_attached[id] || !segs || count == 0) return;
        ensureBus();
        uint8_t  n     = count > MAX_BUS_SERVO_SEGMENTS ? MAX_BUS_SERVO_SEGMENTS : count;
        uint32_t total = 0;
        for (uint8_t i = 0; i < n; i++) {
            _bsegs[id][i].curve = segs[i].curve;
            _bsegs[id][i].dur   = segs[i].dur ? segs[i].dur : 1;
            _bsegs[id][i].value = segs[i].value;
            total += _bsegs[id][i].dur;
        }
        if (padToMs > total && n < MAX_BUS_SERVO_SEGMENTS) {   // trailing hold → arrive together
            uint32_t padMs = padToMs - total;
            _bsegs[id][n].curve = CURVE_LINEAR;
            _bsegs[id][n].dur   = padMs > 0xFFFF ? 0xFFFF : (uint16_t)padMs;
            _bsegs[id][n].value = (flags & GESTURE_FLAG_ABSOLUTE) ? _bsegs[id][n - 1].value : 0;
            n++;
        }
        n = pardaloteApplyModsInPlace(_bsegs[id], n, flags, mod);   // scale · speed · crop
        if (n == 0) { _bsegCount[id] = 0; return; }                 // cropped to nothing
        _bsegCount[id] = n;
        _bsegFlags[id] = flags;
        int32_t from = readPos(_servoId[id]);   // live start (lead-in anchor)
        if (from < 0) from = 0;
        _bsegFrom[id] = from;
        loadBusSegment(id, 0, startMs ? startMs : millis());
        resetPace(id, from);
    }

    // Register a whenDone() callback (nullptr clears it).
    static void setOnGestureDone(int id, PardaloteGestureDone cb) {
        if (validId(id)) _onDone[id] = cb;
    }

    // Immediate write to an absolute position — the DEVICE_BUSSERVO
    // ImmediateWriter for Pardalote.write(). Mirrors CMD_BUSSERVO_WRITE
    // (cancels any gesture, clamps, arms the done poller) + echoes. Uses the
    // same default speed/acc as the browser write().
    static void writeNow(int id, int32_t target) {
        if (!validId(id) || !_attached[id]) return;
        cancelBusGesture(id);
        int pos = (int)target;
        if (_limitSet[id]) pos = constrain(pos, (int)_minPos[id], (int)_maxPos[id]);
        writePos(id, pos, 2400, 50);
        beginAwaitDone(id);
        echoTarget(id, pos);
    }

    // -------------------------------------------------------------------
    // Bus-level primitives, addressed by HARDWARE servo ID (the number
    // scan() returns) — used by discovery (scan/ping), the frame handler,
    // and the logical-id accessors below. Live, blocking bus transactions.
    // -------------------------------------------------------------------
    // Series of a known-attached servo, else default ST.
    static int busSeriesForId(uint8_t servoId) {
        for (int i = 0; i < MAX_BUS_SERVOS; i++)
            if (_attached[i] && _servoId[i] == servoId) return _series[i];
        return BUSSERVO_SERIES_ST;
    }

    // Logical instance bound to a hardware servo ID, or -1 if none is.
    static int logicalForServoId(uint8_t servoId) {
        for (int i = 0; i < MAX_BUS_SERVOS; i++)
            if (_attached[i] && _servoId[i] == servoId) return i;
        return -1;
    }

    // Ping a range of IDs; fill out[] with those that respond, return count.
    static int scanBus(uint8_t* out, int max, int first, int last) {
        ensureBus();
        int n = 0;
        for (int sid = first; sid <= last && n < max; sid++)
            if (_st.Ping(sid) != -1 || _sc.Ping(sid) != -1) out[n++] = (uint8_t)sid;
        return n;
    }

    // Present position (counts) for one servo ID, or -1 if it doesn't answer.
    static int readPos(uint8_t servoId) {
        ensureBus();
        bool sc = (busSeriesForId(servoId) == BUSSERVO_SERIES_SC);
        int rc = sc ? _sc.FeedBack(servoId) : _st.FeedBack(servoId);
        if (rc == -1) return -1;
        return sc ? _sc.ReadPos(-1) : _st.ReadPos(-1);
    }

    // Full feedback in one bus transaction.
    static PardaloteBusServoReading readFeedback(uint8_t servoId) {
        ensureBus();
        PardaloteBusServoReading r = { -1, 0, 0, 0, 0, 0, false };
        bool sc = (busSeriesForId(servoId) == BUSSERVO_SERIES_SC);
        int rc = sc ? _sc.FeedBack(servoId) : _st.FeedBack(servoId);
        if (rc == -1) return r;
        r.ok = true;
        if (sc) {
            r.position    = _sc.ReadPos(-1);   r.velocity = _sc.ReadSpeed(-1);
            r.load        = _sc.ReadLoad(-1);  r.voltage  = _sc.ReadVoltage(-1);
            r.temperature = _sc.ReadTemper(-1);
        } else {
            r.position    = _st.ReadPos(-1);   r.velocity = _st.ReadSpeed(-1);
            r.load        = _st.ReadLoad(-1);  r.voltage  = _st.ReadVoltage(-1);
            r.temperature = _st.ReadTemper(-1); r.current = _st.ReadCurrent(-1);
        }
        return r;
    }

    // The servo's own Moving flag (its "am I still moving?" — accounts for
    // deadband/settling). One bus read. Returns 1 = moving, 0 = arrived/idle,
    // -1 if the servo didn't answer.
    static int readMoving(uint8_t servoId) {
        ensureBus();
        bool sc = (busSeriesForId(servoId) == BUSSERVO_SERIES_SC);
        return sc ? _sc.ReadMove(servoId) : _st.ReadMove(servoId);
    }

    // Logical-id read accessors used by the PardaloteBusServo sketch object.
    // A bus servo is addressed by its LOGICAL id (the value attach() returns),
    // exactly like the browser and groups; these map that to the servo's bus
    // (hardware) id and do a live, blocking bus transaction. Only servos that
    // have been attached (are Pardalote instances on the bus) are addressable —
    // there is no drive-by-raw-hardware-id path (that would reach a servo the
    // system doesn't model; on a Pardalote bus every servo is a Pardalote one,
    // so the answer is always "attach it first").
    static int positionById(int id) {
        return (validId(id) && _attached[id]) ? readPos(_servoId[id]) : -1;
    }
    static PardaloteBusServoReading feedbackById(int id) {
        if (validId(id) && _attached[id]) return readFeedback(_servoId[id]);
        return { -1, 0, 0, 0, 0, 0, false };
    }
    static int movingById(int id) {
        return (validId(id) && _attached[id]) ? readMoving(_servoId[id]) : -1;
    }

    // Read the servo's EEPROM min/max ANGLE-LIMIT registers into the cache
    // (_fwLimitMin/Max). One or two blocking bus reads; each register is a
    // word (readWord returns -1 on no answer, so a dead servo caches -1/-1).
    // These are the servo's own firmware limits — read-only, never written.
    static void readFwLimits(int id) {
        if (!validId(id) || !_attached[id]) return;
        ensureBus();
        uint8_t sid = _servoId[id];
        if (isSC(id)) {
            _fwLimitMin[id] = (int16_t)_sc.readWord(sid, SCSCL_MIN_ANGLE_LIMIT_L);
            _fwLimitMax[id] = (int16_t)_sc.readWord(sid, SCSCL_MAX_ANGLE_LIMIT_L);
        } else {
            _fwLimitMin[id] = (int16_t)_st.readWord(sid, SMS_STS_MIN_ANGLE_LIMIT_L);
            _fwLimitMax[id] = (int16_t)_st.readWord(sid, SMS_STS_MAX_ANGLE_LIMIT_L);
        }
    }

    // Send the cached firmware limits to one client: [id, min, max] (raw
    // counts; -1 = unknown). JS interprets min==max==0 as "disabled".
    static void sendFwLimits(uint8_t clientNum, int id) {
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_READ_LIMITS, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(_fwLimitMin[id]);
        fb.addInt(_fwLimitMax[id]);
        Pardalote.sendFrame(clientNum, fb);
    }

    // Send the cached attach-time presence to one client: [id, servoId, present].
    static void sendPresence(uint8_t clientNum, int id) {
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_PRESENT, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(_servoId[id]);
        fb.addInt(_found[id]);
        Pardalote.sendFrame(clientNum, fb);
    }

    // Echo a sketch-issued write to the browser so it sets its cached target
    // exactly as if the browser had written it. Addressed by LOGICAL id (the
    // browser routes by it); echoes the clamped value the board actually applied.
    static void echoTarget(int id, int position) {
        if (!validId(id) || !_attached[id]) return;
        // Mirror writePos's clamp so the browser caches the value the board
        // actually applied — physical range always, soft limits when set.
        position = constrain(position, 0, isSC(id) ? 1023 : 4095);
        if (_limitSet[id]) position = constrain(position, _minPos[id], _maxPos[id]);
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_WRITE, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(position);
        Pardalote.broadcastFrame(fb);
    }

    // Echo a sketch-issued torque change so the browser's cached torque state
    // stays in sync (teach-by-demonstration reads it). Addressed by logical id.
    static void echoTorque(int id, bool on) {
        if (!validId(id) || !_attached[id]) return;
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_TORQUE, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(on ? 1 : 0);
        Pardalote.broadcastFrame(fb);
    }

    // -------------------------------------------------------------------
    // Sketch-created bus servos — PardaloteBusServo.attach("name", servoId).
    //
    // Creation and browser visibility are one act (see the servo extension for
    // the full rationale): a logical instance is bound to the hardware servo
    // ID through the same handler a browser attach uses, and a CMD_SHARE frame
    // (+ attach/mode/torque state) is broadcast so connected browsers
    // materialise arduino.<name> immediately. announce() replays the same
    // sequence for browsers that connect later.
    //
    // Logical ids are allocated from the TOP of the range downward —
    // browser-assigned ids grow from 0 upward, so the two sides can't collide
    // until every slot is in use. Idempotent on the name: attaching again with
    // a name that is already sketch-owned reuses its id.
    //
    // Returns the LOGICAL id — the handle write()/read()/torque() take on the
    // PardaloteBusServo object (and the same id the browser and groups use) —
    // or -1 if no slot is free. The `servoId` argument is the servo's address
    // on the bus; it's stored as the binding and reappears only in scan/
    // SyncWrite, never as the control handle.
    static int sketchAttach(const char* name, int servoId, int series) {
        if (name == nullptr || name[0] == '\0') return -1;

        int id = -1;
        for (int i = 0; i < MAX_BUS_SERVOS; i++) {
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        }
        if (id < 0) {
            for (int i = MAX_BUS_SERVOS - 1; i >= 0; i--) {
                if (!_attached[i] && !_sketchOwned[i]) { id = i; break; }
            }
        }
        if (id < 0) {
            Serial.print(F("BusServo: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        // Attach through the same code path a browser attach takes.
        Pardalote.command(DEVICE_BUSSERVO, CMD_BUSSERVO_ATTACH, id, servoId, series);

        // Tell any connected browsers now (no-op when none are connected;
        // announce() covers future connects): name → attach → mode → torque,
        // the same order announce() replays.
        broadcastShare(id);
        sendAttachState(id, false, 0);

        return id;
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addString(_names[id]);
        Pardalote.broadcastFrame(fb);
    }

    // Emit this instance's attach frame (servo ID + series) followed by its
    // mode and torque state. Shared by sketchAttach (broadcast to everyone)
    // and announce (unicast to one joining client) so the two stay in
    // lockstep. When `unicast` is false, `clientNum` is ignored.
    static void sendAttachState(int id, bool unicast, uint8_t clientNum) {
        FrameBuilder fa;
        fa.begin(CMD_BUSSERVO_ATTACH, DEVICE_BUSSERVO);
        fa.addInt(id); fa.addInt(_servoId[id]); fa.addInt(_series[id]);
        FrameBuilder fm;
        fm.begin(CMD_BUSSERVO_SET_MODE, DEVICE_BUSSERVO);
        fm.addInt(id); fm.addInt(_mode[id]);
        FrameBuilder ft;
        ft.begin(CMD_BUSSERVO_TORQUE, DEVICE_BUSSERVO);
        ft.addInt(id); ft.addInt(_torque[id] ? 1 : 0);

        if (unicast) {
            Pardalote.sendFrame(clientNum, fa);
            Pardalote.sendFrame(clientNum, fm);
            Pardalote.sendFrame(clientNum, ft);
        } else {
            Pardalote.broadcastFrame(fa);
            Pardalote.broadcastFrame(fm);
            Pardalote.broadcastFrame(ft);
        }
    }

    // -------------------------------------------------------------------
    // Main dispatch — TARGET == DEVICE_BUSSERVO.
    // -------------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {

        // ---- global (busless) commands handled before id validation ----
        if (cmd == CMD_BUSSERVO_BUS_CONFIG) {
            if (nparams >= 1) _serialIndex = (int)paramInt(params, 0);
            if (nparams >= 2) _baud        = (uint32_t)paramInt(params, 1);
            if (nparams >= 3) _rxPin       = (int)paramInt(params, 2);
            if (nparams >= 4) _txPin       = (int)paramInt(params, 3);
            _busConfigured = false;   // force re-begin with the new settings
            ensureBus();
            return;
        }
        if (cmd == CMD_BUSSERVO_SCAN) {
            ensureBus();
            int first = (nparams >= 1) ? (int)paramInt(params, 0) : 1;
            int last  = (nparams >= 2) ? (int)paramInt(params, 1) : 20;
            scan(first, last);
            return;
        }
        if (cmd == CMD_BUSSERVO_SYNC_WRITE) {
            ensureBus();
            if (nparams < 1 || payloadLen < 6) return;
            int series = (int)paramInt(params, 0);
            const int REC = 6;                    // servoId(1) pos(2) speed(2) acc(1)
            int count = payloadLen / REC;
            static uint8_t  ids[MAX_BUS_SERVOS];
            static int16_t  positions[MAX_BUS_SERVOS];
            static uint16_t speeds[MAX_BUS_SERVOS];
            static uint8_t  accs[MAX_BUS_SERVOS];
            int n = 0;
            for (int i = 0; i < count && n < MAX_BUS_SERVOS; i++) {
                uint8_t* r   = payload + i * REC;
                ids[n]       = r[0];
                positions[n] = (int16_t)(((uint16_t)r[1] << 8) | r[2]);
                speeds[n]    = (uint16_t)(((uint16_t)r[3] << 8) | r[4]);
                accs[n]      = r[5];
                // Clamp to the series' physical range first (SyncWrite skips
                // writePos, so it needs its own guard against the mod-wrap
                // lurch), then apply the bound instance's soft limits.
                int16_t hi = (series == BUSSERVO_SERIES_SC) ? 1023 : 4095;
                positions[n] = constrain(positions[n], (int16_t)0, hi);
                int lid = logicalForServoId(ids[n]);
                if (lid >= 0 && _limitSet[lid]) {
                    positions[n] = constrain(positions[n], _minPos[lid], _maxPos[lid]);
                }
                n++;
            }
            if (n == 0) return;
            if (series == BUSSERVO_SERIES_SC) {
                // SCSCL has no SyncWritePosEx — write individually (still one frame).
                for (int i = 0; i < n; i++) _sc.WritePos(ids[i], positions[i], 0, speeds[i]);
            } else {
                _st.SyncWritePosEx(ids, n, positions, speeds, accs);
            }
            for (int i = 0; i < n; i++) {
                int lid = logicalForServoId(ids[i]);
                cancelBusGesture(lid);   // a direct sync-write supersedes any gesture on that servo
                beginAwaitDone(lid);
            }
            return;
        }

        // Global bus-servo gesture — one or more channel blocks, each a segment
        // schedule the board sequences locally (see defs.h layout).
        if (cmd == CMD_BUSSERVO_GESTURE) {
            ensureBus();
            uint32_t startMs = millis();   // one shared clock → all channels phase-locked
            uint16_t off = 0;
            while (off + 3 <= payloadLen) {
                int     sid   = payload[off];
                uint8_t flags = payload[off + 1];
                uint8_t count = payload[off + 2];
                off += 3;
                if ((uint32_t)off + (uint32_t)count * 7 > payloadLen) break;   // malformed — stop
                if (validId(sid) && _attached[sid] && count > 0) {
                    uint8_t nseg = count > MAX_BUS_SERVO_SEGMENTS ? MAX_BUS_SERVO_SEGMENTS : count;
                    for (uint8_t i = 0; i < nseg; i++) {
                        const uint8_t* r = payload + off + i * 7;
                        _bsegs[sid][i].curve = r[0];
                        _bsegs[sid][i].dur   = (uint16_t)(((uint16_t)r[1] << 8) | r[2]);
                        _bsegs[sid][i].value = (int32_t)(((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) |
                                                         ((uint32_t)r[5] <<  8) |  (uint32_t)r[6]);
                    }
                    _bsegCount[sid] = nseg;
                    _bsegFlags[sid] = flags;
                    int32_t from = readPos(_servoId[sid]);       // live start (relative anchor)
                    if (from < 0) from = 0;
                    _bsegFrom[sid] = from;
                    loadBusSegment(sid, 0, startMs);
                    resetPace(sid, from);
                }
                off += (uint16_t)count * 7;                       // skip the whole declared block
            }
            return;
        }

        if (nparams < 1) return;
        int id = (int)paramInt(params, 0);
        if (!validId(id)) {
            Serial.print(F("BusServo: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            case CMD_BUSSERVO_ATTACH: {
                if (nparams < 2) return;
                ensureBus();
                _servoId[id] = (uint8_t)paramInt(params, 1);
                _series[id]  = (nparams > 2) ? (uint8_t)paramInt(params, 2) : BUSSERVO_SERIES_ST;
                _attached[id] = true;
                _bsegCount[id] = 0;   // no stale gesture from a previous binding on this id

                // Sensible defaults: position mode, torque on.
                setMode(id, BUSSERVO_MODE_POSITION);
                setTorque(id, true);

                int found = isSC(id) ? _sc.Ping(_servoId[id]) : _st.Ping(_servoId[id]);
                Serial.print(F("BusServo ")); Serial.print(id);
                Serial.print(F(" → servo ID ")); Serial.print(_servoId[id]);
                Serial.print(isSC(id) ? F(" (SC) ") : F(" (ST) "));
                Serial.println(found != -1 ? F("[found]") : F("[NO RESPONSE — check wiring/ID/baud]"));

                // Cache + report presence, so JS learns what Serial just printed.
                _found[id] = (found != -1) ? 1 : 0;
                sendPresence(clientNum, id);

                // Read the servo's own firmware angle limits once, cache them,
                // and tell the requesting browser. announce() replays the cache
                // to later clients, so this EEPROM read happens only at attach.
                readFwLimits(id);
                sendFwLimits(clientNum, id);
                break;
            }

            case CMD_BUSSERVO_DETACH:
                if (_attached[id]) {
                    setTorque(id, false);
                    _attached[id]  = false;
                    _awaitDone[id] = false;
                    _bsegCount[id] = 0;
                    _limitSet[id]  = false;
                    _fwLimitMin[id] = -1;   // cache is per-binding — forget on detach
                    _fwLimitMax[id] = -1;
                    _found[id]      = -1;
                    ExtReadPoll* p = extPollFind(_polls, MAX_BUS_SERVOS, id);
                    if (p) p->instance = -1;   // stop any periodic read
                    Serial.print(F("BusServo ")); Serial.print(id); Serial.println(F(" detached"));
                }
                break;

            case CMD_BUSSERVO_WRITE: {
                if (!_attached[id] || nparams < 2) return;
                cancelBusGesture(id);   // direct write supersedes a running gesture
                int pos   = (int)paramInt(params, 1);
                int speed = (nparams > 2) ? (int)paramInt(params, 2) : 2400;
                int acc   = (nparams > 3) ? (int)paramInt(params, 3) : 50;
                if (_limitSet[id]) pos = constrain(pos, _minPos[id], _maxPos[id]);
                writePos(id, pos, speed, acc);
                beginAwaitDone(id);            // poll for arrival → emit CMD_BUSSERVO_DONE
                break;
            }

            case CMD_BUSSERVO_WRITE_SPEED: {
                if (!_attached[id] || nparams < 2) return;
                cancelBusGesture(id);   // wheel-mode spin supersedes a running gesture
                int speed = (int)paramInt(params, 1);
                int acc   = (nparams > 2) ? (int)paramInt(params, 2) : 50;
                writeSpeed(id, speed, acc);
                _awaitDone[id] = false;        // wheel mode never "arrives"
                break;
            }

            case CMD_BUSSERVO_SET_MODE:
                if (!_attached[id] || nparams < 2) return;
                cancelBusGesture(id);
                setMode(id, (uint8_t)paramInt(params, 1));
                break;

            case CMD_BUSSERVO_TORQUE:
                if (!_attached[id] || nparams < 2) return;
                setTorque(id, paramInt(params, 1) != 0);
                break;

            // READ — params [id, interval?, threshold?]. Always
            // answers the requester immediately. interval > 0 registers a
            // board-side per-client periodic read (ONE bus transaction per
            // poll tick, regardless of client count, gated on position);
            // interval < 0 (JS END) removes this client's registration;
            // absent/0 = one-shot.
            case CMD_BUSSERVO_READ: {
                long ms  = (nparams > 1) ? paramInt(params, 1) : 0;
                long thr = (nparams > 2) ? paramInt(params, 2) : 0;

                if (ms < 0) {   // END — unregister this client
                    ExtReadPoll* p = extPollFind(_polls, MAX_BUS_SERVOS, id);
                    if (p) p->removeClient(clientNum);
                    break;
                }

                if (!_attached[id]) return;
                FrameBuilder fb;
                int32_t pos = buildRead(fb, id);
                Pardalote.sendFrame(clientNum, fb);

                if (ms > 0) {
                    ExtReadPoll* p = extPollGet(_polls, MAX_BUS_SERVOS, id);
                    if (!p) break;
                    p->setClient(clientNum, (uint16_t)constrain(ms, 1, 65535),
                                 thr > 0 ? (uint16_t)thr : DEFAULT_THRESHOLD);
                    p->seed(clientNum, pos, millis());
                }
                break;
            }

            // READ_LIMITS — force a fresh EEPROM read of the servo's firmware
            // angle limits, refresh the cache, and answer the requester.
            // (The attach-time read + announce cover the normal path; this is
            // the explicit readFirmwareLimits() refresh.)
            case CMD_BUSSERVO_READ_LIMITS: {
                if (!_attached[id]) return;
                readFwLimits(id);
                sendFwLimits(clientNum, id);
                break;
            }

            case CMD_BUSSERVO_SET_LIMITS: {
                // Software limits — clamped in board RAM on every write path
                // (browser, sketch, SyncWrite). Deliberately does NOT touch
                // the servo's EEPROM limit registers: no EEPROM wear, no
                // reliance on the unverified unLockEprom/writeWord path.
                if (!_attached[id] || nparams < 3) return;
                int16_t lo = (int16_t)paramInt(params, 1);
                int16_t hi = (int16_t)paramInt(params, 2);
                _minPos[id]   = min(lo, hi);
                _maxPos[id]   = max(lo, hi);
                _limitSet[id] = (nparams < 4) || paramInt(params, 3) != 0;
                break;
            }

            case CMD_BUSSERVO_CALIBRATE: {
                if (!_attached[id]) return;
                // Declare the current physical position as centre. Feetech's
                // CalibrationOfs() writes the homing offset so present position
                // reads ~half-scale here (2048 for ST, 512 for SC) — mirrors
                // LeRobot's "centre on resolution/2" calibration.
                uint8_t sid = _servoId[id];
                if (isSC(id)) _sc.CalibrationOfs(sid);
                else          _st.CalibrationOfs(sid);
                Serial.print(F("BusServo ")); Serial.print(id); Serial.println(F(" centred"));
                break;
            }

            case CMD_BUSSERVO_SET_ID: {
                if (!_attached[id] || nparams < 2) return;
                uint8_t oldId = _servoId[id];
                uint8_t newId = (uint8_t)paramInt(params, 1);
                // ID lives in EEPROM: unlock → write → lock. Only safe with a
                // single servo on the bus (otherwise every servo takes the ID).
                if (isSC(id)) {
                    _sc.unLockEprom(oldId); _sc.writeByte(oldId, SCSCL_ID, newId); _sc.LockEprom(newId);
                } else {
                    _st.unLockEprom(oldId); _st.writeByte(oldId, SMS_STS_ID, newId); _st.LockEprom(newId);
                }
                _servoId[id] = newId;
                Serial.print(F("BusServo ")); Serial.print(id);
                Serial.print(F(" servo ID ")); Serial.print(oldId);
                Serial.print(F(" → ")); Serial.println(newId);
                break;
            }

            case CMD_BUSSERVO_PING: {
                if (nparams < 2) return;
                uint8_t sid = (uint8_t)paramInt(params, 1);
                bool sc = _attached[id] ? isSC(id) : false;
                int found = sc ? _sc.Ping(sid) : _st.Ping(sid);
                FrameBuilder fb;
                fb.begin(CMD_BUSSERVO_PING, DEVICE_BUSSERVO);
                fb.addInt(id); fb.addInt(sid); fb.addInt(found != -1 ? 1 : 0);
                Pardalote.broadcastFrame(fb);
                break;
            }

            default:
                Serial.print(F("BusServo: unknown cmd 0x")); Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Poll response — one FeedBack() bus transaction, then read the cached
    // present values (passing -1 reads from the latched feedback packet).
    // buildRead() does the transaction and fills the frame; returns the
    // present position (or -1) for threshold gating.
    // -------------------------------------------------------------------
    static int32_t buildRead(FrameBuilder& fb, int id) {
        uint8_t sid = _servoId[id];
        int pos = -1, speed = 0, load = 0, voltage = 0, temp = 0, current = 0;
        bool sc = isSC(id);
        int rc = sc ? _sc.FeedBack(sid) : _st.FeedBack(sid);
        if (rc != -1) {
            if (sc) {
                pos     = _sc.ReadPos(-1);
                speed   = _sc.ReadSpeed(-1);
                load    = _sc.ReadLoad(-1);
                voltage = _sc.ReadVoltage(-1);
                temp    = _sc.ReadTemper(-1);
            } else {
                pos     = _st.ReadPos(-1);
                speed   = _st.ReadSpeed(-1);
                load    = _st.ReadLoad(-1);
                voltage = _st.ReadVoltage(-1);
                temp    = _st.ReadTemper(-1);
                current = _st.ReadCurrent(-1);
            }
        }
        fb.begin(CMD_BUSSERVO_READ, DEVICE_BUSSERVO);
        fb.addInt(id);
        fb.addInt(pos);
        fb.addInt(speed);
        fb.addInt(load);
        fb.addInt(voltage);
        fb.addInt(temp);
        fb.addInt(current);
        return pos;
    }

    // -------------------------------------------------------------------
    // Bus scan — ping a range of IDs and report which respond. Reports the
    // first up-to-15 found in a single frame [count, id1, ...].
    // -------------------------------------------------------------------
    static void scan(int first, int last) {
        FrameBuilder fb;
        fb.begin(CMD_BUSSERVO_SCAN, DEVICE_BUSSERVO);
        uint8_t found[15];
        int n = 0;
        for (int sid = first; sid <= last && n < 15; sid++) {
            // Try ST first, then SC — either responding means the ID is live.
            if (_st.Ping(sid) != -1 || _sc.Ping(sid) != -1) found[n++] = (uint8_t)sid;
        }
        fb.addInt(n);
        for (int i = 0; i < n; i++) fb.addInt(found[i]);
        Pardalote.broadcastFrame(fb);
        Serial.print(F("BusServo: scan found ")); Serial.print(n); Serial.println(F(" servo(s)"));
    }

    // -------------------------------------------------------------------
    // Loop hook. Three concerns: (1) render running gestures via the streaming
    // interpolator; (2) poll the Moving flag of any servo awaiting arrival after
    // a PLAIN write and broadcast CMD_BUSSERVO_DONE when it settles (or times
    // out); (3) periodic position reads for the browser. Reads are status
    // queries — they run concurrently with the servo's own motion.
    // -------------------------------------------------------------------
    static void loop() {
        uint32_t now = millis();

        // (1) Continuous gesture rendering, paced to the real servo (see serviceGesture):
        // streams setpoints and freezes the schedule whenever the servo lags, so the
        // stream can't outrun the hardware.
        serviceGesture(now);

        // (2) Arrival poller for plain (non-gesture) writes → CMD_BUSSERVO_DONE.
        for (int id = 0; id < MAX_BUS_SERVOS; id++) {
            if (!_attached[id] || !_awaitDone[id]) continue;
            if (now - _awaitStartMs[id]   < MOVE_STARTUP_MS) continue;   // let it start moving
            if (now - _lastMovePollMs[id] < MOVE_POLL_MS)    continue;   // ~30 Hz
            _lastMovePollMs[id] = now;

            int mv = readMoving(_servoId[id]);            // 1=moving, 0=settled, -1=no answer
            if (mv == 0 || mv == 1) _lastRespMs[id] = now;   // a valid answer keeps a long move alive

            bool arrived = (mv == 0);
            bool lost    = (now - _lastRespMs[id]   > MOVE_NO_RESP_MS);   // servo stopped answering
            bool tooLong = (now - _awaitStartMs[id] > MOVE_MAX_MS);       // absolute ceiling
            if (!arrived && !lost && !tooLong) continue;

            _awaitDone[id] = false;

            // A plain awaited write settled (or was lost / timed out) — report
            // the landing position. Gestures don't use this poller; they run and
            // finish in serviceGesture() (which has its own pace-gate reads).
            int pos = readPos(_servoId[id]);
            FrameBuilder fb;
            fb.begin(CMD_BUSSERVO_DONE, DEVICE_BUSSERVO);
            fb.addInt(id);
            fb.addInt(pos);
            Pardalote.broadcastFrame(fb);
        }

        // Board-side periodic reads — ONE FeedBack() transaction per due
        // registration, then per-client gating on position.
        for (int i = 0; i < MAX_BUS_SERVOS; i++) {
            ExtReadPoll& p = _polls[i];
            if (!p.due(now)) continue;
            if (!validId(p.instance) || !_attached[p.instance]) { p.instance = -1; continue; }
            const int id = p.instance;

            // Back-off for a servo that has stopped answering: poll it slowly
            // (BUSSERVO_LOST_RETRY_MS) instead of on every due interval. Each read
            // to a dead servo blocks up to IOTimeOut (5 ms); a bus-wide dropout
            // (e.g. a power brownout under load) would otherwise block loop() ~5 ms
            // per servo EVERY pass, and on the UNO R4 that sustained blocking
            // starves the WiFiS3/native-USB transport and drops the connection.
            // Responding servos (found == 1) are never throttled — full poll rate,
            // no effect on relay latency. The one cost is that recovery is noticed
            // up to BUSSERVO_LOST_RETRY_MS later.
            if (_found[id] == 0 && (int32_t)(now - _lostRetryAt[id]) < 0) continue;

            FrameBuilder fb;
            const int32_t pos = buildRead(fb, id);

            // Live presence off the read result (pos == -1 means no answer):
            // a servo that appears (e.g. driver board powered up after boot) or
            // disappears mid-session updates the cache — so announce() stays
            // accurate — and logs the change to Serial, matching the browser's
            // 'presence' event. (JS tracks the browser side from this same pos.)
            const int8_t nowFound = (pos != -1) ? 1 : 0;
            if (nowFound != _found[id]) {
                _found[id] = nowFound;
                Serial.print(F("BusServo ")); Serial.print(id);
                Serial.print(F(" servo ID ")); Serial.print(_servoId[id]);
                Serial.println(nowFound ? F(" — now responding [found]")
                                        : F(" — stopped responding [LOST]"));
            }
            // Still not answering — schedule the next retry a back-off away.
            if (nowFound == 0) _lostRetryAt[id] = now + BUSSERVO_LOST_RETRY_MS;

            for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
                if (p.gate(c, pos, now)) Pardalote.sendFrame(c, fb);
        }

        // Gesture-active edge (start / finish / superseding write).
        for (int i = 0; i < MAX_BUS_SERVOS; i++)
            if (_attached[i] || _gestureActive[i]) updateGestureState(i);
    }

    // Client disconnect — drop its read registrations.
    static void disconnect(uint8_t clientNum) {
        extPollDropClient(_polls, MAX_BUS_SERVOS, clientNum);
    }

    // -------------------------------------------------------------------
    // Announce — tell a new client the bus config and replay each binding.
    // Present position is live on the servo, so it's read on demand, not
    // replayed here.
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_BUSSERVO);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_BUS_SERVOS);
        Pardalote.sendFrame(clientNum, fb);

        if (_busConfigured) {
            FrameBuilder fc;
            fc.begin(CMD_BUSSERVO_BUS_CONFIG, DEVICE_BUSSERVO);
            fc.addInt(_serialIndex); fc.addInt((int32_t)_baud);
            fc.addInt(_rxPin); fc.addInt(_txPin);
            Pardalote.sendFrame(clientNum, fc);
        }

        for (int i = 0; i < MAX_BUS_SERVOS; i++) {
            if (!_attached[i]) continue;

            // Sketch-created bus servo: send its SHARE frame FIRST so the
            // browser materialises arduino.<name> before the attach/state
            // frames below arrive to sync it.
            if (_sketchOwned[i]) {
                FrameBuilder fsh;
                fsh.begin(CMD_SHARE, DEVICE_BUSSERVO);
                fsh.addInt(i);
                fsh.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fsh);
            }

            // Replay attach (servo ID + series), mode, and torque.
            sendAttachState(i, true, clientNum);

            if (_limitSet[i]) {
                FrameBuilder fl;
                fl.begin(CMD_BUSSERVO_SET_LIMITS, DEVICE_BUSSERVO);
                fl.addInt(i); fl.addInt(_minPos[i]); fl.addInt(_maxPos[i]); fl.addInt(1);
                Pardalote.sendFrame(clientNum, fl);
            }

            // Replay the cached firmware limits (read once at attach) — no
            // EEPROM re-read per connecting client.
            sendFwLimits(clientNum, i);
            // Replay the cached attach-time presence too.
            sendPresence(clientNum, i);

            // Tell a reconnecting browser this servo is mid-gesture (existence).
            if (_gestureActive[i]) sendGestureState(clientNum, i, 1);
        }
    }
};

// -------------------------------------------------------------------
// PardaloteBusServo — sketch-facing bus object.
//
// THE BUS IS THE HARDWARE YOU ATTACH. A serial bus is either a Pardalote
// bus or it isn't — never shared. Every servo on a Pardalote bus is
// Pardalote hardware; a servo you want to keep private lives on a
// SEPARATE UART that you drive with the raw SCServo library, and Pardalote
// never touches it. (This mirrors PWM servos: a private one uses the plain
// Servo lib on its own pin — the difference is only that the boundary is
// per-bus here, because bus servos share one wire, versus per-pin there.)
//
// Discovery vs control:
//   uint8_t ids[16];
//   int n = PardaloteBusServo.scan(ids, 16);   // DISCOVERY: hardware ids on
//                                              // the bus (all Pardalote's)
//   int j = PardaloteBusServo.attach("wrist", ids[0]);  // adopt → LOGICAL id
//   PardaloteBusServo.write(j, 2048);          // CONTROL: by logical id
//   int pos = PardaloteBusServo.read(j);       // feedback, −1 if no answer
// A scan is blocking (a ping timeout per silent ID) — call it in setup()
// or on demand, not in a tight loop.
// -------------------------------------------------------------------
class PardaloteBusServoAccess {
public:
    // attach(name, servoId, series?) — adopt a servo on the Pardalote bus (by
    // its bus/hardware id) into a named Pardalote instance, visible to every
    // browser as arduino.<name>. series is BUSSERVO_SERIES_ST (default, STS/
    // SMS) or BUSSERVO_SERIES_SC. Returns the LOGICAL id — the handle write()/
    // read()/torque() take below, and the same id the browser and groups use —
    // or -1 if no slot is free. Names longer than MAX_SHARE_NAME (15) are
    // truncated. Idempotent per name. The bus is brought up automatically
    // (Serial1 @ 1 Mbps unless changed via configureBus on the browser side).
    int attach(const char* name, int servoId,
               int series = BUSSERVO_SERIES_ST) const {
        return BusServoExt::sketchAttach(name, servoId, series);
    }

    // configureBus(rxPin, txPin, [serialIndex], [baud]) — set the bus UART from a
    // HEADLESS sketch (the board-side twin of the browser's configureBus). Call
    // once in setup() BEFORE attach(). The rx/tx pins are ESP32-only; UNO R4 is
    // fixed to Serial1 (D0/D1) and ignores them. serialIndex 1 = Serial1 (default),
    // 2 = Serial2 (ESP32). baud 0 keeps the default (1 Mbps).
    void configureBus(int rxPin, int txPin, int serialIndex = 1, uint32_t baud = 0) const {
        BusServoExt::configureBus(rxPin, txPin, serialIndex, baud);
    }

    // scan(out, max, first?, last?) — DISCOVERY: ping the bus, report the
    // hardware ids that respond. On a Pardalote bus every responder is
    // Pardalote hardware; attach() the ones you want to drive.
    int scan(uint8_t* out, int max, int first = 1, int last = 30) const {
        return BusServoExt::scanBus(out, max, first, last);
    }

    // Reads — by LOGICAL id (what attach() returned). Live bus transactions.
    int read(int id)     const { return BusServoExt::positionById(id); }         // position (counts)
    int position(int id) const { return BusServoExt::positionById(id); }         // alias
    PardaloteBusServoReading feedback(int id) const { return BusServoExt::feedbackById(id); }

    // Arrival check — one bus read of the servo's own Moving flag. Opt-in
    // (the servo can't notify you; you ask). Both false if it didn't answer.
    bool isMoving(int id) const { return BusServoExt::movingById(id) == 1; }
    bool arrived(int id)  const { return BusServoExt::movingById(id) == 0; }

    // Writes — by LOGICAL id, routed through the same handler the browser uses
    // (soft limits enforced; the done-poller is armed; the target is auto-
    // echoed to the browser so its record matches, like a browser write).
    void write(int id, int position, int speed = 2400, int acc = 50) const {
        Pardalote.command(DEVICE_BUSSERVO, CMD_BUSSERVO_WRITE, id, position, speed, acc);
        BusServoExt::echoTarget(id, position);
    }
    void torque(int id, bool on) const {
        Pardalote.command(DEVICE_BUSSERVO, CMD_BUSSERVO_TORQUE, id, on ? 1 : 0);
        BusServoExt::echoTorque(id, on);
    }

    // gesture(id, segs, count, flags?) — play a SEGMENT SCHEDULE on the board,
    // the sketch-side twin of JS busServo.gesture(). segs is a PardaloteSeg[]
    // (counts; absolute by default, flags = 0 for relative). A board-side
    // streaming interpolator renders each segment's easing curve on its own
    // clock (including CURVE_BACK overshoot). Coordinate via Pardalote.gesture().
    void gesture(int id, const PardaloteSeg* segs, uint8_t count,
                 uint8_t flags = GESTURE_FLAG_ABSOLUTE,
                 const PardaloteGestureMod& mod = PardaloteGestureMod()) const {
        BusServoExt::startGesture(id, segs, count, flags, millis(), 0, mod);
    }
    // onGestureDone(id, cb) — board-side whenDone(): cb(id) on the last segment.
    void onGestureDone(int id, PardaloteGestureDone cb) const { BusServoExt::setOnGestureDone(id, cb); }
};
inline PardaloteBusServoAccess PardaloteBusServo;

// Self-register — runs before setup().
INSTALL_EXTENSION(DEVICE_BUSSERVO, BusServoExt::handle, BusServoExt::announce,
                  BusServoExt::disconnect, BusServoExt::loop)

// Register the board-side gesture starter so Pardalote.gesture() can drive bus
// servos in a coordinated group by DEVICE_BUSSERVO id (see internal/gesture.h).
INSTALL_GESTURE(DEVICE_BUSSERVO, BusServoExt::startGesture)

// Register the immediate writer so Pardalote.write() can drive bus servos.
INSTALL_WRITER(DEVICE_BUSSERVO, BusServoExt::writeNow)

#endif
