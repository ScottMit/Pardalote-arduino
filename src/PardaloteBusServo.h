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

    // Gesture segment schedule (CMD_BUSSERVO_GESTURE). A bus servo runs its
    // OWN motion, so there is no per-tick curve to render: each segment is one
    // position write at a distance/duration-matched speed, and the sequencer
    // advances to the next segment when the servo's Moving flag SETTLES (the
    // same feedback the DONE poller uses) — not on a timer, so a saturated
    // segment just takes longer and the next still fires on true arrival. The
    // per-segment `curve` byte is accepted but NOT rendered inside a segment
    // (bus-servo expression comes from segment decomposition + lane overlap);
    // `from` is captured live at gesture start, then chained from each target.
    static const uint8_t MAX_BUS_SERVO_SEGMENTS = 12;
    static const int     BUS_SEG_MAX_SPEED      = 4095;   // hardware/lib ceiling (authored dur wins)
    struct BSeg { uint8_t curve; uint16_t dur; int32_t value; };
    inline static BSeg     _bsegs[MAX_BUS_SERVOS][MAX_BUS_SERVO_SEGMENTS] = {};
    inline static uint8_t  _bsegCount[MAX_BUS_SERVOS]  = {};   // 0 = no gesture running
    inline static uint8_t  _bsegIndex[MAX_BUS_SERVOS]  = {};
    inline static uint8_t  _bsegFlags[MAX_BUS_SERVOS]  = {};   // GESTURE_FLAG_*
    inline static int32_t  _bsegFrom[MAX_BUS_SERVOS]   = {};   // start of the current segment
    inline static int32_t  _bsegTarget[MAX_BUS_SERVOS] = {};   // end of the current segment (chained)
    inline static uint16_t _bsegDurMs[MAX_BUS_SERVOS]  = {};   // authored duration — segment's time floor

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

    // Issue segment `idx` as one position write at a distance/duration-matched
    // speed, then arm the settle poller. `from` is _bsegFrom[id] (captured at
    // gesture start, chained from each target); target = from+delta (relative)
    // or the value itself (absolute), clamped to soft limits / series range.
    static void loadBusSegment(int id, uint8_t idx) {
        const BSeg& seg = _bsegs[id][idx];
        int32_t from    = _bsegFrom[id];
        int32_t target  = (_bsegFlags[id] & GESTURE_FLAG_ABSOLUTE) ? seg.value : from + seg.value;
        if (_limitSet[id]) target = constrain(target, (int32_t)_minPos[id], (int32_t)_maxPos[id]);
        else               target = constrain(target, (int32_t)0, (int32_t)(isSC(id) ? 1023 : 4095));

        uint16_t dur    = seg.dur ? seg.dur : 1;
        float    durSec = dur / 1000.0f;
        long     dist   = labs((long)target - (long)from);
        int      speed  = (int)lroundf((float)dist / durSec);
        if (speed < 1)               speed = 1;                 // Feetech: 0 = full speed
        if (speed > BUS_SEG_MAX_SPEED) speed = BUS_SEG_MAX_SPEED;

        _bsegIndex[id]  = idx;
        _bsegTarget[id] = target;
        _bsegDurMs[id]  = dur;
        writePos(id, target, speed, 50);
        beginAwaitDone(id);   // loop() polls the Moving flag → advance or DONE
    }

    // Drop any running gesture (a direct write / mode change / detach
    // supersedes it). No-op when none is active. Does NOT emit DONE — the
    // superseding command owns completion.
    static void cancelBusGesture(int id) { if (validId(id)) _bsegCount[id] = 0; }

public:
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
                    loadBusSegment(sid, 0);
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
    // Loop hook — polls the Moving flag of any servo awaiting arrival (after
    // a position write) at ~30 Hz, and broadcasts CMD_BUSSERVO_DONE when it
    // settles (or after a timeout). Reading doesn't interrupt the servo's
    // motion — it's a status query, run concurrently with its control loop.
    // -------------------------------------------------------------------
    static void loop() {
        uint32_t now = millis();
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

            // Gesture segment that settled EARLY: hold the lane until its
            // authored duration elapses. Keeps group lanes phase-locked and
            // makes a zero-distance pad/hold actually wait (it would otherwise
            // settle instantly). Error paths (lost/tooLong) skip the floor.
            if (_bsegCount[id] > 0 && arrived && !lost && !tooLong &&
                (now - _awaitStartMs[id] < _bsegDurMs[id])) continue;

            _awaitDone[id] = false;

            // Gesture sequencing — a segment just settled. On real arrival with
            // more segments, chain to the next (no DONE). On the last segment,
            // or if the move was lost/timed out, finish and emit DONE below.
            if (_bsegCount[id] > 0) {
                if (arrived && _bsegIndex[id] + 1 < _bsegCount[id]) {
                    _bsegFrom[id] = _bsegTarget[id];        // chain from the commanded target
                    loadBusSegment(id, _bsegIndex[id] + 1); // re-arms _awaitDone
                    continue;
                }
                _bsegCount[id] = 0;                          // gesture complete (or aborted)
            }

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
};
inline PardaloteBusServoAccess PardaloteBusServo;

// Self-register — runs before setup().
INSTALL_EXTENSION(DEVICE_BUSSERVO, BusServoExt::handle, BusServoExt::announce,
                  BusServoExt::disconnect, BusServoExt::loop)

#endif
