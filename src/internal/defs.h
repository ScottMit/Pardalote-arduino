// ==============================================================
// defs.h
// Pardalote Protocol Constants
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
// ==============================================================

#ifndef DEFS_H
#define DEFS_H

// -------------------------------------------------------------------
// Protocol Version
// -------------------------------------------------------------------
// Wire-compatibility contract between any JS build and any firmware
// build. MAJOR changes when old clients can no longer talk (forces a
// MAJOR product release); MINOR marks backward-compatible additions.
// Independent of the product version below.
#define PROTOCOL_VERSION_MAJOR 1
#define PROTOCOL_VERSION_MINOR 0

// Product version — the release humans see. Canonical copies live in
// library.properties (Arduino) and package.json (JS); this string lets
// a sketch print what it's running.
#define PARDALOTE_VERSION "1.1.0"

// -------------------------------------------------------------------
// ADC Resolution
// Default bits used by analogRead() on each platform.
// Override in your sketch before including Pardalote:
//   #define ADC_RESOLUTION_BITS 14   // e.g. after analogReadResolution(14)
// -------------------------------------------------------------------
#ifndef ADC_RESOLUTION_BITS
  #if defined(PLATFORM_UNO_R4) || defined(PLATFORM_UNO_R4_MINIMA)
    #define ADC_RESOLUTION_BITS 10   // Arduino compat default (hardware is 14-bit)
  #elif defined(PLATFORM_ESP32)
    #define ADC_RESOLUTION_BITS 12   // ESP32 default
  #else
    #define ADC_RESOLUTION_BITS 10   // safe fallback
  #endif
#endif

// -------------------------------------------------------------------
// Core Commands (CMD byte, 0x00–0x0B). The range 0x00–0x0F is RESERVED
// for the core — extension commands start at 0x10 (see the rule at the
// Extension Device IDs section).
// -------------------------------------------------------------------
#define CMD_HELLO         0x00  // Arduino → JS on connect: [major, minor, adcBits, bootId] + board string
                                // bootId (param 3): random 31-bit token generated once
                                // per boot — JS compares it across reconnects to tell "board
                                // rebooted" (drop board-originated state) from "network blip"
                                // (keep everything). Older firmware omits it; JS treats absent as 0.
#define CMD_ANNOUNCE      0x01  // Arduino → JS per extension: [version, maxInstances]
#define CMD_PIN_MODE      0x02  // JS → Arduino: set pin mode [mode]; Arduino → JS: announce pin
                                // config [mode] or [mode, interval, threshold] when the board
                                // itself polls the pin (share() with an interval)
#define CMD_DIGITAL_WRITE 0x03  // JS → Arduino: write value; Arduino → JS: announce output state
#define CMD_DIGITAL_READ  0x04  // JS → Arduino: [interval?, threshold?] read/register (threshold;
                                // 0 = board default); Arduino → JS: [value]
#define CMD_ANALOG_WRITE  0x05
#define CMD_ANALOG_READ   0x06  // params as CMD_DIGITAL_READ; analog default threshold = ADC
                                // noise floor (analogMax >> 8, min 1)
#define CMD_END           0x07  // Stop a periodic read (per requesting client)
#define CMD_PING          0x08  // JS → Arduino: heartbeat request
#define CMD_PONG          0x09  // Arduino → JS: heartbeat response
#define CMD_SYNC_COMPLETE 0x0A  // Arduino → JS: all announce frames sent; JS fires 'ready'
#define CMD_MESSAGE       0x0B  // Both ways: user-defined key/value message (see Message Channel below)
#define CMD_AUTH          0x0C  // Connection key — set on the board with requireKey(), works over
                                // BOTH transports. Over WiFi it latches against the wrong board on
                                // a shared network; over USB the cable already picks the board, so
                                // the key is a board-IDENTITY check ("is this the board my sketch
                                // expects?") — it catches a student grabbing the wrong physical board.
                                // JS → Arduino: no params + payload: UTF-8 key. Sent first after the
                                //   socket opens (WiFi) or in the serial probe loop (USB).
                                // Arduino → JS: [reason] then the board drops the client —
                                //   1 = board requires a key and none arrived in time,
                                //   2 = wrong key.
                                // Success sends no AUTH reply: the normal HELLO is the acceptance.
                                // This is an accident-prevention latch, not security: the key
                                // crosses the wire in cleartext (ws:// no TLS, or plain over USB).
#define CMD_SERIAL_BUSY   0x0D  // Arduino → JS (serial only): a WiFi-active board that is LISTENING
                                // for a USB takeover received a plain probe (no takeover flag). It
                                // means "I'm on WiFi — reconnect with a picker gesture to switch me
                                // to USB." The board stays on WiFi. JS surfaces it as 'usbBusy' and
                                // stops probing (no reconnect churn). See the transport listen/switch.
#define CMD_REBOOT        0x0E  // Arduino → JS (serial): sent once at the very top of begin(), before
                                // the boot-watch window. A machine-readable "I just (re)booted" marker
                                // (a real framed message, NOT the human "=== Pardalote ===" banner —
                                // that stays cosmetic). A browser still holding the port from a prior
                                // session sees it and immediately resumes takeover-probing, so its probe
                                // lands in the boot-watch window and the board switches straight back to
                                // serial — the fast recovery from a reset while USB-connected.
// Next free core cmd: 0x0F.

// -------------------------------------------------------------------
// WebSocket client capacity — shared by the core (per-client pin read
// gating) and extensions (per-client sensor read gating). The serial
// transport has exactly one client, permanently client 0 — the same
// per-client machinery serves it as a degenerate case.
// -------------------------------------------------------------------
#define PARDALOTE_MAX_CLIENTS 4

// -------------------------------------------------------------------
// Transport selection — tokens for begin(int). Values are API tokens,
// not wire constants.
//   begin()                 — WiFi + WebSocket AND listen on USB for a
//                             deliberate (picker-gesture) takeover; the
//                             board drops WiFi and switches to USB when one
//                             arrives (one-way, reboot to return to WiFi).
//   begin(PARDALOTE_WIFI)   — WiFi only; does NOT listen on USB (opt-out
//                             so nobody grabs the board over the cable).
//   begin(PARDALOTE_SERIAL) — USB serial only; WiFi never started. Speaks
//                             the same binary protocol over Serial, COBS-
//                             framed with a CRC8 (see serial_transport.h).
// On boards with no radio (UNO R4 Minima) every form starts serial, the
// only transport the hardware can have.
// -------------------------------------------------------------------
#define PARDALOTE_SERIAL 1
#define PARDALOTE_WIFI   2

// Longest connection key begin("key") accepts (excl. NUL). Longer keys
// are truncated with a Serial warning.
#define PARDALOTE_KEY_MAX 32

// -------------------------------------------------------------------
// Pin tracking
// Upper bound on pin numbers the core will track for announce/re-register.
// Covers all pins on UNO R4 WiFi (~20) and ESP32 variants (~40).
// -------------------------------------------------------------------
#define MAX_PIN_NUMBER 64

// -------------------------------------------------------------------
// Pin Modes (param to CMD_PIN_MODE)
// -------------------------------------------------------------------
#define MODE_INPUT          0
#define MODE_OUTPUT         1
#define MODE_INPUT_PULLUP   2
#define MODE_INPUT_PULLDOWN 3   // ESP32 only
#define ANALOG_INPUT_MODE   8

// The four MODE_* values above are internal wire codes: a sketch passes
// Arduino's own INPUT / OUTPUT / INPUT_PULLUP / INPUT_PULLDOWN to share() and
// the library maps them to these. ANALOG_INPUT_MODE is the exception — it's the
// one pin mode a sketch names directly, because Arduino has no equivalent
// ("analog input, auto-polled to the browser"). It is deliberately suffixed
// _MODE rather than a bare ANALOG_INPUT: pin-mode names are an unscoped, crowded
// #define space shared by every core (ESP32 defines ANALOG; some Pycom builds
// define ANALOG_INPUT as 0x0), and the _MODE suffix keeps us clear of them
// without needing a fragile #ifndef guard. The JS side uses the same name
// (const ANALOG_INPUT_MODE = 8); this value is the wire constant and must match
// on both sides.

// -------------------------------------------------------------------
// RULE: extension CMD values MUST be >= 0x10. 0x00–0x0F is reserved for
// core commands — CMD_MESSAGE (0x0B) is routed by its cmd byte ALONE
// (message flags in the target high byte can exceed RESERVED_START), so
// an extension cmd that collides with a core cmd routed this way is
// silently misdispatched on both sides of the wire.
//
// Extension Device IDs (TARGET >= 200)
// -------------------------------------------------------------------
#define RESERVED_START        200
#define DEVICE_NEO_PIXEL      200
#define DEVICE_SERVO          201
#define DEVICE_ULTRASONIC     202
// DEVICE_IMU 203, DEVICE_CAMERA 204 and DEVICE_STEPPER 205 are defined
// alongside their command blocks lower in this file. Next free ID: 206.

// -------------------------------------------------------------------
// NeoPixel Commands (0x5C–0x61)
// -------------------------------------------------------------------
// Numbered after the encoder block. NeoPixel historically sat at
// 0x0A–0x0F, INSIDE the core range — and CMD_MESSAGE (0x0B) is routed by
// cmd alone, before target dispatch, so it silently swallowed every
// CMD_NEO_SET_PIXEL frame. Hence the >= 0x10 rule above.
#define CMD_NEO_INIT       0x5C  // params: [instanceId, pin, numPixels, type]
#define CMD_NEO_SET_PIXEL  0x5D  // params: [instanceId, index, r, g, b (, w)]
#define CMD_NEO_FILL       0x5E  // params: [instanceId, color, first, count]
#define CMD_NEO_CLEAR      0x5F  // params: [instanceId]
#define CMD_NEO_BRIGHTNESS 0x60  // params: [instanceId, value]
#define CMD_NEO_SHOW       0x61  // params: [instanceId]

// -------------------------------------------------------------------
// Servo Commands (0x14–0x1D)
// -------------------------------------------------------------------
#define CMD_SERVO_ATTACH             0x14  // params: [instanceId, pin, minPulse, maxPulse]
#define CMD_SERVO_DETACH             0x15  // params: [instanceId]
#define CMD_SERVO_WRITE              0x16  // params: [instanceId, angle]
#define CMD_SERVO_WRITE_MICROSECONDS 0x17  // params: [instanceId, microseconds]
#define CMD_SERVO_READ               0x18  // params: [instanceId]  — response: [instanceId, angle]
#define CMD_SERVO_ATTACHED           0x19  // params: [instanceId]  — response: [instanceId, 0|1]
#define CMD_SERVO_WRITE_TIMED        0x1A  // params: [instanceId, angle, durationMs] — board interpolates
#define CMD_SERVO_SYNC_TIMED         0x1B  // JS→Ar (global): [durationMs] + payload:
                                           //   N × { logicalId u8, targetAngle u8 } (2 bytes each)
                                           // All listed servos interpolate over the SAME duration → arrive together
#define CMD_SERVO_STOP               0x1C  // params: [instanceId] — cancel a timed move, hold current angle
#define CMD_SERVO_DONE          0x1D  // Ar→JS (unsolicited): [instanceId, angle] — timed move reached target
// Numbered after the stepper switch block (0x52–0x53) — the 0x14–0x1D servo
// block was full; dispatch is by (deviceId, cmd) so the gap is cosmetic only.
#define CMD_SERVO_SET_LIMITS    0x54  // JS→Ar: [instanceId, minAngle, maxAngle, enabled] — soft angle
                                      //   limits, clamped ON THE BOARD (browser and sketch writes alike).
                                      // Ar→JS (announce): same shape, replays limit state.

// -------------------------------------------------------------------
// Expressive-motion gesture band (0x58–0x5A) — one code per actuator
// type. A gesture is a per-channel SEGMENT SCHEDULE the board plays
// LOCALLY (no per-segment round-trips): push it once, the board advances
// segment→segment on its own millis() clock, so multi-channel gestures
// stay phase-locked and arrive-together survives. Completion reuses the
// type's existing DONE frame (CMD_SERVO_DONE etc.) → whenDone().
//
// Payload = one or more channel blocks, back to back:
//   channel: [ logicalId u8, flags u8, segCount u8, segment × segCount ]
//   segment: [ curve u8, dur u16 (ms, big-endian), value i32 (big-endian) ]
// flags: bit0 = reference frame (0 relative-delta / 1 absolute-target),
//        bit1 = loop (reserved — playback not yet implemented).
// `value` is a per-segment DELTA (relative) or TARGET (absolute), in the
// actuator's native unit. `from` is captured on-board at each segment
// start (dynamic capture), so relative gestures need no absolute truth.
// -------------------------------------------------------------------
#define CMD_SERVO_GESTURE       0x58  // JS→Ar (global): payload = servo channel blocks (see above)
#define CMD_STEPPER_GESTURE     0x59  // JS→Ar (global): stepper channel blocks — MODE_EASED segment player
#define CMD_BUSSERVO_GESTURE    0x5A  // JS→Ar (global): bus-servo channel blocks — feedback-sequenced segments

// Gesture channel flags (defs.h ↔ pardalote.js must agree).
#define GESTURE_FLAG_ABSOLUTE   0x01  // value is an absolute target, not a relative delta
#define GESTURE_FLAG_LOOP       0x02  // repeat the schedule (reserved)

// Shared easing curve ids — the ONE numbered table used by every surface
// (this firmware, the standalone follower's PROGMEM gestures, and
// pardalote.js). Keep the formulas identical across surfaces; see
// pardaloteEase() below and curveShape() in pardalote.js. Curated set — 0x05+
// (elastic, bounce, …) reserved for later.
#define CURVE_LINEAR       0   // t
#define CURVE_EASE_IN      1   // t^2                 — accelerate from rest
#define CURVE_EASE_OUT     2   // 1-(1-t)^2           — decelerate into rest
#define CURVE_EASE_IN_OUT  3   // smoothstep t^2(3-2t)
#define CURVE_BACK         4   // overshoot past the target, then settle

// The one on-device easing implementation — used by every extension's
// segment player (servo, stepper, …) and MUST match curveShape() in
// pardalote.js. `t` in [0,1]; CURVE_BACK returns slightly >1 mid-flight
// (the overshoot), which position players re-clamp and velocity players
// render as a brief reverse near the end.
static inline float pardaloteEase(uint8_t curve, float t) {
    switch (curve) {
        case CURVE_EASE_IN:     return t * t;
        case CURVE_EASE_OUT:    return t * (2.0f - t);              // 1-(1-t)^2
        case CURVE_EASE_IN_OUT: return t * t * (3.0f - 2.0f * t);   // smoothstep
        case CURVE_BACK: {                                          // easeOutBack, s = 1.70158
            float k = t - 1.0f;
            return 1.0f + 2.70158f * k * k * k + 1.70158f * k * k;
        }
        default:                return t;                           // CURVE_LINEAR
    }
}

// -------------------------------------------------------------------
// Ultrasonic Commands (0x1E–0x27)
// -------------------------------------------------------------------
#define CMD_ULTRASONIC_ATTACH      0x1E  // params: [instanceId, trigPin, echoPin]
#define CMD_ULTRASONIC_DETACH      0x1F  // params: [instanceId]
#define CMD_ULTRASONIC_READ        0x20  // params: [instanceId, unit, interval]
#define CMD_ULTRASONIC_SET_TIMEOUT 0x21  // params: [instanceId, timeoutMs]

// -------------------------------------------------------------------
// Ultrasonic Units
// -------------------------------------------------------------------
#define UNIT_CM   0
#define UNIT_INCH 1

// -------------------------------------------------------------------
// IMU (6-DOF) Device ID and Commands (0x28–0x2F)
// Designed for MPU-6050; adaptable to other I2C IMUs by swapping the
// I2C register reads in PardaloteIMU.h (see comments there).
// -------------------------------------------------------------------
#define DEVICE_IMU  203

// -------------------------------------------------------------------
// Camera Device ID and Commands (0x30–0x32)
// ESP32-S3 only — MJPEG stream and JPEG snapshot served over HTTP.
// -------------------------------------------------------------------
#define DEVICE_CAMERA          204

#define CMD_CAMERA_INIT        0x30  // JS→Ar: [id, port] — start camera + HTTP server
                                     // Ar→JS: [id, port] — confirms stream is live
#define CMD_CAMERA_SET_RES     0x31  // JS→Ar: [id, framesize]  (framesize_t enum value)
#define CMD_CAMERA_SET_QUALITY 0x32  // JS→Ar: [id, quality]    0 = best, 63 = worst

#define CMD_IMU_ATTACH          0x28  // JS→Ar: [id, addr, sda?, scl?] + model name string in payload
                                      // Ar→JS (announce): [id, addr] + model name string in payload
#define CMD_IMU_DETACH          0x29  // JS→Ar: [id]
#define CMD_IMU_READ            0x2A  // JS→Ar: [id]
                                      // Ar→JS: [id, ax, ay, az, gx, gy, gz, temp]  (floats, g and °/s)
#define CMD_IMU_SET_ACCEL_RANGE 0x2B  // JS→Ar: [id, range]  0=±2g, 1=±4g, 2=±8g, 3=±16g
#define CMD_IMU_SET_GYRO_RANGE  0x2C  // JS→Ar: [id, range]  0=±250, 1=±500, 2=±1000, 3=±2000 °/s
#define CMD_IMU_CALIBRATE       0x2D  // JS→Ar: [id, samples?]
                                      // Ar→JS: [id, ax, ay, az, gx, gy, gz]  offset floats
// Model name strings (e.g. "6050", "LSM6DSOX") are sent in the payload of
// CMD_IMU_ATTACH and matched against SENSORS[i].name in PardaloteIMU.h.
// See imu.js IMU_MODELS for the JS-side list — row order in either table
// is irrelevant; the two are coupled by name only.

// -------------------------------------------------------------------
// Stepper Device ID and Commands (0x33–0x40)
// Motion executes on-board via AccelStepper::run() / runSpeed() in the
// extension loop hook. JS sends targets and motion profiles; the board
// generates the step pulses. Mirrors the AccelStepper API (non-blocking)
// rather than the built-in Stepper library (whose step() blocks and
// would stall Pardalote.run()).
//
// Requires the AccelStepper library (by Mike McCauley), installable via
// Arduino IDE → Manage Libraries.
// -------------------------------------------------------------------
#define DEVICE_STEPPER  205

#define CMD_STEPPER_ATTACH        0x33  // JS→Ar: [id, interface, pin1, pin2, pin3?, pin4?, enPin?, invertMask?]
                                        // Ar→JS (announce): same shape, replays attach state
#define CMD_STEPPER_DETACH        0x34  // JS→Ar: [id]
#define CMD_STEPPER_MOVE_TO       0x35  // JS→Ar: [id, absPosition]   — position mode, accel profile
#define CMD_STEPPER_MOVE          0x36  // JS→Ar: [id, relSteps]      — position mode, accel profile
#define CMD_STEPPER_SET_MAX_SPEED 0x37  // JS→Ar: [id, speed]         — steps/sec ceiling (int or float)
#define CMD_STEPPER_SET_ACCEL     0x38  // JS→Ar: [id, accel]         — steps/sec^2 (int or float)
#define CMD_STEPPER_RUN_SPEED     0x39  // JS→Ar: [id, speed]         — velocity mode, continuous rotation
#define CMD_STEPPER_STOP          0x3A  // JS→Ar: [id]                — decelerate to a stop
#define CMD_STEPPER_SET_POSITION  0x3B  // JS→Ar: [id, position]      — setCurrentPosition (zero / manual home)
#define CMD_STEPPER_ENABLE        0x3C  // JS→Ar: [id, enable]        — EN pin: 1=hold torque, 0=release
#define CMD_STEPPER_SET_LIMITS    0x3D  // JS→Ar: [id, min, max, enabled] — soft position limits (safety)
#define CMD_STEPPER_READ          0x3E  // JS→Ar: [id]
                                        // Ar→JS: [id, position, distanceToGo, speed(f), isRunning]
#define CMD_STEPPER_DONE          0x3F  // Ar→JS (unsolicited): [id, position] — position-mode target reached
#define CMD_STEPPER_HOME          0x40  // JS→Ar: [id, speed?, timeoutMs?] — run the homing routine: seek
                                        //   the limit switch (MIN if configured, else MAX) at `speed`
                                        //   (0 = default, maxSpeed/4), on trip set the counter to the
                                        //   switch's declared coordinate (SET_SWITCH_POS, default 0), back
                                        //   off until released, then travel to home — the origin, 0. DONE
                                        //   fires on arrival. With no switch: plain accel move to 0.
                                        //   timeoutMs caps the SEEK+BACKOFF legs (0 = default 30 s) —
                                        //   an unplugged/stuck switch can't spin the motor forever.
                                        // Ar→JS (unsolicited): [id, position] — homing GAVE UP (timeout);
                                        //   motor hard-stopped where it was. DONE follows (motion settled).
// Timed / coordinated moves. Numbered after the bus-servo block (0x41–0x4E)
// because the 0x33–0x40 stepper block was full; dispatch is by (deviceId, cmd)
// so the numeric gap is cosmetic only.
#define CMD_STEPPER_MOVE_TIMED    0x4F  // JS→Ar: [id, target, durationMs] — arrive in ~duration (constant speed)
#define CMD_STEPPER_SYNC_MOVE     0x50  // JS→Ar (global): [durationMs] + payload:
                                        //   N × { logicalId u8, target i32 } (5 bytes each)
                                        // Board computes matched speeds from its own positions → arrive together
// Limit switches (0x51 is CMD_BUSSERVO_DONE — numbered after it, same
// cosmetic-gap note as 0x4F–0x50).
#define CMD_STEPPER_SET_SWITCH    0x52  // JS→Ar: [id, which, pin, trigger] — configure one limit switch.
                                        //   which: LIMIT_MIN/LIMIT_MAX; pin -1 = clear; trigger 0=LOW,
                                        //   1=HIGH. The coordinate the switch sits at is set separately by
                                        //   SET_SWITCH_POS (default 0).
                                        // Ar→JS (announce): same shape, replays each configured switch.
#define CMD_STEPPER_LIMIT         0x53  // Ar→JS (unsolicited): [id, which, position] — switch tripped,
                                        //   motion was hard-stopped on the board. CMD_STEPPER_DONE follows.
#define CMD_STEPPER_SET_SWITCH_POS 0x54 // JS→Ar: [id, which, coord] — declare the coordinate a limit switch
                                        //   physically sits at, INDEPENDENT of the soft limits. Homing
                                        //   adopts this coordinate when the switch trips (default 0 = the
                                        //   switch is the origin). Lets home sit anywhere relative to the
                                        //   switch (e.g. switch at -500, home the origin at 0).
                                        // Ar→JS (echo + announce): same shape — silent JS sync.
#define CMD_STEPPER_SET_HOME      0x55  // JS→Ar: [id, value?] — re-zero the coordinate frame: the current
                                        //   physical position BECOMES `value` (default 0 = the origin/home).
                                        //   Soft limits and switch positions shift by the same offset so
                                        //   they keep pointing at the same physical spots. home() returns
                                        //   to the origin (0).
                                        // Ar→JS (echo): the board broadcasts the shifted SET_POSITION,
                                        //   SET_LIMITS and SET_SWITCH_POS frames — silent JS sync.
// Uses the next-free device-scoped code (0x57; 0x56 is CMD_SHARE) — sits
// outside the 0x3x stepper block by allocation order, not by category.
#define CMD_STEPPER_HARD_STOP     0x57  // JS→Ar: [id] — instant halt, no decel ramp (cf. CMD_STEPPER_STOP,
                                        //   which decelerates). Keeps the current position; DONE follows.

// -------------------------------------------------------------------
// Sketch-created hardware objects (Ar→JS)
//
// The sketch-facing API is the same verb the browser uses — e.g.
// PardaloteServo.attach("pan", 9) — creation and browser visibility are
// one act (unlike raw pins, where the hardware exists outside Pardalote
// and share() only informs). On the wire, the board tells browsers about
// the new object with CMD_SHARE: a device-scoped command like any other,
// but the VALUE is reserved across ALL extension device IDs — the JS
// core intercepts it generically (before per-instance routing) and
// materialises a browser object of the right class. Shape, for every
// device type:
//
//   Ar→JS: [logicalId] + payload: UTF-8 name → browser creates the
//   extension instance and binds it as arduino.<name>. The normal
//   announce/state frames (ATTACH, WRITE, …) follow and sync it.
//
// Board-created objects allocate logical ids from the TOP of each
// extension's range downward; browser-created ids grow from 0 upward,
// so the two sides can't collide until the range is full.
// Currently implemented by: DEVICE_SERVO.
// -------------------------------------------------------------------
#define CMD_SHARE  0x56

// Longest browser-visible name a sketch can give an object (excl. NUL).
#define MAX_SHARE_NAME  15

// Limit-switch ends (param 1 of SET_SWITCH / LIMIT). Named LIMIT_* because
// some cores define MIN/MAX macros.
#define LIMIT_MIN  0
#define LIMIT_MAX  1

// Stepper interface types (param 1 of CMD_STEPPER_ATTACH) — match AccelStepper
#define STEPPER_DRIVER     1   // STEP/DIR: pin1=STEP, pin2=DIR (TMC2208/2209, A4988, EasyDriver)
#define STEPPER_FULL4WIRE  4   // 4 coil pins (28BYJ-48 via ULN2003, bare bipolar via H-bridge)

// invertMask bits (optional param of CMD_STEPPER_ATTACH):
//   bit0 = DIR inverted, bit1 = STEP inverted, bit2 = ENABLE inverted.
// ENABLE defaults to inverted (mask 0x04) — most driver EN pins are active-LOW.

// -------------------------------------------------------------------
// Bus Servo Device ID and Commands (0x41–0x4E)
// Serial-bus smart servos (Feetech ST/SMS and SC/SCS series) on a shared
// half-duplex UART, e.g. via a Waveshare Serial Bus Servo Driver board.
// Unlike PWM servos, all bus servos share ONE UART and are addressed by a
// hardware servo ID (1–253); positions are raw encoder counts
// (ST: 0–4095, SC: 0–1023), not degrees.
//
// Requires the Feetech/Waveshare SCServo library (SMS_STS + SCSCL classes),
// which handles the packet protocol, half-duplex direction, and the
// sign-magnitude encoding of the offset/speed registers for us.
// -------------------------------------------------------------------
#define DEVICE_BUSSERVO  206

#define CMD_BUSSERVO_BUS_CONFIG  0x41  // JS→Ar (global): [serialIndex, baud, rxPin, txPin]
#define CMD_BUSSERVO_ATTACH      0x42  // JS→Ar: [id, servoId, series]  series 0=ST(0-4095), 1=SC(0-1023)
#define CMD_BUSSERVO_DETACH      0x43  // JS→Ar: [id]
#define CMD_BUSSERVO_WRITE       0x44  // JS→Ar: [id, position, speed, acc]  position-mode goal (counts)
#define CMD_BUSSERVO_WRITE_SPEED 0x45  // JS→Ar: [id, speed, acc]  wheel-mode speed (sign = direction)
#define CMD_BUSSERVO_SET_MODE    0x46  // JS→Ar: [id, mode]  0=position, 1=wheel (continuous)
#define CMD_BUSSERVO_TORQUE      0x47  // JS→Ar: [id, enable]  0 = go limp (hand-pose / read)
#define CMD_BUSSERVO_READ        0x48  // JS→Ar: [id]
                                       // Ar→JS: [id, position, speed, load, voltage, temp, current]
#define CMD_BUSSERVO_SET_LIMITS  0x49  // JS→Ar: [id, minPos, maxPos, enabled] — soft position limits,
                                       //   clamped ON THE BOARD (RAM only; deliberately does NOT touch
                                       //   the servo's EEPROM limit registers — no wear, no unverified
                                       //   unLockEprom path).
#define CMD_BUSSERVO_CALIBRATE   0x4A  // JS→Ar: [id]  set current position as centre (homing offset)
#define CMD_BUSSERVO_SET_ID      0x4B  // JS→Ar: [id, newServoId]  (renumber — one servo on the bus!)
#define CMD_BUSSERVO_PING        0x4C  // JS→Ar: [id, servoId]  Ar→JS: [id, servoId, found]
#define CMD_BUSSERVO_SCAN        0x4D  // JS→Ar: [firstId, lastId]  Ar→JS: [count, id1, id2, ...]
#define CMD_BUSSERVO_SYNC_WRITE  0x4E  // JS→Ar (global): [series] + payload:
                                       //   N × { servoId u8, position i16, speed u16, acc u8 } (6 bytes each)
                                       // One hardware SyncWrite packet — all listed servos latch together.
// (0x4F–0x50 are stepper timed commands, numbered after this block.)
#define CMD_BUSSERVO_DONE        0x51  // Ar→JS (unsolicited): [id, position] — servo settled at its goal.
                                       // The board polls the servo's Moving flag after a write and emits this
                                       // when it stops, so bus servos get a done like steppers/servos.
// Uses 0x62 — the next globally-free command code (NeoPixel ends at 0x61); the
// bus-servo 0x41–0x4E block is full, and dispatch is by (deviceId, cmd) anyway.
#define CMD_BUSSERVO_READ_LIMITS 0x62  // JS→Ar: [id]  read the servo's EEPROM min/max ANGLE-LIMIT registers
                                       // (SMS_STS/SCSCL_MIN/MAX_ANGLE_LIMIT). Ar→JS: [id, min, max] (counts;
                                       // −1 = servo didn't answer). The board reads + caches these at attach
                                       // and replays the cache in announce(); this command forces a fresh read.
                                       // Unlike CMD_BUSSERVO_SET_LIMITS (board-RAM soft limits) these are the
                                       // servo's OWN firmware limits — read-only here, never written.
#define CMD_BUSSERVO_PRESENT     0x63  // Ar→JS: [id, servoId, present] — did the servo answer the attach-time
                                       // ping? present 1 = found, 0 = no response. The board pings at attach,
                                       // caches the result, and replays it in announce() (so late/reconnecting
                                       // browsers learn it too). Gives JS the parity with the serial monitor's
                                       // "[found] / [NO RESPONSE]" line. (0x64 is the next globally-free code.)

// Series (param 2 of CMD_BUSSERVO_ATTACH)
#define BUSSERVO_SERIES_ST  0   // STS / SMS series — 0–4095 counts (STS3215 etc.)
#define BUSSERVO_SERIES_SC  1   // SCS series      — 0–1023 counts (SCS15 etc.)

// Operating mode (param of CMD_BUSSERVO_SET_MODE)
#define BUSSERVO_MODE_POSITION  0
#define BUSSERVO_MODE_WHEEL     1

// -------------------------------------------------------------------
// Message Channel (CMD_MESSAGE 0x0B) — user-defined key/value messages
// that aren't tied to any pin or hardware device. Symmetric: the same
// frame shape flows JS→Ar and Ar→JS.
//
// Routed by CMD (like HELLO/ANNOUNCE/PONG), NOT by the target range —
// the flags in the TARGET high byte can push it past RESERVED_START, so
// the target range check would misroute it to the extension dispatch.
//
// Frame:
//   TARGET      low byte  = value type (MSG_TYPE_*)
//               high byte = flags (MSG_FLAG_*)
//   NPARAMS     = 1 for INT/BOOL/FLOAT/CHAR (the value) ; 0 for TEXT/BLOB
//   TYPE_MASK   = bit0 set iff FLOAT (so the param decodes as float32)
//   PAYLOAD     = [keyLen:u8][key UTF-8 …][value bytes … (TEXT/BLOB only)]
// -------------------------------------------------------------------
#define MSG_TYPE_INT    0   // int32 param
#define MSG_TYPE_BOOL   1   // int32 param (0/1)
#define MSG_TYPE_FLOAT  2   // float32 param (TYPE_MASK bit0 set)
#define MSG_TYPE_CHAR   3   // int32 param (one code unit)
#define MSG_TYPE_TEXT   4   // UTF-8 string in payload (after the key)
#define MSG_TYPE_BLOB   5   // raw bytes in payload (after the key)

// TARGET high-byte flags.
#define MSG_FLAG_RETAIN     0x01  // board stores the latest value, replays it on connect
#define MSG_FLAG_BROADCAST  0x02  // board relays a browser message to the OTHER browsers too

// Pack / unpack the TARGET field of a message frame.
#define MSG_TARGET(type, flags)  ((uint16_t)(((uint16_t)(flags) << 8) | ((type) & 0xFF)))
#define MSG_TYPE(target)         ((uint8_t)((target) & 0xFF))
#define MSG_FLAGS(target)        ((uint8_t)(((target) >> 8) & 0xFF))

// Longest message key (excl. NUL). Keys are length-prefixed with a u8.
#define MAX_MESSAGE_KEY  24

// -------------------------------------------------------------------
// Rotary Encoder Extension (PardaloteEncoder.h)
//
// Quadrature encoders (KY-040 knobs, motor shaft encoders). Counted in
// interrupt handlers — a 4x state-table decoder that rejects invalid
// transitions, so mechanical bounce never miscounts and edge rates far
// beyond the loop rate are handled. Position is ABSOLUTE (raw
// quadrature steps; a KY-040 detent = 4 steps), so intermediate values
// are disposable: transmission uses the analog model — per-client
// interval as a rate limit + threshold, latest value wins.
// -------------------------------------------------------------------
#define DEVICE_ENCODER  207

#define CMD_ENCODER_ATTACH        0x58  // JS→Ar: [id, pinA, pinB]
#define CMD_ENCODER_DETACH        0x59  // JS→Ar: [id]
#define CMD_ENCODER_READ          0x5A  // JS→Ar: [id, interval?, threshold?] read/register (0 = defaults;
                                        //   interval < 0 = END, unregister). Ar→JS: [id, position]
#define CMD_ENCODER_SET_POSITION  0x5B  // JS→Ar: [id, value] — re-zero/set the count; board echoes a
                                        //   READ to all clients so every mirror adopts the new frame

#endif
