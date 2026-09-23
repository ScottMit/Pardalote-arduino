// ==============================================================
// Pardalote — Board Gestures (bus servos) example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// The bus-servo twin of board-gestures-PWM-servos: the same idle + reshaped
// "react" demo, on two Feetech serial-bus servos instead of PWM ones. Values
// are in COUNTS (ST/SMS = 0–4095, centre 2048) instead of degrees; everything
// else — idle loop, the scale / speed / crop shaping cycle, onGestureDone() —
// is identical, because the gesture surface is the same across actuator types.
//
// Expressive motion authored and run ON THE BOARD — no browser needed.
// "Whoever speaks is in control": the browser can compose-and-play a gesture,
// and so can the sketch. This is the sketch speaking.
//
// SHAPING (this example's focus): the same authored react is RESHAPED on the
// fly — scale (amplitude), speed (tempo), crop (play a slice). Each button
// press steps to the next reshaping so you can feel each transform back-to-back:
//   0 plain · 1 gentle (scale 0.5) · 2 big (scale 1.4) · 3 fast (speed 2) ·
//   4 slow (speed 0.5) · 5 first-half only (crop 0–0.5) ·
//   6 asymmetric (tilt scaled 1.6, a touch quick) · 7 pan-only (per-actuator mod)
// The react is a RELATIVE round-trip, so scale/crop visibly change it; the idle
// is absolute, so it always re-centres afterwards (even after a crop).
//
// Requires the Feetech / Waveshare SCServo library (install "SCServo" by FT&WS
// from the Library Manager). Bus gestures render eased curves on the board via
// a streaming interpolator, so the shaping reads the same as on PWM servos.
//
// Hardware:
//   - A Waveshare Serial Bus Servo Driver (or equivalent) on a hardware UART:
//     UNO R4 WiFi → Serial1 (D0/D1, fixed); ESP32 → set BUS_RX / BUS_TX below.
//   - Two Feetech ST/SMS servos on the bus, IDs 1 (pan) and 2 (tilt), each with
//     its own 6–7.4 V supply.
//   - Optional: a momentary button from pin 2 to GND (internal pull-up). With no
//     button wired the head simply idles forever.
// ==============================================================

#include <Pardalote.h>
#include <PardaloteBusServo.h>

const int PAN_ID     = 1;    // Feetech bus IDs (set unique IDs on the servos first)
const int TILT_ID    = 2;
const int BUTTON_PIN = 2;

// ESP32 bus-UART pins (ignored on a UNO R4, which is fixed to Serial1 = D0/D1).
const int BUS_RX = 18;
const int BUS_TX = 19;

int pan, tilt;   // logical ids from attach()

// --- Authored gestures --------------------------------------------------
// Counts (0–4095, centre 2048). Flash const, ~0 RAM. { curve, duration-ms, value }.

// Idle: a slow, easy sweep that loops forever (absolute targets).
static const PardaloteSeg PAN_IDLE[] = {
    { CURVE_EASE_IN_OUT, 1600, 1748 },
    { CURVE_EASE_IN_OUT, 1600, 2348 },
    { CURVE_EASE_IN_OUT, 1200, 2048 },
};
static const PardaloteSeg TILT_IDLE[] = {
    { CURVE_EASE_IN_OUT, 1400, 1908 },
    { CURVE_EASE_IN_OUT, 1400, 2188 },
    { CURVE_EASE_IN_OUT, 1600, 2048 },   // shorter total → padded to arrive with pan
};

// React: a quick head-cock — pan turns and returns, tilt dips (with a little
// CURVE_BACK overshoot) and returns. RELATIVE (played with flags = 0): a net-zero
// round-trip from wherever the head is, so scale and crop visibly reshape it.
static const PardaloteSeg REACT_PAN[] = {
    { CURVE_EASE_OUT,    200,  300 },   // turn
    { CURVE_EASE_IN_OUT, 300, -300 },   // back to centre
};
static const PardaloteSeg REACT_TILT[] = {
    { CURVE_BACK,        200, -350 },   // dip, slight overshoot
    { CURVE_EASE_IN_OUT, 300,  350 },   // back up
};

enum Mood { IDLING, REACTING };
Mood mood = IDLING;
int  reactStep = 0;
const int NUM_REACTS = 8;

// Two-servo idle as ONE coordinated, phase-locked gesture (shorter tilt lane
// padded to arrive with pan) — the board-side twin of JS group.gesture().
void playIdle() {
    Pardalote.gesture()
        .add(DEVICE_BUSSERVO, pan,  PAN_IDLE,  3)
        .add(DEVICE_BUSSERVO, tilt, TILT_IDLE, 3)
        .play();
}

// Play the react, reshaped by `step`. Same authored segments every time — only
// scale / speed / crop change. speed and crop go on the builder (group-wide, so
// the lanes stay phase-locked); scale can be group-wide or per-lane.
void playReact(int step) {
    switch (step) {
        case 0:  // plain — the authored motion, unshaped
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false)
                .play();
            break;
        case 1:  // gentle — half the amplitude
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false)
                .scale(0.5f).play();
            break;
        case 2:  // big — emphatic
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false)
                .scale(1.4f).play();
            break;
        case 3:  // fast — twice the tempo
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false)
                .speed(2.0f).play();
            break;
        case 4:  // slow — half the tempo
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false)
                .speed(0.5f).play();
            break;
        case 5:  // first half only — the outward move, no return (ends off-home;
                 // the absolute idle re-centres it on the next loop)
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false)
                .crop(0.0f, 0.5f).play();
            break;
        case 6:  // asymmetric — tilt scaled bigger than pan (per-lane laneScale),
                 // and a touch quick (group speed)
            Pardalote.gesture()
                .add(DEVICE_BUSSERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_BUSSERVO, tilt, REACT_TILT, 2, false, 1.6f)   // laneScale on tilt
                .speed(1.2f).play();
            break;
        case 7:  // per-actuator mod — one servo, a PardaloteGestureMod{scale,speed}
                 // passed straight to gesture() (pan only; tilt holds)
            PardaloteBusServo.gesture(pan, REACT_PAN, 2, /*flags=*/0, PardaloteGestureMod(0.7f, 1.5f));
            break;
    }
}

// Pan drives the sequence: when its lane finishes, decide what's next. A react
// just finished → return to idling; otherwise loop the idle.
void onPanDone(int /*id*/) {
    if (mood == REACTING) mood = IDLING;
    playIdle();
}

void setup() {
    Pardalote.begin();
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Bus UART (ESP32 pins; UNO R4 ignores them). Call before attach().
    PardaloteBusServo.configureBus(BUS_RX, BUS_TX);

    // Sketch-created bus servos — the browser sees these automatically.
    pan  = PardaloteBusServo.attach("pan",  PAN_ID);
    tilt = PardaloteBusServo.attach("tilt", TILT_ID);
    PardaloteBusServo.torque(pan,  true);
    PardaloteBusServo.torque(tilt, true);

    PardaloteBusServo.onGestureDone(pan, onPanDone);   // chain gestures headlessly
    playIdle();                                        // start moving on our own
}

void loop() {
    Pardalote.run();

    // Board-authored reaction — no browser involved. On a falling edge (button
    // pressed) while idling, interrupt the idle with the next reshaping of the
    // react; onPanDone() then returns the head to its idle loop. Each press steps
    // through the shaping cycle (see the header) so you can compare them.
    static bool wasPressed = false;
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (pressed && !wasPressed && mood == IDLING) {
        mood = REACTING;
        playReact(reactStep);
        reactStep = (reactStep + 1) % NUM_REACTS;
    }
    wasPressed = pressed;
}
