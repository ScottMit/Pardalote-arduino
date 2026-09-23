// ==============================================================
// Pardalote — Board Gestures (PWM servos) example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// Two standard PWM servos. For the same demo on Feetech serial-bus servos,
// see the board-gestures-bus-servos example.
//
// Expressive motion authored and run ON THE BOARD — no browser needed.
// Pardalote's rule is "whoever speaks is in control": the browser can
// compose-and-play a gesture (arduino.pan.gesture([...])), and so can the
// sketch. This example is the sketch speaking.
//
// A two-servo "creature head" (pan + tilt) idles on its own, and each time a
// button is pressed it plays a quick "react" — all decided by the board. A
// gesture is an ordered list of eased segments the board plays on its own
// millis() clock; Pardalote.gesture() plays several servos phase-locked so they
// arrive together; onGestureDone() chains one gesture into the next.
//
// SHAPING (this example's focus): the same authored react is RESHAPED on the
// fly — scale (amplitude), speed (tempo), crop (play a slice) — without editing
// its segments. Each button press steps to the next reshaping so you can feel
// each transform back-to-back:
//   0 plain · 1 gentle (scale 0.5) · 2 big (scale 1.4) · 3 fast (speed 2) ·
//   4 slow (speed 0.5) · 5 first-half only (crop 0–0.5) ·
//   6 asymmetric (tilt scaled 1.6, a touch quick) · 7 pan-only (per-actuator mod)
// The react is a RELATIVE round-trip, so scale/crop visibly change it; the idle
// is absolute, so it always re-centres afterwards (even after a crop leaves the
// head off-home).
//
// Browser side (OPTIONAL — just to watch):
//   The sketch creates the servos, so a connected browser sees them as
//   arduino.pan / arduino.tilt automatically and can take over with a
//   write() at any time (last speaker wins).
//
// Hardware:
//   - Two standard PWM servos on pins 9 (pan) and 10 (tilt).
//   - Optional: a momentary button from pin 2 to GND (uses the internal
//     pull-up). With no button wired the head simply idles forever.
//   - ESP32Servo is used on ESP32; the built-in Servo library elsewhere.
// ==============================================================

#include <Pardalote.h>
#include <PardaloteServo.h>

const int PAN_PIN    = 9;
const int TILT_PIN   = 10;
const int BUTTON_PIN = 2;

int pan, tilt;   // logical ids from attach()

// --- Authored gestures --------------------------------------------------
// Absolute degrees. Because Pardalote is 32-bit only, these const arrays
// live in flash and cost no RAM. { curve, duration-ms, target-degrees }.

// Idle: a slow, easy sweep that loops forever.
static const PardaloteSeg PAN_IDLE[] = {
    { CURVE_EASE_IN_OUT, 1600,  60 },
    { CURVE_EASE_IN_OUT, 1600, 120 },
    { CURVE_EASE_IN_OUT, 1200,  90 },
};
static const PardaloteSeg TILT_IDLE[] = {
    { CURVE_EASE_IN_OUT, 1400,  80 },
    { CURVE_EASE_IN_OUT, 1400, 100 },
    { CURVE_EASE_IN_OUT, 1600,  90 },   // shorter total → padded to arrive with pan
};

// React: a quick head-cock — pan turns and returns, tilt dips (with a little
// CURVE_BACK overshoot) and returns. RELATIVE (played with flags = 0), so it's a
// net-zero round-trip that runs from wherever the head is — and so that scale
// and crop visibly reshape it (scale multiplies a relative delta; an absolute
// target has no anchor to scale about).
static const PardaloteSeg REACT_PAN[] = {
    { CURVE_EASE_OUT,    200,  45 },   // turn
    { CURVE_EASE_IN_OUT, 300, -45 },   // back to centre
};
static const PardaloteSeg REACT_TILT[] = {
    { CURVE_BACK,        200, -40 },   // dip, slight overshoot
    { CURVE_EASE_IN_OUT, 300,  40 },   // back up
};

enum Mood { IDLING, REACTING };
Mood mood = IDLING;
int  reactStep = 0;                    // which reshaping the next press plays
const int NUM_REACTS = 8;

// Play the two-servo idle as ONE coordinated, phase-locked gesture. The
// shorter tilt lane is padded with a trailing hold so both lanes finish
// together (the board-side twin of JS group.gesture()).
void playIdle() {
    Pardalote.gesture()
        .add(DEVICE_SERVO, pan,  PAN_IDLE,  3)
        .add(DEVICE_SERVO, tilt, TILT_IDLE, 3)
        .play();
}

// Play the react, reshaped by `step`. Same authored segments every time — only
// the scale / speed / crop change. speed and crop go on the builder (group-wide,
// so the lanes stay phase-locked); scale can be group-wide or per-lane.
void playReact(int step) {
    switch (step) {
        case 0:  // plain — the authored motion, unshaped
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false)
                .play();
            break;
        case 1:  // gentle — half the amplitude
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false)
                .scale(0.5f).play();
            break;
        case 2:  // big — emphatic
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false)
                .scale(1.4f).play();
            break;
        case 3:  // fast — twice the tempo
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false)
                .speed(2.0f).play();
            break;
        case 4:  // slow — half the tempo
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false)
                .speed(0.5f).play();
            break;
        case 5:  // first half only — the outward move, no return (ends off-home;
                 // the absolute idle re-centres it on the next loop)
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false)
                .crop(0.0f, 0.5f).play();
            break;
        case 6:  // asymmetric — tilt scaled bigger than pan (per-lane laneScale),
                 // and a touch quick (group speed)
            Pardalote.gesture()
                .add(DEVICE_SERVO, pan,  REACT_PAN,  2, false)
                .add(DEVICE_SERVO, tilt, REACT_TILT, 2, false, 1.6f)   // laneScale on tilt
                .speed(1.2f).play();
            break;
        case 7:  // per-actuator mod — one servo, a PardaloteGestureMod{scale,speed}
                 // passed straight to gesture() (pan only; tilt holds)
            PardaloteServo.gesture(pan, REACT_PAN, 2, /*flags=*/0, PardaloteGestureMod(0.7f, 1.5f));
            break;
    }
}

// The pan servo drives the sequence: whenever its lane finishes, decide what
// plays next. (Only pan carries a done-callback — pan moves in every step — so
// there's one clear place the sequence advances.) A react just finished → return
// to idling; otherwise loop the idle.
void onPanDone(int /*id*/) {
    if (mood == REACTING) mood = IDLING;
    playIdle();
}

void setup() {
    Pardalote.begin();
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Sketch-created servos — the browser sees these automatically.
    pan  = PardaloteServo.attach("pan",  PAN_PIN);
    tilt = PardaloteServo.attach("tilt", TILT_PIN);

    PardaloteServo.onGestureDone(pan, onPanDone);   // chain gestures headlessly
    playIdle();                                     // start moving on our own
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
