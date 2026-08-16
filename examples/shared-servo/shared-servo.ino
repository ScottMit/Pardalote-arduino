// ==============================================================
// Pardalote — Shared Servo example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// The SKETCH creates the servo:
//
//     pan = PardaloteServo.attach("pan", SERVO_PIN);
//
// and the browser sees it automatically as arduino.pan — a full
// Servo instance, identical to one created with arduino.add().
// No JS-side setup at all.
//
// Both sides then drive the same servo: this sketch nods it to a
// new pose every few seconds, and the browser can grab it with the
// mouse at any time. Last writer wins; both stay in sync (sketch
// writes are auto-echoed to the browser).
//
// Wiring:
//   Servo signal on pin 9 (change SERVO_PIN to suit), 5 V + GND.
// ==============================================================

#include <Pardalote.h>
#include <PardaloteServo.h>

const int SERVO_PIN = 9; // e.g. pin 9 for UNO R4, GPIO 18 for ESP32

int pan = -1;                     // logical id, auto assigned by attach()
unsigned long lastMove = 0;

void setup() {
    Pardalote.begin();

    // Create the servo and make it visible to every browser as
    // arduino.pan. Returns the logical id for the calls below.
    pan = PardaloteServo.attach("pan", SERVO_PIN);

    PardaloteServo.write(pan, 90);
}

void loop() {
    Pardalote.run();

    // Every 4 s, glide to a new pose (1 s, board-interpolated).
    // The browser's arduino.pan.angle follows automatically.
    if (millis() - lastMove > 4000) {
        lastMove = millis();
        int target = random(20, 161);
        PardaloteServo.writeTimed(pan, target, 1000);
    }
}
