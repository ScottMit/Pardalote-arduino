// ==============================================================
// Pardalote — Servo example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// Including <PardaloteServo.h> is enough to add servo support.
// The extension self-registers; no further setup is required.
//
// Browser side:
//   arduino.add('pan', new Servo());
//   arduino.on('ready', () => {
//       arduino.pan.attach(9);
//       arduino.pan.write(90);
//   });
// ==============================================================

#include <Pardalote.h>
#include <PardaloteServo.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
