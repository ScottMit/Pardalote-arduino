// ==============================================================
// Pardalote — Ultrasonic example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// Supports HC-SR04 and similar sensors, 3-wire or 4-wire.
//
// Browser side:
//   arduino.add('sonar', new Ultrasonic());
//   arduino.on('ready', () => {
//       arduino.sonar.attach(7, 8);   // 4-wire: trig=7, echo=8
//       arduino.sonar.read(200, CM);  // poll every 200 ms
//   });
// ==============================================================

#include <Pardalote.h>
#include <PardaloteUltrasonic.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
