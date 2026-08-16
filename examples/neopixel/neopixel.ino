// ==============================================================
// Pardalote — NeoPixel example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// Requires the Adafruit NeoPixel library (Tools → Manage Libraries).
//
// Browser side:
//   arduino.add('strip', new NeoPixel());
//   arduino.on('ready', () => {
//       arduino.strip.init(6, 30);
//       arduino.strip.setPixelColor(0, 255, 0, 0);
//       arduino.strip.show();
//   });
// ==============================================================

#include <Pardalote.h>
#include <PardaloteNeoPixel.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
