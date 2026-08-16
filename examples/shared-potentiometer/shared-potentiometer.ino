// ==============================================================
// Pardalote — Shared Input: Potentiometer example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// A potentiometer wired to A0. The Arduino tells the browser to start
// polling it — the browser doesn't have to declare the pin itself.
//
// Wiring:
//   Pot wiper    → A0
//   Pot one end  → 3.3 V
//   Pot other end → GND
// ==============================================================

#include <Pardalote.h>

const int POT = A0; // e.g. A0 for UNO R4, GPIO 36 (ADC1) for ESP32

void setup() {
    Pardalote.begin();
    pinMode(POT, INPUT);

    // Tell the browser "A0 is an analog input."
    // The JS side responds by auto-starting a poll at its default
    // interval (200 ms) — no pinMode/analogRead in the browser code.
    Pardalote.share(POT, ANALOG_INPUT_MODE);
}

void loop() {
    Pardalote.run();
    // Nothing else needed — the browser's poll requests are handled
    // by Pardalote.run() above, which also fires the analogRead and
    // broadcasts the result.
}
