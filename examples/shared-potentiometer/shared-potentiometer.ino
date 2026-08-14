// ==============================================================
// Pardalote — Shared Input: Potentiometer
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

const int POT = A0; // ESP IO 36

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
