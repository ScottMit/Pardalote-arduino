// ==============================================================
// Pardalote — Stepper example
//
// Including <PardaloteStepper.h> is enough to add stepper support.
// The extension self-registers; no further setup is required.
//
// Requires the AccelStepper library (by Mike McCauley):
//   Arduino IDE → Tools → Manage Libraries → search "AccelStepper".
//
// Wiring (STEP/DIR driver — TMC2208/2209, A4988, EasyDriver):
//   driver STEP → your STEP pin
//   driver DIR  → your DIR pin
//   driver EN   → your EN pin (optional; active-LOW on most drivers)
//   Give the motor its own supply — do not power it from the board's 5V.
//
// Browser side:
//   arduino.add('x', new Stepper());
//   arduino.on('ready', () => {
//       arduino.x.attach(2, 3, 4);   // STEP, DIR, EN
//       arduino.x.setMaxSpeed(1000);
//       arduino.x.setAcceleration(500);
//       arduino.x.moveTo(2000);
//   });
// ==============================================================

#include <Pardalote.h>
#include <PardaloteStepper.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
