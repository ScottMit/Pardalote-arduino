// ==============================================================
// Pardalote — Servo example
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
