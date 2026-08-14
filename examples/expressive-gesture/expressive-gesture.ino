// Pardalote — expressive gesture
//
// Board side of examples/expressive-gesture/ (browser). The browser creates
// two PWM servos as a pan/tilt "head" and plays authored GESTURES on them —
// animation-style eased motion (nods, shakes, a curious cock) that the board
// runs on its own clock. All the motion is authored in the browser; the sketch
// just needs the servo extension.
//
// Wiring: two hobby servos on any two PWM-capable pins (defaults 9 and 10 in
// the web page). Give the servos their own 5V supply with a common ground.

#include <Pardalote.h>
#include <PardaloteServo.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
