// Pardalote — coordinated motion
//
// Board side of examples/coordinated-motion/ (browser). The browser
// creates a group of motors and drives them together; the sketch only
// needs the extension(s) for the motor types you use — simplest is all
// three, so you can switch type without re-flashing.

#include <Pardalote.h>
#include <PardaloteServo.h>
#include <PardaloteBusServo.h>
#include <PardaloteStepper.h>

void setup() {
    Pardalote.begin(); 
}

void loop() {
    Pardalote.run();   
}
