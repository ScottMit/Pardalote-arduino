// ==============================================================
// Pardalote — Coordinated Motion example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// This sketch includes all the motor control extensions so that
// you can choose between different motor types in the web browser.
// The browser example creates a group of motors and drives them together;
// the sketch only needs the extension(s) for the motor types you use.
//
// Requires the following libraries:
//   - the Feetech / Waveshare SCServo library (SMS_STS + SCSCL classes).
//     Install "SCServo" by FT&WS from the Library Manager, or a ZIP
//     from the Waveshare Bus Servo Adapter wiki / Feetech SDK.
//   - ESP32Servo for PWM servos.
//   - AccelStepper by Mike McCauley for stepper motors.
//
// Hardware, one or more of the following:
//   - A Waveshare Serial Bus Servo Driver board (or equivalent adapter)
//     and one or more Feetech ST/SMS (0–4095) or SC/SCS (0–1023) servos.
//   - One or more standard PWM servos.
//   - A stepper motor driver board and one or more stepper motors.
//   - A power supply capable of powering your motors.
// ==============================================================

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
