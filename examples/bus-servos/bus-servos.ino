// ==============================================================
// Pardalote — Bus Servo example
// https://github.com/ScottMit/Pardalote
// Copyright (C) 2026 Scott Mitchell — GPL-3.0-or-later. See LICENSE.
//
// Including <PardaloteBusServo.h> is enough to add serial-bus servo
// support. The extension self-registers; no further setup is required.
//
// Requires the Feetech / Waveshare SCServo library (SMS_STS + SCSCL
// classes). Install "SCServo" by FT&WS from the Library Manager, or a ZIP
// from the Waveshare Bus Servo Adapter wiki / Feetech SDK.
//
// Hardware:
//   - A Waveshare Serial Bus Servo Driver board (or equivalent adapter)
//     wired to a hardware UART: UNO R4 WiFi → Serial1 (D0/D1),
//     ESP32 → Serial1 or Serial2 (set the pins with configureBus()).
//   - One or more Feetech ST/SMS (0–4095) or SC/SCS (0–1023) servos,
//     daisy-chained on the bus, each with a unique ID.
//   - The servos need their own power supply (6–7.4 V typical).
//
// Browser side:
//   arduino.add('joint', new BusServo());
//   arduino.on('ready', () => {
//       arduino.joint.attach(1);     // servo ID 1
//       arduino.joint.center();      // → 2048 (ST)
//   });
// ==============================================================

#include <Pardalote.h>
#include <PardaloteBusServo.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
