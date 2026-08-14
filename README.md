# Pardalote (Arduino library)

Arduino-side library for the Pardalote project — control hardware from a web browser over WiFi or USB serial.

A minimal sketch:

```cpp
#include <Pardalote.h>

void setup() { Pardalote.begin(); }
void loop()  { Pardalote.run();   }
```

Extensions are opt-in headers that self-register when included — add only the ones your sketch uses. The full set:

```cpp
#include <Pardalote.h>
#include <PardaloteServo.h>       // hobby / PWM servos
#include <PardaloteStepper.h>     // stepper motors (STEP/DIR or 4-wire)
#include <PardaloteBusServo.h>    // Feetech ST/SC serial bus servos
#include <PardaloteNeoPixel.h>    // WS2812 / NeoPixel LEDs
#include <PardaloteUltrasonic.h>  // HC-SR04 ultrasonic distance sensor
#include <PardaloteIMU.h>         // MPU-6050 accelerometer + gyro
#include <PardaloteEncoder.h>     // quadrature rotary encoder
#include <PardaloteCamera.h>      // camera streaming (ESP32 only)

void setup() { Pardalote.begin(); }
void loop()  { Pardalote.run();   }
```

See the [main Pardalote README](https://github.com/ScottMit/Pardalote#readme) for the full guide, JavaScript API, and protocol documentation.
