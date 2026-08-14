# Pardalote (Arduino library)

Arduino-side library for the Pardalote project — control hardware from a web browser over WiFi or USB serial.

A minimal sketch:

```cpp
#include <Pardalote.h>

void setup() { Pardalote.begin(); }
void loop()  { Pardalote.run();   }
```

Optional extensions self-register when included:

```cpp
#include <Pardalote.h>
#include <PardaloteServo.h>
#include <PardaloteNeoPixel.h>

void setup() { Pardalote.begin(); }
void loop()  { Pardalote.run();   }
```

See the [project README](../../../README.md) for the full guide, JavaScript API, and protocol documentation.
