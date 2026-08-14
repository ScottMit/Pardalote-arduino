// ==============================================================
// Pardalote — Messaging
//
// Named key/value messages between the sketch and the browser, with
// no pin or hardware device involved:
//
//   Pardalote.send(key, value, flags)  — send (int/float/bool/char/text/blob)
//   Pardalote.watch(key, cb)           — handle one key
//   Pardalote.onMessage(cb)            — handle every key
//
// This sketch:
//   • drives pin 13 (built-in LED on some boards) from a "led" message (browser → board),
//   • logs every incoming message to Serial (onMessage),
//   • sends a retained "uptime" once a second (board → every browser,
//     and replayed to browsers that connect later).
//
// Pair it with examples/messaging/ (browser page) in the project repo.
// ==============================================================

#include <Pardalote.h>

const int LIGHT = 13;
unsigned long lastTick = 0;

// Handle the "led" key: value is a bool (on/off).
void onLed(const Message& m) {
    if (m.asBool()) digitalWrite(LIGHT, HIGH);
    else digitalWrite(LIGHT, LOW);
}

// Handle every message (a simple log to Serial).
void onAnyMessage(const Message& m) {
    Serial.print(F("message: "));
    Serial.print(m.key);
    Serial.print(F(" = "));
    switch (m.type) {
        case MSG_TYPE_INT:   Serial.println(m.asInt());          break;
        case MSG_TYPE_BOOL:  Serial.println(m.asBool() ? F("true") : F("false")); break;
        case MSG_TYPE_FLOAT: Serial.println(m.asFloat());        break;
        case MSG_TYPE_CHAR:  Serial.println(m.asChar());         break;
        case MSG_TYPE_TEXT:  Serial.println(m.text);             break;
        case MSG_TYPE_BLOB:  Serial.print(m.length); Serial.println(F(" bytes")); break;
    }
}

void setup() {
    Pardalote.begin();
    pinMode(LIGHT, OUTPUT);

    Pardalote.watch("led", onLed);
    Pardalote.onMessage(onAnyMessage);
}

void loop() {
    Pardalote.run();

    // Once a second, publish an uptime counter. retain = every browser that
    // connects later immediately gets the current value.
    if (millis() - lastTick >= 1000) {
        lastTick += 1000;
        Pardalote.send("uptime", (int)(millis() / 1000), MSG_FLAG_RETAIN);
    }
}
