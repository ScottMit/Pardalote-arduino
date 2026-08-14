// ==============================================================
// internal/led_matrix.h
// UNO R4 LED matrix helpers — shows "Pardalote" at boot, then
// scrolls the IP address until a browser connects. No-op on other
// platforms so callers don't need to guard.
// ==============================================================

#pragma once

// Call once during PardaloteClass::begin() — starts the matrix and
// scrolls the boot text. No-op on non-UNO R4 platforms.
void ledMatrixBegin();

// Call from PardaloteClass::run(). While no browser is connected it
// re-scrolls the IP each time the previous animation finishes, so the
// address stays readable. Once a browser connects it stops re-arming —
// the in-flight scroll finishes and the matrix goes quiet, freeing the
// loop cycles it was spending rebuilding the animation (this rebuild
// competes with _ws.loop() for the UNO R4's single core). Scrolling
// resumes automatically if every client disconnects. Pass whether any
// browser is currently connected. No-op on non-UNO R4.
void ledMatrixLoop(bool anyConnected);
