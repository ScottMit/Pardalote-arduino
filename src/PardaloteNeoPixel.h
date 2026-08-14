// ==============================================================
// PardaloteNeoPixel.h
// Pardalote NeoPixel Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0 License
//
// Add #include <PardaloteNeoPixel.h> to your sketch.
// Requires the Adafruit NeoPixel library.
//
// Pixel updates are buffered on the Arduino side by the
// Adafruit library — LEDs only update when CMD_NEO_SHOW arrives.
// ==============================================================

#ifndef PARDALOTE_NEOPIXEL_H
#define PARDALOTE_NEOPIXEL_H

#include <Adafruit_NeoPixel.h>
#include "Pardalote.h"

#define MAX_STRIPS 4

class NeoPixelExt {
private:
    inline static Adafruit_NeoPixel* _strips[MAX_STRIPS]   = {};
    inline static int16_t  _pins[MAX_STRIPS]                = { -1,-1,-1,-1 };
    inline static uint16_t _numPixels[MAX_STRIPS]           = {};  // matches Adafruit_NeoPixel::numPixels() type
    inline static uint32_t _types[MAX_STRIPS]               = {};
    inline static bool     _initialized[MAX_STRIPS]         = {};

    // Sketch-created strips (PardaloteNeoPixel.attach("name", pin, count)).
    // The name is what the browser binds (arduino.<name>); announce() replays
    // a CMD_SHARE frame for these so every connecting browser materialises the
    // object. Browser-created strips have _sketchOwned = false. Mirrors the
    // actuator extensions' sketch-attach path.
    inline static bool     _sketchOwned[MAX_STRIPS] = {};
    inline static char     _names[MAX_STRIPS][MAX_SHARE_NAME + 1] = {};

    static bool validId(int id) { return id >= 0 && id < MAX_STRIPS; }

public:
    // -------------------------------------------------------------------
    // Sketch-facing read accessors (used by the PardaloteNeoPixel object).
    // -------------------------------------------------------------------
    static bool initializedId(int id) { return validId(id) && _initialized[id]; }
    static int  numPixelsId(int id)   { return (validId(id) && _initialized[id]) ? _numPixels[id] : 0; }
    static int  listAttached(int* out, int max) {
        int n = 0;
        for (int i = 0; i < MAX_STRIPS && n < max; i++) if (_initialized[i]) out[n++] = i;
        return n;
    }

    // -------------------------------------------------------------------
    // Sketch-created strips — PardaloteNeoPixel.attach("name", pin, count).
    //
    // Creation and browser visibility are one act (see the servo extension
    // for the full rationale): the strip is initialised through the same
    // handler a browser init uses, and a CMD_SHARE frame (+ the init state)
    // is broadcast so connected browsers materialise arduino.<name>
    // immediately. announce() replays the sequence for later connects.
    //
    // Logical ids allocated TOP-DOWN (browser add() grows from 0 up), so the
    // two sides can't collide until every slot is used. Idempotent per name.
    // Returns the logical id, or -1 if no slot is free.
    // -------------------------------------------------------------------
    static int sketchAttach(const char* name, int pin, int count, uint32_t type) {
        if (name == nullptr || name[0] == '\0') return -1;

        int id = -1;
        for (int i = 0; i < MAX_STRIPS; i++)
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        if (id < 0)
            for (int i = MAX_STRIPS - 1; i >= 0; i--)
                if (!_initialized[i] && !_sketchOwned[i]) { id = i; break; }
        if (id < 0) {
            Serial.print(F("NeoPixel: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        // Init through the same code path a browser init takes.
        Pardalote.command(DEVICE_NEO_PIXEL, CMD_NEO_INIT, id, pin, count, (int32_t)type);

        // Tell any connected browsers now (no-op when none are connected;
        // announce() covers future connects): name → init.
        broadcastShare(id);
        FrameBuilder fi;
        fi.begin(CMD_NEO_INIT, DEVICE_NEO_PIXEL);
        fi.addInt(id); fi.addInt(_pins[id]);
        fi.addInt(_numPixels[id]); fi.addInt((int32_t)_types[id]);
        Pardalote.broadcastFrame(fi);

        return id;
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_NEO_PIXEL);
        fb.addInt(id);
        fb.addString(_names[id]);
        Pardalote.broadcastFrame(fb);
    }

    // -------------------------------------------------------------------
    // Main dispatch
    // -------------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {
        if (nparams < 1) return;
        int id = (int)paramInt(params, 0);
        if (!validId(id)) {
            Serial.print(F("NeoPixel: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            case CMD_NEO_INIT: {
                if (nparams < 4) return;
                int     pin   = (int)paramInt(params, 1);
                int     num   = (int)paramInt(params, 2);
                uint32_t type = (uint32_t)paramInt(params, 3);

                if (_strips[id]) { delete _strips[id]; _strips[id] = nullptr; }
                _strips[id]      = new Adafruit_NeoPixel(num, pin, type);
                _pins[id]        = (int16_t)pin;
                _numPixels[id]   = (uint16_t)num;
                _types[id]       = type;
                _initialized[id] = true;

                _strips[id]->begin();
                _strips[id]->clear();
                _strips[id]->show();

                Serial.print(F("NeoPixel ")); Serial.print(id);
                Serial.print(F(" init: pin=")); Serial.print(pin);
                Serial.print(F(" pixels=")); Serial.println(num);
                break;
            }

            case CMD_NEO_SET_PIXEL: {
                if (!_initialized[id] || nparams < 5) return;
                int index = (int)paramInt(params, 1);
                if (index < 0 || index >= _numPixels[id]) return;
                uint8_t r = (uint8_t)paramInt(params, 2);
                uint8_t g = (uint8_t)paramInt(params, 3);
                uint8_t b = (uint8_t)paramInt(params, 4);
                if (nparams > 5) {
                    uint8_t w = (uint8_t)paramInt(params, 5);
                    _strips[id]->setPixelColor(index, _strips[id]->Color(r, g, b, w));
                } else {
                    _strips[id]->setPixelColor(index, _strips[id]->Color(r, g, b));
                }
                break;
            }

            case CMD_NEO_FILL: {
                if (!_initialized[id] || nparams < 2) return;
                uint32_t color = (uint32_t)paramInt(params, 1);
                uint16_t first = (nparams > 2) ? (uint16_t)paramInt(params, 2) : 0;
                uint16_t count = (nparams > 3) ? (uint16_t)paramInt(params, 3) : 0;
                _strips[id]->fill(color, first, count);
                break;
            }

            case CMD_NEO_CLEAR:
                if (!_initialized[id]) return;
                _strips[id]->clear();
                break;

            case CMD_NEO_BRIGHTNESS:
                if (!_initialized[id] || nparams < 2) return;
                _strips[id]->setBrightness((uint8_t)paramInt(params, 1));
                break;

            case CMD_NEO_SHOW:
                if (!_initialized[id]) return;
                _strips[id]->show();
                break;

            default:
                Serial.print(F("NeoPixel: unknown cmd 0x"));
                Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Called on each new client connection.
    // Sends full strip state — init config, brightness, and every pixel
    // colour — so connecting clients immediately reflect the live state.
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_NEO_PIXEL);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_STRIPS);
        Pardalote.sendFrame(clientNum, fb);

        for (int i = 0; i < MAX_STRIPS; i++) {
            if (!_initialized[i]) continue;

            // Sketch-created strip: send its SHARE frame FIRST so the browser
            // materialises arduino.<name> before the state frames below.
            if (_sketchOwned[i]) {
                FrameBuilder fsh;
                fsh.begin(CMD_SHARE, DEVICE_NEO_PIXEL);
                fsh.addInt(i);
                fsh.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fsh);
            }

            // Init frame — tells JS the pin, length, and pixel type
            FrameBuilder fi;
            fi.begin(CMD_NEO_INIT, DEVICE_NEO_PIXEL);
            fi.addInt(i);
            fi.addInt(_pins[i]);
            fi.addInt(_numPixels[i]);
            fi.addInt((int32_t)_types[i]);
            Pardalote.sendFrame(clientNum, fi);

            // Brightness frame
            FrameBuilder fbr;
            fbr.begin(CMD_NEO_BRIGHTNESS, DEVICE_NEO_PIXEL);
            fbr.addInt(i);
            fbr.addInt(_strips[i]->getBrightness());
            Pardalote.sendFrame(clientNum, fbr);

            // One SET_PIXEL frame per pixel using getPixelColor() readback.
            // Adafruit stores colours in its own software buffer so this is
            // reliable regardless of whether show() has been called.
            for (uint16_t j = 0; j < _numPixels[i]; j++) {
                uint32_t c = _strips[i]->getPixelColor(j);
                uint8_t w = (c >> 24) & 0xFF;
                uint8_t r = (c >> 16) & 0xFF;
                uint8_t g = (c >>  8) & 0xFF;
                uint8_t b =  c        & 0xFF;

                FrameBuilder fp;
                fp.begin(CMD_NEO_SET_PIXEL, DEVICE_NEO_PIXEL);
                fp.addInt(i);
                fp.addInt(j);
                fp.addInt(r);
                fp.addInt(g);
                fp.addInt(b);
                if (w > 0) fp.addInt(w);
                Pardalote.sendFrame(clientNum, fp);
            }
        }
    }
};

// -------------------------------------------------------------------
// PardaloteNeoPixel — sketch-facing collection of LED strips.
//
// Create a strip from the sketch (the browser sees it automatically as
// arduino.<name> — a full NeoPixel instance, identical to a browser-created
// one):
//   int ring = PardaloteNeoPixel.attach("ring", 6, 24);   // name, pin, count
//   PardaloteNeoPixel.fill(ring, 0, 0, 40);               // dim blue
//   PardaloteNeoPixel.show(ring);
//
// Colours are 0–255 per channel. Like the PWM servo, the inside/outside
// boundary is per strip (each owns its pin — no shared bus): a strip you want
// to keep private shouldn't be here at all — drive it with the plain
// Adafruit_NeoPixel library directly. Pixel writes drive the board's buffer;
// browsers materialise the strip and its current colours on connect (via
// announce), but per-frame sketch animation is not pushed live (unlike an
// actuator's target echo — a whole framebuffer is too heavy to mirror).
// -------------------------------------------------------------------
class PardaloteNeoPixelAccess {
public:
    // attach(name, pin, count, type?) — create a strip and make it visible to
    // browsers as arduino.<name>. Returns the logical id for setPixel()/show()/
    // etc., or -1 if no slot is free. `type` is an Adafruit pixel-type constant
    // (default NEO_GRB + NEO_KHZ800). Names >MAX_SHARE_NAME (15) are truncated.
    // Idempotent per name.
    int attach(const char* name, int pin, int count,
               uint32_t type = NEO_GRB + NEO_KHZ800) const {
        return NeoPixelExt::sketchAttach(name, pin, count, type);
    }

    int  scan(int* out, int max) const { return NeoPixelExt::listAttached(out, max); }
    bool attached(int id)        const { return NeoPixelExt::initializedId(id); }
    int  numPixels(int id)       const { return NeoPixelExt::numPixelsId(id); }

    // Pixel ops — routed through the same handler the browser uses. Buffered:
    // nothing lights up until show(). Colours 0–255.
    void setPixel(int id, int index, int r, int g, int b) const {
        Pardalote.command(DEVICE_NEO_PIXEL, CMD_NEO_SET_PIXEL, id, index, r, g, b);
    }
    void fill(int id, int r, int g, int b) const {
        // Pack to Adafruit's 0x00RRGGBB (== strip.Color(r,g,b)); fill whole strip.
        int32_t color = ((int32_t)(uint8_t)r << 16) | ((int32_t)(uint8_t)g << 8) | (uint8_t)b;
        Pardalote.command(DEVICE_NEO_PIXEL, CMD_NEO_FILL, id, color);
    }
    void clear(int id)               const { Pardalote.command(DEVICE_NEO_PIXEL, CMD_NEO_CLEAR, id); }
    void brightness(int id, int val) const { Pardalote.command(DEVICE_NEO_PIXEL, CMD_NEO_BRIGHTNESS, id, val); }
    void show(int id)                const { Pardalote.command(DEVICE_NEO_PIXEL, CMD_NEO_SHOW, id); }
};
inline PardaloteNeoPixelAccess PardaloteNeoPixel;

INSTALL_EXTENSION(DEVICE_NEO_PIXEL, NeoPixelExt::handle, NeoPixelExt::announce)

#endif
