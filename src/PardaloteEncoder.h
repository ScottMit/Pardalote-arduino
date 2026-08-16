// ==============================================================
// PardaloteEncoder.h
// Pardalote Rotary Encoder Extension (quadrature)
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
//
// Quadrature encoders: KY-040 style knobs, optical/magnetic motor
// shaft encoders. Add #include <PardaloteEncoder.h> to your sketch.
//
// WHY INTERRUPTS: direction lives in the ORDER the two pins change
// and counting lives in never missing a step — a hand-flicked knob
// produces ~1,600 edges/s, far beyond loop()-rate sampling. Both pins
// raise a CHANGE interrupt feeding a 4x state-table decoder; invalid
// transitions (contact bounce) contribute nothing, so no debounce
// timing is needed and counts can't run backwards from chatter.
//
// TRANSMISSION: position is absolute, so intermediate values are
// disposable (unlike button edges). The loop hook snapshots the ISR
// counter every pass and offers it through the standard per-client
// gate — each browser's interval is a rate limit, threshold in raw
// quadrature steps (default 1). A KY-040 detent = 4 steps; the JS
// side scales to detents if asked (setStepsPerDetent).
//
// The KY-040's push button is NOT part of this extension — it's just
// a switch. Wire it to any pin and use arduino.pin(SW).on('change'),
// which delivers debounced edges instantly.
//
// Sketch-created encoders (browser sees arduino.<name> automatically):
//   int knob = PardaloteEncoder.attach("knob", 2, 3);   // name, pinA, pinB
//   long pos = PardaloteEncoder.read(knob);
//   PardaloteEncoder.setPosition(knob, 0);
// ==============================================================

#ifndef PARDALOTE_ENCODER_H
#define PARDALOTE_ENCODER_H

#include "Pardalote.h"

#define MAX_ENCODERS 4

// ISRs live in IRAM on the ESP32 (flash-cache misses inside an interrupt
// handler crash); other cores don't need or define the attribute.
#if defined(PLATFORM_ESP32) && defined(IRAM_ATTR)
  #define PARDALOTE_ISR IRAM_ATTR
#else
  #define PARDALOTE_ISR
#endif

class EncoderExt {
private:
    inline static int16_t _pinA[MAX_ENCODERS]     = { -1, -1, -1, -1 };
    inline static int16_t _pinB[MAX_ENCODERS]     = { -1, -1, -1, -1 };
    inline static bool    _attached[MAX_ENCODERS] = {};

    // ISR state. _count is written in interrupt context and read in the
    // loop — a single aligned 32-bit word, atomic on these MCUs
    // (Cortex-M4 / Xtensa / RISC-V), so no critical section is needed
    // for reads. Writes from the loop (setPosition) briefly mask
    // interrupts so a concurrent ISR can't interleave.
    inline static volatile int32_t _count[MAX_ENCODERS] = {};
    inline static volatile uint8_t _state[MAX_ENCODERS] = {};

    // Sketch-created encoders (PardaloteEncoder.attach("name", a, b)).
    inline static bool _sketchOwned[MAX_ENCODERS] = {};
    inline static char _names[MAX_ENCODERS][MAX_SHARE_NAME + 1] = {};

    // Periodic reads — per-client registration + gating.
    // Threshold in raw quadrature steps; default 1 (every step).
    inline static ExtReadPoll _polls[MAX_ENCODERS] = {};
    static constexpr uint16_t DEFAULT_THRESHOLD = 1;

    static bool validId(int id) { return id >= 0 && id < MAX_ENCODERS; }

    // ------------------------------------------------------------------
    // Quadrature decoding. State = (previous AB << 2) | current AB.
    // The table maps each of the 16 transitions to -1/0/+1; the four
    // "impossible" transitions (both pins flipping at once — always
    // bounce or noise) map to 0, which is what makes this decoder
    // debounce-free: chatter walks the state back and forth and sums
    // to zero, never to a phantom count.
    // ------------------------------------------------------------------
    inline static const int8_t QDELTA[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0,
    };

    static void PARDALOTE_ISR update(int id) {
        const uint8_t ab = (uint8_t)((digitalRead(_pinA[id]) << 1) | digitalRead(_pinB[id]));
        const uint8_t st = (uint8_t)(((_state[id] & 0x03) << 2) | ab);
        _state[id] = ab;
        _count[id] += QDELTA[st];
    }

    // One static trampoline per instance slot — attachInterrupt() wants a
    // plain function pointer, portably across UNO R4 and ESP32 cores.
    static void PARDALOTE_ISR isr0() { update(0); }
    static void PARDALOTE_ISR isr1() { update(1); }
    static void PARDALOTE_ISR isr2() { update(2); }
    static void PARDALOTE_ISR isr3() { update(3); }
    static void (*isrFor(int id))() {
        switch (id) {
            case 0: return isr0;
            case 1: return isr1;
            case 2: return isr2;
            default: return isr3;
        }
    }

    static void sendReadTo(uint8_t clientNum, int id, int32_t pos) {
        FrameBuilder fb;
        fb.begin(CMD_ENCODER_READ, DEVICE_ENCODER);
        fb.addInt(id);
        fb.addInt(pos);
        Pardalote.sendFrame(clientNum, fb);
    }

    static void broadcastRead(int id, int32_t pos) {
        FrameBuilder fb;
        fb.begin(CMD_ENCODER_READ, DEVICE_ENCODER);
        fb.addInt(id);
        fb.addInt(pos);
        Pardalote.broadcastFrame(fb);
    }

    static void doAttach(int id, int a, int b) {
        if (_attached[id]) doDetach(id);
        pinMode(a, INPUT_PULLUP);
        pinMode(b, INPUT_PULLUP);
        _pinA[id]  = (int16_t)a;
        _pinB[id]  = (int16_t)b;
        _count[id] = 0;
        _state[id] = (uint8_t)((digitalRead(a) << 1) | digitalRead(b));
        _attached[id] = true;
        attachInterrupt(digitalPinToInterrupt(a), isrFor(id), CHANGE);
        attachInterrupt(digitalPinToInterrupt(b), isrFor(id), CHANGE);
        Serial.print(F("Encoder ")); Serial.print(id);
        Serial.print(F(" attached: A=")); Serial.print(a);
        Serial.print(F(" B=")); Serial.println(b);
    }

    static void doDetach(int id) {
        if (!_attached[id]) return;
        detachInterrupt(digitalPinToInterrupt(_pinA[id]));
        detachInterrupt(digitalPinToInterrupt(_pinB[id]));
        _attached[id] = false;
        _pinA[id] = _pinB[id] = -1;
        ExtReadPoll* p = extPollFind(_polls, MAX_ENCODERS, id);
        if (p) p->instance = -1;   // stop any periodic read
        Serial.print(F("Encoder ")); Serial.print(id);
        Serial.println(F(" detached"));
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_ENCODER);
        fb.addInt(id);
        fb.addString(_names[id]);
        Pardalote.broadcastFrame(fb);
    }

public:
    // -------------------------------------------------------------------
    // Sketch-facing accessors (used by the PardaloteEncoder object).
    // -------------------------------------------------------------------
    static bool attachedId(int id) { return validId(id) && _attached[id]; }
    static int  listAttached(int* out, int max) {
        int n = 0;
        for (int i = 0; i < MAX_ENCODERS && n < max; i++) if (_attached[i]) out[n++] = i;
        return n;
    }

    static int32_t readId(int id) {
        return (validId(id) && _attached[id]) ? _count[id] : 0;
    }

    static void setPositionId(int id, int32_t value) {
        if (!validId(id) || !_attached[id]) return;
        noInterrupts();
        _count[id] = value;
        interrupts();
        broadcastRead(id, value);   // every mirror adopts the new frame
    }

    // -------------------------------------------------------------------
    // Sketch-created encoders — PardaloteEncoder.attach("name", a, b).
    // Same pattern as the other extensions: creation and browser
    // visibility are one act; logical ids allocated TOP-DOWN (browser
    // add() grows from 0 up). Idempotent per name.
    // -------------------------------------------------------------------
    static int sketchAttach(const char* name, int a, int b) {
        if (name == nullptr || name[0] == '\0') return -1;

        int id = -1;
        for (int i = 0; i < MAX_ENCODERS; i++)
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        if (id < 0)
            for (int i = MAX_ENCODERS - 1; i >= 0; i--)
                if (!_attached[i] && !_sketchOwned[i]) { id = i; break; }
        if (id < 0) {
            Serial.print(F("Encoder: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        doAttach(id, a, b);

        // Tell any connected browsers now (announce() covers future connects).
        broadcastShare(id);
        FrameBuilder fa;
        fa.begin(CMD_ENCODER_ATTACH, DEVICE_ENCODER);
        fa.addInt(id); fa.addInt(a); fa.addInt(b);
        Pardalote.broadcastFrame(fa);
        return id;
    }

    // -------------------------------------------------------------------
    // Command handler
    // -------------------------------------------------------------------
    static void handle(uint8_t clientNum,
                       uint8_t cmd, uint16_t typeMask,
                       uint8_t* params, uint8_t nparams,
                       uint8_t* payload, uint16_t payloadLen) {
        if (nparams < 1) return;
        int id = (int)paramInt(params, 0);
        if (!validId(id)) {
            Serial.print(F("Encoder: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            case CMD_ENCODER_ATTACH: {
                if (nparams < 3) return;
                int a = (int)paramInt(params, 1);
                int b = (int)paramInt(params, 2);
                // Skip if already in the requested state — avoids an
                // interrupt detach/attach cycle on JS reconnect.
                if (_attached[id] && _pinA[id] == a && _pinB[id] == b) break;
                doAttach(id, a, b);
                break;
            }

            case CMD_ENCODER_DETACH:
                doDetach(id);
                break;

            // READ — params [id, interval?, threshold?]. Always
            // answers the requester immediately. interval > 0 registers a
            // per-client stream (rate-limited, threshold in raw steps);
            // interval < 0 (JS END) unregisters; absent/0 = one-shot.
            case CMD_ENCODER_READ: {
                long ms  = (nparams > 1) ? paramInt(params, 1) : 0;
                long thr = (nparams > 2) ? paramInt(params, 2) : 0;

                if (ms < 0) {   // END — unregister this client
                    ExtReadPoll* p = extPollFind(_polls, MAX_ENCODERS, id);
                    if (p) p->removeClient(clientNum);
                    break;
                }

                if (!_attached[id]) return;
                const int32_t pos = _count[id];
                sendReadTo(clientNum, id, pos);

                if (ms > 0) {
                    ExtReadPoll* p = extPollGet(_polls, MAX_ENCODERS, id);
                    if (!p) break;
                    p->setClient(clientNum, (uint16_t)constrain(ms, 1, 65535),
                                 thr > 0 ? (uint16_t)thr : DEFAULT_THRESHOLD);
                    p->seed(clientNum, pos, millis());
                }
                break;
            }

            case CMD_ENCODER_SET_POSITION: {
                if (!_attached[id] || nparams < 2) return;
                setPositionId(id, paramInt(params, 1));
                break;
            }

            default:
                Serial.print(F("Encoder: unknown cmd 0x"));
                Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Loop hook — the counter is maintained by the ISRs (looking is
    // free), so every pass just offers the current position through each
    // client's gate: interval = rate limit, threshold in steps. A spin
    // suppressed by the rate limit self-resolves — the next pass after
    // the spacing expires still sees position != lastSent and transmits
    // the latest value.
    // -------------------------------------------------------------------
    static void loop() {
        const unsigned long now = millis();
        for (int i = 0; i < MAX_ENCODERS; i++) {
            ExtReadPoll& p = _polls[i];
            if (p.instance == -1 || !p.clientMask) continue;
            if (!validId(p.instance) || !_attached[p.instance]) { p.instance = -1; continue; }
            const int32_t pos = _count[p.instance];
            for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
                if (p.gate(c, pos, now)) sendReadTo(c, p.instance, pos);
        }
    }

    // Client disconnect — drop its read registrations.
    static void disconnect(uint8_t clientNum) {
        extPollDropClient(_polls, MAX_ENCODERS, clientNum);
    }

    // -------------------------------------------------------------------
    // Announce — extension presence, then each attached encoder's pins
    // and current position, so a connecting browser's mirror seeds
    // without waiting for the knob to move.
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_ENCODER);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_ENCODERS);
        Pardalote.sendFrame(clientNum, fb);

        for (int i = 0; i < MAX_ENCODERS; i++) {
            if (!_attached[i]) continue;

            if (_sketchOwned[i]) {
                FrameBuilder fsh;
                fsh.begin(CMD_SHARE, DEVICE_ENCODER);
                fsh.addInt(i);
                fsh.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fsh);
            }

            FrameBuilder fa;
            fa.begin(CMD_ENCODER_ATTACH, DEVICE_ENCODER);
            fa.addInt(i);
            fa.addInt(_pinA[i]);
            fa.addInt(_pinB[i]);
            Pardalote.sendFrame(clientNum, fa);

            sendReadTo(clientNum, i, _count[i]);
        }
    }
};

// -------------------------------------------------------------------
// PardaloteEncoder — sketch-facing collection of encoders.
//
//   int knob = PardaloteEncoder.attach("knob", 2, 3);
//   long pos = PardaloteEncoder.read(knob);      // raw quadrature steps
//   PardaloteEncoder.setPosition(knob, 0);       // re-zero (echoed to browsers)
// -------------------------------------------------------------------
class PardaloteEncoderAccess {
public:
    // attach(name, pinA, pinB) — create an encoder and make it visible to
    // browsers as arduino.<name>. Returns the logical id, or -1 if full.
    int attach(const char* name, int pinA, int pinB) const {
        return EncoderExt::sketchAttach(name, pinA, pinB);
    }

    int  scan(int* out, int max) const { return EncoderExt::listAttached(out, max); }
    bool attached(int id)        const { return EncoderExt::attachedId(id); }

    // read(id) — current position in raw quadrature steps (KY-040: 4/detent).
    long read(int id) const { return EncoderExt::readId(id); }

    // setPosition(id, value) / zero(id) — re-zero the frame; browsers adopt it.
    void setPosition(int id, long value) const { EncoderExt::setPositionId(id, (int32_t)value); }
    void zero(int id)                    const { EncoderExt::setPositionId(id, 0); }
};
inline PardaloteEncoderAccess PardaloteEncoder;

INSTALL_EXTENSION(DEVICE_ENCODER, EncoderExt::handle, EncoderExt::announce,
                  EncoderExt::disconnect, EncoderExt::loop)

#endif
