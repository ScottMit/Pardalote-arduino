// ==============================================================
// internal/serial_transport.h
// USB-serial transport — the same Pardalote binary frames, carried
// over Serial instead of a WebSocket.
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0 License
//
// A WebSocket gives the protocol two things a raw byte stream does
// not: message boundaries and reliability. This transport restores
// both with an envelope around each message (a message = one frame,
// or one JS batch of frames — exactly the unit a WS binary message
// carried):
//
//   0x00  0xA5  COBS( message bytes + CRC8 )  0x00
//
//   - COBS encoding guarantees the body contains no 0x00, so 0x00 is
//     an unambiguous delimiter and the decoder can ALWAYS resync on
//     the next one — a dropped or corrupted byte loses one message,
//     never the link.
//   - The CRC8 (poly 0x07) rejects corrupted messages.
//   - The 0xA5 marker distinguishes an envelope from debug text.
//
// Debug-text coexistence: Serial is also the sketch's print console.
// ASCII text contains no 0x00, so Serial.print output passes between
// envelopes untouched; the JS side collects it and surfaces it as the
// 'log' event. This is deliberate — students see their Serial.print
// output in the browser without opening the IDE.
//
// Connection semantics (serial has no CONNECTED/DISCONNECTED events):
//   - connect  = first valid inbound envelope (the JS probe);
//   - disconnect = no valid inbound envelope for TIMEOUT_MS. The JS
//     heartbeat pings every 3 s once connected, so a closed page or
//     unplugged cable is declared dead well inside the timeout. This
//     also stops us writing into a port nobody is draining (which can
//     block Serial.write on native-USB boards and stall loop()).
// ==============================================================

#pragma once

#include <Arduino.h>

// Decoded-message capacity — must hold the largest JS batch (a group
// write is a handful of ~24-byte frames; a text message tops out near
// the WS path's limits). Raw COBS input is at most decoded+overhead,
// so the raw accumulator adds a small margin.
#ifndef PARDALOTE_SERIAL_RX_BUF
#define PARDALOTE_SERIAL_RX_BUF 520
#endif

#define PARDALOTE_SERIAL_MAGIC       0xA5
#define PARDALOTE_SERIAL_TIMEOUT_MS  8000UL

typedef void (*PardaloteSerialMessageSink)(uint8_t* data, size_t len);
typedef void (*PardaloteSerialEventSink)();

class PardaloteSerialTransport {
public:
    // Sinks are plain function pointers (same pattern as the WS event
    // trampoline) so this file needs no knowledge of PardaloteClass.
    void begin(PardaloteSerialMessageSink onMessage,
               PardaloteSerialEventSink   onConnect,
               PardaloteSerialEventSink   onDisconnect);

    // Call every loop() pass: drains Serial through the decoder and
    // runs the rx-timeout disconnect check.
    void loop(unsigned long now);

    // Envelope + write one outbound message. COBS is stream-encoded
    // straight to Serial (no send buffer). No-op until connected.
    void send(const uint8_t* data, size_t len);

    bool connected() const { return _connected; }

    // -------- Listen mode (WiFi is the active transport, watching USB) --------
    // A WiFi-active board arms this to sniff USB for a takeover probe WITHOUT
    // committing to serial. Decoded messages go to the listen sink; no
    // connect/disconnect events, no rx-timeout, _connected stays false. The
    // core decides (from the message) whether to switch — at which point it
    // calls begin() to promote to full connected mode.
    void beginListen(PardaloteSerialMessageSink onListen);
    void loopListen(unsigned long now);   // drain + decode, route to listen sink
    bool listening() const { return _listening; }

    // Boot-watch helpers: the WiFi config window (wifi_config.cpp) drains Serial
    // itself (it also watches for the 'w' config key), so it feeds bytes to the
    // listen decoder one at a time rather than calling loopListen(). A completed
    // takeover probe fires the listen sink as usual. decoderInText() is true when
    // the decoder is between envelopes, so a loose 'w' can be told from a 'w'
    // byte inside an envelope body.
    void feedListen(uint8_t b, unsigned long now) { if (_listening) _feed(b, now); }
    bool decoderInText() const { return _state == ST_TEXT; }

    // Emit one envelope while NOT connected — the listen-mode nudges
    // (CMD_SERIAL_BUSY / CMD_AUTH reject). Normal send() no-ops until
    // connected; this bypasses that guard.
    void sendUnconnected(const uint8_t* data, size_t len);

private:
    void _feed(uint8_t b, unsigned long now);
    void _envelopeDone(unsigned long now);
    void _writeEnvelope(const uint8_t* data, size_t len);
    static uint8_t _crc8(const uint8_t* data, size_t len);

    // Decoder states. TEXT swallows the sketch's own debug bytes (the
    // board never receives text from JS — anything outside an envelope
    // is discarded); DELIM has just consumed a 0x00 and decides what
    // the next byte starts; FRAME accumulates COBS bytes to the
    // closing 0x00.
    enum State : uint8_t { ST_TEXT, ST_DELIM, ST_FRAME };

    uint8_t  _state = ST_TEXT;
    uint8_t  _buf[PARDALOTE_SERIAL_RX_BUF];
    size_t   _len  = 0;
    bool     _overflow  = false;
    bool     _connected = false;
    bool     _listening = false;
    unsigned long _lastRx = 0;

    PardaloteSerialMessageSink _onMessage    = nullptr;
    PardaloteSerialEventSink   _onConnect    = nullptr;
    PardaloteSerialEventSink   _onDisconnect = nullptr;
    PardaloteSerialMessageSink _onListen     = nullptr;
};
