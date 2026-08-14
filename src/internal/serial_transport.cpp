// ==============================================================
// internal/serial_transport.cpp
// USB-serial transport implementation. See the design notes in
// serial_transport.h.
// ==============================================================

#include "serial_transport.h"

// -------------------------------------------------------------------
// CRC8, polynomial 0x07, init 0x00. Bytewise loop — message sizes are
// small enough that a lookup table isn't worth its 256 bytes.
// -------------------------------------------------------------------
uint8_t PardaloteSerialTransport::_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

void PardaloteSerialTransport::begin(PardaloteSerialMessageSink onMessage,
                                     PardaloteSerialEventSink   onConnect,
                                     PardaloteSerialEventSink   onDisconnect) {
    _onMessage    = onMessage;
    _onConnect    = onConnect;
    _onDisconnect = onDisconnect;
    _state = ST_TEXT;
    _len = 0;
    _overflow  = false;
    _connected = false;
    _listening = false;   // full connected mode — promote out of listen
}

void PardaloteSerialTransport::beginListen(PardaloteSerialMessageSink onListen) {
    _onListen  = onListen;
    _state = ST_TEXT;
    _len = 0;
    _overflow  = false;
    _connected = false;
    _listening = true;
}

void PardaloteSerialTransport::loop(unsigned long now) {
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        _feed((uint8_t)c, now);
    }

    // Liveness: the JS side pings every 3 s once connected (and probes
    // every 500 ms before that), so a long silence means the page is
    // gone or the cable is out.
    if (_connected && now - _lastRx > PARDALOTE_SERIAL_TIMEOUT_MS) {
        _connected = false;
        if (_onDisconnect) _onDisconnect();
    }
}

// Listen-mode drain: decode probes but never connect. The listen sink may
// call begin() to promote to connected mode (a takeover) — that clears
// _listening, so the while-guard exits and leftover bytes are left for the
// connected loop() to re-read from a fresh decoder.
void PardaloteSerialTransport::loopListen(unsigned long now) {
    while (_listening && Serial.available() > 0) {
        int c = Serial.read();
        if (c < 0) break;
        _feed((uint8_t)c, now);
    }
}

void PardaloteSerialTransport::_feed(uint8_t b, unsigned long now) {
    switch (_state) {

        case ST_TEXT:
            // The board's own echo of debug text can't arrive here (we
            // only read what the HOST sent); anything outside an
            // envelope is line noise or a mis-framed remnant — drop it
            // and wait for the next delimiter.
            if (b == 0x00) _state = ST_DELIM;
            break;

        case ST_DELIM:
            if (b == PARDALOTE_SERIAL_MAGIC) {
                _state = ST_FRAME;
                _len = 0;
                _overflow = false;
            } else if (b != 0x00) {
                _state = ST_TEXT;   // back-to-back 0x00s stay in DELIM
            }
            break;

        case ST_FRAME:
            if (b == 0x00) {        // closing delimiter — decode
                _state = ST_DELIM;  // the closer doubles as the next opener —
                                    // set BEFORE decode so a listen-sink switch
                                    // (which re-begin()s and resets _state) wins
                _envelopeDone(now);
            } else if (_len < sizeof(_buf)) {
                _buf[_len++] = b;
            } else {
                _overflow = true;   // swallow to the delimiter, then discard
            }
            break;
    }
}

// COBS-decode _buf in place, check the CRC, deliver. In-place is safe:
// the write index always trails the read index.
void PardaloteSerialTransport::_envelopeDone(unsigned long now) {
    if (_overflow || _len < 2) return;   // minimum: 1 COBS code + CRC

    size_t r = 0, w = 0;
    while (r < _len) {
        uint8_t code = _buf[r++];
        if (code == 0 || r + (size_t)(code - 1) > _len) return;   // malformed
        for (uint8_t i = 1; i < code; i++) _buf[w++] = _buf[r++];
        if (code != 0xFF && r < _len) _buf[w++] = 0x00;
    }
    if (w < 1) return;

    const size_t msgLen = w - 1;                       // last byte is the CRC
    if (_crc8(_buf, msgLen) != _buf[msgLen]) return;   // corrupted — drop, stay synced

    // Listen mode: hand the message to the core's listen handler and never
    // connect. The handler may promote us to connected mode (a takeover).
    if (_listening) {
        if (msgLen > 0 && _onListen) _onListen(_buf, msgLen);
        return;
    }

    _lastRx = now;
    if (!_connected) {
        _connected = true;
        if (_onConnect) _onConnect();
    }
    if (msgLen > 0 && _onMessage) _onMessage(_buf, msgLen);
}

// -------------------------------------------------------------------
// Outbound: 0x00, 0xA5, COBS(data + crc), 0x00 — COBS is generated on
// the fly (scan ahead to the next zero, emit the code, emit the run),
// treating the CRC as a virtual final byte so no send buffer is needed.
// -------------------------------------------------------------------
void PardaloteSerialTransport::send(const uint8_t* data, size_t len) {
    if (!_connected || len == 0) return;
    _writeEnvelope(data, len);
}

// Emit one envelope regardless of _connected — used for the listen-mode
// nudges before any client is committed.
void PardaloteSerialTransport::sendUnconnected(const uint8_t* data, size_t len) {
    if (len == 0) return;
    _writeEnvelope(data, len);
}

void PardaloteSerialTransport::_writeEnvelope(const uint8_t* data, size_t len) {
    const uint8_t crc = _crc8(data, len);
    const size_t  total = len + 1;                       // data + CRC
    auto at = [&](size_t i) -> uint8_t { return i < len ? data[i] : crc; };

    Serial.write((uint8_t)0x00);
    Serial.write((uint8_t)PARDALOTE_SERIAL_MAGIC);

    // Canonical COBS: each block is a code byte (run+1) then the run of
    // non-zero bytes; a code < 0xFF encodes one consumed zero. A message
    // that ends on a zero (or a full 254-run) gets a final empty block
    // (code 0x01) so the decoder reproduces it exactly.
    size_t pos = 0;
    while (true) {
        size_t run = 0;
        while (run < 254 && pos + run < total && at(pos + run) != 0x00) run++;

        Serial.write((uint8_t)(run + 1));                // COBS code
        for (size_t i = 0; i < run; i++) Serial.write(at(pos + i));
        pos += run;

        if (run == 254) {                                // full block — no zero consumed
            if (pos >= total) { Serial.write((uint8_t)0x01); break; }
            continue;
        }
        if (pos >= total) break;                         // ended at the data end
        pos++;                                           // consume the 0x00 this block encodes
        if (pos == total) { Serial.write((uint8_t)0x01); break; }   // trailing zero
    }

    Serial.write((uint8_t)0x00);
}
