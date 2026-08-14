// ==============================================================
// protocol.h
// Pardalote Binary Frame Parsing and Building
// Part of Pardalote — version in library.properties
// ==============================================================
//
// Frame layout (all multi-byte values big-endian):
//
//  Byte 0      CMD          uint8
//  Bytes 1–2   TARGET       uint16   pin number or extension device ID
//  Byte 3      NPARAMS      uint8    number of params (max 16 for typed mask)
//  Bytes 4–5   TYPE_MASK    uint16   bit i=0 → int32, bit i=1 → float32
//  Bytes 6–7   PAYLOAD_LEN  uint16   byte length of trailing payload blob
//  Bytes 8+    PARAMS       NPARAMS × 4 bytes (big-endian int32 or float32)
//  Bytes 8+N*4 PAYLOAD      PAYLOAD_LEN bytes (strings, bitmaps, etc.)
//
// ==============================================================

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

#define FRAME_HEADER_SIZE 8
#define MAX_PARAMS        16

// -------------------------------------------------------------------
// Parsed frame — all pointers into the original WebSocket buffer.
// Do not hold references past the webSocketEvent callback.
// -------------------------------------------------------------------
struct Frame {
    uint8_t  cmd;
    uint16_t target;
    uint8_t  nparams;
    uint16_t typeMask;
    uint16_t payloadLen;
    uint8_t* params;    // NPARAMS * 4 bytes
    uint8_t* payload;   // PAYLOAD_LEN bytes
    size_t   totalLen;  // full byte length of this frame
    bool     valid;
};

// -------------------------------------------------------------------
// Parse one frame from buf at position pos.
// Returns an invalid Frame if the buffer is too short.
// -------------------------------------------------------------------
inline Frame parseFrame(uint8_t* buf, size_t pos, size_t len) {
    Frame f = {};
    if (pos + FRAME_HEADER_SIZE > len) { f.valid = false; return f; }

    f.cmd        = buf[pos];
    f.target     = ((uint16_t)buf[pos + 1] << 8) | buf[pos + 2];
    f.nparams    = buf[pos + 3];
    f.typeMask   = ((uint16_t)buf[pos + 4] << 8) | buf[pos + 5];
    f.payloadLen = ((uint16_t)buf[pos + 6] << 8) | buf[pos + 7];
    f.totalLen   = FRAME_HEADER_SIZE + (size_t)f.nparams * 4 + f.payloadLen;

    if (pos + f.totalLen > len) { f.valid = false; return f; }

    f.params  = buf + pos + FRAME_HEADER_SIZE;
    f.payload = f.params + (size_t)f.nparams * 4;
    f.valid   = true;
    return f;
}

// -------------------------------------------------------------------
// Param accessors — extension handlers call these by index.
// The extension author knows from the protocol definition which
// params are int32 and which are float32.
// -------------------------------------------------------------------
inline int32_t paramInt(uint8_t* p, int i) {
    return ((int32_t)(uint8_t)p[i * 4]     << 24) |
           ((int32_t)(uint8_t)p[i * 4 + 1] << 16) |
           ((int32_t)(uint8_t)p[i * 4 + 2] <<  8) |
            (uint8_t)p[i * 4 + 3];
}

inline float paramFloat(uint8_t* p, int i) {
    uint32_t raw = ((uint32_t)(uint8_t)p[i * 4]     << 24) |
                   ((uint32_t)(uint8_t)p[i * 4 + 1] << 16) |
                   ((uint32_t)(uint8_t)p[i * 4 + 2] <<  8) |
                    (uint8_t)p[i * 4 + 3];
    float f;
    memcpy(&f, &raw, 4);
    return f;
}

inline bool paramIsFloat(uint16_t typeMask, int i) {
    return (typeMask >> i) & 1;
}

// -------------------------------------------------------------------
// FrameBuilder — construct outgoing frames (Arduino → JS).
//
// Usage:
//   FrameBuilder fb;
//   fb.begin(CMD_SERVO_READ, DEVICE_SERVO);
//   fb.addInt(instanceId);
//   fb.addInt(angle);
//   Pardalote.sendFrame(clientNum, fb);
// -------------------------------------------------------------------
class FrameBuilder {
public:
    uint8_t  buf[256];
    uint8_t  nparams    = 0;
    uint16_t typeMask   = 0;
    uint16_t payloadLen = 0;
    bool     valid      = true;   // becomes false on overflow; finish() returns 0

    void begin(uint8_t cmd, uint16_t target) {
        nparams    = 0;
        typeMask   = 0;
        payloadLen = 0;
        valid      = true;
        buf[0] = cmd;
        buf[1] = (target >> 8) & 0xFF;
        buf[2] =  target       & 0xFF;
    }

    bool addInt(int32_t v) {
        if (!valid) return false;
        if (nparams >= MAX_PARAMS) {
            Serial.print(F("[FrameBuilder] addInt: param count exceeds MAX_PARAMS="));
            Serial.println(MAX_PARAMS);
            valid = false; return false;
        }
        if (FRAME_HEADER_SIZE + (nparams + 1) * 4 + payloadLen > sizeof(buf)) {
            Serial.println(F("[FrameBuilder] addInt: buffer overflow"));
            valid = false; return false;
        }
        uint8_t* p = buf + FRAME_HEADER_SIZE + nparams * 4;
        p[0] = (v >> 24) & 0xFF;
        p[1] = (v >> 16) & 0xFF;
        p[2] = (v >>  8) & 0xFF;
        p[3] =  v        & 0xFF;
        nparams++;
        return true;
    }

    bool addFloat(float v) {
        if (!valid) return false;
        if (nparams >= MAX_PARAMS) {
            Serial.print(F("[FrameBuilder] addFloat: param count exceeds MAX_PARAMS="));
            Serial.println(MAX_PARAMS);
            valid = false; return false;
        }
        if (FRAME_HEADER_SIZE + (nparams + 1) * 4 + payloadLen > sizeof(buf)) {
            Serial.println(F("[FrameBuilder] addFloat: buffer overflow"));
            valid = false; return false;
        }
        uint32_t raw;
        memcpy(&raw, &v, 4);
        uint8_t* p = buf + FRAME_HEADER_SIZE + nparams * 4;
        p[0] = (raw >> 24) & 0xFF;
        p[1] = (raw >> 16) & 0xFF;
        p[2] = (raw >>  8) & 0xFF;
        p[3] =  raw        & 0xFF;
        typeMask |= (uint16_t)(1u << nparams);
        nparams++;
        return true;
    }

    // Append a string into the payload section (no null terminator — JS reads payloadLen bytes).
    // Must be called after all addInt / addFloat calls.
    bool addString(const char* s) {
        if (!valid) return false;
        uint16_t len = (uint16_t)strlen(s);
        if ((size_t)FRAME_HEADER_SIZE + nparams * 4 + payloadLen + len > sizeof(buf)) {
            Serial.print(F("[FrameBuilder] addString: buffer overflow ("));
            Serial.print(len); Serial.println(F(" bytes)"));
            valid = false; return false;
        }
        uint8_t* p = buf + FRAME_HEADER_SIZE + nparams * 4 + payloadLen;
        memcpy(p, s, len);
        payloadLen += len;
        return true;
    }

    // Append raw bytes into the payload section (after all addInt/addFloat).
    // Used by the message channel to pack [keyLen][key][value] into the payload.
    bool addBytes(const uint8_t* data, uint16_t len) {
        if (!valid) return false;
        if ((size_t)FRAME_HEADER_SIZE + nparams * 4 + payloadLen + len > sizeof(buf)) {
            Serial.print(F("[FrameBuilder] addBytes: buffer overflow ("));
            Serial.print(len); Serial.println(F(" bytes)"));
            valid = false; return false;
        }
        uint8_t* p = buf + FRAME_HEADER_SIZE + nparams * 4 + payloadLen;
        memcpy(p, data, len);
        payloadLen += len;
        return true;
    }

    // Append a single byte into the payload section.
    bool addByte(uint8_t b) { return addBytes(&b, 1); }

    // Call finish() to write the header fields and get the total frame size.
    // Returns 0 if any addInt/addFloat/addString reported overflow — callers
    // should treat 0 as "do not send."
    size_t finish() {
        if (!valid) return 0;
        buf[3] =  nparams;
        buf[4] = (typeMask   >> 8) & 0xFF;
        buf[5] =  typeMask         & 0xFF;
        buf[6] = (payloadLen >> 8) & 0xFF;
        buf[7] =  payloadLen       & 0xFF;
        return FRAME_HEADER_SIZE + nparams * 4 + payloadLen;
    }
};

// Outgoing frames are sent via Pardalote.sendFrame(num, fb) or
// Pardalote.broadcastFrame(fb) — see Pardalote.h.

#endif
