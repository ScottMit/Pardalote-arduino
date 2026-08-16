// ==============================================================
// Pardalote.h
// Arduino-side library for the Pardalote project.
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
//
// Minimal sketch:
//   #include <Pardalote.h>
//   void setup() { Pardalote.begin(); }
//   void loop()  { Pardalote.run(); }
//
// Optional extensions:
//   #include <PardaloteServo.h>
//   #include <PardaloteNeoPixel.h>
// ==============================================================

#ifndef PARDALOTE_H
#define PARDALOTE_H

#include <Arduino.h>

#include "internal/platform.h"
#include "internal/defs.h"
#include "internal/protocol.h"
#include "internal/frame_names.h"
#include "internal/extensions.h"
#include "internal/serial_transport.h"
#ifndef PARDALOTE_NO_WIFI
  #include <WebSocketsServer.h>
  #include "internal/wifi_config.h"
#endif

// -------------------------------------------------------------------
// Message channel — a user-defined key/value delivered to watch() /
// onMessage() callbacks. The `type` field says which accessor is valid.
// For TEXT, `text` is a NUL-terminated copy (truncated if very long);
// `length` is the true byte length. For BLOB, `blob`/`length` point
// straight into the receive buffer (valid only during the callback).
// -------------------------------------------------------------------
struct Message {
    const char*    key;
    uint8_t        type;        // MSG_TYPE_*
    int32_t        intValue;    // INT / BOOL / CHAR
    float          floatValue;  // FLOAT
    const char*    text;        // TEXT (NUL-terminated)
    const uint8_t* blob;        // BLOB
    uint16_t       length;      // TEXT / BLOB byte length

    bool  asBool()  const { return intValue != 0; }
    int   asInt()   const { return (int)intValue; }
    float asFloat() const { return floatValue; }
    char  asChar()  const { return (char)intValue; }
};
typedef void (*PardaloteMessageHandler)(const Message&);

// -------------------------------------------------------------------
// Frame monitor — every frame in/out, decoded. `params`/`payload`
// point into the working buffer and are valid only during the callback.
// -------------------------------------------------------------------
enum PardaloteFrameDir { PARDALOTE_FRAME_IN = 0, PARDALOTE_FRAME_OUT = 1 };

struct FrameEvent {
    uint8_t        dir;         // PARDALOTE_FRAME_IN / _OUT
    uint8_t        cmd;
    uint16_t       target;
    uint8_t        nparams;
    uint16_t       typeMask;
    const uint8_t* params;
    uint16_t       payloadLen;
    const uint8_t* payload;
    const char*    name;        // decoded command name, or nullptr
};
typedef void (*PardaloteFrameHandler)(const FrameEvent&);

// -------------------------------------------------------------------
// Compile-time WiFi credentials via secrets.h in the sketch folder.
//
// The sketch folder is on the include path when the .ino compiles,
// but NOT when the library .cpps compile. So __has_include("secrets.h")
// can only succeed in the user's TU. The binder below copies the macros
// into _pardaloteSecrets (declared in wifi_config.h, defined in
// wifi_config.cpp), which wifiConfigConnect() reads at runtime.
// -------------------------------------------------------------------
#if __has_include("secrets.h")
  #include "secrets.h"
#endif

#if defined(SECRET_SSID) && !defined(PARDALOTE_NO_WIFI)
  namespace {
    struct _PardaloteSecretBinder {
        _PardaloteSecretBinder() {
            _pardaloteSecrets.ssid = SECRET_SSID;
            #ifdef SECRET_PASS
              _pardaloteSecrets.pass = SECRET_PASS;
            #else
              _pardaloteSecrets.pass = nullptr;
            #endif
        }
    };
    static _PardaloteSecretBinder _pardalote_secret_binder;
  }
#endif

// -------------------------------------------------------------------
// PardaloteClass
// -------------------------------------------------------------------
class PardaloteClass {
public:
    PardaloteClass();

    // Call from setup(). Three forms:
    //   begin()                  — WiFi + WebSocket server AND listen on USB
    //                              for a deliberate (picker-gesture) takeover:
    //                              when one arrives the board drops WiFi and
    //                              switches to USB (one-way; reboot to return
    //                              to WiFi). The default.
    //   begin(PARDALOTE_WIFI)    — WiFi only; does NOT listen on USB. The
    //                              opt-out for "nobody grabs my board over
    //                              the cable."
    //   begin(PARDALOTE_SERIAL)  — USB serial only, WiFi never started
    //                              (arduino.connectSerial() in the browser).
    // On boards with no radio (UNO R4 Minima) every form starts serial,
    // the only transport the hardware can have.
    void begin();
    void begin(int transport);

    // Optional — call BEFORE begin(). Require a connection key on EITHER
    // transport. Over WiFi it latches against connecting to the wrong board
    // on a shared network; over USB the cable already picks the board, so
    // the key becomes a board-IDENTITY check that catches "you grabbed the
    // wrong board" (wrong key → refused, with a browser-console message).
    // An accident-prevention latch, NOT security: the key crosses the wire
    // in cleartext. Composes with every begin() form.
    void requireKey(const char* key);

    // Call from loop() — services the WebSocket, runs periodic reads,
    // dispatches per-extension housekeeping.
    void run();

    // Used by extensions to push frames back to clients.
    // Phase 5 makes these the only API (the free-function wrappers go away).
    void sendFrame(uint8_t clientNum, FrameBuilder& fb);
    void broadcastFrame(FrameBuilder& fb);

    // Run an extension command locally from the sketch — the same code path a
    // browser command takes. Used by the PardaloteServo / PardaloteStepper
    // write helpers; not usually called directly.
    void command(uint16_t deviceId, uint8_t cmd, int32_t a)                       { int32_t p[1] = { a };       _command(deviceId, cmd, p, 1); }
    void command(uint16_t deviceId, uint8_t cmd, int32_t a, int32_t b)            { int32_t p[2] = { a, b };    _command(deviceId, cmd, p, 2); }
    void command(uint16_t deviceId, uint8_t cmd, int32_t a, int32_t b, int32_t c) { int32_t p[3] = { a, b, c }; _command(deviceId, cmd, p, 3); }
    void command(uint16_t deviceId, uint8_t cmd, int32_t a, int32_t b, int32_t c, int32_t d) { int32_t p[4] = { a, b, c, d }; _command(deviceId, cmd, p, 4); }
    void command(uint16_t deviceId, uint8_t cmd, int32_t a, int32_t b, int32_t c, int32_t d, int32_t e) { int32_t p[5] = { a, b, c, d, e }; _command(deviceId, cmd, p, 5); }
    void command(uint16_t deviceId, uint8_t cmd, int32_t a, int32_t b, int32_t c, int32_t d, int32_t e, int32_t f) { int32_t p[6] = { a, b, c, d, e, f }; _command(deviceId, cmd, p, 6); }

    // Inspection helpers.
    const char* boardName() const { return PARDALOTE_BOARD; }
    bool        anyConnected() const { return _connectedClients != 0; }

    // -----------------------------------------------------------------------
    // Sharing pin state with the browser.
    //
    // share(pin, mode) — tell the browser "this pin exists, it's in this mode."
    //     Accepts Arduino's INPUT / OUTPUT / INPUT_PULLUP / INPUT_PULLDOWN, or
    //     Pardalote's ANALOG_INPUT_MODE. For input modes the JS side will start
    //     polling automatically — so the browser gets values flowing without
    //     having to declare the pin itself.
    //
    // send(pin, value) — push a current value to the browser. JS caches it,
    //     fires arduino.onChange(pin, ...) handlers, makes it available via
    //     arduino.digitalRead(pin) / analogRead(pin).
    //
    // Both calls do NOT manipulate the pin — they only inform the browser.
    // The sketch is still responsible for the actual pinMode / digitalWrite /
    // digitalRead via the standard Arduino API. share() and send() exist
    // purely to keep the browser in sync.
    //
    // See examples/shared-control-example/ and examples/shared-input-example/.
    // -----------------------------------------------------------------------
    // Optional interval (ms) starts board-driven periodic reads of the pin —
    // values flow to every browser with no JS call needed. Optional
    // threshold sets what counts as a meaningful change (0 = board default:
    // 1 for digital pins, the ADC noise floor for analog).
    void share(uint8_t pin, uint8_t mode,
               uint16_t interval = 0, uint16_t threshold = 0);
    void send (uint8_t pin, int     value);

    // -----------------------------------------------------------------------
    // Message channel — send a named value that isn't tied to any pin or
    // device. The key is a string, so these overloads never collide with the
    // pin send(pin, value) above (pins are numeric). Every connected browser
    // receives it (arduino.watch(key) / arduino.on('message')). Pass
    // MSG_FLAG_RETAIN so browsers that connect later get the latest value on
    // connect; MSG_FLAG_BROADCAST is a browser-side flag (see the docs) and
    // is a no-op here (a sketch send already reaches every browser).
    //
    //   Pardalote.send("temp", 22.5);                 // float
    //   Pardalote.send("mode", "idle", MSG_FLAG_RETAIN);
    //   Pardalote.watch("cmd", onCmd);                // handler for one key
    //   Pardalote.onMessage(onAny);                   // handler for all keys
    // -----------------------------------------------------------------------
    void send(const char* key, int         value, uint8_t flags = 0);
    void send(const char* key, double      value, uint8_t flags = 0);
    void send(const char* key, bool        value, uint8_t flags = 0);
    void send(const char* key, char        value, uint8_t flags = 0);
    void send(const char* key, const char* text,  uint8_t flags = 0);
    void sendBlob(const char* key, const uint8_t* data, uint16_t len, uint8_t flags = 0);

    void watch(const char* key, PardaloteMessageHandler cb);
    void onMessage(PardaloteMessageHandler cb);

    // Frame monitor — cb fires for every frame in and out (decoded). Costs
    // nothing until a handler is registered.
    void onFrame(PardaloteFrameHandler cb);

private:
    void _command(uint16_t deviceId, uint8_t cmd, const int32_t* params, uint8_t n);

    static constexpr uint8_t  MAX_WS_CLIENTS  = PARDALOTE_MAX_CLIENTS;
    static constexpr uint32_t HELLO_DELAY_MS  = 50;
    static constexpr int      NUM_ACTIONS     = 20;

    // Message channel capacities.
    static constexpr int      NUM_WATCHERS    = 12;   // watch(key) callbacks
    static constexpr int      NUM_RETAINED    = 8;    // retained keys re-announced on connect
    static constexpr int      RETAIN_VALUE_MAX = 48;  // bytes stored per retained TEXT/BLOB

    // One watched-pin entry per pin. Looking and telling are decoupled
    // (when looking is free, look always; rate-limit the telling):
    //
    //   DIGITAL — watched on every run() pass. A level change (outside a
    //   short bounce lockout) is transmitted IMMEDIATELY to every client
    //   whose threshold it clears — a client's interval does NOT delay
    //   edges (buttons must not lose or lag their presses). Idle pins
    //   transmit nothing.
    //
    //   ANALOG — sampled every ANALOG_SAMPLE_MS (an ADC read has real
    //   cost; the loop also generates stepper pulses). Each client hears
    //   about a change at most once per its interval — the interval is a
    //   per-client minimum spacing between updates, nothing more. A
    //   change on a dormant pin therefore goes out within one sample.
    //
    // threshold 0 on the wire = board default (1 digital, ADC noise
    // floor analog — see defaultThreshold()).
    //
    // Registrations come from two places:
    //   - a browser sends CMD_DIGITAL_READ / CMD_ANALOG_READ
    //     [interval, threshold] → per-client entry, cleared when that
    //     client disconnects or sends CMD_END;
    //   - the sketch calls share(pin, mode, interval, threshold) →
    //     boardOwned, survives disconnects, serves every client that
    //     hasn't registered its own preference.
    struct Action {
        int16_t       id;             // pin number; -1 = empty slot
        uint8_t       cmd;            // CMD_DIGITAL_READ or CMD_ANALOG_READ
        unsigned long lastSample;     // analog: last ADC read
        int32_t       lastLevel;      // digital: last accepted level (-1 = none yet)
        unsigned long lockoutUntil;   // digital: bounce lockout deadline

        // Sketch-side registration (share with an interval).
        bool          boardOwned;
        uint16_t      boardInterval;
        uint16_t      boardThreshold; // resolved (never 0)

        // Per-client registrations. Bit c set = client c registered.
        uint8_t       clientMask;
        uint8_t       seededMask;     // client has received its first value
        uint16_t      interval[MAX_WS_CLIENTS];    // ms — min spacing between updates
        uint16_t      threshold[MAX_WS_CLIENTS];   // resolved (never 0)
        int32_t       lastSent[MAX_WS_CLIENTS];
        unsigned long lastSendTime[MAX_WS_CLIENTS];
    };

    // Digital bounce lockout: the first transition is accepted (and sent)
    // instantly; further transitions are ignored for this long. A change
    // still pending when the lockout expires is picked up then, so even a
    // tap shorter than the lockout delivers both edges.
    static constexpr uint16_t DEBOUNCE_MS = 15;

    // Analog sampling cadence — decoupled from client intervals. Fast
    // enough that a dormant pin's change feels instant; cheap enough
    // (~20–100 µs per read) not to disturb stepper pulse generation.
    static constexpr uint16_t ANALOG_SAMPLE_MS = 10;

    struct Watcher {
        char                    key[MAX_MESSAGE_KEY + 1];
        PardaloteMessageHandler cb;
    };

    struct Retained {
        bool     used = false;
        uint8_t  keyLen;
        char     key[MAX_MESSAGE_KEY + 1];
        uint8_t  type;
        int32_t  intVal;                       // INT / BOOL / CHAR
        float    floatVal;                     // FLOAT
        uint8_t  valueBuf[RETAIN_VALUE_MAX];   // TEXT / BLOB bytes
        uint16_t valueLen;                     // TEXT / BLOB length
    };

#ifndef PARDALOTE_NO_WIFI
    WebSocketsServer _ws{81};
    WifiStore        _wifiStore;
#endif

    // Transport selection — set once in begin(). The serial transport's
    // single client is permanently client 0; all the per-client machinery
    // below serves it as a degenerate case (only bit 0 ever set).
    enum : uint8_t { TRANSPORT_WIFI = 0, TRANSPORT_SERIAL = 1 };
    uint8_t                  _transport = TRANSPORT_WIFI;
    PardaloteSerialTransport _serialT;

    // begin() (default) sets this: WiFi is active but the board also sniffs
    // USB for a takeover probe and switches on a gesture-backed one.
    // begin(PARDALOTE_WIFI) leaves it false (no USB listen).
    bool _serialListen = false;
    bool _begun = false;   // a begin() form has run — requireKey() too late now
    bool _rebootAnnounced = false;   // CMD_REBOOT sent once per boot (see _announceReboot)
    // Listen-window auth state for the prospective serial client (kept apart
    // from _authed[], whose slots belong to WS clients while WiFi is active).
    bool _listenAuthed  = false;   // a matching key arrived during listen
    bool _listenKeyTried = false;  // a (wrong) key was tried during listen

    // Boot-watch: during the "press 'w'" config window we also watch USB for a
    // takeover probe. If one arrives we skip WiFi entirely and go serial — the
    // fast path for the common "board reset onto WiFi, then switch" case (an
    // ESP32 DTR-resets when the port opens). _bootWatch gates _handleListenMessage
    // to set _bootTakeover instead of running the (WiFi-teardown) runtime switch.
    bool _bootWatch    = false;
    bool _bootTakeover = false;

    // Connection key (WebSocket transport only; serial implies physical
    // possession and skips auth). Empty _key = no auth required.
    // Unauthed clients receive nothing (no HELLO, no announce, no
    // broadcasts) and their frames are dropped, except CMD_AUTH.
    static constexpr uint32_t AUTH_TIMEOUT_MS = 3000;
    char     _key[PARDALOTE_KEY_MAX + 1] = {};
    bool     _keyRequired = false;
    bool     _authed[MAX_WS_CLIENTS]       = {};
    uint32_t _authDeadline[MAX_WS_CLIENTS] = {};

    Action   _actions[NUM_ACTIONS];
    uint8_t  _corePinModes[MAX_PIN_NUMBER];
    uint8_t  _corePinValues[MAX_PIN_NUMBER];
    uint8_t  _connectedClients = 0;
    bool     _pendingHello[MAX_WS_CLIENTS] = {};
    uint32_t _helloAfter[MAX_WS_CLIENTS]   = {};

    // Boot id — random 31-bit token generated once in begin(), sent in
    // HELLO. Lets the browser distinguish "board rebooted (possibly new
    // firmware)" from "same board-run, network blip" and drop stale
    // board-originated state on reboot. Never 0 (0 = "unknown" JS-side).
    uint32_t _bootId = 0;

    Watcher  _watchers[NUM_WATCHERS];
    uint8_t  _watcherCount = 0;
    Retained _retained[NUM_RETAINED];
    PardaloteMessageHandler _messageHandler = nullptr;
    PardaloteFrameHandler   _frameHandler   = nullptr;

#ifndef PARDALOTE_NO_WIFI
    // WebSocket library wants a free/static function pointer — this
    // trampoline forwards to the global Pardalote instance.
    static void _wsEventTrampoline(uint8_t num, WStype_t type,
                                   uint8_t* payload, size_t length);

    void _handleWsEvent(uint8_t num, WStype_t type,
                        uint8_t* payload, size_t length);
    void _beginWifi();
    // Runtime WiFi→serial switch (a USB takeover): drop the WS server and the
    // WiFi association (radio stays powered — the sketch may want WiFi), then
    // promote the serial transport to connected mode.
    void _switchToSerial();
    // Listen-mode message handler: a probe arrived on USB while WiFi is active.
    // Decides nudge (stay on WiFi) vs takeover (switch). Never connects a client.
    void _handleListenMessage(uint8_t* data, size_t len);
    void _sendListenFrame(uint8_t cmd, int32_t reason);   // one nudge over USB
    static void _serialListenTrampoline(uint8_t* data, size_t len);
    // Boot-watch byte handler passed to wifiConfigInit (see PardaloteBootProbe).
    int  _handleBootByte(uint8_t b);
    static int _bootProbeByte(uint8_t b);
#endif
    void _beginCommon();
    void _beginSerial();
    // Emit a CMD_REBOOT frame over serial at boot (see CMD_REBOOT in defs.h) so a
    // browser still holding the port resumes probing and recovers fast.
    void _announceReboot();

    // Serial transport sinks (free-function trampolines, same pattern as
    // the WS event trampoline).
    static void _serialMessageTrampoline(uint8_t* data, size_t len);
    static void _serialConnectTrampoline();
    static void _serialDisconnectTrampoline();
    void _onClientConnected(uint8_t num);
    void _onClientDisconnected(uint8_t num);

    // Shared inbound path — one binary message (one frame, or a batch),
    // from either transport.
    void _handleBinary(uint8_t num, uint8_t* payload, size_t length);
    void _handleAuthFrame(uint8_t num, const Frame& f);
    void _rejectClient(uint8_t num, int32_t reason);
    bool _clientReady(uint8_t c) const {
        return (_connectedClients & (1 << c)) && _authed[c];
    }

    // Raw transport write — the ONLY place bytes leave the board.
    void _sendRaw(uint8_t clientNum, uint8_t* buf, size_t len);

    void _handleCoreFrame(uint8_t clientNum, const Frame& f);
    void _pollActions(unsigned long now);
    void _sendReadTo(uint8_t clientNum, int pin, uint8_t cmd, int32_t val);
    void _seedActions(uint8_t clientNum);
    void _sendHello(uint8_t clientNum);
    void _announcePins(uint8_t clientNum);
    void _sendSyncComplete(uint8_t clientNum);
    int  _getSlot(int id);
    void _initAction(int slot, int pin, uint8_t cmd);
    void _registerClientRead(uint8_t clientNum, int pin, uint8_t cmd,
                             uint16_t interval, uint16_t threshold,
                             int32_t seedVal);
    void _unregisterAction(int id);
    void _unregisterClient(uint8_t clientNum);
    void _offerToClients(Action& a, int32_t val, unsigned long now, bool respectSpacing);
    static uint16_t _defaultThreshold(uint8_t cmd);

    // Message channel internals.
    void _emitMessage(uint8_t type, uint8_t flags, const char* key,
                      int32_t intVal, float floatVal,
                      const uint8_t* value, uint16_t valueLen);
    void _buildMessageFrame(FrameBuilder& fb, uint8_t type, uint8_t flags,
                            const char* key, uint8_t keyLen,
                            int32_t intVal, float floatVal,
                            const uint8_t* value, uint16_t valueLen);
    void _handleMessageFrame(uint8_t clientNum, const Frame& f, uint8_t* frameStart);
    void _dispatchMessage(const Message& m);
    void _storeRetained(const char* key, uint8_t keyLen, uint8_t type,
                        int32_t intVal, float floatVal,
                        const uint8_t* value, uint16_t valueLen);
    void _announceMessages(uint8_t clientNum);
    void _emitFrame(uint8_t dir, const Frame& f);
    void _emitFrameOut(uint8_t* buf, size_t len);

    void _platformInit();
    void _platformLoop();
};

extern PardaloteClass Pardalote;

#endif
