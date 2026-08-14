// ==============================================================
// Pardalote.cpp
// PardaloteClass implementation.
// ==============================================================

#include "Pardalote.h"
#include "internal/led_matrix.h"

#ifdef PLATFORM_ESP32
#include <esp_system.h>   // esp_random() — hardware RNG for the boot id
#endif

// -------------------------------------------------------------------
// Global singleton — _pardaloteSecrets lives in wifi_config.cpp.
// -------------------------------------------------------------------
PardaloteClass Pardalote;

// -------------------------------------------------------------------
// Constructor
// -------------------------------------------------------------------
PardaloteClass::PardaloteClass() {
    for (int i = 0; i < NUM_ACTIONS; i++) _actions[i].id = -1;
    memset(_corePinModes,  0xFF, sizeof(_corePinModes));
    memset(_corePinValues, 0,    sizeof(_corePinValues));
}

// -------------------------------------------------------------------
// begin() — called from setup(). Three public forms (see Pardalote.h);
// all funnel through _beginCommon() + one transport starter.
// -------------------------------------------------------------------
// begin() is WiFi + a USB listen: WiFi is the active transport, but the
// board also watches USB and switches to it on a deliberate (picker-gesture)
// takeover — one-way, reboot to return to WiFi. begin(PARDALOTE_WIFI) is
// WiFi with the listen off; begin(PARDALOTE_SERIAL) is USB only. One
// exception (Scott's call): on a board with no radio (UNO R4 Minima) every
// form starts serial — the only transport the hardware can have.
void PardaloteClass::begin() {
#ifdef PARDALOTE_NO_WIFI
    _beginSerial();
    Serial.println(F("[Pardalote] (no WiFi on this board — serial transport started)"));
#else
    _serialListen = true;
    _beginWifi();
#endif
}

void PardaloteClass::begin(int transport) {
    if (transport == PARDALOTE_SERIAL) { _beginSerial(); return; }
#ifdef PARDALOTE_NO_WIFI
    _beginSerial();
    Serial.println(F("[Pardalote] (no WiFi on this board — serial transport started)"));
#else
    // PARDALOTE_WIFI = WiFi only, no USB listen. Any other value is treated
    // as the default (listen on) rather than leaving the board unreachable.
    _serialListen = (transport != PARDALOTE_WIFI);
    _beginWifi();
#endif
}

// requireKey() — set BEFORE begin(). Stashes the key; the transport starters
// read _key/_keyRequired. Works on either transport (see the CMD_AUTH note in
// defs.h). Idempotent-ish; truncates over-long keys with a warning.
void PardaloteClass::requireKey(const char* key) {
    if (!key || !key[0]) return;
    if (_begun) {
        Serial.println(F("[Pardalote] requireKey() must be called before begin() — ignored"));
        return;
    }
    if (strlen(key) > PARDALOTE_KEY_MAX)
        Serial.println(F("[Pardalote] connection key truncated to 32 chars"));
    strncpy(_key, key, PARDALOTE_KEY_MAX);
    _key[PARDALOTE_KEY_MAX] = 0;
    _keyRequired = true;
}

// Shared by both transports: Serial console + boot id.
void PardaloteClass::_beginCommon() {
    Serial.begin(115200);

    // Boot id — random 31-bit token, new every boot, sent in HELLO so the
    // browser can tell "board rebooted" from "network blip". Masked to 31
    // bits so it survives the int32 wire param without sign confusion.
    // Only needs to differ between consecutive boots of the same board.
    // (On the WiFi path this runs AFTER the WiFi connect, so micros() has
    // accumulated association/DHCP jitter — the entropy for non-ESP32
    // boards. The serial path has no such jitter to harvest; micros()
    // spread at begin() is smaller there, which is acceptable — a boot-id
    // collision only costs a missed stale-state drop.)
#ifdef PLATFORM_ESP32
    _bootId = esp_random() & 0x7FFFFFFF;
#else
    randomSeed(micros() ^ (millis() << 16));
    _bootId = (uint32_t)random(0x7FFFFFFF);
#endif
    if (_bootId == 0) _bootId = 1;   // 0 is reserved as "unknown" JS-side

    Serial.print(F("Board: "));
    Serial.println(F(PARDALOTE_BOARD));
}

// Emit one CMD_REBOOT frame over serial, once per boot (guarded so the
// boot-takeover path, where _beginWifi then _beginSerial both call this, sends
// it only once). A browser still holding the port from a pre-reset session sees
// it and resumes takeover-probing — the fast recovery from a reset-while-USB.
void PardaloteClass::_announceReboot() {
    if (_rebootAnnounced) return;
    _rebootAnnounced = true;
    FrameBuilder fb;
    fb.begin(CMD_REBOOT, 0x0000);
    size_t n = fb.finish();
    if (n) _serialT.sendUnconnected(fb.buf, n);
}

#ifndef PARDALOTE_NO_WIFI
void PardaloteClass::_beginWifi() {
    _transport = TRANSPORT_WIFI;
    _begun = true;

    Serial.begin(115200);
    _announceReboot();   // tell a browser still holding the port that we rebooted

    // Boot-watch: arm the USB listen so the config window can catch a takeover
    // probe. If the browser opened the port (which DTR-resets an ESP32 into this
    // boot) and is probing with takeover intent, we skip WiFi entirely — no 5 s
    // window wait, no failed-network timeouts, ~1 s to serial instead of ~10.
    _bootWatch = _serialListen;
    _bootTakeover = false;
    if (_bootWatch) _serialT.beginListen(_serialListenTrampoline);

    PardaloteBootProbe probe = _bootWatch ? _bootProbeByte : nullptr;
    wifiConfigInit(_wifiStore, probe);

    if (_bootTakeover) {
        _bootWatch = false;
        Serial.println(F("[Pardalote] USB takeover at boot — skipping WiFi, starting serial"));
        _beginSerial();
        return;
    }

    _platformInit();

    // Keep watching USB through the (blocking) WiFi connect. A takeover during a
    // slow or unreachable connect aborts it and goes serial too — covering the
    // case where the boot-watch window was missed because something else held
    // the port during boot (e.g. the Arduino IDE Serial Monitor). _bootWatch is
    // still set, so _handleListenMessage flags _bootTakeover rather than running
    // the runtime switch (whose WiFi teardown assumes the WS server is up).
    if (!wifiConfigConnect(_wifiStore, probe)) {
        _bootWatch = false;
        WiFi.disconnect();   // drop the half-open association (radio stays on)
        Serial.println(F("[Pardalote] USB takeover during WiFi connect — starting serial"));
        _beginSerial();
        return;
    }
    _bootWatch = false;

#ifdef PLATFORM_UNO_R4
    // WiFiS3 sets WL_CONNECTED before DHCP completes — wait for a real IP
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0)) delay(100);
#endif

    _beginCommon();   // boot id AFTER the WiFi connect (jitter = entropy)

    // Key set via requireKey() before begin() (see the CMD_AUTH note in defs.h).
    if (_keyRequired) Serial.println(F("Connection key required"));

    Serial.print(F("IP: "));
    Serial.println(WiFi.localIP());

    _ws.begin();
    _ws.onEvent(_wsEventTrampoline);
    Serial.println(F("WebSocket server started on port 81"));

#ifdef PLATFORM_ESP32
    WiFi.setSleep(false);   // disable modem sleep — prevents latency on incoming frames
#endif

    // Arm the USB listen AFTER the WiFi bring-up (and its Serial config
    // window) has finished, so nothing else is reading Serial. A takeover
    // probe now switches the board to USB; a plain probe gets a CMD_SERIAL_BUSY
    // nudge. begin(PARDALOTE_WIFI) leaves _serialListen false → no listen.
    if (_serialListen) {
        _serialT.beginListen(_serialListenTrampoline);
        Serial.println(F("Listening on USB — connectSerial() switches the board to serial"));
    }
}
#endif   // PARDALOTE_NO_WIFI

void PardaloteClass::_beginSerial() {
    _transport = TRANSPORT_SERIAL;
    _begun = true;
    _beginCommon();
    _announceReboot();   // no-op if _beginWifi already announced (boot-takeover path)
    // No _platformInit(): the UNO R4 LED matrix scroll exists to show the
    // IP address — there is no IP here, and the rebuild work it does per
    // loop() is exactly what the R4's WebSocket lesson taught us to avoid.
    _serialT.begin(_serialMessageTrampoline,
                   _serialConnectTrampoline,
                   _serialDisconnectTrampoline);
    if (_keyRequired) Serial.println(F("Connection key required"));
    Serial.println(F("Serial transport ready — connect with arduino.connectSerial()"));
}

// -------------------------------------------------------------------
// run() — called from loop()
// -------------------------------------------------------------------
void PardaloteClass::run() {
    if (_transport == TRANSPORT_SERIAL) {
        _serialT.loop(millis());
    } else {
#ifndef PARDALOTE_NO_WIFI
        _ws.loop();
        _platformLoop();
        // Watch USB for a takeover probe (begin() default). A gesture-backed
        // takeover here calls _switchToSerial(), flipping _transport — the
        // next run() pass takes the serial branch above.
        if (_serialListen) _serialT.loopListen(millis());
#endif
    }
    loopAll();

#ifdef PLATFORM_ESP32
    delay(1);   // yield to FreeRTOS idle task — prevents TG0WDT watchdog reset
#endif

    if (!anyConnected()) return;

    unsigned long now = millis();

    // Auth timeout — a connected client that never presented the key
    // (old JS, or a page that wasn't given one) is rejected so it gets a
    // clear reason instead of a silent HELLO that never comes. Applies to
    // both transports now that keys work over USB (a keyed serial client
    // that never sends AUTH is rejected the same way).
    if (_keyRequired) {
        for (int c = 0; c < MAX_WS_CLIENTS; c++) {
            if (!(_connectedClients & (1 << c)) || _authed[c]) continue;
            if ((int32_t)(now - _authDeadline[c]) > 0) _rejectClient(c, 1);
        }
    }

    // Send deferred HELLO + announce to any newly connected client.
    for (int c = 0; c < MAX_WS_CLIENTS; c++) {
        if (!_clientReady(c)) continue;
        if (!_pendingHello[c] || now < _helloAfter[c]) continue;
        _pendingHello[c] = false;
        _sendHello(c);
        _announcePins(c);
        _seedActions(c);   // current value of every polled pin, this client only
        announceAll(c);
        _announceMessages(c);
        _sendSyncComplete(c);
    }

    _pollActions(now);
}

// -------------------------------------------------------------------
// Watched pins. Looking is decoupled from telling (see the Action
// comment in Pardalote.h):
//
//   DIGITAL — read on every pass (a GPIO read is nanoseconds). The
//   first level change is accepted instantly, then a DEBOUNCE_MS
//   lockout swallows contact bounce; a change still pending when the
//   lockout expires is accepted then, so both edges of even a very
//   short tap are delivered. Edges are transmitted immediately —
//   client intervals do not delay them.
//
//   ANALOG — read every ANALOG_SAMPLE_MS. Clients hear about changes
//   through their own gate: at least `interval` ms since that client's
//   last update AND at least `threshold` counts of movement since the
//   value that client last saw. Because sampling is continuous, a
//   change suppressed by the spacing gate is re-offered a sample later
//   and goes out the moment the spacing expires.
// -------------------------------------------------------------------
void PardaloteClass::_pollActions(unsigned long now) {
    for (int i = 0; i < NUM_ACTIONS; i++) {
        Action& a = _actions[i];
        if (a.id == -1) continue;

        if (a.cmd == CMD_DIGITAL_READ) {
            const int32_t val = digitalRead(a.id);
            if (val == a.lastLevel) continue;
            if (a.lastLevel != -1 && now < a.lockoutUntil) continue;  // bouncing
            a.lastLevel    = val;
            a.lockoutUntil = now + DEBOUNCE_MS;
            _offerToClients(a, val, now, false);   // edges bypass spacing
        } else {
            if (now - a.lastSample < ANALOG_SAMPLE_MS) continue;
            a.lastSample = now;
            _offerToClients(a, analogRead(a.id), now, true);
        }
    }
}

// Offer a fresh value to every connected client through its own gate.
// Effective config: the client's own registration, else the sketch's
// share() settings, else the defaults (passive client following someone
// else's watch). `respectSpacing` = false for digital edges.
void PardaloteClass::_offerToClients(Action& a, int32_t val,
                                     unsigned long now, bool respectSpacing) {
    for (uint8_t c = 0; c < MAX_WS_CLIENTS; c++) {
        const uint8_t bit = 1 << c;
        if (!_clientReady(c)) continue;   // connected AND authed

        uint16_t interval, threshold;
        if (a.clientMask & bit) {
            interval  = a.interval[c];
            threshold = a.threshold[c];
        } else if (a.boardOwned) {
            interval  = a.boardInterval;
            threshold = a.boardThreshold;
        } else {
            interval  = 0;
            threshold = _defaultThreshold(a.cmd);
        }

        if (a.seededMask & bit) {
            if (respectSpacing && now - a.lastSendTime[c] < interval) continue;
            int32_t delta = val - a.lastSent[c];
            if (delta < 0) delta = -delta;
            if (delta < threshold) continue;
        }

        _sendReadTo(c, a.id, a.cmd, val);
        a.lastSent[c]     = val;
        a.lastSendTime[c] = now;
        a.seededMask     |= bit;
    }
}

// -------------------------------------------------------------------
// Client lifecycle — shared by both transports. The WS event handler
// and the serial transport's connect/disconnect sinks both land here.
// -------------------------------------------------------------------
void PardaloteClass::_onClientConnected(uint8_t num) {
    if (_connectedClients & (1 << num)) return;    // already connected
    _connectedClients |= (1 << num);
    // A client is authed immediately when no key is required. With a key set,
    // both transports must present it (over USB the key is a board-identity
    // check — see the CMD_AUTH note in defs.h); nothing reaches an unauthed
    // client until AUTH matches.
    _authed[num] = !_keyRequired;
    _authDeadline[num] = millis() + AUTH_TIMEOUT_MS;
    if (_authed[num]) {
        _pendingHello[num] = true;
        _helloAfter[num]   = millis() + HELLO_DELAY_MS;
    }
    Serial.print('['); Serial.print(num); Serial.println(F("] Connected"));
}

void PardaloteClass::_onClientDisconnected(uint8_t num) {
    if (!(_connectedClients & (1 << num))) return;  // already disconnected
    _connectedClients  &= ~(1 << num);
    _pendingHello[num]  = false;
    _authed[num]        = false;
    // Drop this client's read registrations; slots with no remaining
    // registrations are freed. boardOwned actions (sketch share with
    // an interval) survive — the sketch, not a client, owns them.
    _unregisterClient(num);
    disconnectAll(num);
    Serial.print('['); Serial.print(num); Serial.println(F("] Disconnected"));
}

// -------------------------------------------------------------------
// Inbound binary — one message (a frame, or a JS batch of frames),
// from either transport. Unauthed WS clients get exactly one verb:
// CMD_AUTH; everything else is dropped until the key checks out.
// -------------------------------------------------------------------
void PardaloteClass::_handleBinary(uint8_t num, uint8_t* payload, size_t length) {
    size_t pos = 0;
    while (pos < length) {
        Frame f = parseFrame(payload, pos, length);
        if (!f.valid) break;

        if (!_authed[num]) {
            if (f.cmd == CMD_AUTH && f.target < RESERVED_START)
                _handleAuthFrame(num, f);
            pos += f.totalLen;
            continue;
        }

        _emitFrame(PARDALOTE_FRAME_IN, f);   // frame monitor tap

        if (f.cmd == CMD_AUTH) {
            // Authed already (key matched, or none required) — a repeat
            // AUTH is harmless; ignore it.
        } else if (f.cmd == CMD_MESSAGE) {
            // Routed by cmd, not target range — the flags in the
            // target high byte can push it past RESERVED_START.
            _handleMessageFrame(num, f, payload + pos);
        } else if (f.target < RESERVED_START) {
            _handleCoreFrame(num, f);
        } else {
            dispatchExtension(num, f.target, f.cmd, f.typeMask,
                              f.params, f.nparams,
                              f.payload, f.payloadLen);
        }
        pos += f.totalLen;
    }
}

// AUTH: payload is the UTF-8 key. Match → arm the normal HELLO path
// (HELLO is the acceptance; no AUTH reply). Mismatch → reject + close.
void PardaloteClass::_handleAuthFrame(uint8_t num, const Frame& f) {
    const size_t keyLen = strlen(_key);
    if (f.payloadLen == keyLen && keyLen > 0 &&
        memcmp(f.payload, _key, keyLen) == 0) {
        _authed[num]       = true;
        _pendingHello[num] = true;
        _helloAfter[num]   = millis() + HELLO_DELAY_MS;
        Serial.print('['); Serial.print(num); Serial.println(F("] Key accepted"));
    } else {
        _rejectClient(num, 2);
    }
}

// Send CMD_AUTH [reason] (1 = key required, 2 = wrong key), then drop the
// client. Works over both transports (keys work over USB too). The JS side
// surfaces it as the 'authFail' event and stops reconnecting.
void PardaloteClass::_rejectClient(uint8_t num, int32_t reason) {
    FrameBuilder fb;
    fb.begin(CMD_AUTH, 0x0000);
    fb.addInt(reason);
    size_t len = fb.finish();
    // Direct raw send — sendFrame() requires _authed, and a rejected client
    // is by definition not authed. _sendRaw routes to the active transport.
    if (len) _sendRaw(num, fb.buf, len);
    Serial.print('['); Serial.print(num);
    Serial.println(reason == 2 ? F("] Rejected: wrong key") : F("] Rejected: no key presented"));
#ifndef PARDALOTE_NO_WIFI
    if (_transport == TRANSPORT_WIFI) {
        _ws.disconnect(num);                  // fires WStype_DISCONNECTED → cleanup
        return;
    }
#endif
    // Serial: the board can't close the host's port (JS closes it on authFail
    // and stops reconnecting). Clear board-side state so the slot is clean.
    _onClientDisconnected(num);
}

// -------------------------------------------------------------------
// Serial transport sinks — the serial client is permanently client 0.
// -------------------------------------------------------------------
void PardaloteClass::_serialMessageTrampoline(uint8_t* data, size_t len) {
    Pardalote._handleBinary(0, data, len);
}
void PardaloteClass::_serialConnectTrampoline()    { Pardalote._onClientConnected(0); }
void PardaloteClass::_serialDisconnectTrampoline() { Pardalote._onClientDisconnected(0); }

#ifndef PARDALOTE_NO_WIFI
void PardaloteClass::_serialListenTrampoline(uint8_t* data, size_t len) {
    Pardalote._handleListenMessage(data, len);
}

// -------------------------------------------------------------------
// USB listen mode — a WiFi-active board (begin() default) watches USB for a
// deliberate takeover. Only two verbs matter here: CMD_AUTH (the board-
// identity key, when one is required) and CMD_HELLO (the probe, carrying the
// takeover flag in param 0). A plain probe gets a CMD_SERIAL_BUSY nudge and
// the board stays on WiFi; a gesture-backed takeover (with a matching key,
// if required) switches the board to serial. No client is ever "connected"
// here — the switch, then the normal serial handshake, does that.
// -------------------------------------------------------------------
void PardaloteClass::_handleListenMessage(uint8_t* data, size_t len) {
    size_t pos = 0;
    while (pos < len) {
        Frame f = parseFrame(data, pos, len);
        if (!f.valid) break;

        if (f.cmd == CMD_AUTH) {
            const size_t keyLen = strlen(_key);
            if (f.payloadLen == keyLen && keyLen > 0 &&
                memcmp(f.payload, _key, keyLen) == 0) {
                _listenAuthed = true;
            } else {
                _listenKeyTried = true;   // reported against the HELLO below
            }
        } else if (f.cmd == CMD_HELLO) {
            const bool takeover = (f.nparams >= 1 && paramInt(f.params, 0) != 0);
            if (takeover && (!_keyRequired || _listenAuthed)) {
                // Authorised takeover. During the boot window, just flag it so
                // _beginWifi skips WiFi and starts serial (nothing to tear down
                // yet). At runtime, do the full WiFi→serial switch.
                if (_bootWatch) { _bootTakeover = true; return; }
                _switchToSerial();
                return;   // _transport changed; stop draining in listen mode
            }
            // Not an authorised takeover. At runtime, nudge the browser; during
            // the boot window stay silent and let WiFi come up — the runtime
            // listen handles a plain/keyless probe once the board is on WiFi.
            if (!_bootWatch) {
                if (!takeover) _sendListenFrame(CMD_SERIAL_BUSY, 0);      // no gesture
                else           _sendListenFrame(CMD_AUTH, _listenKeyTried ? 2 : 1);  // wrong/absent key
            }
        }
        pos += f.totalLen;
    }
}

// Boot-watch byte handler (see PardaloteBootProbe). Feeds one byte to the USB
// envelope decoder; a completed takeover probe sets _bootTakeover. A loose 'w'
// (between envelopes) is the WiFi config keystroke.
int PardaloteClass::_bootProbeByte(uint8_t b) { return Pardalote._handleBootByte(b); }
// Returns: 2 = USB takeover, 1 = loose 'w' (config key), 3 = other loose text (a
// menu keystroke — the caller uses the raw byte), 0 = a 0x00 delimiter or a byte
// consumed by an in-progress envelope (not menu input).
int PardaloteClass::_handleBootByte(uint8_t b) {
    _serialT.feedListen(b, millis());                  // completed envelope → _handleListenMessage
    if (_bootTakeover)                          return 2;   // USB takeover — skip WiFi
    // A byte that leaves the decoder in TEXT and isn't a 0x00 delimiter is loose
    // input, not envelope data — so it can double as config-menu keystrokes.
    if (b != 0x00 && _serialT.decoderInText())  return (b == 'w') ? 1 : 3;
    return 0;
}

// Emit one framed message over USB while still WiFi-active (not connected as
// a serial client) — the listen-mode nudges.
void PardaloteClass::_sendListenFrame(uint8_t cmd, int32_t reason) {
    FrameBuilder fb;
    fb.begin(cmd, 0x0000);
    if (cmd == CMD_AUTH) fb.addInt(reason);
    size_t n = fb.finish();
    if (n) _serialT.sendUnconnected(fb.buf, n);
}

// A USB takeover was accepted: release WiFi and promote the serial transport.
void PardaloteClass::_switchToSerial() {
    // Drop every WS client cleanly (fires extension disconnect hooks, clears
    // per-client read registrations and auth state).
    for (uint8_t c = 0; c < MAX_WS_CLIENTS; c++)
        if (_connectedClients & (1 << c)) _onClientDisconnected(c);

    _ws.close();          // stop the WebSocket server
    WiFi.disconnect();    // drop the association — radio stays powered so the
                          // sketch can still use WiFi for its own purposes
    Serial.println(F("[Pardalote] USB takeover — WiFi released, switching to serial"));

    _serialListen   = false;
    _listenAuthed   = false;
    _listenKeyTried = false;
    _transport      = TRANSPORT_SERIAL;

    // Full connected serial mode. JS keeps probing (and re-sends AUTH if a key
    // is set), so the normal HELLO→announce→SYNC_COMPLETE handshake runs and
    // client 0 comes up — re-authing through the standard path if keyed.
    _serialT.begin(_serialMessageTrampoline,
                   _serialConnectTrampoline,
                   _serialDisconnectTrampoline);
}


// -------------------------------------------------------------------
// WebSocket trampoline + event handler
// -------------------------------------------------------------------
void PardaloteClass::_wsEventTrampoline(uint8_t num, WStype_t type,
                                        uint8_t* payload, size_t length) {
    Pardalote._handleWsEvent(num, type, payload, length);
}

void PardaloteClass::_handleWsEvent(uint8_t num, WStype_t type,
                                    uint8_t* payload, size_t length) {
    switch (type) {

        // Deduplicate state changes. The WebSocketsServer library on the
        // UNO R4's WiFiS3 stack is known to fire spurious DISCONNECTED
        // events for slots that were never connected (or that we already
        // processed a disconnect for). Processing them re-clears the action
        // table, re-fires extension disconnect hooks, and spams Serial —
        // enough overhead to cause visible lag in NeoPixel updates and
        // servo sweeps. Only act on actual state transitions.
        // (The dedupe now lives in _onClientConnected/_onClientDisconnected,
        // shared with the serial transport's sinks.)
        case WStype_DISCONNECTED:
            _onClientDisconnected(num);
            break;

        case WStype_CONNECTED:
            _onClientConnected(num);
            break;

        case WStype_BIN:
            _handleBinary(num, payload, length);
            break;

        default:
            break;
    }
}
#endif   // PARDALOTE_NO_WIFI

// -------------------------------------------------------------------
// Core frame handler
// -------------------------------------------------------------------
void PardaloteClass::_handleCoreFrame(uint8_t clientNum, const Frame& f) {
    int pin = (int)f.target;

    switch (f.cmd) {

        // JS → board HELLO request. The serial transport has no CONNECTED
        // event, so the browser probes with a HELLO frame after opening the
        // port; answering (re)introduces this board — HELLO + announce +
        // SYNC_COMPLETE — even when the board never noticed the previous
        // page go away (e.g. a reload inside the rx-timeout window).
        // Harmless from a WS client too.
        case CMD_HELLO: {
            if (!_pendingHello[clientNum]) {
                _pendingHello[clientNum] = true;
                _helloAfter[clientNum]   = millis();   // no delay — the link is warm
            }
            break;
        }

        case CMD_PIN_MODE: {
            if (f.nparams < 1) return;
            int pardaloteMode = (int)paramInt(f.params, 0);
            uint8_t arduinoMode;
            switch (pardaloteMode) {
                case MODE_INPUT:          arduinoMode = INPUT;        break;
                case MODE_OUTPUT:         arduinoMode = OUTPUT;       break;
                case MODE_INPUT_PULLUP:   arduinoMode = INPUT_PULLUP; break;
                case ANALOG_INPUT_MODE:   arduinoMode = INPUT;        break;
#ifdef PLATFORM_ESP32
                case MODE_INPUT_PULLDOWN: arduinoMode = INPUT_PULLDOWN; break;
#endif
                default:
                    Serial.print(F("Invalid pinMode: "));
                    Serial.println(pardaloteMode);
                    return;
            }
            pinMode(pin, arduinoMode);
            _unregisterAction(pin);
            if (pin >= 0 && pin < MAX_PIN_NUMBER)
                _corePinModes[pin] = (uint8_t)pardaloteMode;
            break;
        }

        case CMD_DIGITAL_WRITE: {
            if (f.nparams < 1) return;
            int32_t wval = paramInt(f.params, 0);
            digitalWrite(pin, (int)wval);
            if (pin >= 0 && pin < MAX_PIN_NUMBER)
                _corePinValues[pin] = (uint8_t)wval;
            // Echo back to all clients so every browser sees the new state.
            FrameBuilder fb;
            fb.begin(CMD_DIGITAL_WRITE, (uint16_t)pin);
            fb.addInt(wval);
            broadcastFrame(fb);
            break;
        }

        case CMD_ANALOG_WRITE:
            if (f.nparams < 1) return;
            analogWrite(pin, (int)paramInt(f.params, 0));
            break;

        // Read request/registration: params [interval?, threshold?].
        // The requesting client always gets an immediate reading (seeds its
        // mirror); interval > 0 additionally registers a per-client periodic
        // read. threshold 0 / absent = board default.
        case CMD_DIGITAL_READ:
        case CMD_ANALOG_READ: {
            const int32_t val = (f.cmd == CMD_ANALOG_READ)
                                ? analogRead(pin) : digitalRead(pin);
            _sendReadTo(clientNum, pin, f.cmd, val);

            const long ms  = (f.nparams > 0) ? paramInt(f.params, 0) : 0;
            const long thr = (f.nparams > 1) ? paramInt(f.params, 1) : 0;
            if (ms > 0) {
                _registerClientRead(clientNum, pin, f.cmd,
                                    (uint16_t)constrain(ms, 1, 65535),
                                    (uint16_t)constrain(thr, 0, 65535),
                                    val);
            }
            break;
        }

        case CMD_END: {
            // Remove only THIS client's registration; the slot lives on
            // while other clients (or the sketch) still want it.
            for (int i = 0; i < NUM_ACTIONS; i++) {
                Action& a = _actions[i];
                if (a.id != pin) continue;
                a.clientMask &= ~(1 << clientNum);
                if (a.clientMask == 0 && !a.boardOwned) a.id = -1;
                break;
            }
            break;
        }

        case CMD_PING: {
            FrameBuilder fb;
            fb.begin(CMD_PONG, 0x0000);
            sendFrame(clientNum, fb);
            break;
        }
    }
}

// -------------------------------------------------------------------
// Periodic-read plumbing
// -------------------------------------------------------------------

// Default change threshold: any change for digital; the ADC noise floor
// (~0.4% of full range, min 1) for analog. A jittering ADC otherwise
// defeats send-on-change entirely.
uint16_t PardaloteClass::_defaultThreshold(uint8_t cmd) {
    if (cmd != CMD_ANALOG_READ) return 1;
    uint16_t t = (uint16_t)(((1UL << ADC_RESOLUTION_BITS) - 1) >> 8);
    return t > 0 ? t : 1;
}

// Send one reading to one client.
void PardaloteClass::_sendReadTo(uint8_t clientNum, int pin, uint8_t cmd,
                                 int32_t val) {
    FrameBuilder fb;
    fb.begin(cmd, (uint16_t)pin);
    fb.addInt(val);
    sendFrame(clientNum, fb);
}

// Fresh slot: no registrations yet.
void PardaloteClass::_initAction(int slot, int pin, uint8_t cmd) {
    Action& a = _actions[slot];
    a.id             = (int16_t)pin;
    a.cmd            = cmd;
    a.lastSample     = 0;
    a.lastLevel      = -1;   // digital: adopt the first level silently
    a.lockoutUntil   = 0;
    a.boardOwned     = false;
    a.boardInterval  = 0;
    a.boardThreshold = 0;
    a.clientMask     = 0;
    a.seededMask     = 0;
}

// Register (or update) one client's watch on a pin. seedVal is the
// reading just sent to the client, so gating starts from it.
void PardaloteClass::_registerClientRead(uint8_t clientNum, int pin,
                                         uint8_t cmd, uint16_t interval,
                                         uint16_t threshold, int32_t seedVal) {
    int slot = _getSlot(pin);
    if (slot < 0) return;
    Action& a = _actions[slot];

    // New pin, or the pin switched digital<->analog: (re)start clean.
    // A cmd switch invalidates every registration — the JS side always
    // ends the old read before registering the new kind.
    if (a.id == -1 || a.cmd != cmd) _initAction(slot, pin, cmd);
    if (cmd == CMD_DIGITAL_READ && a.lastLevel == -1) a.lastLevel = seedVal;

    const uint8_t bit = 1 << clientNum;
    a.clientMask         |= bit;
    a.interval[clientNum] = interval;
    a.threshold[clientNum] = threshold > 0 ? threshold : _defaultThreshold(cmd);
    a.lastSent[clientNum]     = seedVal;
    a.lastSendTime[clientNum] = millis();
    a.seededMask         |= bit;
}

// Drop every registration a departing client held.
void PardaloteClass::_unregisterClient(uint8_t clientNum) {
    const uint8_t bit = 1 << clientNum;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        Action& a = _actions[i];
        if (a.id == -1) continue;
        a.clientMask &= ~bit;
        a.seededMask &= ~bit;
        if (a.clientMask == 0 && !a.boardOwned) a.id = -1;
    }
}

// Send a newly connected client the current value of every polled pin,
// so its mirror seeds without waiting for a change to occur.
void PardaloteClass::_seedActions(uint8_t clientNum) {
    const uint8_t bit = 1 << clientNum;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        Action& a = _actions[i];
        if (a.id == -1) continue;
        const int32_t val = (a.cmd == CMD_ANALOG_READ)
                            ? analogRead(a.id) : digitalRead(a.id);
        _sendReadTo(clientNum, a.id, a.cmd, val);
        a.lastSent[clientNum]     = val;
        a.lastSendTime[clientNum] = millis();
        a.seededMask             |= bit;
    }
}

// -------------------------------------------------------------------
// HELLO + announce
// -------------------------------------------------------------------
void PardaloteClass::_sendHello(uint8_t clientNum) {
    FrameBuilder fb;
    fb.begin(CMD_HELLO, 0x0000);
    fb.addInt(PROTOCOL_VERSION_MAJOR);
    fb.addInt(PROTOCOL_VERSION_MINOR);
    fb.addInt(ADC_RESOLUTION_BITS);
    fb.addInt((int32_t)_bootId);   // param[3]: boot id (older JS ignores it)
    fb.addString(PARDALOTE_BOARD);
    sendFrame(clientNum, fb);
}

void PardaloteClass::_announcePins(uint8_t clientNum) {
    for (int pin = 0; pin < MAX_PIN_NUMBER; pin++) {
        if (_corePinModes[pin] == 0xFF) continue;

        FrameBuilder fm;
        fm.begin(CMD_PIN_MODE, (uint16_t)pin);
        fm.addInt(_corePinModes[pin]);
        // Board-owned poll (share with interval): include its settings so
        // the connecting browser registers the pin without a READ round trip.
        for (int i = 0; i < NUM_ACTIONS; i++) {
            if (_actions[i].id == pin && _actions[i].boardOwned) {
                fm.addInt(_actions[i].boardInterval);
                fm.addInt(_actions[i].boardThreshold);
                break;
            }
        }
        sendFrame(clientNum, fm);

        if (_corePinModes[pin] == MODE_OUTPUT) {
            FrameBuilder fv;
            fv.begin(CMD_DIGITAL_WRITE, (uint16_t)pin);
            fv.addInt(_corePinValues[pin]);
            sendFrame(clientNum, fv);
        }
    }
}

void PardaloteClass::_sendSyncComplete(uint8_t clientNum) {
    FrameBuilder fb;
    fb.begin(CMD_SYNC_COMPLETE, 0x0000);
    sendFrame(clientNum, fb);
}

// -------------------------------------------------------------------
// Sharing pin state with the browser. See the API comments in
// Pardalote.h for the public-facing contract.
// -------------------------------------------------------------------
void PardaloteClass::share(uint8_t pin, uint8_t mode,
                           uint16_t interval, uint16_t threshold) {
    // Map Arduino's mode constants to Pardalote's protocol modes.
    // INPUT / OUTPUT / INPUT_PULLUP happen to align numerically with
    // MODE_INPUT / MODE_OUTPUT / MODE_INPUT_PULLUP — we map explicitly
    // so the mapping is auditable and any future divergence is caught.
    uint8_t pardaloteMode;
    switch (mode) {
        case INPUT:             pardaloteMode = MODE_INPUT;          break;
        case OUTPUT:            pardaloteMode = MODE_OUTPUT;         break;
        case INPUT_PULLUP:      pardaloteMode = MODE_INPUT_PULLUP;   break;
#if defined(PLATFORM_ESP32) && defined(INPUT_PULLDOWN)
        case INPUT_PULLDOWN:    pardaloteMode = MODE_INPUT_PULLDOWN; break;
#endif
        case ANALOG_INPUT_MODE: pardaloteMode = ANALOG_INPUT_MODE;   break;
        default: return;   // unrecognised — silently skip
    }

    // Cache so future client connects see the right state via announce.
    if (pin < MAX_PIN_NUMBER) _corePinModes[pin] = pardaloteMode;

    // An interval on an input mode registers a board-owned periodic read:
    // the board polls the pin itself and readings flow to every browser,
    // with no JS read call and no round trip. Survives client disconnects.
    const bool inputMode = (pardaloteMode == MODE_INPUT ||
                            pardaloteMode == MODE_INPUT_PULLUP ||
                            pardaloteMode == MODE_INPUT_PULLDOWN ||
                            pardaloteMode == ANALOG_INPUT_MODE);
    if (inputMode && interval > 0) {
        const uint8_t cmd = (pardaloteMode == ANALOG_INPUT_MODE)
                            ? CMD_ANALOG_READ : CMD_DIGITAL_READ;
        int slot = _getSlot(pin);
        if (slot >= 0) {
            Action& a = _actions[slot];
            if (a.id == -1 || a.cmd != cmd) _initAction(slot, pin, cmd);
            a.boardOwned     = true;
            a.boardInterval  = interval;
            a.boardThreshold = threshold > 0 ? threshold : _defaultThreshold(cmd);
        }
    }

    if (!anyConnected()) return;
    FrameBuilder fb;
    fb.begin(CMD_PIN_MODE, (uint16_t)pin);
    fb.addInt(pardaloteMode);
    if (inputMode && interval > 0) {
        // Tell browsers the board is polling — JS registers the pin
        // without sending a READ back. Older JS ignores the extra params.
        fb.addInt(interval);
        fb.addInt(threshold);
    }
    broadcastFrame(fb);
}

void PardaloteClass::send(uint8_t pin, int value) {
    // Cache so future client connects see the right state via announce.
    if (pin < MAX_PIN_NUMBER) _corePinValues[pin] = (uint8_t)value;

    if (!anyConnected()) return;
    FrameBuilder fb;
    fb.begin(CMD_DIGITAL_WRITE, (uint16_t)pin);
    fb.addInt(value);
    broadcastFrame(fb);
}

// ===================================================================
// Message channel — user-defined key/value messages (CMD_MESSAGE).
// ===================================================================

// Public send() overloads. The key is a string, so these never collide
// with send(uint8_t pin, int value) — pins are numeric.
void PardaloteClass::send(const char* key, int value, uint8_t flags) {
    _emitMessage(MSG_TYPE_INT, flags, key, (int32_t)value, 0.0f, nullptr, 0);
}
void PardaloteClass::send(const char* key, double value, uint8_t flags) {
    _emitMessage(MSG_TYPE_FLOAT, flags, key, 0, (float)value, nullptr, 0);
}
void PardaloteClass::send(const char* key, bool value, uint8_t flags) {
    _emitMessage(MSG_TYPE_BOOL, flags, key, value ? 1 : 0, 0.0f, nullptr, 0);
}
void PardaloteClass::send(const char* key, char value, uint8_t flags) {
    _emitMessage(MSG_TYPE_CHAR, flags, key, (int32_t)(uint8_t)value, 0.0f, nullptr, 0);
}
void PardaloteClass::send(const char* key, const char* text, uint8_t flags) {
    _emitMessage(MSG_TYPE_TEXT, flags, key, 0, 0.0f,
                 (const uint8_t*)text, (uint16_t)strlen(text));
}
void PardaloteClass::sendBlob(const char* key, const uint8_t* data, uint16_t len, uint8_t flags) {
    _emitMessage(MSG_TYPE_BLOB, flags, key, 0, 0.0f, data, len);
}

// Build [header][param?][keyLen][key][value?] for a message frame.
void PardaloteClass::_buildMessageFrame(FrameBuilder& fb, uint8_t type, uint8_t flags,
        const char* key, uint8_t keyLen, int32_t intVal, float floatVal,
        const uint8_t* value, uint16_t valueLen) {
    fb.begin(CMD_MESSAGE, MSG_TARGET(type, flags));
    switch (type) {
        case MSG_TYPE_FLOAT: fb.addFloat(floatVal); break;
        case MSG_TYPE_INT:
        case MSG_TYPE_BOOL:
        case MSG_TYPE_CHAR:  fb.addInt(intVal);     break;
        default: break;   // TEXT / BLOB carry no param
    }
    fb.addByte(keyLen);
    fb.addBytes((const uint8_t*)key, keyLen);
    if ((type == MSG_TYPE_TEXT || type == MSG_TYPE_BLOB) && valueLen)
        fb.addBytes(value, valueLen);
}

// Sketch-originated send: retain if asked, then broadcast to every browser.
// A sketch's own watchers do NOT fire (that would echo your own send —
// same rule as pin send() not calling onChange locally).
void PardaloteClass::_emitMessage(uint8_t type, uint8_t flags, const char* key,
        int32_t intVal, float floatVal, const uint8_t* value, uint16_t valueLen) {
    if (!key) return;
    uint8_t keyLen = (uint8_t)strnlen(key, MAX_MESSAGE_KEY);

    if (flags & MSG_FLAG_RETAIN)
        _storeRetained(key, keyLen, type, intVal, floatVal, value, valueLen);

    if (!anyConnected()) return;
    FrameBuilder fb;
    _buildMessageFrame(fb, type, flags, key, keyLen, intVal, floatVal, value, valueLen);
    broadcastFrame(fb);
}

// Browser → board: decode, retain, deliver to watchers, relay if broadcast.
void PardaloteClass::_handleMessageFrame(uint8_t clientNum, const Frame& f, uint8_t* frameStart) {
    if (f.payloadLen < 1) return;
    uint8_t type   = MSG_TYPE(f.target);
    uint8_t flags  = MSG_FLAGS(f.target);
    uint8_t keyLen = f.payload[0];
    if ((uint16_t)1 + keyLen > f.payloadLen) return;

    const uint8_t* keyPtr = f.payload + 1;
    const uint8_t* valPtr = keyPtr + keyLen;
    uint16_t       valLen = f.payloadLen - 1 - keyLen;

    char keyBuf[MAX_MESSAGE_KEY + 1];
    uint8_t kl = keyLen > MAX_MESSAGE_KEY ? MAX_MESSAGE_KEY : keyLen;
    memcpy(keyBuf, keyPtr, kl);
    keyBuf[kl] = 0;

    Message m = {};
    m.key  = keyBuf;
    m.type = type;

    char textBuf[128];
    switch (type) {
        case MSG_TYPE_FLOAT:
            m.floatValue = (f.nparams > 0) ? paramFloat(f.params, 0) : 0.0f;
            break;
        case MSG_TYPE_INT:
        case MSG_TYPE_BOOL:
        case MSG_TYPE_CHAR:
            m.intValue = (f.nparams > 0) ? paramInt(f.params, 0) : 0;
            break;
        case MSG_TYPE_TEXT: {
            uint16_t n = valLen < sizeof(textBuf) - 1 ? valLen : sizeof(textBuf) - 1;
            memcpy(textBuf, valPtr, n);
            textBuf[n] = 0;
            m.text   = textBuf;
            m.length = valLen;
            break;
        }
        case MSG_TYPE_BLOB:
            m.blob   = valPtr;
            m.length = valLen;
            break;
    }

    if (flags & MSG_FLAG_RETAIN)
        _storeRetained(keyBuf, kl, type, m.intValue, m.floatValue, valPtr, valLen);

    _dispatchMessage(m);

    // Relay to the OTHER browsers (the board is the hub). Send the exact
    // received bytes; receivers process it as an ordinary inbound message.
    // (No-op on the serial transport — there are no other browsers.)
    if (flags & MSG_FLAG_BROADCAST) {
        for (int c = 0; c < MAX_WS_CLIENTS; c++) {
            if (c != clientNum && _clientReady((uint8_t)c))
                _sendRaw((uint8_t)c, frameStart, f.totalLen);
        }
    }
}

void PardaloteClass::_dispatchMessage(const Message& m) {
    for (uint8_t i = 0; i < _watcherCount; i++) {
        if (_watchers[i].cb && strncmp(_watchers[i].key, m.key, MAX_MESSAGE_KEY) == 0)
            _watchers[i].cb(m);
    }
    if (_messageHandler) _messageHandler(m);
}

void PardaloteClass::_storeRetained(const char* key, uint8_t keyLen, uint8_t type,
        int32_t intVal, float floatVal, const uint8_t* value, uint16_t valueLen) {
    bool scalar = (type != MSG_TYPE_TEXT && type != MSG_TYPE_BLOB);
    if (!scalar && valueLen > RETAIN_VALUE_MAX) {
        Serial.print(F("[Pardalote] retain: value too large for key '"));
        Serial.print(key); Serial.println(F("' — not stored"));
        return;
    }

    Retained* slot = nullptr;
    for (int i = 0; i < NUM_RETAINED; i++)
        if (_retained[i].used && strncmp(_retained[i].key, key, MAX_MESSAGE_KEY) == 0) { slot = &_retained[i]; break; }
    if (!slot)
        for (int i = 0; i < NUM_RETAINED; i++)
            if (!_retained[i].used) { slot = &_retained[i]; break; }
    if (!slot) { Serial.println(F("[Pardalote] retain table full")); return; }

    slot->used = true;
    memcpy(slot->key, key, keyLen);
    slot->key[keyLen] = 0;
    slot->keyLen   = keyLen;
    slot->type     = type;
    slot->intVal   = intVal;
    slot->floatVal = floatVal;
    if (scalar) {
        slot->valueLen = 0;
    } else {
        memcpy(slot->valueBuf, value, valueLen);
        slot->valueLen = valueLen;
    }
}

void PardaloteClass::_announceMessages(uint8_t clientNum) {
    for (int i = 0; i < NUM_RETAINED; i++) {
        Retained& r = _retained[i];
        if (!r.used) continue;
        FrameBuilder fb;
        _buildMessageFrame(fb, r.type, MSG_FLAG_RETAIN, r.key, r.keyLen,
                           r.intVal, r.floatVal, r.valueBuf, r.valueLen);
        sendFrame(clientNum, fb);
    }
}

void PardaloteClass::watch(const char* key, PardaloteMessageHandler cb) {
    for (uint8_t i = 0; i < _watcherCount; i++)
        if (strncmp(_watchers[i].key, key, MAX_MESSAGE_KEY) == 0) { _watchers[i].cb = cb; return; }
    if (_watcherCount >= NUM_WATCHERS) { Serial.println(F("[Pardalote] watch table full")); return; }
    strncpy(_watchers[_watcherCount].key, key, MAX_MESSAGE_KEY);
    _watchers[_watcherCount].key[MAX_MESSAGE_KEY] = 0;
    _watchers[_watcherCount].cb = cb;
    _watcherCount++;
}

void PardaloteClass::onMessage(PardaloteMessageHandler cb) { _messageHandler = cb; }
void PardaloteClass::onFrame(PardaloteFrameHandler cb)     { _frameHandler   = cb; }

// Frame monitor delivery.
void PardaloteClass::_emitFrame(uint8_t dir, const Frame& f) {
    if (!_frameHandler) return;
    FrameEvent ev;
    ev.dir        = dir;
    ev.cmd        = f.cmd;
    ev.target     = f.target;
    ev.nparams    = f.nparams;
    ev.typeMask   = f.typeMask;
    ev.params     = f.params;
    ev.payloadLen = f.payloadLen;
    ev.payload    = f.payload;
    ev.name       = pardaloteFrameName(f.target, f.cmd);
    _frameHandler(ev);
}

void PardaloteClass::_emitFrameOut(uint8_t* buf, size_t len) {
    if (!_frameHandler) return;
    Frame f = parseFrame(buf, 0, len);
    if (f.valid) _emitFrame(PARDALOTE_FRAME_OUT, f);
}

// -------------------------------------------------------------------
// Public send methods — extensions call Pardalote.sendFrame /
// Pardalote.broadcastFrame from phase 5 onward.
// -------------------------------------------------------------------
void PardaloteClass::sendFrame(uint8_t clientNum, FrameBuilder& fb) {
    if (clientNum >= MAX_WS_CLIENTS) return;   // loopback client (sketch command) has no socket
    if (!_authed[clientNum]) return;           // nothing reaches an unauthed client
    size_t len = fb.finish();
    if (len == 0) return;
    _emitFrameOut(fb.buf, len);
    _sendRaw(clientNum, fb.buf, len);
}

// The ONLY place bytes leave the board — routes to the active transport.
void PardaloteClass::_sendRaw(uint8_t clientNum, uint8_t* buf, size_t len) {
    if (_transport == TRANSPORT_SERIAL) {
        if (clientNum == 0) _serialT.send(buf, len);
        return;
    }
#ifndef PARDALOTE_NO_WIFI
    _ws.sendBIN(clientNum, buf, len);
#endif
}

// -------------------------------------------------------------------
// Local command dispatch — a sketch drives an extension through the same
// handler the browser uses. Params are int32, packed big-endian, then run
// against the extension with a loopback client id (>= MAX_WS_CLIENTS, so
// any reply is dropped by sendFrame; command frames don't reply anyway).
// -------------------------------------------------------------------
void PardaloteClass::_command(uint16_t deviceId, uint8_t cmd, const int32_t* params, uint8_t n) {
    if (n > MAX_PARAMS) n = MAX_PARAMS;
    uint8_t buf[MAX_PARAMS * 4];
    for (uint8_t i = 0; i < n; i++) {
        int32_t p = params[i];
        buf[i * 4]     = (p >> 24) & 0xFF;
        buf[i * 4 + 1] = (p >> 16) & 0xFF;
        buf[i * 4 + 2] = (p >>  8) & 0xFF;
        buf[i * 4 + 3] =  p        & 0xFF;
    }
    dispatchExtension(0xFF, deviceId, cmd, 0, buf, n, nullptr, 0);
}

void PardaloteClass::broadcastFrame(FrameBuilder& fb) {
    size_t len = fb.finish();
    if (len == 0) return;
    _emitFrameOut(fb.buf, len);
    for (int c = 0; c < MAX_WS_CLIENTS; c++) {
        if (_clientReady((uint8_t)c))
            _sendRaw((uint8_t)c, fb.buf, len);
    }
}

// -------------------------------------------------------------------
// Action registry helpers
// -------------------------------------------------------------------
int PardaloteClass::_getSlot(int id) {
    int empty = -1;
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (_actions[i].id == id)  return i;
        if (empty == -1 && _actions[i].id == -1) empty = i;
    }
    if (empty == -1) Serial.println(F("Action table full"));
    return empty;
}

void PardaloteClass::_unregisterAction(int id) {
    for (int i = 0; i < NUM_ACTIONS; i++) {
        if (_actions[i].id == id) { _actions[i].id = -1; return; }
    }
}

// -------------------------------------------------------------------
// Platform init / loop
// -------------------------------------------------------------------
void PardaloteClass::_platformInit() {
    ledMatrixBegin();
}

void PardaloteClass::_platformLoop() {
    ledMatrixLoop(anyConnected());
}
