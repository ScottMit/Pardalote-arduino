// ==============================================================
// PardaloteIMU.h
// Pardalote Generic IMU Extension
// Part of Pardalote — version in library.properties
// by Scott Mitchell
// GPL-3.0-or-later License
//
// Add to your sketch:
//
//   #include <PardaloteIMU.h>
//
// Supports multiple sensor families through a descriptor table.
// The JS side selects the sensor by passing the model name string in
// the payload of CMD_IMU_ATTACH. See imu.js IMU_MODELS for the JS-side
// list. Row order in SENSORS[] no longer matters — names are matched
// directly, so reordering or inserting rows is safe.
//
// Currently supported sensors (model name in quotes):
//   "6050"     TDK InvenSense MPU-6050   6-DOF   I2C 0x68 / 0x69
//   "6500"     TDK InvenSense MPU-6500   6-DOF   I2C 0x68 / 0x69
//   "9250"     TDK InvenSense MPU-9250   9-DOF*  I2C 0x68 / 0x69
//   "9255"     TDK InvenSense MPU-9255   9-DOF*  I2C 0x68 / 0x69
//   "LSM6DS3"  STMicroelectronics LSM6DS3  6-DOF I2C 0x6A / 0x6B
//   "LSM6DSOX" STMicroelectronics LSM6DSOX 6-DOF I2C 0x6A / 0x6B
//   (* magnetometer not yet implemented — 6-DOF data only)
//
// Adding a new sensor:
//   1. Add scale arrays if the sensor uses different sensitivity values.
//   2. Add a row to SENSORS[] with the correct register addresses,
//      byte offsets, and endianness. The .name field is the wire identifier
//      JS sends — keep it consistent with imu.js IMU_MODELS keys.
//   3. Add a matching entry to IMU_MODELS in imu.js (same key string).
//   Everything else — the protocol, commands, calibration — is generic.
//
// Response data (CMD_IMU_READ):
//   ax, ay, az  in g         (float, calibration-corrected)
//   gx, gy, gz  in °/s       (float, calibration-corrected)
//   temp        in °C         (float, sensor formula applied)
// ==============================================================

#ifndef PARDALOTE_IMU_H
#define PARDALOTE_IMU_H

#include <Wire.h>
#include "Pardalote.h"

#define MAX_IMUS 2

// -------------------------------------------------------------------
// Sensor descriptor — one row per supported sensor in SENSORS[].
//
// To add a sensor, fill in all fields from the datasheet and push
// a new row onto the end of SENSORS[]. Match it with a new entry
// in imu.js IMU_MODELS with code = that row's index.
// -------------------------------------------------------------------
struct SensorDef {
    const char*    name;         // wire identifier (matched against payload of CMD_IMU_ATTACH)
                                 // and used in Serial debug messages

    uint8_t        dof;          // 6 or 9 (9 = has magnetometer, data not yet sent)

    // WHO_AM_I identity check — run once on attach.
    uint8_t        whoReg;       // register address of WHO_AM_I
    uint8_t        whoVal;       // expected value

    // Pre-init write — runs before the WHO_AM_I check.
    // For MPU family: clears SLEEP bit in PWR_MGMT_1 (0x6B → 0x00).
    // For LSM6 family: sets BDU and IF_INC in CTRL3_C (0x12 → 0x44).
    // Set preReg = 0 to skip.
    uint8_t        preReg;
    uint8_t        preVal;
    uint8_t        preDelayMs;   // settling time after the write

    // 14-byte data burst — all sensors pack accel + temp + gyro into
    // a contiguous block readable with a single I2C transaction.
    uint8_t        dataStart;    // first register of the block
    bool           littleEndian; // false = MSB first (MPU), true = LSB first (LSM6)

    // Byte offsets within the 14-byte block for each measurement.
    // Each offset points to the first byte of a 2-byte signed value.
    // For big-endian: first byte = high byte.
    // For little-endian: first byte = low byte.
    uint8_t        axOff, ayOff, azOff;  // accelerometer
    uint8_t        txOff;                // temperature
    uint8_t        gxOff, gyOff, gzOff;  // gyroscope

    // Range configuration registers
    uint8_t        accelCfgReg;
    uint8_t        gyroCfgReg;

    // Scale tables — indexed by range param (0–3).
    // accelFs/gyroFs contain the full byte written to the config register.
    // For MPU family these are pure FS bits; for LSM6 they include ODR bits.
    const float*   accelLsb;   // [4] inverse sensitivity: LSB per g
    const float*   gyroLsb;    // [4] inverse sensitivity: LSB per °/s
    const uint8_t* accelFs;    // [4] byte written to accelCfgReg for each range
    const uint8_t* gyroFs;     // [4] byte written to gyroCfgReg  for each range

    // Temperature formula: temp_°C = raw_int16 / tempDiv + tempOff
    float          tempDiv;
    float          tempOff;
};

// -------------------------------------------------------------------
// Scale and config tables
// -------------------------------------------------------------------

// MPU family — same sensitivity across 6050, 6500, 9250, 9255
inline constexpr float    MPU_A_LSB[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
inline constexpr float    MPU_G_LSB[4] = { 131.0f, 65.5f, 32.8f, 16.4f };
inline constexpr uint8_t  MPU_A_FS[4]  = { 0x00, 0x08, 0x10, 0x18 };   // AFS_SEL bits → ACCEL_CONFIG (0x1C)
inline constexpr uint8_t  MPU_G_FS[4]  = { 0x00, 0x08, 0x10, 0x18 };   // FS_SEL bits  → GYRO_CONFIG  (0x1B)

// LSM6 family — ±2g/4g/8g/16g accel; ±250/500/1000/2000 °/s gyro
// Full register bytes include ODR=416 Hz (bits [7:4] = 0x07) ORed with FS bits.
// Accel CTRL1_XL: FS bits [3:2] → 00=±2g 10=±4g 11=±8g 01=±16g
// Gyro  CTRL2_G : FS bits [3:1] → 000=±250 010=±500 011=±1000 100=±2000
inline constexpr float    LSM_A_LSB[4] = { 16393.0f, 8197.0f, 4098.0f, 2049.0f };
inline constexpr float    LSM_G_LSB[4] = { 114.3f, 57.1f, 28.6f, 14.3f };
inline constexpr uint8_t  LSM_A_FS[4]  = { 0x70, 0x78, 0x7C, 0x74 };   // ODR|FS → CTRL1_XL (0x10)
inline constexpr uint8_t  LSM_G_FS[4]  = { 0x70, 0x74, 0x76, 0x78 };   // ODR|FS → CTRL2_G  (0x11)

// -------------------------------------------------------------------
// Sensor table — each row is one supported sensor.
// The .name field is matched against the model name string the JS side
// sends in the payload of CMD_IMU_ATTACH. Row order is irrelevant.
//
// MPU data block (big-endian, 14 bytes from 0x3B):
//   [0-1]=AX  [2-3]=AY  [4-5]=AZ  [6-7]=TEMP  [8-9]=GX  [10-11]=GY  [12-13]=GZ
//
// LSM6 data block (little-endian, 14 bytes from 0x20):
//   [0-1]=TEMP  [2-3]=GX  [4-5]=GY  [6-7]=GZ  [8-9]=AX  [10-11]=AY  [12-13]=AZ
// -------------------------------------------------------------------
inline constexpr SensorDef SENSORS[] = {
// ── TDK InvenSense MPU family ─────────────────────────────────────────────────────────
//  name         dof  whoReg  whoVal  preReg  preVal  preDelay  dataStart  LE
//               axOff ayOff azOff txOff gxOff gyOff gzOff  accelCfg  gyroCfg
//               accelLsb  gyroLsb  accelFs  gyroFs  tempDiv  tempOff
    { "6050",  6,  0x75, 0x68,   0x6B,  0x00,    10,      0x3B,    false,
      0, 2, 4, 6, 8, 10, 12,   0x1C, 0x1B,
      MPU_A_LSB, MPU_G_LSB, MPU_A_FS, MPU_G_FS,   340.0f, 36.53f },

    { "6500",  6,  0x75, 0x70,   0x6B,  0x00,    10,      0x3B,    false,
      0, 2, 4, 6, 8, 10, 12,   0x1C, 0x1B,
      MPU_A_LSB, MPU_G_LSB, MPU_A_FS, MPU_G_FS,   340.0f, 36.53f },

    { "9250",  9,  0x75, 0x71,   0x6B,  0x00,    10,      0x3B,    false,
      0, 2, 4, 6, 8, 10, 12,   0x1C, 0x1B,
      MPU_A_LSB, MPU_G_LSB, MPU_A_FS, MPU_G_FS,   340.0f, 36.53f },

    { "9255",  9,  0x75, 0x73,   0x6B,  0x00,    10,      0x3B,    false,
      0, 2, 4, 6, 8, 10, 12,   0x1C, 0x1B,
      MPU_A_LSB, MPU_G_LSB, MPU_A_FS, MPU_G_FS,   340.0f, 36.53f },

// ── STMicroelectronics LSM6 family ────────────────────────────────────────────────────
//  name         dof  whoReg  whoVal  preReg  preVal  preDelay  dataStart  LE
//               axOff ayOff azOff txOff gxOff gyOff gzOff  accelCfg  gyroCfg
//               accelLsb  gyroLsb  accelFs  gyroFs  tempDiv  tempOff
    { "LSM6DS3",   6,  0x0F, 0x69,   0x12,  0x44,     5,      0x20,    true,
      8, 10, 12, 0, 2, 4, 6,   0x10, 0x11,
      LSM_A_LSB, LSM_G_LSB, LSM_A_FS, LSM_G_FS,   256.0f, 25.0f },

    { "LSM6DSOX",  6,  0x0F, 0x6C,   0x12,  0x44,     5,      0x20,    true,
      8, 10, 12, 0, 2, 4, 6,   0x10, 0x11,
      LSM_A_LSB, LSM_G_LSB, LSM_A_FS, LSM_G_FS,   256.0f, 25.0f },
};
inline constexpr uint8_t NUM_SENSORS = sizeof(SENSORS) / sizeof(SENSORS[0]);

// One IMU's live reading (returned by PardaloteIMU.read()). Accel in g,
// gyro in °/s, temp in °C. ok = false if the I2C read failed.
struct PardaloteIMUReading {
    float ax, ay, az;
    float gx, gy, gz;
    float temp;
    bool  ok;
};

// -------------------------------------------------------------------
// ImuExt — extension class
// -------------------------------------------------------------------
class ImuExt {
private:
    inline static uint8_t            _addr[MAX_IMUS]       = {};
    inline static uint8_t            _accelRange[MAX_IMUS] = {};
    inline static uint8_t            _gyroRange[MAX_IMUS]  = {};
    inline static bool               _attached[MAX_IMUS]   = {};
    inline static bool               _calibrated[MAX_IMUS] = {};
    inline static const SensorDef*   _def[MAX_IMUS]        = {};   // pointer into SENSORS[]

    // Software calibration offsets applied in readSensor()
    inline static float _calAx[MAX_IMUS] = {};
    inline static float _calAy[MAX_IMUS] = {};
    inline static float _calAz[MAX_IMUS] = {};
    inline static float _calGx[MAX_IMUS] = {};
    inline static float _calGy[MAX_IMUS] = {};
    inline static float _calGz[MAX_IMUS] = {};

    // Sketch-created sensors (PardaloteIMU.attach("name", "6050")). The name is
    // what the browser binds (arduino.<name>); announce() replays a CMD_SHARE
    // frame for these. Browser-created sensors have _sketchOwned = false.
    inline static bool  _sketchOwned[MAX_IMUS] = {};
    inline static char  _names[MAX_IMUS][MAX_SHARE_NAME + 1] = {};

    static bool validId(int id)   { return id >= 0 && id < MAX_IMUS; }

    // Periodic reads — per-client registration. IMU streams
    // are integrated/smoothed in JS, where dropped samples cause drift — so
    // there is NO threshold here: every registered client receives every
    // sample at its own interval (threshold 0 in the shared gate = pass on
    // each interval tick). The win is per-client rates and ONE I2C
    // transaction per poll tick regardless of client count.
    inline static ExtReadPoll _polls[MAX_IMUS] = {};

    // One I2C read, frame filled for sending. False if the read failed.
    static bool buildRead(FrameBuilder& fb, int id) {
        float ax, ay, az, gx, gy, gz, temp;
        if (!readSensor(id, ax, ay, az, gx, gy, gz, temp)) return false;
        fb.begin(CMD_IMU_READ, DEVICE_IMU);
        fb.addInt(id);
        fb.addFloat(ax); fb.addFloat(ay); fb.addFloat(az);
        fb.addFloat(gx); fb.addFloat(gy); fb.addFloat(gz);
        fb.addFloat(temp);
        return true;
    }

    // Look up SENSORS[] by name string (payload is not null-terminated).
    // Returns the index, or -1 if not found.
    static int findSensor(const uint8_t* payload, uint16_t len) {
        if (!payload || len == 0) return -1;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (strlen(SENSORS[i].name) == len &&
                memcmp(SENSORS[i].name, payload, len) == 0) {
                return i;
            }
        }
        return -1;
    }

    // ---- I2C helpers ------------------------------------------------
    static void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    static uint8_t readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t count) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        uint8_t n = (uint8_t)Wire.requestFrom((int)addr, (int)count);
        for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
        return n;
    }

    // Assemble two consecutive bytes into a signed int16.
    // littleEndian = false → buf[0] is high byte (MPU family)
    // littleEndian = true  → buf[0] is low  byte (LSM6 family)
    static inline int16_t get16(const uint8_t* buf, uint8_t off, bool le) {
        return le
            ? (int16_t)((uint16_t)buf[off + 1] << 8 | buf[off])
            : (int16_t)((uint16_t)buf[off]     << 8 | buf[off + 1]);
    }

    // ---- Sensor init ------------------------------------------------
    // Runs the pre-init write, checks WHO_AM_I, and applies initial ranges.
    static bool initDevice(int id) {
        const SensorDef* def  = _def[id];
        const uint8_t    addr = _addr[id];

        // Pre-init write (wake-up or global config enable)
        if (def->preReg != 0) {
            writeReg(addr, def->preReg, def->preVal);
            if (def->preDelayMs) delay(def->preDelayMs);
        }

        // Identity check
        uint8_t who = 0;
        readRegs(addr, def->whoReg, &who, 1);
        if (who != def->whoVal) {
            Serial.print(F("IMU WHO_AM_I=0x")); Serial.print(who, HEX);
            Serial.print(F(", expected 0x"));   Serial.println(def->whoVal, HEX);
            return false;
        }

        // Apply initial range configuration
        writeReg(addr, def->accelCfgReg, def->accelFs[_accelRange[id]]);
        writeReg(addr, def->gyroCfgReg,  def->gyroFs[_gyroRange[id]]);
        return true;
    }

    // Bring up I2C, bind the sensor descriptor, and initialise the device.
    // Shared by the CMD_IMU_ATTACH handler (browser) and sketchAttach (board)
    // so both take the identical path. sda/scl are ESP32-only (ignored else,
    // -1 = board default). Returns false if the device didn't identify.
    static bool attachDevice(int id, uint8_t addr, int code, int sda, int scl) {
#if defined(PLATFORM_ESP32)
        if (sda >= 0 && scl >= 0) { if (!ensureWire(sda, scl)) ensureWire(); }
        else                      ensureWire();
#else
        (void)sda; (void)scl;
        ensureWire();
#endif
        _addr[id]       = addr;
        _def[id]        = &SENSORS[code];
        _accelRange[id] = 0;
        _gyroRange[id]  = 0;
        _calibrated[id] = false;
        _calAx[id] = _calAy[id] = _calAz[id] = 0.0f;
        _calGx[id] = _calGy[id] = _calGz[id] = 0.0f;

        if (!initDevice(id)) return false;
        _attached[id] = true;
        return true;
    }

    // ---- Data read --------------------------------------------------
    // Reads the 14-byte burst block and converts to physical units.
    // Byte offsets and endianness come from the sensor descriptor.
    static bool readSensor(int id,
                           float& ax, float& ay, float& az,
                           float& gx, float& gy, float& gz,
                           float& temp) {
        const SensorDef* def = _def[id];
        uint8_t buf[14];
        if (readRegs(_addr[id], def->dataStart, buf, 14) != 14) return false;

        const bool  le     = def->littleEndian;
        const float aScale = def->accelLsb[_accelRange[id]];
        const float gScale = def->gyroLsb[_gyroRange[id]];

        ax   = (float)get16(buf, def->axOff, le) / aScale - _calAx[id];
        ay   = (float)get16(buf, def->ayOff, le) / aScale - _calAy[id];
        az   = (float)get16(buf, def->azOff, le) / aScale - _calAz[id];
        gx   = (float)get16(buf, def->gxOff, le) / gScale - _calGx[id];
        gy   = (float)get16(buf, def->gyOff, le) / gScale - _calGy[id];
        gz   = (float)get16(buf, def->gzOff, le) / gScale - _calGz[id];
        temp = (float)get16(buf, def->txOff, le) / def->tempDiv + def->tempOff;
        return true;
    }

public:
    // -------------------------------------------------------------------
    // Sketch-facing accessors (used by the PardaloteIMU object).
    // -------------------------------------------------------------------
    static bool attachedId(int id) { return validId(id) && _attached[id]; }
    static int  listAttached(int* out, int max) {
        int n = 0;
        for (int i = 0; i < MAX_IMUS && n < max; i++) if (_attached[i]) out[n++] = i;
        return n;
    }
    // Blocking I2C read of one sensor — sketch-local (does not broadcast).
    static PardaloteIMUReading readData(int id) {
        PardaloteIMUReading r = { 0,0,0, 0,0,0, 0, false };
        if (validId(id) && _attached[id])
            r.ok = readSensor(id, r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.temp);
        return r;
    }

    // -------------------------------------------------------------------
    // Sketch-created IMUs — PardaloteIMU.attach("name", "6050", addr?).
    //
    // Creation and browser visibility are one act (see the servo extension for
    // the rationale). The model NAME rides in a payload, which Pardalote.
    // command() can't carry, so this shares the attach path with the handler
    // via attachDevice() instead of round-tripping a frame. A CMD_SHARE frame
    // (+ attach/range state) is broadcast so browsers materialise arduino.
    // <name>; announce() replays it. Logical ids allocated TOP-DOWN. Idempotent
    // per name. addr = 0 → the model's default (0x68 MPU family, 0x6A LSM6).
    // Returns the logical id, or -1 (bad name / unknown model / no slot / the
    // device didn't identify on the bus).
    // -------------------------------------------------------------------
    static int sketchAttach(const char* name, const char* model,
                            int addr, int sda, int scl) {
        if (name == nullptr || name[0] == '\0') return -1;

        int code = findSensor((const uint8_t*)model, model ? (uint16_t)strlen(model) : 0);
        if (code < 0) {
            Serial.print(F("IMU: unknown model \""));
            if (model) Serial.print(model);
            Serial.println(F("\""));
            return -1;
        }

        int id = -1;
        for (int i = 0; i < MAX_IMUS; i++)
            if (_sketchOwned[i] && strcmp(_names[i], name) == 0) { id = i; break; }
        if (id < 0)
            for (int i = MAX_IMUS - 1; i >= 0; i--)
                if (!_attached[i] && !_sketchOwned[i]) { id = i; break; }
        if (id < 0) {
            Serial.print(F("IMU: no free slot for '"));
            Serial.print(name); Serial.println('\'');
            return -1;
        }

        uint8_t a = (addr > 0) ? (uint8_t)addr
                               : (strncmp(model, "LSM", 3) == 0 ? 0x6A : 0x68);
        if (!attachDevice(id, a, code, sda, scl)) return -1;   // leaves slot free

        strncpy(_names[id], name, MAX_SHARE_NAME);
        _names[id][MAX_SHARE_NAME] = '\0';
        _sketchOwned[id] = true;

        // Tell any connected browsers now (announce() covers future connects):
        // name → attach (model in payload) → ranges.
        broadcastShare(id);
        FrameBuilder fa;
        fa.begin(CMD_IMU_ATTACH, DEVICE_IMU);
        fa.addInt(id); fa.addInt(a); fa.addString(_def[id]->name);
        Pardalote.broadcastFrame(fa);
        FrameBuilder fr;
        fr.begin(CMD_IMU_SET_ACCEL_RANGE, DEVICE_IMU);
        fr.addInt(id); fr.addInt(_accelRange[id]);
        Pardalote.broadcastFrame(fr);
        FrameBuilder fg;
        fg.begin(CMD_IMU_SET_GYRO_RANGE, DEVICE_IMU);
        fg.addInt(id); fg.addInt(_gyroRange[id]);
        Pardalote.broadcastFrame(fg);

        return id;
    }

    static void broadcastShare(int id) {
        FrameBuilder fb;
        fb.begin(CMD_SHARE, DEVICE_IMU);
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
            Serial.print(F("IMU: invalid id ")); Serial.println(id);
            return;
        }

        switch (cmd) {

            // ── Attach ──────────────────────────────────────────────
            // Params: [id, addr, sda?, scl?]
            // Payload: model name string (e.g. "6050", "LSM6DSOX")
            case CMD_IMU_ATTACH: {
                if (nparams < 2) return;
                uint8_t addr = (uint8_t)paramInt(params, 1);

                int code = findSensor(payload, payloadLen);
                if (code < 0) {
                    Serial.print(F("IMU: unknown model \""));
                    if (payload && payloadLen) Serial.write(payload, payloadLen);
                    Serial.println(F("\""));
                    return;
                }

                // Skip if already in the requested state — avoids redundant I2C
                // init traffic and duplicate Serial output on JS reconnect /
                // 'ready' re-attach.
                bool stateChanged = !_attached[id]
                                    || _addr[id] != addr
                                    || _def[id]  != &SENSORS[code];
                if (!stateChanged) break;

                // On ESP32, optional custom SDA/SCL via params[2,3].
                int sda = -1, scl = -1;
#if defined(PLATFORM_ESP32)
                if (nparams >= 4) { sda = (int)paramInt(params, 2); scl = (int)paramInt(params, 3); }
#endif
                if (!attachDevice(id, addr, code, sda, scl)) return;

                Serial.print(F("IMU ")); Serial.print(id);
                Serial.print(F(" attached: ")); Serial.print(_def[id]->name);
                Serial.print(F(" at 0x")); Serial.println(addr, HEX);
                break;
            }

            // ── Detach ──────────────────────────────────────────────
            case CMD_IMU_DETACH: {
                _attached[id]   = false;
                _calibrated[id] = false;
                _def[id]        = nullptr;
                ExtReadPoll* p = extPollFind(_polls, MAX_IMUS, id);
                if (p) p->instance = -1;   // stop any periodic read
                Serial.print(F("IMU ")); Serial.print(id);
                Serial.println(F(" detached"));
                break;
            }

            // ── Read ────────────────────────────────────────────────
            // Params [id, interval?]. Always answers the requester
            // immediately. interval > 0 registers a board-side per-client
            // periodic read (no threshold — see _polls comment); interval < 0
            // (JS END) removes this client's registration; absent/0 = one-shot.
            case CMD_IMU_READ: {
                long ms = (nparams > 1) ? paramInt(params, 1) : 0;

                if (ms < 0) {   // END — unregister this client
                    ExtReadPoll* p = extPollFind(_polls, MAX_IMUS, id);
                    if (p) p->removeClient(clientNum);
                    break;
                }

                if (!_attached[id]) return;
                FrameBuilder fb;
                if (!buildRead(fb, id)) return;
                Pardalote.sendFrame(clientNum, fb);

                if (ms > 0) {
                    ExtReadPoll* p = extPollGet(_polls, MAX_IMUS, id);
                    if (!p) break;
                    p->setClient(clientNum, (uint16_t)constrain(ms, 1, 65535), 0);
                    p->seed(clientNum, 0, millis());
                }
                break;
            }

            // ── Accel range ─────────────────────────────────────────
            case CMD_IMU_SET_ACCEL_RANGE: {
                if (!_attached[id] || nparams < 2) return;
                uint8_t r = (uint8_t)constrain((int)paramInt(params, 1), 0, 3);
                _accelRange[id] = r;
                writeReg(_addr[id], _def[id]->accelCfgReg, _def[id]->accelFs[r]);
                const int gs[] = { 2, 4, 8, 16 };
                Serial.print(F("IMU ")); Serial.print(id);
                Serial.print(F(" accel ±")); Serial.print(gs[r]); Serial.println(F("g"));
                break;
            }

            // ── Gyro range ──────────────────────────────────────────
            case CMD_IMU_SET_GYRO_RANGE: {
                if (!_attached[id] || nparams < 2) return;
                uint8_t r = (uint8_t)constrain((int)paramInt(params, 1), 0, 3);
                _gyroRange[id] = r;
                writeReg(_addr[id], _def[id]->gyroCfgReg, _def[id]->gyroFs[r]);
                const int dps[] = { 250, 500, 1000, 2000 };
                Serial.print(F("IMU ")); Serial.print(id);
                Serial.print(F(" gyro ±")); Serial.print(dps[r]); Serial.println(F("°/s"));
                break;
            }

            // ── Calibrate ───────────────────────────────────────────
            // Place sensor flat, Z-axis pointing UP before calling.
            // After calibration: ax≈0g, ay≈0g, az≈+1g, gx≈gy≈gz≈0.
            // Blocks the loop for approx. samples × 2 ms.
            case CMD_IMU_CALIBRATE: {
                if (!_attached[id]) return;
                int samples = (nparams > 1)
                    ? constrain((int)paramInt(params, 1), 10, 1000)
                    : 200;

                Serial.print(F("IMU ")); Serial.print(id);
                Serial.print(F(": calibrating (")); Serial.print(samples);
                Serial.println(F(" samples — keep flat and still)"));

                _calAx[id] = _calAy[id] = _calAz[id] = 0.0f;
                _calGx[id] = _calGy[id] = _calGz[id] = 0.0f;

                double sAx=0, sAy=0, sAz=0, sGx=0, sGy=0, sGz=0;
                int good = 0;
                for (int i = 0; i < samples; i++) {
                    float ax, ay, az, gx, gy, gz, t;
                    if (readSensor(id, ax, ay, az, gx, gy, gz, t)) {
                        sAx+=ax; sAy+=ay; sAz+=az;
                        sGx+=gx; sGy+=gy; sGz+=gz;
                        good++;
                    }
                    delay(2);
                }
                if (good < 10) {
                    Serial.println(F("IMU: calibration failed — too few good reads"));
                    return;
                }

                _calAx[id] = (float)(sAx / good);
                _calAy[id] = (float)(sAy / good);
                _calAz[id] = (float)(sAz / good) - 1.0f;  // preserve +1g on Z
                _calGx[id] = (float)(sGx / good);
                _calGy[id] = (float)(sGy / good);
                _calGz[id] = (float)(sGz / good);
                _calibrated[id] = true;
                Serial.println(F("IMU: calibration complete"));

                FrameBuilder fb;
                fb.begin(CMD_IMU_CALIBRATE, DEVICE_IMU);
                fb.addInt(id);
                fb.addFloat(_calAx[id]); fb.addFloat(_calAy[id]); fb.addFloat(_calAz[id]);
                fb.addFloat(_calGx[id]); fb.addFloat(_calGy[id]); fb.addFloat(_calGz[id]);
                Pardalote.broadcastFrame(fb);
                break;
            }

            default:
                Serial.print(F("IMU: unknown cmd 0x")); Serial.println(cmd, HEX);
                break;
        }
    }

    // -------------------------------------------------------------------
    // Called on each new client connection
    // -------------------------------------------------------------------
    static void announce(uint8_t clientNum) {
        FrameBuilder fb;
        fb.begin(CMD_ANNOUNCE, DEVICE_IMU);
        fb.addInt(PROTOCOL_VERSION_MAJOR);
        fb.addInt(MAX_IMUS);
        Pardalote.sendFrame(clientNum, fb);

        for (int i = 0; i < MAX_IMUS; i++) {
            if (!_attached[i] || !_def[i]) continue;

            // Sketch-created sensor: send its SHARE frame FIRST so the browser
            // materialises arduino.<name> before the state frames below.
            if (_sketchOwned[i]) {
                FrameBuilder fsh;
                fsh.begin(CMD_SHARE, DEVICE_IMU);
                fsh.addInt(i);
                fsh.addString(_names[i]);
                Pardalote.sendFrame(clientNum, fsh);
            }

            // Re-send attach state. Model name is carried in the payload so
            // the JS side can identify the sensor by its string key.
            FrameBuilder fa;
            fa.begin(CMD_IMU_ATTACH, DEVICE_IMU);
            fa.addInt(i);
            fa.addInt(_addr[i]);
            fa.addString(_def[i]->name);
            Pardalote.sendFrame(clientNum, fa);

            FrameBuilder fr;
            fr.begin(CMD_IMU_SET_ACCEL_RANGE, DEVICE_IMU);
            fr.addInt(i); fr.addInt(_accelRange[i]);
            Pardalote.sendFrame(clientNum, fr);

            FrameBuilder fg;
            fg.begin(CMD_IMU_SET_GYRO_RANGE, DEVICE_IMU);
            fg.addInt(i); fg.addInt(_gyroRange[i]);
            Pardalote.sendFrame(clientNum, fg);

            if (_calibrated[i]) {
                FrameBuilder fc;
                fc.begin(CMD_IMU_CALIBRATE, DEVICE_IMU);
                fc.addInt(i);
                fc.addFloat(_calAx[i]); fc.addFloat(_calAy[i]); fc.addFloat(_calAz[i]);
                fc.addFloat(_calGx[i]); fc.addFloat(_calGy[i]); fc.addFloat(_calGz[i]);
                Pardalote.sendFrame(clientNum, fc);
            }
        }
    }

    // -------------------------------------------------------------------
    // Loop hook — board-side periodic reads. ONE I2C transaction per due
    // registration; every registered client at its own interval (no
    // threshold — see the _polls comment).
    // -------------------------------------------------------------------
    static void loop() {
        const unsigned long now = millis();
        for (int i = 0; i < MAX_IMUS; i++) {
            ExtReadPoll& p = _polls[i];
            if (!p.due(now)) continue;
            if (!validId(p.instance) || !_attached[p.instance]) { p.instance = -1; continue; }
            FrameBuilder fb;
            if (!buildRead(fb, p.instance)) continue;
            for (uint8_t c = 0; c < PARDALOTE_MAX_CLIENTS; c++)
                if (p.gate(c, 0, now)) Pardalote.sendFrame(c, fb);
        }
    }

    // Client disconnect — drop its read registrations.
    static void disconnect(uint8_t clientNum) {
        extPollDropClient(_polls, MAX_IMUS, clientNum);
    }
};

// -------------------------------------------------------------------
// PardaloteIMU — sketch-facing collection of IMUs.
//
// Create an IMU from the sketch (the browser sees it automatically as
// arduino.<name> — a full IMU instance, identical to a browser-created one):
//   int imu = PardaloteIMU.attach("imu", "6050");   // name, model
//   PardaloteIMUReading r = PardaloteIMU.read(imu); // accel g / gyro °·s⁻¹
//   if (r.ok) { float ax = r.ax; ... }
//
// Models: "6050", "6500", "9250", "9255" (0x68 default), "LSM6DS3",
// "LSM6DSOX" (0x6A default). Pass an explicit address for the AD0-HIGH
// variant, and on ESP32 optionally custom SDA/SCL pins. All IMUs share the
// one I2C bus (ensureWire), so — like a Pardalote serial-servo bus — the
// bus itself is Pardalote's; a private IMU means talking to Wire yourself.
// read() does a blocking I2C burst; the browser polls on its own timer.
// -------------------------------------------------------------------
class PardaloteIMUAccess {
public:
    // attach(name, model?, addr?, sda?, scl?) — create an IMU and make it
    // visible to browsers as arduino.<name>. model defaults to "6050"; addr 0
    // → the model's default. Returns the logical id for read()/etc., or -1
    // (unknown model / no slot / the device didn't answer on the bus). Names
    // >MAX_SHARE_NAME (15) are truncated. Idempotent per name.
    int attach(const char* name, const char* model = "6050",
               int addr = 0, int sda = -1, int scl = -1) const {
        return ImuExt::sketchAttach(name, model, addr, sda, scl);
    }

    int  scan(int* out, int max) const { return ImuExt::listAttached(out, max); }
    bool attached(int id)        const { return ImuExt::attachedId(id); }

    // read(id) — one blocking I2C burst → accel (g), gyro (°/s), temp (°C).
    // Check .ok before using the values.
    PardaloteIMUReading read(int id) const { return ImuExt::readData(id); }

    // Ranges: 0..3 → accel ±2/4/8/16 g, gyro ±250/500/1000/2000 °/s.
    void setAccelRange(int id, int range) const { Pardalote.command(DEVICE_IMU, CMD_IMU_SET_ACCEL_RANGE, id, range); }
    void setGyroRange(int id, int range)  const { Pardalote.command(DEVICE_IMU, CMD_IMU_SET_GYRO_RANGE, id, range); }

    // calibrate(id) — hold the sensor flat & still; averages `samples` reads
    // to zero the offsets (blocks ~samples × 2 ms). Echoed to browsers.
    void calibrate(int id, int samples = 200) const { Pardalote.command(DEVICE_IMU, CMD_IMU_CALIBRATE, id, samples); }
};
inline PardaloteIMUAccess PardaloteIMU;

INSTALL_EXTENSION(DEVICE_IMU, ImuExt::handle, ImuExt::announce,
                  ImuExt::disconnect, ImuExt::loop)

#endif
