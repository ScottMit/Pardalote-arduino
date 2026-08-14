// ==============================================================
// frame_names.h
// Best-effort (deviceId/target, cmd) → human-readable command name.
// Used by the frame monitor (Pardalote.onFrame) so traffic reads as
// "SERVO_WRITE" rather than "0x16". Returns nullptr for anything not
// in the table — the caller falls back to a hex label.
//
// This table is maintained BY HAND alongside defs.h. When you add a
// command there, add its name here (and to _FRAME_NAMES in pardalote.js).
// ==============================================================

#ifndef FRAME_NAMES_H
#define FRAME_NAMES_H

#include <Arduino.h>
#include "defs.h"

inline const char* pardaloteFrameName(uint16_t target, uint8_t cmd) {
    // Routed by cmd regardless of target — check these first.
    if (cmd == CMD_MESSAGE) return "MESSAGE";
    if (cmd == CMD_SHARE)   return "SHARE";

    // Core frames (pin number or 0).
    if (target < RESERVED_START) {
        switch (cmd) {
            case CMD_HELLO:         return "HELLO";
            case CMD_ANNOUNCE:      return "ANNOUNCE";
            case CMD_PIN_MODE:      return "PIN_MODE";
            case CMD_DIGITAL_WRITE: return "DIGITAL_WRITE";
            case CMD_DIGITAL_READ:  return "DIGITAL_READ";
            case CMD_ANALOG_WRITE:  return "ANALOG_WRITE";
            case CMD_ANALOG_READ:   return "ANALOG_READ";
            case CMD_END:           return "END";
            case CMD_PING:          return "PING";
            case CMD_PONG:          return "PONG";
            case CMD_SYNC_COMPLETE: return "SYNC_COMPLETE";
            case CMD_AUTH:          return "AUTH";
            case CMD_SERIAL_BUSY:   return "SERIAL_BUSY";
            case CMD_REBOOT:        return "REBOOT";
            default:                return nullptr;
        }
    }

    // Extension frames — scoped by device id.
    switch (target) {
        case DEVICE_NEO_PIXEL:
            switch (cmd) {
                case CMD_NEO_INIT:       return "NEO_INIT";
                case CMD_NEO_SET_PIXEL:  return "NEO_SET_PIXEL";
                case CMD_NEO_FILL:       return "NEO_FILL";
                case CMD_NEO_CLEAR:      return "NEO_CLEAR";
                case CMD_NEO_BRIGHTNESS: return "NEO_BRIGHTNESS";
                case CMD_NEO_SHOW:       return "NEO_SHOW";
            }
            break;
        case DEVICE_SERVO:
            switch (cmd) {
                case CMD_SERVO_ATTACH:             return "SERVO_ATTACH";
                case CMD_SERVO_DETACH:             return "SERVO_DETACH";
                case CMD_SERVO_WRITE:              return "SERVO_WRITE";
                case CMD_SERVO_WRITE_MICROSECONDS: return "SERVO_WRITE_MICROSECONDS";
                case CMD_SERVO_READ:               return "SERVO_READ";
                case CMD_SERVO_ATTACHED:           return "SERVO_ATTACHED";
                case CMD_SERVO_WRITE_TIMED:        return "SERVO_WRITE_TIMED";
                case CMD_SERVO_SYNC_TIMED:         return "SERVO_SYNC_TIMED";
                case CMD_SERVO_STOP:               return "SERVO_STOP";
                case CMD_SERVO_DONE:               return "SERVO_DONE";
                case CMD_SERVO_SET_LIMITS:         return "SERVO_SET_LIMITS";
            }
            break;
        case DEVICE_ULTRASONIC:
            switch (cmd) {
                case CMD_ULTRASONIC_ATTACH:      return "ULTRASONIC_ATTACH";
                case CMD_ULTRASONIC_DETACH:      return "ULTRASONIC_DETACH";
                case CMD_ULTRASONIC_READ:        return "ULTRASONIC_READ";
                case CMD_ULTRASONIC_SET_TIMEOUT: return "ULTRASONIC_SET_TIMEOUT";
            }
            break;
        case DEVICE_IMU:
            switch (cmd) {
                case CMD_IMU_ATTACH:          return "IMU_ATTACH";
                case CMD_IMU_DETACH:          return "IMU_DETACH";
                case CMD_IMU_READ:            return "IMU_READ";
                case CMD_IMU_SET_ACCEL_RANGE: return "IMU_SET_ACCEL_RANGE";
                case CMD_IMU_SET_GYRO_RANGE:  return "IMU_SET_GYRO_RANGE";
                case CMD_IMU_CALIBRATE:       return "IMU_CALIBRATE";
            }
            break;
        case DEVICE_CAMERA:
            switch (cmd) {
                case CMD_CAMERA_INIT:        return "CAMERA_INIT";
                case CMD_CAMERA_SET_RES:     return "CAMERA_SET_RES";
                case CMD_CAMERA_SET_QUALITY: return "CAMERA_SET_QUALITY";
            }
            break;
        case DEVICE_STEPPER:
            switch (cmd) {
                case CMD_STEPPER_ATTACH:        return "STEPPER_ATTACH";
                case CMD_STEPPER_DETACH:        return "STEPPER_DETACH";
                case CMD_STEPPER_MOVE_TO:       return "STEPPER_MOVE_TO";
                case CMD_STEPPER_MOVE:          return "STEPPER_MOVE";
                case CMD_STEPPER_SET_MAX_SPEED: return "STEPPER_SET_MAX_SPEED";
                case CMD_STEPPER_SET_ACCEL:     return "STEPPER_SET_ACCEL";
                case CMD_STEPPER_RUN_SPEED:     return "STEPPER_RUN_SPEED";
                case CMD_STEPPER_STOP:          return "STEPPER_STOP";
                case CMD_STEPPER_SET_POSITION:  return "STEPPER_SET_POSITION";
                case CMD_STEPPER_ENABLE:        return "STEPPER_ENABLE";
                case CMD_STEPPER_SET_LIMITS:    return "STEPPER_SET_LIMITS";
                case CMD_STEPPER_READ:          return "STEPPER_READ";
                case CMD_STEPPER_DONE:          return "STEPPER_DONE";
                case CMD_STEPPER_HOME:          return "STEPPER_HOME";
                case CMD_STEPPER_MOVE_TIMED:    return "STEPPER_MOVE_TIMED";
                case CMD_STEPPER_SYNC_MOVE:     return "STEPPER_SYNC_MOVE";
                case CMD_STEPPER_SET_SWITCH:    return "STEPPER_SET_SWITCH";
                case CMD_STEPPER_LIMIT:         return "STEPPER_LIMIT";
                case CMD_STEPPER_SET_SWITCH_POS: return "STEPPER_SET_SWITCH_POS";
                case CMD_STEPPER_SET_HOME:      return "STEPPER_SET_HOME";
                case CMD_STEPPER_HARD_STOP:     return "STEPPER_HARD_STOP";
            }
            break;
        case DEVICE_BUSSERVO:
            switch (cmd) {
                case CMD_BUSSERVO_BUS_CONFIG:  return "BUSSERVO_BUS_CONFIG";
                case CMD_BUSSERVO_ATTACH:      return "BUSSERVO_ATTACH";
                case CMD_BUSSERVO_DETACH:      return "BUSSERVO_DETACH";
                case CMD_BUSSERVO_WRITE:       return "BUSSERVO_WRITE";
                case CMD_BUSSERVO_WRITE_SPEED: return "BUSSERVO_WRITE_SPEED";
                case CMD_BUSSERVO_SET_MODE:    return "BUSSERVO_SET_MODE";
                case CMD_BUSSERVO_TORQUE:      return "BUSSERVO_TORQUE";
                case CMD_BUSSERVO_READ:        return "BUSSERVO_READ";
                case CMD_BUSSERVO_SET_LIMITS:  return "BUSSERVO_SET_LIMITS";
                case CMD_BUSSERVO_CALIBRATE:   return "BUSSERVO_CALIBRATE";
                case CMD_BUSSERVO_SET_ID:      return "BUSSERVO_SET_ID";
                case CMD_BUSSERVO_PING:        return "BUSSERVO_PING";
                case CMD_BUSSERVO_SCAN:        return "BUSSERVO_SCAN";
                case CMD_BUSSERVO_SYNC_WRITE:  return "BUSSERVO_SYNC_WRITE";
                case CMD_BUSSERVO_DONE:        return "BUSSERVO_DONE";
                case CMD_BUSSERVO_READ_LIMITS: return "BUSSERVO_READ_LIMITS";
                case CMD_BUSSERVO_PRESENT:     return "BUSSERVO_PRESENT";
            }
            break;

        case DEVICE_ENCODER:
            switch (cmd) {
                case CMD_ENCODER_ATTACH:       return "ENCODER_ATTACH";
                case CMD_ENCODER_DETACH:       return "ENCODER_DETACH";
                case CMD_ENCODER_READ:         return "ENCODER_READ";
                case CMD_ENCODER_SET_POSITION: return "ENCODER_SET_POSITION";
            }
            break;
    }
    return nullptr;
}

#endif
