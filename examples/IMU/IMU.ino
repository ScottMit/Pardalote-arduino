// ==============================================================
// Pardalote — IMU example
//
// Supports InvenSense MPU-6050/6500/9250/9255 and STMicro
// LSM6DS3/LSM6DSOX. No third-party library is needed — the
// extension reads sensor registers directly over I2C.
//
// Browser side:
//   arduino.add('imu', new IMU('6050'));
//   arduino.on('ready', () => {
//       arduino.imu.attach(0x68);
//       arduino.imu.read(20);   // poll at 50 Hz
//   });
// ==============================================================

#include <Pardalote.h>
#include <PardaloteIMU.h>

void setup() {
    Pardalote.begin();
}

void loop() {
    Pardalote.run();
}
