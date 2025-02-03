#include <stdint.h>
#include <Streaming.h>
#include <Wire.h>

const uint8_t drvCnt = 5;

// these pins needs a DRV8243 sleep reset pulse before they activate
const uint8_t drvSleepPins[drvCnt] = {
  14, // AUX nSLEEP
  15, // LOCK nSLEEP
  13, // Section 1/2 nSLEEP
   3, // Section 3/4 nSLEEP
   7  // Section 5/6 nSLEEP
};

// set/leave these PCA pins LOW(default) to allow DRV operation or HIGH to for Standby (Hi-Z outputs)
const uint8_t drvOffPins[3] = {
  2,  // Sec 1/2 DRVOFF
  6,  // Sec 3/4 DRVOFF
  8   // Sec 5/6 DRVOFF
};

typedef enum {
  LOCK_DRV,
  AUX_DRV,
  SEC12_DRV,
  SEC34_DRV,
  SEC56_DRV,
} DRV_ID;

void outputsInit() {
  Serial << "\r\nInitializing Outputs";
  Serial << "\r\n- v5.0d PCA9685 I2C PWM IO extenders (2 should be detected)";
  outputs.begin();              // Adafruit_PWMServoDriver 
  Wire.setClock(1000000);
  outputs.setPWMFreq(1526);     // the maximum, to hopefully mitigate switching/frequency noise
  outputs.setOutputMode(false); // false: open drain, true: totempole (push/pull)
  
  Wire.beginTransmission(0x70);
  Serial.print("\r\n  - RGB PCA9685 ");
  if (Wire.endTransmission() == 0)
    Serial.print("found");
  else
    Serial.print("*NOT found!*");

  Wire.beginTransmission(0x44);
  Serial.print("\r\n  - Sections/Lock/Aux PCA9685 ");
  if (Wire.endTransmission() == 0)
    Serial.print("found");
  else
    Serial.print("*NOT found!*");

  // put all DRVs to sleep (Hi-Z outputs & they're ready to wake up)
  Serial.print("\r\n    - All LOCK/AUX/Section DRVs set to sleep (Hi-Z outputs)");
  for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
    outputs.setPin(drvSleepPins[drvNum], 0, 0); // sets PCA9685 pin LOW 0V
  }

  delayMicroseconds(150);  // wait max tSLEEP (120uS) for Sleep mode to settle in

  // wake/activate LOCK & AUX
  Serial.print("\r\n- Enabling LOCK/AUX outputs");
  // LOCK still needs signal from Autosteer code before its output is HIGH
  //outputs.setPin(drvSleepPins[DRV_ID::LOCK_DRV], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz, send nSLEEP reset pulse
  // AUX's output is HIGH as soon as it wakes up
  outputs.setPin(drvSleepPins[DRV_ID::AUX_DRV], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz, send nSLEEP reset pulse
}

