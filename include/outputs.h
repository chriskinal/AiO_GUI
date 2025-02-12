#include <stdint.h>
#include <Streaming.h>
#include <Wire.h>

const uint8_t drvCnt = 5;

// these pins needs a DRV8243 sleep reset pulse before they activate
const uint8_t drvSleepPins[drvCnt] = {
  15, // AUX nSLEEP
  14, // LOCK nSLEEP
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
  Serial.print("\r\nInitializing Machine/Section Outputs");
  Serial.print("\r\n- v5.0d PCA9685 I2C PWM IO extender controls (5) DRV8243 for Section/Lock/Aux outputs");
  // only nSLEEP pin is controlled on Aux & Lock by PCA9685

  outputs.begin();              // Adafruit_PWMServoDriver 
  Wire.setClock(1000000);
  outputs.setPWMFreq(1526);     // the maximum, to hopefully mitigate switching/frequency noise
  outputs.setOutputMode(false); // false: open drain, true: totempole (push/pull)
  
  Wire.beginTransmission(0x44);
  Serial.print("\r\n  - Sections/Lock/Aux PCA9685 ");
  if (Wire.endTransmission() == 0)
    Serial.print("found");
  else
    Serial.print("*NOT found!*");

  // put all DRVs to sleep (Hi-Z outputs & they're ready to wake up)
  Serial.print("\r\n    - All LOCK/AUX/Section DRVs set to sleep (Hi-Z outputs)");
  for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
    outputs.setPin(drvSleepPins[drvNum], 0, 0); // sets PCA9685 pin LOW 0V, puts DRV in Sleep mode
  }

  delayMicroseconds(150);  // wait max tSLEEP (120uS) for Sleep mode to settle in

  // wake/activate LOCK & AUX
  // LOCK still needs signal from Autosteer code before its output is HIGH
  Serial.print("\r\n- Enabling LOCK DRV, output controlled by Autosteer/Lock Btn");
  outputs.setPin(drvSleepPins[DRV_ID::LOCK_DRV], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz, send nSLEEP reset pulse
  
  // AUX's output is HIGH as soon as it wakes up
  Serial.print("\r\n- Enabling AUX Output");
  outputs.setPin(drvSleepPins[DRV_ID::AUX_DRV], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz, send nSLEEP reset pulse
}


const uint8_t numMachineOutputs = 6;
const uint8_t machinePCA9685OutputPins[numMachineOutputs] = { 0, 1, 4, 5, 10, 9 };
//const uint8_t Machine_PCA9685_DRV_OFF_Pins[3] = { 2, 6, 8 };
//const uint8_t Machine_PCA9685_DRV_Sleep_Pins[3] = { 13, 3, 7 };

void initMachineOutputs() {
  // set all DRV signals HIGH before waking so that outputs are Hi-Z (PWM bridge mode)
  for (uint8_t i = 0; i < numMachineOutputs; i++) {
    //digitalWrite(machineOutputPins[i], !machinePTR->config.isPinActiveHigh);  // set OFF
    outputs.setPin(machinePCA9685OutputPins[i], 0, !machinePTR->config.isPinActiveHigh); // HIGH signal sets DRV output HI-Z
  }

  // issue DRV nSLEEP reset pulse to wake them up
  outputs.setPin(drvSleepPins[DRV_ID::SEC12_DRV], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz
  outputs.setPin(drvSleepPins[DRV_ID::SEC34_DRV], 187, 1);
  outputs.setPin(drvSleepPins[DRV_ID::SEC56_DRV], 187, 1);
}

// callback function triggered by Machine class to update "machine" outputs
// this updates the Machine Module Pin Configuration outputs
// - sections 1-16, Hyd Up/Down, Tramline Right/Left, Geo Stop
void updateMachineOutputs()
{
  Serial.print("\r\n\nMachine Outputs update");
  for (uint8_t i = 1; i <= numMachineOutputs; i++) {
    Serial.printf("\r\n- Pin %2i: %i ", machinePCA9685OutputPins[i-1], machinePTR->states.functions[machinePTR->config.pinFunction[i]]);
    Serial.print(machinePTR->functionNames[machinePTR->config.pinFunction[i]]);

    //digitalWrite(machinePCA9685OutputPins[i], machinePTR->state.functions[machinePTR->config.pinFunction[i]] == machinePTR->config.isPinActiveHigh);
    outputs.setPin(machinePCA9685OutputPins[i-1], 0,
      machinePTR->states.functions[machinePTR->config.pinFunction[i]]);// == machinePTR->config.isPinActiveHigh); // == does an XOR operation
    
    
    //outputs.setPin(machinePCA9685OutputPins[0], 0, 1); // sets PCA9685 pin HIGH
  }
}
