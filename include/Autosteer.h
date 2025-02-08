#ifndef AUTOSTEER_H_
#define AUTOSTEER_H_
#include "common.h"
/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/
const uint8_t PWM_Frequency = 2;
const float LOW_HIGH_DEGREES = 3.0; // How many degrees before decreasing Max PWM

bool testBothWasSensors = false;
bool adcDebug = false;
bool useInternalADC = true;   // v5.0 Proto only uses Teensy ADC
bool useExternalADS = false;

#include <EEPROM.h>

uint32_t autoSteerLastTime, currentTime;
elapsedMillis autoSteerUpdateTimer;

void calcSteeringPID(void);
void motorDrive(void);

void adcSetup()
{
  Serial.print("\r\n- ADC check:");
  analogReadResolution(12);
  analogReadAveraging(16);
  // detect input on Teensy WAS_SENSOR_PIN
  /*pinMode(WAS_SENSOR_PIN, INPUT_PULLDOWN);
  uint16_t pullDown = analogRead(WAS_SENSOR_PIN);
  pinMode(WAS_SENSOR_PIN, INPUT_PULLUP);
  uint16_t pullUp = analogRead(WAS_SENSOR_PIN);
  uint16_t pullDiff = abs(pullUp - pullDown);*/
  pinMode(WAS_SENSOR_PIN, INPUT_DISABLE); // don't forget to disable the internal resistor !!
  /*Serial.printf("\r\n  - Teensy Internal ADC pDn:%4i, pUp:%4i, diff:%4i", pullDown, pullUp, pullDiff);

  if (pullDiff < 500) // v4.0, A0 floating 3960 diff, MCP plugged in 140 diff max
  {
    Serial.print("\r\n  - using Teensy ADC");
    useInternalADC = true;
    autoSteerEnabled = true;
    LEDs.set(LED_ID::STEER, STEER_STATE::WAS_READY);
  }*/

  autoSteerEnabled = true;    // need other checks for valid WAS input but for now enable AS anyways

} // end adcSetup()

void steerConfigInit()
{
  if (steerConfig.SteerButton == 0 && steerConfig.SteerSwitch == 0)
  {
    // currentState = 0;
    prevSteerReading = 1;
  }

  if (steerConfig.PressureSensor)
  {
    pinMode(KICKOUT_A_PIN, INPUT_DISABLE);
  }
  else
  {
    pinMode(KICKOUT_A_PIN, INPUT_PULLUP);
  }

  BNO.isSwapXY = !steerConfig.IsUseY_Axis;
}

void steerSettingsInit()
{
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup()
{
  Serial.print("\r\n\nAutoSteer setup");
  // PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_PIN, 490);
    analogWriteFrequency(PWM2_PIN, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_PIN, 122);
    analogWriteFrequency(PWM2_PIN, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_PIN, 3921);
    analogWriteFrequency(PWM2_PIN, 3921);
  }

  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, LOW); // keep DRV8701 Cytron asleep

  // keep pulled high and drag low to activate, noise free safe
  pinMode(STEER_PIN, INPUT_PULLUP);
  pinMode(KICKOUT_D_PIN, INPUT_PULLUP); // also set by Encoder library

  // Disable pullup/down resistors for analog input pins
  pinMode(WORK_PIN, INPUT_DISABLE);     // input driven by MCP6002 opamp
  pinMode(CURRENT_PIN, INPUT_DISABLE);  // input driven by MCP6002 opamp

  uint16_t as_ee_read = EE_ver;
  EEPROM.get(1, as_ee_read);

  if (as_ee_read != EE_ver)
  { // if value in eeprom does not match, overwrite with defaults
    EEPROM.put(1, EE_ver);
    EEPROM.put(100, steerSettings);
    EEPROM.put(200, steerConfig);
    Serial.print("\r\n- ** EEPROM reset to defaults! **");
  }
  else
  {
    EEPROM.get(100, steerSettings); // read the Settings
    EEPROM.get(200, steerConfig);
    Serial.print("\r\n- loaded settings/config from EEPROM");
  }

  steerSettingsInit();
  steerConfigInit();
  adcSetup();

  if (!autoSteerEnabled)
  {
    Serial.print("\r\n- ** AutoSteer is disabled, GPS only mode **");
    Serial.print("\r\n  - ** likely no WAS input detected **");
    return;
  }

  Serial.print("\r\n- AutoSteer enabled, setup complete");
  LEDs.set(LED_ID::STEER, STEER_STATE::AUTOSTEER_READY);
} // End of autosteerSetup

void autoSteerUpdate()
{
  ASusage.timeIn();

  if (autoSteerUpdateTimer > 9)
  {                             // update AS loop every 10ms (100hz)
    autoSteerUpdateTimer -= 10; // or = 0?

    // ******************************* Steer Switch/Button *******************************
    // Steer input logic all setup so that '1' (HIGH) is ON, and '0' (LOW) is OFF
    steerReading = !digitalRead(STEER_PIN); // read steer input switch/button, invert reading to match On/Off logic

    if (steerConfig.SteerSwitch == 1) // steer "Switch" mode (on - off)
    {
      // new code for steer "Switch" mode that keeps AutoSteer OFF after current/pressure kickout until switch is cycled
      if (steerReading == LOW)
      {                            // switching OFF
        steerState = steerReading; // set OFF
        if (prevSteerReading != steerState)
        {
          //char msg[] = "AutoSteer Switch OFF";
          //char msgTime = 2;
          LEDs.activateBlueFlash(LED_ID::STEER);
        }
      }
      else if (steerReading == HIGH && prevSteerReading == LOW)
      {                            // switch ON after prev being OFF
        steerState = steerReading; // set ON
        //char msg[] = "AutoSteer Switch ON";
        //char msgTime = 2;
        LEDs.activateBlueFlash(LED_ID::STEER);
      }
      prevSteerReading = steerReading;
    }

    else if (steerConfig.SteerButton == 1) // steer "Button" mode (momentary)
    {
      if (steerReading == HIGH && prevSteerReading == LOW)
      { // button is pressed
        steerState = !steerState;
        LEDs.activateBlueFlash(LED_ID::STEER);
        /*char *msg;
        if (steerState)
          msg = (char *)"AutoSteer Btn ON";
        else
          msg = (char *)"AutoSteer Btn OFF";
        char msgTime = 2;*/
        // UDP.SendUdpFreeForm(1, msg, strlen(msg), msgTime, UDP.broadcastIP, UDP.portAgIO_9999);
      }
      prevSteerReading = steerReading; // get ready to detect next press

      if (guidanceStatusChanged)
        steerState = guidanceStatus; // allows AoG to turn AS on/off in parallel with Btn
    }

    else // No steer switch or button
    {
      // If steering is OFF and AoG's GUI btn is switched ON
      if (guidanceStatusChanged && guidanceStatus == 1 && steerState == 0 && prevSteerReading == 1)
      {
        prevSteerReading = steerState;
        steerState = 1;
        LEDs.activateBlueFlash(LED_ID::STEER);
      }

      // If steering is ON and AoG's GUI btn is switched OFF
      if (guidanceStatusChanged && guidanceStatus == 0 && steerState == 1 && prevSteerReading == 0)
      {
        prevSteerReading = steerState;
        steerState = 0;
        LEDs.activateBlueFlash(LED_ID::STEER);
      }
    }

    // ******************* Kickouts ( Encoders / Pressure / Current ) *******************
    if (steerConfig.ShaftEncoder)
    {
      if (encoderType == 1) // single input
      {
        pulseCount = encoder.readCount();
        if (pulseCount != lastEnc)
        {
          lastEnc = pulseCount;
        }
      }
      else if (encoderType == 2) // dual input (quadrature encoder)
      {
        pulseCount = abs(encoder.readPosition());
        if (pulseCount != lastEnc)
        {
          Serial << "\r\npulseCount:" << pulseCount;
          lastEnc = pulseCount;
        }
      }
      if (pulseCount >= steerConfig.PulseCountMax)
      {
        steerState = 0; // reset values like it turned off
        prevSteerReading = !steerState;
      }
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(KICKOUT_A_PIN);          // >> 4);    // to scale 12 bit down to 8 bit
      sensorSample *= 0.15;                                     // for 5v sensor, scale down to try matching old AIO
      sensorSample = min(sensorSample, 255);                    // limit to 1 byte (0-255)
      sensorReading = sensorReading * 0.8 + sensorSample * 0.2; // filter

      if (sensorReading >= steerConfig.PulseCountMax)
      {                 // if reading exceeds kickout setpoint
        steerState = 0; // turn OFF autoSteer
        prevSteerReading = !steerState;
      }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
      if (keyaDetected)
      {
        sensorReading = sensorReading * 0.7 + KeyaCurrentSensorReading * 0.3; // then use keya current data
      }
      else
      { // otherwise continue using analog input on PCB
        sensorSample = (float)analogRead(CURRENT_PIN);
        //Serial << "\r\n" << sensorSample - 45.0;
        sensorSample -= 45.0;     // zero current offset
        //sensorSample = abs(3100 - sensorSample) * 0.0625; // 3100 is like old firmware, 3150 is center (zero current) value on Matt's v4.0 Micro
        sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
        //Serial << " " << sensorReading << " max:" << steerConfig.PulseCountMax;
        if (sensorReading >= steerConfig.PulseCountMax)
        {
          steerState = 0; // turn OFF autoSteer
          prevSteerReading = !steerState;
        }
      }
    }

    uint16_t read = analogRead(WORK_PIN) > ANALOG_TRIG_THRES + ANALOG_TRIG_HYST ? LOW : HIGH;  // read work input

    if (read != workInput)
    {
      Serial.printf("\r\nWORK input(%i): %s", read, (read == 1 ? "ON" : "OFF"));
      workInput = read;
    }

    switchByte = 0;
    switchByte |= (kickoutInput << 2); // put remote in bit 2, Matt - not exaclty sure what this does
    switchByte |= (!steerState << 1);  // put inverted steerInput status in bit 1 position
    switchByte |= !workInput;          // put inverted workInput into bit 0

    // ***************************** READ WAS *****************************
    // useExternalADS = true;

#ifndef JD_DAC_H
    if (adcDebug)
      Serial.printf("\r\n%6i", millis());
    if (useInternalADC || testBothWasSensors)
    {
      steeringPosition = int(float(analogRead(WAS_SENSOR_PIN)) * 3.23);
      if (adcDebug)
        Serial.printf(" Teensy ADC(x3.23):%5i", steeringPosition);
    }
#else
    DACusage.timeIn();
    jdDac.update();
    // static int16_t oldSteer;
    int16_t newDacSteering = (jdDac.getWAS() >> 1); // read JD SWS instead to display on AoG
    // if (adcDebug && (newDacSteering > oldSteer +10 || newDacSteering < oldSteer -10)) Serial.printf("\r\n%6i  DAC_ADS-ch0(/2):%5i", millis(), newDacSteering);
    if (adcDebug || (analogRead(WORK_PIN) > ANALOG_TRIG_THRES ? LOW : HIGH))
      Serial.printf("\r\n%6i  DAC_ADS-ch0(/2):%5i", millis(), newDacSteering);
    steeringPosition = newDacSteering;
    // oldSteer = steeringPosition;
    DACusage.timeOut();

    // if (!digitalRead(KICKOUT_D_PIN)) jdDac.readAllSWS();
#endif

    // DETERMINE ACTUAL STEERING POSITION
    // convert position to steer angle. 32 counts per degree of steer pot position in my case
    //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
    if (steerConfig.InvertWAS)
    {
      steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset); // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
    }
    else
    {
      steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset); // 1/2 of full scale
      steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
    }

    if (steerAngleActual < 0)
      steerAngleActual = (steerAngleActual * steerSettings.AckermanFix); // Ackerman fix
    steerAngleError = steerAngleActual - steerAngleSetPoint;             // calculate the steering error
    // if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

    // If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250)
      watchdogTimer = WATCHDOG_FORCE_VALUE;

    // Serial.print("\r\nAS wd: "); Serial.print(watchdogTimer);
    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      // Enable H Bridge for Cytron, or JD DAC (triple analog output)
#ifdef JD_DAC_H
      jdDac.steerEnable(true); // select IBT2 for JD DAC control
                               // jdDac.ch4Enable(true);
#else
      digitalWrite(SLEEP_PIN, HIGH);
#endif

      calcSteeringPID(); // do the pid
      motorDrive();      // out to motors the pwm value

      LEDs.set(LED_ID::STEER, STEER_STATE::AUTOSTEER_ACTIVE, true);
    }
    else
    {
      // we've lost the comm to AgOpenGPS, or just stop request
      // Disable H Bridge for IBT2, hyd aux, etc for cytron
      pwmDrive = 0; // turn off steering motor
      pulseCount = 0;
      encoder.write(0);

#ifdef JD_DAC_H
      jdDac.steerEnable(false);
      // jdDac.ch4Enable(false);
#else
      digitalWrite(SLEEP_PIN, LOW);   // sleep mode
      //digitalWrite(PWM1_PIN, LOW);    // if both PWM pins are low, even if !sleep, the outputs are Hi-Z
      //digitalWrite(PWM2_PIN, LOW);
#endif

      motorDrive(); // out to motors the pwm value

      LEDs.set(LED_ID::STEER, STEER_STATE::AUTOSTEER_READY, true);
    }
  }

  ASusage.timeOut();
} // end of autoSteerLoop

#endif /* AUTOSTEER_H_ */