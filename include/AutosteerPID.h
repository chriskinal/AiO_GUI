// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef AUTOSTEERPID_H_
#define AUTOSTEERPID_H_

void calcSteeringPID(void)
{
#ifdef JD_DAC_H
  float pValue = steerSettings.Kp * steerAngleSetPoint; // only use set point, not error for two track JD
  float errorAbs = abs(steerAngleSetPoint);
#else
  float pValue = steerSettings.Kp * steerAngleError;
  float errorAbs = abs(steerAngleError);
#endif

  pwmDrive = (int16_t)pValue;

  // add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0)
    pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0)
    pwmDrive += steerSettings.minPWM;

  int16_t newHighPWM = 0;

  // from 0-3 deg error, scale newHighPWM from lowPWM(minPWM*1.2)-highPWM
  if (errorAbs < LOW_HIGH_DEGREES)
  {
    newHighPWM = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  }
  else
    newHighPWM = steerSettings.highPWM;

  // limit the pwm drive
  //  causes oscillation in pwmDrive
  if (pwmDrive > newHighPWM)
    pwmDrive = newHighPWM;
  if (pwmDrive < -newHighPWM)
    pwmDrive = -newHighPWM;

  if (steerConfig.MotorDriveDirection)
    pwmDrive *= -1;

  // *** This needs testing, so far it's the only alternative steering output this board should support (Cytron or Danfoss only, or can bus Keya)
  if (steerConfig.IsDanfoss)
  {
    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, -250, 250));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2; // Divide by 4
    pwmDrive += 128;          // add Center Pos.
  }
}

// #########################################################################################

void motorDrive(void)
{
  // Keya can bus output, always send pwmDrive to keya, SteerKeya function will deal with it
  SteerKeya(pwmDrive); // use this for in tractor

#ifdef JD_DAC_H
  // For JD_DAC.h, MCP4728 QUAD DAC steering
  // scale pwmDrive to DAC output
  // 0 PWM (no WAS change needed) = 2048 centered DAC output (4096 / 2 to get center voltage)
  DACusage.timeIn();
  if (gpsSpeed < (float)steerConfig.MinSpeed / 10.0)
    pwmDrive = 0;
  pwmDisplay = jdDac.steerOutput(pwmDrive);
  // jdDac.ch4Output(pwmDrive);  // now used by OGX
  DACusage.timeOut();
#else
  // Cytron drv8701P Driver PWM1 + PWM2 Signal (like prev IBT2 option)
  if (pwmDrive > 0)
  {
    analogWrite(PWM2_PIN, 0); // Turn off before other one on
    analogWrite(PWM1_PIN, pwmDrive);
  }
  else
  {
    pwmDrive = -1 * pwmDrive;
    analogWrite(PWM1_PIN, 0); // Turn off before other one on
    analogWrite(PWM2_PIN, pwmDrive);
  }

  pwmDisplay = pwmDrive;
#endif




}

#endif /* AUTOSTEERPID_H_ */