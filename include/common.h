// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef COMMON_H_
#define COMMON_H_

#include "Arduino.h"
#include "HardwareSerial.h"
#include "elapsedMillis.h"
#include "EEPROM.h"
#include <stdint.h>
#include <Streaming.h>
#include "IPAddress.h"
#include "pcb.h"
#include "misc.h"
#include "mongoose.h"
#include "mongoose_glue.h"
#include "RingBuf.h"

// Networking variables
struct NetConfigStruct
{
  static const uint8_t defaultIP[5] = {192, 168, 5, 126};
  uint8_t currentIP[5] = {192, 168, 5, 126};
  uint8_t gatewayIP[5] = {192, 168, 5, 1};
  uint8_t broadcastIP[5] = {192, 168, 5, 255};
};
NetConfigStruct const defaultNet;
NetConfigStruct netConfig = defaultNet;

struct mg_connection *sendAgio;
const uint16_t EE_ver = 2404; // if value in eeprom does not match, overwrite with defaults

// Led indicators. 1000ms RGB update, 255/64/127 RGB brightness balance levels for v5.0a
#include "LEDS.h"
//#include "LEDS_old.h"
LEDS LEDs = LEDS(1000, 255, 64, 127);
// End

// Usage stats
ProcessorUsage GUIusage((char *)"GUI   ");
ProcessorUsage BNOusage((char *)"BNO   ");
ProcessorUsage GPS1usage((char *)"GPS1  ");
ProcessorUsage GPS2usage((char *)"GPS2  ");
ProcessorUsage PGNusage((char *)"PGN   ");
ProcessorUsage ASusage((char *)"AG    ");
ProcessorUsage NTRIPusage((char *)"NTRIP ");
ProcessorUsage RS232usage((char *)"RS232 ");
ProcessorUsage LOOPusage((char *)"Loop  ");
ProcessorUsage IMU_Husage((char *)"IMU_H ");
ProcessorUsage NMEA_Pusage((char *)"NMEA_H");
ProcessorUsage RTKusage((char *)"Radio ");
ProcessorUsage UBX_Pusage((char *)"UBX_H ");
ProcessorUsage UDP_Susage((char *)"UDP_S ");
ProcessorUsage DACusage((char *)"DAC   ");
ProcessorUsage MACHusage((char *)"MACH  ");
ProcessorUsage LEDSusage((char *)"LEDS  ");
ProcessorUsage ESP32usage((char *)"ESP32 ");
ProcessorUsage KEYAusage((char *)"KEYA  ");
/*const uint8_t numCpuUsageTasks = 18;
ProcessorUsage *cpuUsageArray[numCpuUsageTasks] = {
    &BNOusage, &GPS1usage, &GPS2usage, &PGNusage, &ASusage, &NTRIPusage,
    &RS232usage, &LOOPusage, &IMU_Husage, &NMEA_Pusage, &RTKusage, &UBX_Pusage,
    &UDP_Susage, &DACusage, &MACHusage, &LEDSusage, &ESP32usage, &GUIusage};*/
HighLowHzStats gps2Stats;
HighLowHzStats gps1Stats;
HighLowHzStats relJitterStats;
HighLowHzStats relTtrStats;
HighLowHzStats bnoStats;

elapsedMillis bufferStatsTimer;
uint32_t testCounter;
bool printCpuUsages = false;
bool printStats = false;
// End

// Autosteer variables
// Variables for settings - 0 is false
struct SteerConfigStruct
{
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0; // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0; // 1 if switch selected
  uint8_t SteerButton = 0; // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 3;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0; // Set to 0 to use X Axis, 1 to use Y avis
  uint8_t MinSpeed = 0;
};
SteerConfigStruct const defaultSteerConfig; // 9 bytes
struct SteerConfigStruct steerConfig = defaultSteerConfig;
float gpsSpeed;
uint8_t guidanceStatus = 0, prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;
float steerAngleSetPoint = 0, steerAngleError = 0;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;
uint8_t aog2Count = 0;

uint8_t xte = 0;

// pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float highLowPerDeg = 0;

// Variables for settings
struct SteerSettingsStruct
{
  uint8_t Kp = 40;     // proportional gain
  uint8_t lowPWM = 10; // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 150; // max PWM value
  float steerSensorCounts = 120;
  float AckermanFix = 1; // sent as percent
};
SteerSettingsStruct defaultSteerSettings;                        // 11 bytes
struct SteerSettingsStruct steerSettings = defaultSteerSettings; // don't need 'struct' in front?

uint8_t steerReading, prevSteerReading = 1; // currentState = 0
int16_t pulseCount = 0;                     // Steering Wheel Encoder
int16_t lastEnc = -999;
bool autoSteerEnabled = false;
float steerAngleActual = 0;
int16_t steeringPosition = 0; // from steering sensor (WAS)
// End

// Switches/Sensors
uint8_t kickoutInput = 0, workInput = 0, steerState = 0, switchByte = 0;
float sensorReading, sensorSample;

const int16_t ANALOG_TRIG_THRES = 100;
const uint8_t ANALOG_TRIG_HYST = 10;
// End

// GNSS processing and variables
#include "NMEA.h"
NMEAParser<5> nmeaParser;
bool nmeaDebug = 0, nmeaDebug2 = 0, extraCRLF;

#include "UBXParser.h"
UBX_Parser ubxParser;

#include "FUSEImu.h"
FUSE_Imu fuseImu;

bool USB1DTR = false; // to track bridge mode state
bool USB2DTR = false;
uint32_t GPS1BAUD; // to track baud changes for bridge mode
uint32_t GPS2BAUD;
#define PANDA_SINGLE 1
#define PAOGI_DUAL 0
bool startup = false;
elapsedMillis LEDTimer;
bool posReady, gpsActive;
bool ggaTimeout, relposnedTimeout;
uint32_t dualTime;
uint16_t ggaMissed;

struct GPSConfigStruct
{
  uint8_t gpsSync = 1;
  bool gpsPass = false; // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
};
GPSConfigStruct gpsConfig;

const uint8_t syncLUT[12] = {10,9,8,7,6,5,4,3,2,1,0};

bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;
// End

// GUI variables
settings aio_settings;
#include "configfuncs.h"

// Autosteer encoder class instance read single or double input values in Autosteer.ino
#include "Encoder.h"
Encoder encoder(KICKOUT_D_PIN, KICKOUT_A_PIN);
// End

// IMU class instance Roomba Vac mode for BNO085
#include "BNO_RVC.h"
BNO_RVC BNO;
// End
RingBuf<BNO_RVC_DATA, 11> bnoRing;
BNO_RVC_DATA bnoRingData;

// Keya CANBus steer motor
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;
int8_t KeyaCurrentSensorReading = 0;
bool keyaDetected = false;
// End

#include "Machine.h"
MACHINE* machinePTR;

// for enabling & controlling AUX, LOCK & Section/Machine outputs
#define I2C_WIRE       Wire           // used for PCA9685 aux, lock & section outputs (0x44), & RGB LEDs (0x70) in LEDS.h
#include "Adafruit_PWMServoDriver.h"  // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
Adafruit_PWMServoDriver outputs = Adafruit_PWMServoDriver(0x44, I2C_WIRE); // RGB instance is 0x44 unless A2 Low solder jumper is closed, then 0x40
#include "outputs.h"

#endif // COMMON_H_