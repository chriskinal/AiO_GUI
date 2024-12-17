#include "Arduino.h"
#include "HardwareSerial.h"
#include "elapsedMillis.h"
#include <stdint.h>
#include <Streaming.h>
#include "IPAddress.h"
#include "pcb.h"
#include "misc.h"

// Networkign variables
static const uint8_t defaultIP[5] = {192, 168, 5, 126};
uint8_t currentIP[5] = {192, 168, 5, 126};
uint8_t broadcastIP[5] = {defaultIP[0], defaultIP[1], defaultIP[2], 255};

// Led indicators. 1000ms RGB update, 255/64/127 RGB brightness balance levels for v5.0a
#include "LEDS.h"
LEDS LEDs = LEDS(1000, 255, 64, 127);
// End

// Usage stats
ProcessorUsage BNOusage           ((char*)"BNO   ");
ProcessorUsage GPS1usage          ((char*)"GPS1  ");
ProcessorUsage GPS2usage          ((char*)"GPS2  ");
ProcessorUsage PGNusage           ((char*)"PGN   ");
ProcessorUsage ASusage            ((char*)"AG    ");
ProcessorUsage NTRIPusage         ((char*)"NTRIP ");
ProcessorUsage RS232usage         ((char*)"RS232 ");  
ProcessorUsage LOOPusage          ((char*)"Loop  ");
ProcessorUsage IMU_Husage         ((char*)"IMU_H ");
ProcessorUsage NMEA_Pusage        ((char*)"NMEA_H");
ProcessorUsage RTKusage           ((char*)"Radio ");
ProcessorUsage UBX_Pusage         ((char*)"UBX_H ");
ProcessorUsage UDP_Susage         ((char*)"UDP_S ");
ProcessorUsage DACusage           ((char*)"DAC   ");
ProcessorUsage MACHusage          ((char*)"MACH  ");
ProcessorUsage LEDSusage          ((char*)"LEDS  ");
ProcessorUsage ESP32usage         ((char*)"ESP32 ");
const uint8_t numCpuUsageTasks = 17;
ProcessorUsage* cpuUsageArray[numCpuUsageTasks] = { 
  &BNOusage, &GPS1usage, &GPS2usage, &PGNusage, &ASusage, &NTRIPusage,
  &RS232usage, &LOOPusage, &IMU_Husage, &NMEA_Pusage, &RTKusage, &UBX_Pusage,
  &UDP_Susage, &DACusage, &MACHusage, &LEDSusage, &ESP32usage
};
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
struct SteerConfigStruct {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;  // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  // 1 if switch selected
  uint8_t SteerButton = 0;  // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 3;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;  // Set to 0 to use X Axis, 1 to use Y avis
  uint8_t MinSpeed = 0;
};
SteerConfigStruct const defaultSteerConfig;  // 9 bytes
struct SteerConfigStruct steerConfig = defaultSteerConfig;

uint8_t steerReading, prevSteerReading = 1;  // currentState = 0
int16_t pulseCount = 0;                      // Steering Wheel Encoder
int16_t lastEnc = -999;

// Switches/Sensors
uint8_t kickoutInput = 0, workInput = 0, steerState = 0, switchByte = 0;
float sensorReading, sensorSample;

const int16_t ANALOG_TRIG_THRES = 100;
const uint8_t ANALOG_TRIG_HYST = 10;
// End

// GNSS processing and variables
#include "NMEA.h"
NMEAParser<4> nmeaParser;
bool nmeaDebug = 0, nmeaDebug2 = 0, extraCRLF;

#include "UBXParser.h"
UBX_Parser ubxParser;

#include "FUSEImu.h"
FUSE_Imu fuseImu;

bool USB1DTR = false;               // to track bridge mode state
bool USB2DTR = false;
uint32_t GPS1BAUD;                  // to track baud changes for bridge mode
uint32_t GPS2BAUD;
#define PANDA_SINGLE 1
#define PAOGI_DUAL 0
bool startup = false;
elapsedMillis LEDTimer;
elapsedMillis imuPandaSyncTimer;
bool posReady, gpsActive, imuPandaSyncTrigger;
bool ggaTimeout, relposnedTimeout;
uint32_t dualTime;
uint16_t ggaMissed;

bool udpPassthrough = false;  // False = GPS neeeds to send GGA, VTG & HPR messages. True = GPS needs to send KSXT messages only.
bool gotCR = false;
bool gotLF = false;
bool gotDollar = false;
char msgBuf[254];
int msgBufLen = 0;
// End

// Autosteer encoder class instance read single or double input values in Autosteer.ino
#include "Encoder.h"
Encoder encoder(KICKOUT_D_PIN, KICKOUT_A_PIN);
// End

// IMU class instance Roomba Vac mode for BNO085
#include "BNO_RVC.h"
BNO_RVC BNO;
// End

// Keya CANBus steer motor
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;
int8_t KeyaCurrentSensorReading = 0;
bool keyaDetected = false;
// End

// USB Port tracking variables
//bool USB1DTR = false;               // to track bridge mode state
//bool USB2DTR = false;
// End