#include "HardwareSerial.h"
#include "elapsedMillis.h"
#include <stdint.h>
#include <Streaming.h>
#include "IPAddress.h"

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

// Ring buffer for transferring data from UDP events for processing
#include <RingBuf.h>
struct pgnData
{
  IPAddress remoteIP;
  byte data[40];
  uint16_t length;
};
struct ntripData
{
  byte data[256];
  uint16_t length;
};

RingBuf *PGN_buf = RingBuf_new(sizeof(struct pgnData), 10);
RingBuf *NTRIP_buf = RingBuf_new(sizeof(struct ntripData), 10);
// End

// Machine stuff
#include "clsPCA9555.h" // https://github.com/nicoverduin/PCA9555
PCA9555 outputs(0x20);  // 0x20 - I2C addr (A0-A2 grounded), interrupt pin causes boot loop
#include "machine.h"
MACHINE machine;      // also used for v4 as it suppresses machine PGN debug messages
const uint8_t pcaOutputPinNumbers[8] = { 1, 0, 12, 15, 9, 8, 6, 7 };    // all 8 PCA9555 section/machine output pin numbers on v5.0a
const uint8_t pcaInputPinNumbers[]  = { 14, 13, 11, 10, 2, 3, 4, 5 };   // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a
// End

// Keya CANBus sterr motor
#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;
int8_t KeyaCurrentSensorReading = 0;
bool keyaDetected = false;
// End

// AiO board hardware
uint8_t GPS1rxbuffer[128];      // seems large enough
uint8_t GPS1txbuffer[256];      // large enough for 256 byte AgIO NTRIP packet
uint8_t GPS2rxbuffer[128];      // seems large enough
uint8_t GPS2txbuffer[256];      // large enough for 256 byte AgIO NTRIP packet
uint8_t RTKrxbuffer[64];        // don't know what size is needed, larger buffer if GPS baud is lower then RTK radio baud
buffer[256];   // large enough to hold a few NMEA sentences as ext terminal bauds are usually slow
//uint8_t RS232rxbuffer[256]; // not needed unless custom rs232 rx code is added
uint8_t ESP32rxbuffer[256];   // don't know what size is needed
uint8_t ESP32txbuffer[256];   // don't know what size is needed
const char inoVersion[] = "AiO v5.0a OGX - " __DATE__;

// ********* IO Defines *********
const uint8_t WAS_SENSOR_PIN = A15;     // WAS input
const uint8_t SPEEDPULSE_PIN = 18;      
const uint8_t SPEEDPULSE10_PIN = 19;   // 1/10 speedpulse output, strictly for human visualization
#include "misc.h"
SpeedPulse speedPulse(SPEEDPULSE_PIN, SPEEDPULSE10_PIN);     // misc.h

const uint8_t PIEZO1 = 37;
const uint8_t PIEZO2 = 36;

// Cytron/DRV8701
#define DIR_PIN           6     // DRV Dir pin
#define PWM_PIN           9     // DRV PWM pin
#define SLEEP_PIN         4     // DRV Sleep pin, LOCK output

// Switches/Sensors
#define STEER_PIN         2
#define WORK_PIN        A17
#define KICKOUT_D_PIN     3     // REMOTE
#define CURRENT_PIN     A13     // CURRENT sense from on board DRV8701
#define KICKOUT_A_PIN   A12     // PRESSURE

// ********* Serial Assignments *********
#define SerialRTK Serial3               // RTK radio
//HardwareSerial SerialRTK = Serial3;   // causes boot loop

HardwareSerial* SerialIMU = &Serial6;   // IMU BNO-085 in RVC serial mode

// HardwareSerial *SerialGPS = &Serial5;   // Main postion receiver (GGA & VTG)
#define SerialGPS1 Serial5

// HardwareSerial *SerialGPS2 = &Serial8; // Dual heading receiver  (relposNED)
#define SerialGPS2 Serial8

// HardwareSerial *SerialRS232 = &Serial7; // RS232
#define SerialRS232 Serial7

// HardwareSerial *SerialESP32 = &Serial2; // ESP32
#define SerialESP32 Serial2

//const int32_t baudGPS = 921600;
const int32_t baudGPS = 460800;
const int32_t baudRTK = 460800;     // most are using Xbee radios with default of 115200
const int32_t baudRS232 = 38400;
const int32_t baudESP32 = 460800;

extern "C" uint32_t set_arm_clock(uint32_t frequency);  // required prototype to set CPU speed
// End
