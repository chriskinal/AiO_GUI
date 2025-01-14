#ifndef PCB_H_
#define PCB_H_
#define AIOv50a
#include "Arduino.h"

const uint8_t encoderType = 1; // 1 - single input
                               // 2 - dual input (quadrature encoder), uses Kickout_A (Pressure) & Kickout_D (Remote) inputs
                               // 3 - variable duty cycle, for future updates

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype to set CPU speed

// ********* IO Defines *********
const uint8_t WAS_SENSOR_PIN = A15; // WAS input
const uint8_t SPEEDPULSE_PIN = 18;
const uint8_t SPEEDPULSE10_PIN = 19; // 1/10 speedpulse output, strictly for human visualization
#include "misc.h"
SpeedPulse speedPulse(SPEEDPULSE_PIN, SPEEDPULSE10_PIN); // misc.h

const uint8_t PIEZO1 = 37;
const uint8_t PIEZO2 = 36;

// Cytron/DRV8701
#define DIR_PIN 6   // DRV Dir pin
#define PWM_PIN 9   // DRV PWM pin
#define SLEEP_PIN 4 // DRV Sleep pin, LOCK output

// Switches/Sensors
#define STEER_PIN 2
#define WORK_PIN A17
#define KICKOUT_D_PIN 3   // REMOTE
#define CURRENT_PIN A13   // CURRENT sense from on board DRV8701
#define KICKOUT_A_PIN A12 // PRESSURE

// ********* Serial Assignments *********
#define SerialRTK Serial3 // RTK radio
// HardwareSerial SerialRTK = Serial3;   // causes boot loop

HardwareSerial *SerialIMU = &Serial6; // IMU BNO-085 in RVC serial mode

// HardwareSerial *SerialGPS = &Serial5;   // Main postion receiver (GGA & VTG)
#define SerialGPS1 Serial5

// HardwareSerial *SerialGPS2 = &Serial8; // Dual heading receiver  (relposNED)
#define SerialGPS2 Serial8

// HardwareSerial *SerialRS232 = &Serial7; // RS232
#define SerialRS232 Serial7

// HardwareSerial *SerialESP32 = &Serial2; // ESP32
#define SerialESP32 Serial2

// const int32_t baudGPS = 921600;
const int32_t baudGPS = 460800;
const int32_t baudRTK = 460800; // most are using Xbee radios with default of 115200
const int32_t baudRS232 = 38400;
const int32_t baudESP32 = 460800;

// constexpr int buffer_size = 512;
uint8_t GPS1rxbuffer[128]; // seems large enough
uint8_t GPS1txbuffer[256]; // large enough for 256 byte AgIO NTRIP packet
uint8_t GPS2rxbuffer[128]; // seems large enough
uint8_t GPS2txbuffer[256]; // large enough for 256 byte AgIO NTRIP packet
uint8_t RTKrxbuffer[64];   // don't know what size is needed, larger buffer if GPS baud is lower then RTK radio baud

uint8_t RS232txbuffer[256]; // large enough to hold a few NMEA sentences as ext terminal bauds are usually slow
// uint8_t RS232rxbuffer[256]; // not needed unless custom rs232 rx code is added
uint8_t ESP32rxbuffer[256]; // don't know what size is needed
uint8_t ESP32txbuffer[256]; // don't know what size is needed

#endif // PCB_H_