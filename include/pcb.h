#ifndef PCB_H_
#define PCB_H_
#define AIOv50a
const char inoVersion[] = "AiO v5.0d Web GUI - " __DATE__;

const uint8_t encoderType = 1; // 1 - single input
                               // 2 - dual input (quadrature encoder), uses Kickout_A (Pressure) & Kickout_D (Remote) inputs
                               // 3 - variable duty cycle, for future updates

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype to set CPU speed

// ********* IO Defines *********
const uint8_t WAS_SENSOR_PIN = A15;  // WAS input

const uint8_t SPEEDPULSE_PIN = 33;   // actual speed pulse output via optocoupler
const uint8_t SPEEDPULSE10_PIN = 37; // 1/10 speedpulse output, strictly for human visualization with onboard LED
#include "misc.h"
SpeedPulse speedPulse(SPEEDPULSE_PIN, SPEEDPULSE10_PIN); // misc.h

const uint8_t BUZZER = 36;  // electromagnetic buzzer driven by NFET

// Cytron/DRV8701
#define SLEEP_PIN 4 // DRV8701-Cytron Sleep pin, LOCK output
#define PWM1_PIN  5 // DRV8701-Cytron PWM pin
#define PWM2_PIN  6 // DRV8701-Cytron PWM2 pin (prev Dir pin)

// Switches/Sensors
#define STEER_PIN       2 // Switch/btn input for autosteer engage/disengage
#define WORK_PIN      A17 // Analog input, can also be used for Digital switches, see UI for settings
#define KICKOUT_D_PIN   3 // REMOTE, encoder or other digital disengage input
#define CURRENT_PIN   A13 // CURRENT sense from on board DRV8701
#define KICKOUT_A_PIN A12 // Analog PRESSURE (can also be used for 2nd Quadrature encodeder input)

// ********* Serial Assignments *********
HardwareSerial *SerialIMU = &Serial4; // IMU BNO-085 in RVC serial mode
#define SerialRTK   Serial3 // RTK radio
#define SerialGPS1  Serial5 // GPS1 UART (Right F9P, or UM982)
#define SerialGPS2  Serial8 // GPS2 UART (Left F9P)
#define SerialRS232 Serial7 // RS232 UART
#define SerialESP32 Serial2 // ESP32 UART (for ESP32 WiFi Bridge)

// const int32_t baudGPS = 921600;
const int32_t baudGPS = 921600;
const int32_t baudRTK = 115200; // most are using Xbee radios with default of 115200
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
uint8_t ESP32rxbuffer[256]; // don't know what size is needed, 128 is likely sufficient
uint8_t ESP32txbuffer[256]; // don't know what size is needed, 128 is likely sufficient

#endif // PCB_H_