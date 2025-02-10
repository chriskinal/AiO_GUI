![AgOpenGPS](https://github.com/chriskinal/AiO_GUI/blob/main/media/agopengps%20name%20logo.png)
[AOG Download](https://github.com/agopengps-official/AgOpenGPS/releases)<br>
[AOG Forum](https://discourse.agopengps.com/)<br>
[AOG YouTube](https://youtube.com/@AgOpenGPS)

# 100hz RVC firmware for All-In-One Proto
### *** very alpha testing/dev stage ***

### Project Overview
This firmware is based on the official [All-In-One v4.x I2C](https://github.com/AgOpenGPS-Official/Firmware_Teensy_AiO-v4_I2C) firmware with some code copied from the ["Ace"](https://github.com/farmerbriantee/Ace) project and as is common for Arduino projects, the Internet. It is designed to run on a new AiO Prototype with hopes to release the design files sometime in 2025. It is built on a bare metal [Mongoose](https://mongoose.ws/documentation) base which includes the TCP/UDP stack and a [Web UI Wizard](https://mongoose.ws/wizard/#/). To start, we plan to support the usual v4.x AiO settings/options/operations via AOG's UI & PGNs with plans to add new settings in the Teesny's hosted Web UI to support the new board's capabilities (analog WORK input, RGB status LEDs brightness, RS232 NMEA output, Section/Machine control, speaker/buzzer, external RS232 IMU, ESP32 WiFi Bridge, AutoSteer Kickout options, etc).

### To Do
too many to list, much many changes coming

### To Contribute
There is a lot of active dev work expected in the next few months. The planned work flow for contributing is to create a branch from main, make some changes/improvments and create a pull request back to main but for now if you have a feature request it might be better to create an issue so that there's a log and we don't forget about it. We might also already be working/planning on it.

![PCB 3D Top](https://github.com/chriskinal/AiO_GUI/blob/main/media/top%203d.png)
![PCB 2D Top](https://github.com/chriskinal/AiO_GUI/blob/main/media/top%202d.png)

