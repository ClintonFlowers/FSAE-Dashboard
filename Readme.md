# KSU Motorsports Dashboard & Data Logger

![KS-2 Dashboard](/Dash.JPG "KS-2 Dashboard")

This is a custom dashboard / shift-light module created for the Kennesaw State University Motorsports student-built race car, the KS-2.
In a 5" x 1.25" x 0.5" footprint, it has:

* 20 individually-addressable RGB LED's (Neopixels)
* 3 14-segment alphanumeric LED elements for string output via scrolling text
* CAN bus communication with solderable termination jumper
* Onboard 12V to 5V switching converter plus overvoltage, transient, overcurrent, and reverse-voltage protection (i.e., Partially Student Resistant)
* A 9-DOF (accelerometer/gyro/magnetometer) inertial measurement unit with integrated sensor fusion
* A battery-backed Realtime Clock
* A Micro SD Card slot
* A 5-way Tactile Switch/Joystick
* Expansion capability via a 0.1" header at the bottom left, designed for a 16x2 character matrix or any I2C device

It's based on a combination of a Atmega328P (Arduino Uno) and a Atmega2560 (Arduino Mega) to ensure team members can program it into the future.

I've included a PDF to view the schematic without EagleCAD, and the models (STL/SLDPRT) of the enclosure pieces. The version 1 faceplate and backplate were designed for Formlabs clear resin, as the backplate's integrated cable-tie strain-relief loops would may break more easily on FDM prints. Newer designs (version 2+) use a thin waterjet polycarbonate faceplate for simplicity.

Feel free to contact me with any questions,

Clinton Flowers

clintonflowers222@gmail.com

   Copyright 2017 Clinton Flowers

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
