# KSU Motorsports Dashboard & Data Logger

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

It's based on a combination of a Atmega328P (Arduino Uno) and a Atmega2560 (Arduino Mega) to ensure team members can program it into the future.

I've included a PDF to view the schematic without EagleCAD, and the models (STL/SLDPRT) of the enclosure pieces. The faceplate and backplate were designed for Formlabs clear resin, as the backplate's integrated cable-tie strain-relief loops would break on a FDM print and the faceplate obviously can't be opaque. Newer designs will use a thin waterjet polycarbonate faceplate for simplicity.

All portions of the hardware work individually, but the final human-readable firmware is still in flux, pending a working racecar :)

Feel free to contact me with any questions (and job offers are always welcome),
Clinton Flowers
clintonflowers222@gmail.com
