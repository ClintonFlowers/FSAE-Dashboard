#ifndef MEGA_LIB
#define MEGA_LIB

#include <Arduino.h>

#define NUMBER_OF_ALPHANUMERICS 3

// These arrays are used to map the alphanumeric LED's to the associated code character. 
// Each bit position corresponds to a segment of the LED; adding the correct bits together and then shifting that byte into the driver will light up the segments for that character
// Since there are 15 positions per digit, we use 2 8-bit bytes, so a Top and a Bottom
const char characterList[] =    {' ',  '$',  '0',  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  'A',  'B',  'C',  'D',  'E',  'F',  'G',  'H',  'I',  'J',  'K',  'L',  'M',  'N',  'O',  'P',  'Q',  'R',  'S',  'T',  'U',  'V',  'W',  'X',  'Y',  'Z'};
const byte characterBottoms[] = {0x00, 0x65, 0xe2, 0x20, 0xc1, 0x61, 0x21, 0x61, 0xe1, 0x20, 0xe1, 0x21, 0xa1, 0x65, 0xc0, 0x64, 0xc1, 0x81, 0xe0, 0xa1, 0x44, 0xe0, 0x89, 0xc0, 0xa0, 0xa8, 0xe0, 0x81, 0xe8, 0x89, 0x60, 0x04, 0xe0, 0x82, 0xaa, 0x0a, 0x05, 0x42};
const byte characterTops[] =    {0x00, 0x6a, 0x56, 0x14, 0x0e, 0x0e, 0x4c, 0x4a, 0x4a, 0x06, 0x4e, 0x4e, 0x4e, 0x2e, 0x42, 0x26, 0x42, 0x42, 0x4a, 0x4c, 0x22, 0x04, 0x50, 0x40, 0x55, 0x45, 0x46, 0x4e, 0x46, 0x4e, 0x0b, 0x22, 0x44, 0x50, 0x44, 0x11, 0x4c, 0x12};

// Various control variables for the TLC59281RGER Constant Current LED Driver
const int blankPin = 65;
const int latchPin = 64;
const int clockPin = 63;
const int dataPin = 62;
// Pins for the joystick on the rear of the device, accounting for improper hardware placement. Unused in current code.
const int centerPin = PE6;  // Center in schematic, J_Common as installed. 
const int rightPin = 19;    // J_Left in schematic
const int leftPin = PE7;    // J_Right in schematic
const int downPin = 2;      // J_Up in schematic
const int upPin = 3;        // J_Down in schematic
// How many (14-segment + dp) alphanumeric displays are connected

extern byte dataOne, dataTwo, dataThree, dataFour, dataFive, dataSix;

extern void shiftOut(int myDataPin, int myClockPin, byte myDataOut);
extern void showText (String whatToWrite);


#endif
