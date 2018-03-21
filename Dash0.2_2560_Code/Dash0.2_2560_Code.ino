//----------------------- MEGA/2560

/*
  KSU Motorsports/FSAE KS-2 Dashboard ATmega2560 Firmware
  Clinton Flowers, 2017
  For programming via ISP, see https://learn.sparkfun.com/tutorials/installing-an-arduino-bootloader/connecting-the-programmer
  Don't forget the capacitor on the reset pin of the programmer Arduino, and don't forget to use "Upload using Programmer"
  If you're using this for a student project (e.g., a Formula SAE car) and need help with it feel free to contact me @ clintonflowers222@gmail.com
  
  MSCan_Sniffer was used and is Copyright (c) 2013 David Will - davidwill@gmail.com The MIT License (MIT)
 */

// Note - CAN data is transmitted in the blind.
// Communications with Megasquirt can only be verified by *RECEIVED* data.
// To enable a megasquirt-compatible RPM read, uncomment below
#define MS_REQUEST

#include <SPI.h>
#include <Wire.h>
#include "RTClib.h" // Adafruit RTCLib Library, from Library Manager
#include <SD.h>
// IMU Includes
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor library, from Library Manager
#include <Adafruit_BNO055.h> // Adafruit BNO055 library, from Library Manager
#include <utility/imumaths.h>
//#define BNO055_SAMPLERATE_DELAY_MS (10) // 1s / 10ms = 100Hz

#define WATCHDOG
#ifdef WATCHDOG
#include <avr/wdt.h> // Watchdog timer for possible lock-ups
#endif

#define DEBUG
#ifdef DEBUG
#define debugShow(x) show(x)
#else
#define debugShow(x)
#endif

// Pin definitions specific to how the MCP2515 is wired up.
// If this code is being breadboarded off of the actual PCB: Pin 10 for SparkFun canbus shield
#define CS_PIN    36
#define CAN_INT_PIN   18 // INTERRUPT != PIN. 
/* 
 Board  int.0 int.1 int.2 int.3 int.4 int.5
 Uno, Ethernet  2 3        
 Mega2560 2 3 21  20  19  18
 Leonardo 3 2 0 1 7  
*/

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
const int numberOfAlphanumerics = 3;
// Vars for data passed to shifting function
byte dataOne, dataTwo, dataThree, dataFour, dataFive, dataSix;

//MCP2515 definitions, from the MSCAN_Sniffer Code
// May be improved by throwing all of this into an external included file
#define CAN_READ        0x03
#define CAN_WRITE       0x02
#define CANINTE         0x2B
#define CANINTF         0x2C
#define BFPCTRL         0x0C
#define CANCTRL         0x0F
#define CANSTAT         0x0E

#define CNF1            0x2A
#define CNF2            0x29
#define CNF3            0x28
#define RXB0CTRL        0x60
#define RXB1CTRL        0x70

// TX Buffer 0
#define TXB0CTRL        0x30
#define TXB0SIDH        0x31
#define TXB0SIDL        0x32
#define TXB0EID8        0x33
#define TXB0EID0        0x34
#define TXB0DLC         0x35
#define TXB0D0          0x36
#define TXB0D1          0x37
#define TXB0D2          0x38
#define TXB0D3          0x39
#define TXB0D4          0x3A
#define TXB0D5          0x3B
#define TXB0D6          0x3C
#define TXB0D7          0x3D
// RX Buffer 0
#define RXB0CTRL        0x60
#define RXB0SIDH        0x61
#define RXB0SIDL        0x62
#define RXB0EID8        0x63
#define RXB0EID0        0x64
#define RXB0DLC         0x65
#define RXB0D0          0x66
#define RXB0D1          0x67
#define RXB0D2          0x68
#define RXB0D3          0x69
#define RXB0D4          0x6A
#define RXB0D5          0x6B
#define RXB0D6          0x6C
#define RXB0D7          0x6D
// RX Buffer 1
#define RXB1CTRL        0x70
#define RXB1SIDH        0x71
#define RXB1SIDL        0x72
#define RXB1EID8        0x73
#define RXB1EID0        0x74
#define RXB1DLC         0x75
#define RXB1D0          0x76
#define RXB1D1          0x77
#define RXB1D2          0x78
#define RXB1D3          0x79
#define RXB1D4          0x7A
#define RXB1D5          0x7B
#define RXB1D6          0x7C
#define RXB1D7          0x7D

// Global vars aren't ideal but they simplify the coding process, especially with a substandard IDE or on a microcontroller.
byte SIDH, SIDL, EID8, EID0, DLC;
byte databuffer[7];
unsigned int data; 
byte block, canintf;
unsigned int offset;
byte dataready = 0;
long lastISR = 0;
long lastParse = 0;
volatile long lastMillisInt = 5000; // don't PWM the screen before this many ms
volatile bool lastMillisIntState = LOW;
volatile bool displayingText = true;
long lastShow = 0;

// Fields stored as received over CAN bus. Append new variables of interest here.
// After declaring the variable storage here, add code to request the associated address in the loop, and handle it in the ISR
unsigned int canRPM = 0;
unsigned const int shiftRPM = 12000;
String canVoltage = "0";
int canTPS = 0;
int canGear = 10; // Zeroeth gear = maybe neutral
int canCELStatus = 0; // CANbus Check Engine Light status
int canTPSADC = 0;
int canCELErrorCode = 0;
int canCLT = 0;
int canOLP = -1; // Oil pressure

// Clock (DS3231), IMU (BNO055) Setup
RTC_DS3231 rtc;
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> accel;
int latAccel = 0;
// I2C Addresses
int unoAddress = 5;
int megaAddress = 6;

void setup() {
  // Enable Watchdog Timer
  wdt_enable(WDTO_8S); // 8 seconds to allow for programming on ...most... bootloaders. :contingency:
  // TODO: For when the dash reboots mid-race due to any reason, maybe make this ask the other MCU its millis() over I2C to see if it should say "hooty hoo" etc, or jump right into displaying RPM
  
  pinMode(CS_PIN, OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  delay(1);
  // Check MCP2515 whitepaper for option definitions. These work for Megasquirt.
  CANWrite(CANCTRL, B10000111); // Set MCP2515 to config mode
  CANWrite(CNF1, 0x00); // CNF1 b00000000
  CANWrite(CNF2, 0xA4); // CNF2 b10100100
  CANWrite(CNF3, 0x84); // CNF3 b10000100
  CANWrite(CANCTRL, B00000111); // Set MCP2515 to normal mode
  delay(1);
  CANWrite(RXB0CTRL, B01101000); //RXB0CTRL clear receive buffers
  CANWrite(RXB1CTRL, B01101000); //RXB1CTRL clear receive buffers
  CANWrite(CANINTE, B00000011); // Enable interrupt on RXB0, RXB1
  CANWrite(BFPCTRL, B00001111); // setting interrupts 

  // Joystick interrupts. 
//  pinMode(centerPin, OUTPUT);
//  pinMode(upPin, INPUT_PULLUP);
//  pinMode(downPin, INPUT_PULLUP);
//  pinMode(leftPin, INPUT_PULLUP);
//  pinMode(rightPin, INPUT_PULLUP);
//  digitalWrite(centerPin, LOW);  // Ground the "common" pin (as installed)
//  attachInterrupt(digitalPinToInterrupt(downPin), resetFunction, FALLING);
//  attachInterrupt(digitalPinToInterrupt(upPin), upFunc, FALLING);
//  attachInterrupt(digitalPinToInterrupt(downPin), downFunc, FALLING);
//  attachInterrupt(digitalPinToInterrupt(leftPin), leftFunc, FALLING);
//  attachInterrupt(digitalPinToInterrupt(rightPin), rightFunc, FALLING);

  delay(1);
  Wire.begin(megaAddress);                // Join I2C bus with the address. See 328p code to guarantee its address if uncertain. 
  Wire.setClock(400000L);

  // Config for shiftOut, used for LED drivers
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(blankPin, OUTPUT);

  // Timer interrupt on millis. IIRC this is used for TIMER0_COMPA_vect, to dim the alphanumerics by putting them on a ~50% duty cycle.
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  // IMU Stuff
  if(!bno.begin()){
    showText("NIM"); // No BNO/IMU Detected. Hardware problem!
    delay(200);
  }
  delay(2);
  bno.setExtCrystalUse(true);
  delay(2);
  
  // DS3231 Clock Stuff
  if (! rtc.begin()) {
    showText("NCK"); // The Clock doesn't work. Hardware problem!
    delay(200);
  }
  
  if (rtc.lostPower()) {
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // TODO: Make a clock synchronization procedure 
  }
  delay(1);
  DateTime now = rtc.now();
  delay(1);
  //showText("T " + String(now.hour()) + "." + String(now.minute()) + "." + String(now.second())); // Display the time (uncomment when setting time)
  delay(1);

  // SD Card / Datalogging Stuff
//  if (!SD.begin(A8)) { // The SD card CS line is on pin PC0/Arduino 37/A8 on Dash v0.2
//    showText("NSD");  // No SD card. User or hardware problem.
//    delay(200);
//  }else{
//    showText("SD.");
//    delay(200);
//  }
 
  showText("Hooty Hoo 1.8"); // Firmware version number.
  delay(100);

  // CAN Interrupt setup and CAN stuff
  pinMode(CAN_INT_PIN, INPUT_PULLUP); // Maybe not necessary, but shouldn't hurt
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), ISR_can, FALLING);
  for(int i = 0; i < 3; i++){
    MS_Parse();
    delay(1);
  }
} // End setup()

void loop() {
  // Clear watchdog, delaying forced resets. Sometimes called "kicking" the watchdog
#ifdef WATCHDOG
  wdt_reset();
#endif
// Check engine light test code. This is commented out since I couldn't get the ECU to actually report check engine light status from its CAN address(es)... Maybe it's bugged?
//  if(canCELStatus > 0){ // If there's a CEL status other than 0, display that fact
//    showText("CHK.");
//    delay(100);
//    showText(String(canCELStatus));
//    delay(100);
//  } else if(canCELErrorCode > 0){ // Check for error codes from the other CAN address
//    showText("CEL");
//    delay(100);
//    showText(String(canCELErrorCode));
//    delay(100);
//  } else{
//  }
  sendRpm(); // Send RPM to the 328P via I2C
  delay(1);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  delay(1); // Allow time for fetching data from the IMU
  latAccel = accel.y();
  if(latAccel > 5){ // Display lateral G's if turning hard enough (over 5 m/s^2)
    float latGs = accel.y() / 9.81;
    showText(String(latGs));
  }else if(millis() % 5000 < 2500){
    // If the modulo of the time the dash has been on is < 2.5s, show voltage, otherwise show coolant temp.
    // (That's the programming way of saying alternate showing Batt and CLT every 2.5s)
    showText(String(canVoltage));
  }else{
    showText(String(canCLT));   // Coolant Temp
    //showText(String(canOLP)); // Oil Pressure
  }
    
  
  if (dataready == 1) {
    //MS_Parse(); // Deal with data if it's arrived (shouldn't be necessary since ISR_CAN is working)
    //dataready = 0;
  }else{
    #ifdef MS_REQUEST
    // See http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf for addresses
    MSrequest(7, 24, 2); // MS3 1.3.2 Firmware TPS - block 7, offset 24, 2 byte request
    delay(1); // Don't request stuff over CAN before it can be transmitted, received, and processed. Usually happens way under 1 ms--see oscilloscope traces.
    MSrequest(7, 26, 2); // MS3 1.3.2 Firmware batt - block 7, offset 26, 2 byte request
    delay(1);
    MSrequest(7, 6, 2); // MS3 1.3.2 Firmware RPM - block 7, offset 6, 2 byte request
    delay(1);
    MSrequest(7, 22, 2); // MS3 1.3.2 Firmware CLT - block 7, offset 22, 2 byte request
    delay(1);
    //MSrequest(7, 106, 2); // MS3 1.3.2 Firmware Generic In (Oil Pressure!(?)) - block 7, offset 106, 2 byte request
    //delay(1);
    //MSrequest(7, 270, 1); // MS3 1.3.2 Firmware gear - block 7, offset 270, 1 byte request
    //delay(1); 
    //MSrequest(7, 434, 2); // MS3 1.3.2 Firmware CEL Status - block 7, offset 434, 2 byte request
    //delay(1);
    //MSrequest(7, 226, 2); // MS3 1.3.2 Firmware TPSADC - block 7, offset 226, 2 byte request
    //delay(1);
    //MSrequest(7, 415, 1); // MS3 1.3.2 Firmware CEL Error Code - block 7, offset 415, 1 byte request
    //delay(1);
    #endif
  }
} // End loop()

void resetFunction() { // Software reset - first, try the watchdog timer
  wdt_disable();
  wdt_enable(WDTO_15MS);
  delay(16000); // Try to suicide via watchdog
  while(1){};
} // End reset function

// For debugging/testing the joystick
void upFunc(){
  showText("U");
  delay(100);
}
void downFunc(){
  showText("D");
  delay(100);
}
void leftFunc(){
  showText("L");
  delay(100);
}
void rightFunc(){
  showText("R");
  delay(100);
}

// I2C RequestEvent - Any time the 328P (assumed) requests data from the 2560 over I2C, respond with data
// In the later versions, this is not used, since the 2560 sends the 328P the RPM on its own time. Could delete.
void requestEvent(){
  if(canRPM == 0){ // The RPM is reading zero. Display TPS!
    Wire.write(map(canTPS, 0, 1100, 0, 255)); 
  }else{
    // Return the RPM value
    Wire.write(map(canRPM, 0, 12000, 0, 255));
  }
} // End requestEvent()

// I2C Send RPM - When called, transmits the current RPM to the I2C Slaved 328P
void sendRpm(){
  // ledArgs format: setPixelGroup(int beginPixels, int endPixels, int delayTime, int red, int green, int blue, int mode). See 328p code for most up-to-date format.
  int ledArgs[7] = {1, 1, 0, 0, 40, 10, 0};
  Wire.beginTransmission(unoAddress);
  if(canRPM == 0){ // The RPM is reading zero - The engine is ostensibly off. Display TPS or something!
    ledArgs[1] = map(canTPS, 0, 1050, 0, 10); // Display a TPS reading
    if(canTPS == 0){
      // The TPS and RPM are both exactly zero, so CAN probably isn't working or the ECU is off. 
      // Might as well display the lateral acceleration or something so we know the dash isn't broken when debugging on a bench.
      ledArgs[1] = abs(latAccel);
      if(latAccel > 8){
        showText("PLS");
        delay(200);
        showText("NO");
        delay(200);
        showText("HIT");
        delay(200);
      }
    }
  }else{
    // Display the RPM value
    ledArgs[3] = 128;   // Red
    ledArgs[4] = 50;    // Green
    ledArgs[5] = 0;     // Blue
    ledArgs[1] = map(canRPM, 0, shiftRPM, 0, 10);
    if(canRPM > shiftRPM){ // We're over 'n'k RPM, display hard red
      ledArgs[3] = 64;
      ledArgs[4] = 128;
      ledArgs[5] = 255;
    }else if(canRPM > 15000){
      resetFunction(); // something's broke or breaking yo
    }
  }
  // Send the LED values to the 328P for display
  for(int i = 0; i < 7; i++){
    Wire.write(ledArgs[i]);
  }
  Wire.endTransmission();
} // End sendRpm()


// Main CAN Requester Function, from David Will's MSCan library
void MSrequest(byte block, unsigned int offset, byte req_bytes) {
  /* Request data in Megasquirt format */
  byte SIDH, SIDL, EID8, EID0, DLC, D0, D1, D2;
  //
  SIDH = lowByte(offset  >> 3);
  // var_offset<2:0> SRR IDE msg_type <3:0>
  SIDL = (lowByte((offset << 5)) | B0001000); //set IDE bit
  //      MFFFFTTT msg_type, From, To
  EID8 = B10011000; //:7 msg_req, from id 3 (4:3)
  //      TBBBBBSS To, Block, Spare
  EID0 = ( ( block & B00001111) << 3); // last 4 bits, move them to 6:3
  EID0 = ((( block & B00010000) >> 2) | EID0); // bit 5 goes to :2
  //
  DLC = B00000011;
  D0=(block);
  D1=(offset >> 3);
  D2=(((offset & B00000111) << 5) | req_bytes); // shift offset
  // Disable interrupts to prevent locking up during SPI transfers
  noInterrupts();
  //  digitalWrite(CS_PIN,LOW);
  PORTC &= B11111101; // Set CS pin (port C1, pin 36) low, starting a write
  SPI.transfer(0x40); // Push bits starting at 0x31 (RXB0SIDH)
  SPI.transfer(SIDH); //0x31
  SPI.transfer(SIDL); //0x32
  SPI.transfer(EID8); //0x33
  SPI.transfer(EID0); //0x34
  SPI.transfer(DLC);  //0x35
  SPI.transfer(D0); // 0x36 TXB0D0 my_varblk
  SPI.transfer(D1); // 0x37 TXB0D1 my_offset
  SPI.transfer(D2); // 0x38 TXB0D2 - request 8 bytes(?) from MS3
  //  digitalWrite(CS_PIN,HIGH); // end write
  PORTC |= B00000010; // Set CS pin (port C1, pin 36) high, ending the write
  
  // RTS - Send this buffer down the wire
  //  digitalWrite(CS_PIN,LOW);
  PORTC &= B11111101; // Set CS pin (port C1, pin 36) low, starting a write
  SPI.transfer(B10000001);
  //  digitalWrite(CS_PIN,HIGH);
  PORTC |= B00000010; // Set CS pin (port C1, pin 36) high, ending the write
  interrupts(); // *Don't* move this below CANWrite. Though maybe remove the redundant CANWrite.
  CANWrite(CANINTF,0x00); // Reenabling interrupts above this makes it freeze occasionally, 
                          // but enabling them afterwards below slows loops down a lot
} // End MSrequest()


// CAN Interrupt Service Routine Handler
// Any time the MCP2515 pings the interrupt pin of the 2560, indicating a message is ready in the CAN buffer, this is run
void ISR_can() {
  //Buffer variables are global.
  byte temp;
  lastISR = millis();

  canintf=CANRead(CANINTF); // Reading Interrupt to determine which buffer is full.

  if (canintf & B00000001) {
    SIDH=CANRead(RXB0SIDH);
    SIDL=CANRead(RXB0SIDL);
    EID8=CANRead(RXB0EID8);
    EID0=CANRead(RXB0EID0);
    DLC=CANRead(RXB0DLC);
    databuffer[0]=CANRead(RXB0D0);
    databuffer[1]=CANRead(RXB0D1);
    databuffer[2]=CANRead(RXB0D2);
    databuffer[3]=CANRead(RXB0D3);
    databuffer[4]=CANRead(RXB0D4);
    databuffer[5]=CANRead(RXB0D5);
    databuffer[6]=CANRead(RXB0D6);
    databuffer[7]=CANRead(RXB0D7);
  } 
  else if (canintf & B00000010) {
    SIDH=CANRead(RXB1SIDH);
    SIDL=CANRead(RXB1SIDL);
    EID8=CANRead(RXB1EID8);
    EID0=CANRead(RXB1EID0);
    DLC=CANRead(RXB0DLC);
    databuffer[0]=CANRead(RXB1D0);
    databuffer[1]=CANRead(RXB1D1);
    databuffer[2]=CANRead(RXB1D2);
    databuffer[3]=CANRead(RXB1D3);
    databuffer[4]=CANRead(RXB1D4);
    databuffer[5]=CANRead(RXB1D5);
    databuffer[6]=CANRead(RXB1D6);
    databuffer[7]=CANRead(RXB1D7);
  }

  block=((B01111000 & EID0) >> 3);
  temp=0x00;
  temp=((B00000100 & EID0) << 3);
  block=block | temp;

  offset=SIDH;
  temp=((SIDL & B11100000) >> 5);
  offset=((offset << 3) | temp);

  dataready = 1; // set flag to run parser in loop
  CANWrite(CANINTF, 0x00); // clear interrupt

  MS_Parse();
} // END ISR_can()


void MS_Parse() {
  dataready = 0;
  lastParse = millis();
  
  data=databuffer[0];
  data=((data << 8) | databuffer[1]);

  byte datalength=(DLC & 0x0F);

  byte temp;
  int var_offset;
  var_offset=SIDH;
  temp=((SIDL & B11100000) >> 5);
  var_offset=((var_offset << 3 )| temp);

  int msg_type;
  msg_type=((B00000011 & SIDL) << 1);
  temp=0x00;
  temp=((B10000000 & EID8) >> 7);
  msg_type=msg_type | temp;

  byte from_id, to_id;
  from_id=((B01111000 & EID8) >> 3);
  to_id=((B00000111 & EID8) << 1);
  temp=0x00;
  temp=((B10000000 & EID0) >> 7);
  to_id=to_id | temp;

  int var_block;
  var_block=((B01111000 & EID0) >> 3);
  temp=0x00;
  temp=((B00000100 & EID0) << 3);
  var_block=var_block | temp;

// Determine what data was received and update the associated (global) variable
// This code is, how you say... inelegant
  if(var_offset == 6){            // RPM
    canRPM = data;
  }else if(var_offset == 24){     // TPS
    canTPS = data;
  }else if(var_offset == 26){     // Battery Voltage
    canVoltage = String(((double)data)/10).substring(0,4);
  }else if(var_offset == 270){    // Gear
    canGear = databuffer[0];
  }else if(var_offset == 434){    // CEL Status(?)
    canCELStatus = data;
  }else if(var_offset == 226){    // TPSADC
    canTPSADC = data;
  }else if(var_offset == 22){     // CLT
    canCLT = data / 10;
  } 
//  if(var_offset == 106){   // Oil Pressure
//    canOLP = data / 10;
//  }
//  if(canCELErrorCode == 415){   // CEL Error Code
//    canCELErrorCode = databuffer[0];
//  }
  
  
  if(String(data).length() < 5){
    //showText(String(data));
  }else{
    showText(" . . .");
  }
} // End MS_Parse()


void CANWrite(byte addr, byte data) {
  digitalWrite(CS_PIN,LOW);
  noInterrupts(); // Necessary to avoid freezing during SPI transfer (due to MS_Parse and ISR_can both clearing the interrupt flag)
  SPI.transfer(CAN_WRITE);
  SPI.transfer(addr);
  SPI.transfer(data);
  interrupts();
  digitalWrite(CS_PIN,HIGH);
} // End CANWrite


byte CANRead(byte addr) {
  byte data;
  digitalWrite(CS_PIN,LOW);
  SPI.transfer(CAN_READ);
  SPI.transfer(addr);
  data=SPI.transfer(0x00);
  digitalWrite(CS_PIN,HIGH);
  return data;
} // End CANRead


 // A method to simplify showing text on the 3 alphanumerics
void showText (String whatToWrite){
  int decimalCount = 0;
  String whatToActuallyWrite; // Used after decimal points have been accounted for and merged
  String decimals;
  
  if(whatToWrite.length() > numberOfAlphanumerics*2){
    // The string definitely needs to be scrolled out since there can only be 1 decimal per alphanumeric character
    scrollText(whatToWrite);
    return;
  }
  // Deal with decimals (e.g., printing "12.4" can fit on 3 alphanumeric displays, so merge the decimal)
  for(int i = 0; i < whatToWrite.length(); i++){
    // Append the character to the portion displayed, dealing with decimals
    decimals += ' ';
    if(whatToWrite[i] == '.'){
      // Any time this loop finds a decimal, go to the last 'valid' char and append one
      // (This implementation discards any leading decimals)
      decimals[whatToActuallyWrite.length() - 1] = '.'; 
    }else{
      whatToActuallyWrite += whatToWrite[i];
    }
  }
  if(whatToActuallyWrite.length() <= numberOfAlphanumerics){
    show(whatToActuallyWrite, decimals);
    return;
  }else{
    scrollText(whatToWrite);
    return;
  }
} // End showText()


// Default method for scrolled text
void scrollText(String input){
  scrollText(input, 150, numberOfAlphanumerics);
} // End scrollText(String)


// Scroll the text from left to right across the displays.
// This is done by calling show() on one set of characters,
// shifting left by one character, and then repeating. 
void scrollText(String whatToScroll, int delayTime, int addedPadding){
  int decimalCount = 0;
  String whatToActuallyWrite;
  String decimals;
  // Add padding to the scrolled text based on input request
  for(int i = 0; i < addedPadding; i++){
    whatToScroll = " " + whatToScroll + " ";
  }
  for(int i = 0; i < whatToScroll.length(); i++){
    // Append the character to the portion displayed, dealing with decimals
    decimals += ' ';
    if(whatToScroll[i] == '.'){
      // Any time this loop finds a decimal, go to the last valid char and "place" one
      // (Note: This implementation discards any leading or standalone decimals)
      decimals[whatToActuallyWrite.length() - 1] = '.'; 
    }else{
      whatToActuallyWrite += whatToScroll[i];
    }

    // Send the latest group of 3 alphanumerics to be shown, and remove the earliest character.
    if(whatToActuallyWrite.length() >= numberOfAlphanumerics){
      show(whatToActuallyWrite, decimals);
      decimals.remove(0,1);
      whatToActuallyWrite.remove(0,1);  
      delay(delayTime); // This delay will cause the main loop to stop operating (and thus updating LED's) if text that's too long is being shifted out. Interrupts might be a better choice.
    }
  }
} // End scrollText(String, int, int)


// Time-based method to show text
void show(String whatToActuallyWrite, String decimals){
  // Example Inputs: "123", "1.234", "1.23", "Hello World"
  if(whatToActuallyWrite.length() > numberOfAlphanumerics){
    // The string still needs to be scrolled out.
    return;
  }
  
  // For this version we shift everything to uppercase (until lowercase values are added to the arrays above)
  whatToActuallyWrite.toUpperCase(); // "As of 1.0, toUpperCase() modifies the string in place rather than returning a new one."
  
  // At this point, the string can (probably) be displayed on 3 characters, so do that. 
  // Loop for each of the 3 digits
  byte topDatas[] = {dataTwo, dataFour, dataSix};
  byte bottomDatas[] = {dataOne, dataThree, dataFive};
  // Load the input sequence into the given characters
  // i is the loop for each of the (nominally 3) alphanumeric characters
  for(int i = 0; i < numberOfAlphanumerics; i++){
    for(int j = 0; j < sizeof(characterList); j++){
      if(whatToActuallyWrite[i] == characterList[j]){
        // The first character is the same as the current character. 
        // Load that into the top and bottom bytes.
        bottomDatas[i] = characterBottoms[j];
        topDatas[i] = characterTops[j];
      }
    }
    if(decimals[i] == '.'){
      bottomDatas[i] |= 0x10;  // Append the decimal to this character
    }
  }

  // Ground latchPin and hold low for as long as you are transmitting
  digitalWrite(latchPin, 0);
  // Shift the bytes out
    for(int i =  numberOfAlphanumerics - 1; i >= 0; i--){
      shiftOut(dataPin, clockPin, topDatas[i]);
      shiftOut(dataPin, clockPin, bottomDatas[i]);
    }
  // Return the latch pin high to signal chip that it no longer needs to listen for information
  digitalWrite(latchPin, 1);
  return true;
} // End show()


// The method that actually shifts out the raw data to the shift registers/LED drivers
void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first, on the rising edge of the clock, clock idles low
  // Internal function setup
  int i=0;
  int pinState;
  // Clear everything out just in case to prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);
  //For each bit in the byte myDataOut notice that we are counting down in our for loop
  //This means that %00000001 or "1" will go through such that it will be pin Q0 that lights. 
  for (i = 7; i >= 0; i--)  {
    digitalWrite(myClockPin, 0);
    // If the value passed to myDataOut and a bitmask result true then... so if we are at i=6 and our value is %11010100 it would the code compares it to %01000000 and proceeds to set pinState to 1.
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {  
      pinState= 0;
    }
    digitalWrite(myDataPin, pinState);  //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myClockPin, 1);        //register shifts bits on upstroke of clock pin  
    digitalWrite(myDataPin, 0);         //zero the data pin after shift to prevent bleed through
  }

  // Stop shifting
  digitalWrite(myClockPin, 0);
} // End shiftOut()

// PWM the alphanumeric blank pin to dim it, via interrupts. Otherwise the alphanumeric LED's get fairly (or very) hot with the chosen current set resistor.
SIGNAL(TIMER0_COMPA_vect) {
  PORTK ^= B00001000;  // Toggle K3 aka blankPin
} // End SIGNAL() interrupt



