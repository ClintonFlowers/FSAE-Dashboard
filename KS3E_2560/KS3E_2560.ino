//----------------------- MEGA/2560

/*
  KSU Motorsports/FSAE KS-3E Dashboard ATmega2560 Firmware
  Clinton Flowers, 2018
  For programming via ISP, see https://learn.sparkfun.com/tutorials/installing-an-arduino-bootloader/connecting-the-programmer
  Don't forget the capacitor on the reset pin of the programmer Arduino, and don't forget to use "Upload using Programmer"
  
  If you're using this for a student project (e.g., a Formula SAE car) and need help with it feel free to contact me @ clintonflowers222@gmail.com
  
  Remember, a design goal of this dash was to display only critical information the driver needs to know; K.I.S.S. applies. If this changes, consider Waveshare panels.
  
  MSCan_Sniffer was used and is Copyright (c) 2013 David Will - davidwill@gmail.com The MIT License (MIT)
 */

// Note - CAN data is transmitted in the blind.
// Communications with Megasquirt can only be verified by *RECEIVED* data.
#define MS_REQUEST

#include "Tasker.h"
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h" // Adafruit RTCLib Library, from Library Manager
#include <SD.h>
#include <CAN.h>
// IMU Includes
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor library, from Library Manager
#include <Adafruit_BNO055.h> // Adafruit BNO055 library, from Library Manager
#include <utility/imumaths.h>
//#define BNO055_SAMPLERATE_DELAY_MS (10) // 1s / 10ms = 100Hz

#include <avr/wdt.h> // Watchdog timer for possible lock-ups

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

Tasker tasker;

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

// Some more global vars
// Received CAN data
String canText = "";
int canRed = 0;
int canGreen = 20;
int canBlue = 5;
int canDisplayedSegments = 0;
// Interrupt and flow control variables
byte dataready = 0;
long lastISR = 0;
long lastParse = 0;
volatile long lastMillisInt = 5000; // don't PWM the screen before this many ms
volatile bool lastMillisIntState = LOW;
volatile bool displayingText = true;
long lastShow = 0;

// One better way to do this would be object-oriented, where the CAN fields are instantiated with name, address, etc, and add themselves to an array of CAN field objects

// Clock (DS3231), IMU (BNO055) Setup
RTC_DS3231 rtc;
Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> accel;
int latAccel = 0;
// I2C Addresses
const int unoAddress = 5;
const int megaAddress = 6;

#include "2560Lib.h"

void setup() {
  // Enable Watchdog Timer
  wdt_enable(WDTO_2S); // 8 seconds to allow for programming on ...most... bootloaders. :contingency:
  // TODO: For when the dash reboots mid-race due to any reason, maybe make this ask the other MCU its millis() over I2C to see if it should say "hooty hoo" etc, or jump right into displaying RPM

  tasker.init();

  pinMode(CS_PIN, OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  delay(1);

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
  // See the IR Tire Temp Sensor code on how to create files and handle appending/creating CSVs. Ensure the clocks are synced (DS3231 drifts ~5 seconds/month)
//  if (!SD.begin(A8)) { // The SD card CS line is on pin PC0/Arduino 37/A8 on Dash v0.2
//    showText("NSD");  // No SD card. User or hardware problem.
//    delay(200);
//  }else{
//    showText("SD.");
//    delay(200);
//  }
 
  //showText("Hooty Hoo 1.9"); // Firmware version number.
  delay(100);

  // CAN Interrupt setup and CAN stuff
//  pinMode(CAN_INT_PIN, INPUT_PULLUP); // Maybe not necessary, but shouldn't hurt
//  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), ISR_can, FALLING);
  //if (!CAN.begin(500E3)) {
  //  showText("NCN"); // The CAN doesn't work. Hardware problem!
  //  delay(100);
  //}
  //CAN.onReceive(ISR_can);
//  for(int i = 0; i < 3; i++){
//    MS_Parse();
//    delay(1);
//  }

  
} // End setup()

void loop() {
  // Clear watchdog, delaying forced resets. Sometimes called "kicking" the watchdog
  wdt_reset();
  tasker.taskLoop();
  updateNeopixels(); // Update LEDs of the 328P via I2C
  delay(1);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  delay(1); // Allow time for fetching data from the IMU
  latAccel = accel.y();
  if(latAccel > 10){ // Display lateral G's if turning hard enough (over 5 m/s^2)
    float latGs = accel.y() / 9.81;
    showText(String(latGs));
  }else if(millis() % 5000 < 2500){
    // If the modulo of the time the dash has been on is < 2.5s, show voltage, otherwise show coolant temp.
    // (That's the programming way of saying alternate showing Batt and CLT every 2.5s)
    showText(String(canText));
  }
  
  if (dataready == 1) {
    //MS_Parse(); // Deal with data if it's arrived (shouldn't be necessary since ISR_CAN is working)
    //dataready = 0;
  }
} // End loop()

// I2C Send RPM - When called, transmits the current RPM to the I2C Slaved 328P
void updateNeopixels(){
  // ledArgs format: setPixelGroup(int beginPixels, int endPixels, int delayTime, int red, int green, int blue, int mode). See 328p code for most up-to-date format.
  int ledArgs[7] = {1, 1, 0, canRed, canGreen, canBlue, 0};
  Wire.beginTransmission(unoAddress);
  if(canText == ""){ // The dash has not received a message from the VCU
    ledArgs[1] = (millis() / 1000) % 10;
    if(abs(latAccel) > 5.0){
      // Might as well display the lateral acceleration or something so we know the dash isn't broken when debugging on a bench.
      ledArgs[1] = abs(latAccel);
    }
  }else{
    // Display the RPM value
    ledArgs[3] = canRed;   // Red
    ledArgs[4] = canGreen;    // Green
    ledArgs[5] = canBlue;     // Blue
    ledArgs[1] = canDisplayedSegments;
  }
  // Send the LED values to the 328P for display
  for(int i = 0; i < 7; i++){
    Wire.write(ledArgs[i]);
  }
  Wire.endTransmission();
} // End updateNeopixels()


// CAN Interrupt Service Routine Handler
// Any time the MCP2515 pings the interrupt pin of the 2560, indicating a message is ready in the CAN buffer, this is run
// Handle new CAN messages
void ISR_can(int packetSize) {

  Serial.print("Received ");
    
  if (CAN.packetExtended()) {
    showText("EX");
//    Serial.print("extended ");
  }

  if (CAN.packetRtr()) {
    showText("RRR");
    // Remote transmission request, packet contains no data
//    Serial.print("RTR ");
  }

//  Serial.print("packet with id 0x");
//  Serial.print(CAN.packetId(), HEX);

  int currentByte = 0;
  byte bytes [packetSize];

  if (CAN.packetRtr()) {
    showText("RTR");
//    Serial.print(" and requested length ");
//    Serial.println(CAN.packetDlc());
  } else {
//    Serial.print(" and length ");
//    Serial.println(packetSize);

    // only print packet data for non-RTR packets
    while (CAN.available()) {
      bytes[currentByte] = ((byte)CAN.read());
      currentByte++;
//      Serial.print(bytes[currentByte]);
//      Serial.print(" ");
    }
//    Serial.println();
  }

   if(bytes[0] == (byte)0x01){
//      Serial.println("RPM Received: " + String(0 + bytes[1] + (bytes[2] >> 8)));
      showText("SD.");
      char ch[]={bytes[1],bytes[2],bytes[3]};
      String toDisplay(ch);
      canText = toDisplay;
      showText(toDisplay);
   }else{
    showText("NO.");
   }

//  Serial.println();
//  MS_Parse();
} // END ISR_can()

// Parse new CAN messages
void MS_Parse() {
  
  dataready = 0;
  lastParse = millis();
} // End MS_Parse()
