//----------------------- UNO/328P
// KSU Motorsports/FSAE KS-2 Dashboard ATmega328P Firmware
// Clinton Flowers, 2017

#include <Wire.h>

#include <Adafruit_NeoPixel.h> // Get this from the Arduino IDE Library Manager
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            8

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      20
#define numRows 2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second
int8_t temp = 50;
int unoRpms = 0; // Local storage of engine RPM's
int megaAddress = 6;
int unoAddress = 5;
int ledArgs[7];

void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
  Wire.begin(unoAddress); // Join the I2C bus
  Wire.setClock(400000L);
  Wire.onReceive(receiveRpmHandler);
} // End setup()

void loop() {
  delay(1); // The latest code version is entirely based on I2C-initiated interrupts; see receiveRpmHandler
} // End loop()

void setPixelGroup(int beginPixels, int endPixels, int delayTime, int red, int green, int blue, int mode){
  // Multiple "modes" for different driver preferences on how the lights light up
  if(mode == 0){
   for(int i=beginPixels-1;i<endPixels;i++){
      // Light up each column of the lights, from left to right
      pixels.setPixelColor(i, pixels.Color(red,green,blue)); // Set top row
      pixels.setPixelColor((NUMPIXELS/(numRows-1))-1 - i, pixels.Color(red,green,blue)); // Set bottom row
      if(delayTime > 0){
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(delayTime); // Delay for a period of time (in milliseconds).
      }   
    } 
  }else if(mode == 1){
    for(int i=beginPixels-1;i<endPixels;i++){
      // Light up the columns of the shift lights, from left and right to center
      pixels.setPixelColor(i, pixels.Color(red,green,blue)); // Set top row left
      pixels.setPixelColor((NUMPIXELS/numRows)-1 - i, pixels.Color(red,green,blue)); // Set top row right
      pixels.setPixelColor((NUMPIXELS/(numRows-1))-1 - i, pixels.Color(red,green,blue)); // Set bottom row left
      pixels.setPixelColor(i + (NUMPIXELS/numRows), pixels.Color(red,green,blue)); // Set bottom row right
      if(delayTime > 0){
        pixels.show(); // This sends the updated pixel color to the hardware.
        delay(delayTime); // Delay for a period of time (in milliseconds).
      }
    }
  }
  pixels.show(); // This sends the updated pixel color to the hardware.
} // End setPixelGroup(int, int, int, int, int, int, int)

// Function that executes whenever data is received from the I2C master 2560 uC
// This function is registered as an event/ISR, see setup()
void receiveRpmHandler(int howMany) {
  for(int i = 0; i < sizeof(ledArgs); i++){
    ledArgs[i] = Wire.read();
  }
  pixels.clear();
  setPixelGroup(ledArgs[0], ledArgs[1], ledArgs[2], ledArgs[3], ledArgs[4], ledArgs[5], ledArgs[6]);
} // End receiveRpmHandler()

