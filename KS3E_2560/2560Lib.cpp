#include "2560Lib.h"

// Vars for data passed to shifting function
byte dataOne, dataTwo, dataThree, dataFour, dataFive, dataSix;

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

// Time-based method to show text
void show(String whatToActuallyWrite, String decimals){
  // Example Inputs: "123", "1.234", "1.23", "Hello World"
  if(whatToActuallyWrite.length() > NUMBER_OF_ALPHANUMERICS){
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
  for(int i = 0; i < NUMBER_OF_ALPHANUMERICS; i++){
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
    for(int i =  NUMBER_OF_ALPHANUMERICS - 1; i >= 0; i--){
      shiftOut(dataPin, clockPin, topDatas[i]);
      shiftOut(dataPin, clockPin, bottomDatas[i]);
    }
  // Return the latch pin high to signal chip that it no longer needs to listen for information
  digitalWrite(latchPin, 1);
  return true;
} // End show()


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
    if(whatToActuallyWrite.length() >= NUMBER_OF_ALPHANUMERICS){
      show(whatToActuallyWrite, decimals);
      decimals.remove(0,1);
      whatToActuallyWrite.remove(0,1);  
      delay(delayTime); // This delay will cause the main loop to stop operating (and thus updating LED's) if text that's too long is being shifted out. Interrupts might be a better choice.
    }
  }
} // End scrollText(String, int, int)


// Default method for scrolled text
void scrollText(String input){
  scrollText(input, 150, NUMBER_OF_ALPHANUMERICS);
} // End scrollText(String)


 // A method to simplify showing text on the 3 alphanumerics
void showText (String whatToWrite){
  int decimalCount = 0;
  String whatToActuallyWrite; // Used after decimal points have been accounted for and merged
  String decimals;
  
  if(whatToWrite.length() > NUMBER_OF_ALPHANUMERICS*2){
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
  if(whatToActuallyWrite.length() <= NUMBER_OF_ALPHANUMERICS){
    show(whatToActuallyWrite, decimals);
    return;
  }else{
    scrollText(whatToWrite);
    return;
  }
} // End showText()

// PWM the alphanumeric blank pin to dim it, via interrupts. Otherwise the alphanumeric LED's get fairly (or very) hot with the chosen current set resistor.
SIGNAL(TIMER0_COMPA_vect) {
  PORTK ^= B00001000;  // Toggle K3 aka blankPin
} // End SIGNAL() interrupt
