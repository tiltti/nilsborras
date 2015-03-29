#include "LPD8806.h"
#include "SPI.h"
#include <Wire.h>
#include <LiquidCrystal_v1.h>

#define NUM_LEDS 160

/* 
 * Nilsborras v0.2 - Staircase lightning project
 *
 * @author: Ossi Tiltti / ossi.tiltti (_) gmail (_) com
 * @date:   3. September 2012
 *
 * released under a creative commons "Attribution-NonCommercial-ShareAlike 2.0" license
 * http://creativecommons.org/licenses/by-nc-sa/2.0/de/
 *
 * Thanks for Kritian Gohlke on the Parallax PIR sensor help! 
 * Check out his projects at http://krx.at/
 *  
 */

// VARS

// LCD
// Connect via i2c, default address #0 (A0-A2 not jumpered)
LiquidCrystal lcd(0);

// PIR
// the time we give the sensor to calibrate (10-60 secs according to the datasheet)
// TODO: Set to something reasonable aftesr debugging
int calibrationTime = 3;        

// LED
// Number of RGB LEDs in strand:
// Each meter has 32 leds thus 5 meters = 160 leds
// And when you accidentally insert 12V into your strip this is how much you have afterwards :)
int nLEDs = 150;

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 2000;  

boolean lockLow = true;
boolean takeLowTime;  
boolean lightStatus = false;
boolean lightsOn = false;

int dataPin  = 2;  // LED strip data pin
int clockPin = 3;  // LED strip clock pin
int pirPin1 = 7;   // the digital pin connected to the PIR sensor #1 output
int pirPin2 = 8;   // PIR sensor #2
int buzzPin = 9;   // piezo
int pirLedPin1 = 13;  // LED for pir sensor #1
int pirLedPin2 = 14;  // LED for pir sensor #2
int led2Pin = 12;  // led for detecting if ldr is on/off
int ldrPin = A0;   // ldr input pin

int lightTreshold = 700;  // sensor reading whether to lit the lights or not

unsigned int sensorValue = 0;  // variable to store the value coming from the ldr
long unsigned int low2In;   

// Create the actual strip object
LPD8806 strip = LPD8806(nLEDs, dataPin, clockPin);

//SETUP
void setup(){
  Serial.begin(9600);
  
  // Start up the LCD
  // set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setBacklight(HIGH);
  lcd.print("NilsBorras v1.0");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);

  // Start up the LED strip
  strip.begin();

  // Update the strip, to start they are all 'off'
  strip.show();

  // initialize pins
  
  // pir sensor #1 (Bottom of staircase)
  pinMode(pirPin1, INPUT);
  digitalWrite(pirPin1, LOW);

  // pir sensor #2 (Top of staircase)
  pinMode(pirPin2, INPUT);
  digitalWrite(pirPin2, LOW);

  pinMode(pirLedPin1, OUTPUT);
  pinMode(pirLedPin2, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(buzzPin, OUTPUT);
  pinMode(led2Pin, OUTPUT);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ambient light:");
  lcd.setCursor(0, 1);
  lcd.print("Checking status");
  delay(1000);
  Serial.println("Determining ambient light status...");

  // check the initial ambient light status
  lightStatus = checkLights(lightStatus);  

  //give the sensor some time to calibrate
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating:");
  Serial.print("Calibrating motion sensors ");
  for(int i = 0; i < calibrationTime; i++){
    Serial.print(".");
    lcd.setCursor(i, 1);
    lcd.print(".");
    delay(1000);
    beep(50,10,1);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("*** SENSORS ");
  lcd.setCursor(0, 1);
  lcd.print("    ACTIVE! ***");
  Serial.println(" done!");
  Serial.println("*** SENSORS ACTIVE ***");  
    for(int i =0; i < 5; i++) {
      beep(300,50,1);
    }
   delay(1000);
}


//LOOP
void loop(){
  // check the ambient light status
  lightStatus = checkLights(lightStatus);
  
  // check motion only if it's dark enough
  if(lightStatus) {
    // lit LCD
    lcd.setBacklight(HIGH);
    // sensor #1
    checkMotion(pirPin1);
    // sensor #2
    checkMotion(pirPin2);
  } else {
      // turn off LCD backlight
      // TODO: Might be good to keep always on, this LCD is not really readable without backlight
      // lcd.setBacklight(LOW);
  }
  delay(100);
}

void collisionAlert() {

  // if another motion sensor is trigged when lights are operating, sound collision alert :)
  // TODO: klaxhorn preferred

  wave(strip.Color(127,0,0), 4, 20);        // candy cane
}

// Sound beep via piezo element - useful for debugging movement etc.
void beep(unsigned char delayms, int freq, int times) {
  
  for(int i=0; i<times;i++) {
    analogWrite(buzzPin, freq);    // Almost any value can be used except 0 and 255
    delay(delayms);                // wait for a delayms ms
    analogWrite(buzzPin, 0);       // 0 turns it off
    delay(100);                    // wait for a delayms ms    
  }
}

// Checks whether the ambient light from LDR sensor is above/below threshold
boolean checkLights(boolean existingLightStatus) {

  // read the LDR output
  sensorValue = analogRead(ldrPin);  

  if(sensorValue < lightTreshold) {
    digitalWrite(led2Pin, HIGH);   // set the LED on
    delay(10);
    if(!existingLightStatus) {
      existingLightStatus = true;
      Serial.print("Lightning conditions changed at ");
      Serial.print(millis()/1000);
      Serial.print(" sec to TRUE. LDR sensor value: "); 
      Serial.println(sensorValue, DEC);
      beep(200,10,1);
      delay(200);
    }
    return true;
  }
  else {
    digitalWrite(led2Pin, LOW);   // set the LED off
    delay(10);
    if(existingLightStatus) {
      existingLightStatus = false;
      Serial.print("Lightning conditions changed at ");
      Serial.print(millis()/1000);
      Serial.print(" sec to FALSE. LDR-sensor value: "); 
      Serial.println(sensorValue, DEC);
      beep(20,10,1);
      delay(200);
    }
    return false;
  }
}

// Check the PIR sensor for movement
void checkMotion(int sensorPin) {

  int currentSensor;
  int ledPin;

  if(sensorPin == pirPin1) {
    // sensor #1
    currentSensor = 1;
    ledPin = pirLedPin1;
  } 
  else {
    // sensor #2
    currentSensor = 2;
    ledPin = pirLedPin2;
  }

  if(digitalRead(sensorPin) == HIGH) {
    digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
    if(lockLow){  
      //makes sure we wait for a transition to LOW before any further output is made:
      lockLow = false;            
      Serial.println("---");
      Serial.print("Sensor #");
      Serial.print(currentSensor, DEC);
      Serial.print(" on pin ");
      Serial.print(sensorPin, DEC);
      Serial.print(" detected motion at ");
      Serial.print(millis()/1000);
      Serial.println(" sec"); 
      delay(50);
      beep(200,200,1);
      Serial.print("Light sensor: ");
      if(lightStatus) {                     
          Serial.println("TRUE");
        initLedSequence(sensorPin);
      }
      else { 
        Serial.println("FALSE");
      }
    }         
    takeLowTime = true;
  }

  if(digitalRead(sensorPin) == LOW){       
    digitalWrite(ledPin, LOW);  // the led visualizes the sensors output pin state

    if(takeLowTime){
      lowIn = millis();          // save the time of the transition from high to LOW
      takeLowTime = false;       // make sure this is only done at the start of a LOW phase
    }
    // if the sensor is low for more than the given pause, 
    // we assume that no more motion is going to happen
    if(!lockLow && millis() - lowIn > pause){  
      // makes sure this block of code is only executed again after 
      // a new motion sequence has been detected
      lockLow = true;
      Serial.print("Sensor #");
      Serial.print(currentSensor, DEC);
      Serial.print(" on pin ");
      Serial.print(sensorPin, DEC);
      Serial.print(" motion ended at ");      //output
      Serial.print((millis() - pause)/1000);
      Serial.println(" sec");
      delay(50);
      beep(200,20,1);
      // start turning LED strip off
      if(lightsOn) {
        Serial.println("Turning lights off...");
        darkenStrip(sensorPin);
      }
    }
  }
}

// fill the dots one after the other with said color
// good for testing purposes
void colorWipe(uint32_t c, uint8_t wait, int mode, int sensorPin) {
  int i;

  Serial.print("Initiating LEDs - ");

  if(mode==0) {
    Serial.println("OFF");
  } 
  else {
    Serial.println("ON");
  }
  if(sensorPin == 7) {
    Serial.println("Direction: TOP");
  } 
  else {
    Serial.println("Direction: BOTTOM");
  }

  // change direction based on revert setting
  if(sensorPin==7) {
    for (i=0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
    }
  } 
  else {
    for (i=strip.numPixels(); i > 1; i--) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
    }
  }
}

int initLedSequence(int sensorPin) {

  lightsOn = true;
  colorWipe(strip.Color(127,127,127), 10, 1, sensorPin);   

}

void darkenStrip(int sensorPin) {
  colorWipe(strip.Color(0,0,0), 10, 0, sensorPin);
  strip.show(); // Refresh to turn off last pixel
  lightsOn = false;
}


// Sine wave effect
#define PI 3.14159265
void wave(uint32_t c, int cycles, uint8_t wait) {
  float y;
  byte  r, g, b, r2, g2, b2;

  // Need to decompose color into its r, g, b elements
  g = (c >> 16) & 0x7f;
  r = (c >>  8) & 0x7f;
  b =  c        & 0x7f; 

  for(int x=0; x<(strip.numPixels()*5); x++)
  {
    for(int i=0; i<strip.numPixels(); i++) {
      y = sin(PI * (float)cycles * (float)(x + i) / (float)strip.numPixels());
      if(y >= 0.0) {
        // Peaks of sine wave are white
        y  = 1.0 - y; // Translate Y to 0.0 (top) to 1.0 (center)
        r2 = 127 - (byte)((float)(127 - r) * y);
        g2 = 127 - (byte)((float)(127 - g) * y);
        b2 = 127 - (byte)((float)(127 - b) * y);
      } 
      else {
        // Troughs of sine wave are black
        y += 1.0; // Translate Y to 0.0 (bottom) to 1.0 (center)
        r2 = (byte)((float)r * y);
        g2 = (byte)((float)g * y);
        b2 = (byte)((float)b * y);
      }
      strip.setPixelColor(i, r2, g2, b2);
    }
    strip.show();
    delay(wait);
  }
}

void solidColor(uint32_t c, uint8_t wait) {
  int i;

  for (i=0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

































