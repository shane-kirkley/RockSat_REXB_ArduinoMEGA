/*******************************************************************
  RockSat REX-B Flight Software
  (Current) Avionics Team:
  Nolan Ferguson, Virginia Nystrom, Leina Hutchinson, Shane Kirkley
  Zachery Toelkes, ???
 
********************************************************************/

/*  Pinout located in /docs/ or in Avionics drive
 *    
 */

/*
 * Includes (Arduino and implemented libraries)
 */

#include <SPI.h>
#include <SD.h>

#include "Adafruit_VC0706.h"

/*
 * Globals
 */
 
// ptc08 objects:
 Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial2);
 Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial3);
  
/*
 * Pin Defs     // could someone tell me why const is preferred over #define on arduino? -Shane
 *  (move these to seperate files if/when necessary)
 */
 
 // ACCELEROMETER
const int ACCL_X = A0;
const int ACCL_Y = A1;
const int ACCL_Z = A2;

// TEMP SENSORS
const int TEMP_E = A3;
const int TEMP_C1 = A5;
const int TEMP_C2 = A7;
const int TEMP_AMBIENT = A9;

// PRESSURE SENSORS
const int PRESSURE_E = A4;
const int PRESSURE_C1 = A6;
const int PRESSURE_C2 = A8;

// SD CARD
const int SD_CHIP_SELECT = 53;

// PTC08

/*
 * 
 */

void setup() {
  
  // accelerometer setup
  pinMode(ACCL_X, INPUT);
  pinMode(ACCL_Y, INPUT);
  pinMode(ACCL_Z, INPUT);

  // temp and pressure sensor setup
  pinMode(TEMP_E, INPUT);
  pinMode(TEMP_C1, INPUT);
  pinMode(TEMP_C2, INPUT);
  pinMode(TEMP_AMBIENT, INPUT);
  pinMode(PRESURE_E, INPUT);
  pinMode(PRESSURE_C1, INPUT);
  pinMode(PRESSURE_C2, INPUT);

  // initialize SD card
  pinMode(SD_CHIP_SELECT, OUTPUT);


  // initialize PTC08 front/back
  Serial2.begin(38400); // default BAUD is 38400 for ptc08
  frontCam.begin();
  frontCam.setImageSize(VC0706_320x240); // default size is 640x480
  
  Serial3.begin(38400);
  rearCam.begin();
  frontCam.setImageSize(VC0706_320x240);

}

void loop() {
  // put your main code here, to run repeatedly:

}
