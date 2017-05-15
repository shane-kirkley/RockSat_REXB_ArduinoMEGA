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

#include "ISR.H"
#include "Adafruit_VC0706.h"
 
/*
 * Pin Defs     // could someone tell me why const is preferred over #define on arduino? -Shane
 */
  // ACCELEROMETER
const int ACCL_X = 0;
const int ACCL_Y = 1;
const int ACCL_Z = 2;

// TEMP SENSORS
const int TEMP_E = 3;
const int TEMP_C1 = 5;
const int TEMP_C2 = 7;
const int TEMP_AMBIENT = 9;

// PRESSURE SENSORS
const int PRESSURE_E = 4;
const int PRESSURE_C1 = 6;
const int PRESSURE_C2 = 8;

// SD CARD
const int SD_CHIP_SELECT = 53;

// Digital interrupt
const int GOPRO_INTERRUPT_PIN = 20;
const int MOTOR_INTERRUPT_PIN = 21;

// PTC08
  Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial2);
  Adafruit_VC0706 rearCam = Adafruit_VC0706(&Serial3);

  /*
 * Globals
 */
 uint16_t * sensorDataBuffer;
 uint8_t sensorDataLength = 0;
 uint16_t x_data;
 uint16_t y_data;
 uint16_t z_data;
 

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
  pinMode(PRESSURE_E, INPUT);
  pinMode(PRESSURE_C1, INPUT);
  pinMode(PRESSURE_C2, INPUT);

  // initialize SD card
  pinMode(SD_CHIP_SELECT, OUTPUT);
  if(!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("SD card failure.");
    return;
  }

  
  delay(200); // cam delay
  Serial.begin(9600); // usb serial
  Serial1.begin(15360); // RS232 is 15360 baud
  
  // enable interrupts
      // GO PRO INTERRUPT ON HIGH EDGE ENABLE    
  attachInterrupt(digitalPinToInterrupt(GOPRO_INTERRUPT_PIN), goProTriggerISR, HIGH);
      // DEPLOY INTERRUPT ON HIGH EDGE ENABLE
  attachInterrupt(digitalPinToInterrupt(MOTOR_INTERRUPT_PIN), motorTriggerISR, HIGH);
      // RELEASE INTERRUPT ON LOW EDGE ENABLE
  attachInterrupt(digitalPinToInterrupt(MOTOR_INTERRUPT_PIN), releaseTriggerISR, LOW);
  
  // initialize PTC08 front/back
//  if(frontCam.begin()) {
//    Serial.println("Success");
//  }
//  else {
//    Serial.println("Camera not found");
//    return;
//  }
//  delay(300);
//  frontCam.setImageSize(VC0706_640x480);
//  uint8_t imgsize = frontCam.getImageSize();
//  Serial.println("Image size: ");
//  if (imgsize == VC0706_640x480) Serial.println("640x480");
//  if (imgsize == VC0706_320x240) Serial.println("320x240");
//  if (imgsize == VC0706_160x120) Serial.println("160x120");
//  if (!frontCam.takePicture()) 
//    Serial.println("Failed to take pic.");
//  else 
//    Serial.println("Picture taken!");
// 
//  uint16_t downlinkJPGsize = frontCam.frameLength();
//  int32_t downlinkTime = millis();
//  while(downlinkJPGsize > 0) {
//     uint8_t* downlinkBuffer;
//     uint8_t downlinkBytesToRead = min(32, downlinkJPGsize);
//     downlinkBuffer = frontCam.readPicture(downlinkBytesToRead);
//     Serial1.write(downlinkBuffer, downlinkBytesToRead);
//     downlinkJPGsize -= downlinkBytesToRead;
//  }
//  downlinkTime = millis() - downlinkTime;
//  Serial.println("downlink of image complete!");
//  Serial.print(downlinkTime); 
//  Serial.println(" ms elapsed");
//  
  // BACK: POWER ON
  // BACK: 20 SECONDS BEFORE DEPLOYMENT
  // BOTH: AFTER DEPPLOYMENT
  
    /*-- SD IMAGE SAVING SECTION --*/
//  if (!frontCam.takePicture()) 
//    Serial.println("Failed to take pic.");
//  else 
//    Serial.println("Picture taken!");
//
    // Create an image with the name IMAGExx.JPG
//  char filename[13];
//  strcpy(filename, "IMAGE00.JPG");
//  for (int i = 0; i < 100; i++) {
//    filename[5] = '0' + i/10;
//    filename[6] = '0' + i%10;
//    // create if does not exist, do not open existing, write, sync after write
//    if (! SD.exists(filename)) {
//      break;
//    }
//  }
//
//
//
//  // Open the file for writing
//  File imgFile = SD.open(filename, FILE_WRITE);
//
//  // Get the size of the image (frame) taken  
//  uint16_t jpglen = frontCam.frameLength();
//  Serial.print("Storing ");
//  Serial.print(jpglen, DEC);
//  Serial.print(" byte image.");
//
//  int32_t time = millis();
//  pinMode(8, OUTPUT);
//  // Read all the data up to # bytes!
//  byte wCount = 0; // For counting # of writes
//  while (jpglen > 0) {
//    // read 32 bytes at a time;
//    uint8_t *buffer;
//    uint8_t bytesToRead = min(32, jpglen);
//    buffer = frontCam.readPicture(bytesToRead);
//    imgFile.write(buffer, bytesToRead);
//    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
//      Serial.print('.');
//      wCount = 0;
//    }
//    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
//    jpglen -= bytesToRead;
//  }
//  imgFile.close();
//
//  time = millis() - time;
//  Serial.println("done!");
//  Serial.print(time); 
//  Serial.println(" ms elapsed");

}

void loop() {
  x_data = analogRead(0);
  y_data = analogRead(1);
  z_data = analogRead(2);
  Serial.println("X: ");
  Serial.print(x_data);
  Serial.println("Y: ");
  Serial.print(y_data);
  Serial.println("Z: ");
  Serial.print(z_data);
  // put your main code here, to run repeatedly:

}



