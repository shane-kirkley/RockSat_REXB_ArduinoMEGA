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
 * Pin Defs and Globals 
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

// gopro and motor pins
const int GOPRO_1_PWR = 22;
const int GOPRO_2_PWR = 24;
const int GOPRO_LED_PIN = 26;
const int FRONT_MOTOR_PWR = 28;
const int REAR_MOTOR_PWR = 30;

// PTC08
  // timer1
const int timer1_TCNT = 34286; // (65536 - (16MHz/256)) / 2 -> 1 Hz
const uint8_t rearCamDelay = 20; // rearCamDelay * 500 ms = seconds to delay rear picture
uint8_t timer1_count = 0;         // counts up to rearCamDelay.
  // timer3 - delay after deployment
const int timer3_TCNT = 34286;
const uint8_t camDelayAfterDeploy = 4;
uint8_t timer3_count = 0;

Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial2);
Adafruit_VC0706 rearCam = Adafruit_VC0706(&Serial3);

/*
 * Function prototypes and ISR
 */
void goProTriggerISR();
void motorTriggerISR();
void releaseTriggerISR();
void downlinkFrontCamera();
void downlinkRearCamera();
 
/*
 * Power on setup
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
  pinMode(PRESSURE_E, INPUT);
  pinMode(PRESSURE_C1, INPUT);
  pinMode(PRESSURE_C2, INPUT);

  // UART setup
  Serial.begin(9600); // usb serial
  Serial1.begin(15360); // RS232 is 15360 baud

  // initialize SD card
  pinMode(SD_CHIP_SELECT, OUTPUT);
  if(!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("SD card failure.");
    return;
  }

 // initialize PTC08 front/back
      // PTC08 picture req's:
        //  BACK: POWER ON
        //  BACK: 20 SECONDS BEFORE DEPLOYMENT
        //  BOTH: AFTER DEPLOYMENT
  if(frontCam.begin()) {
    Serial.println("front cam intialized");
  }
  else {
    Serial.println("font camera not found");
    return;
  }
  if(rearCam.begin()) {
    Serial.println("rear cam initialized");
  }
  else {
    Serial.println("font camera not found");
  }
  delay(300);
  frontCam.setImageSize(VC0706_640x480);
  rearCam.setImageSize(VC0706_640x480);
  downlinkRearCamera();  // rear image downlinked on power on
  
  // setup interrupts
  noInterrupts();           // disable global interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = timer1_TCNT;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
      // GO PRO INTERRUPT ON HIGH EDGE ENABLE    
  attachInterrupt(digitalPinToInterrupt(GOPRO_INTERRUPT_PIN), goProTriggerISR, HIGH);
      // DEPLOY INTERRUPT ON HIGH EDGE ENABLE
  attachInterrupt(digitalPinToInterrupt(MOTOR_INTERRUPT_PIN), motorTriggerISR, HIGH);
      // RELEASE INTERRUPT ON LOW EDGE ENABLE
  attachInterrupt(digitalPinToInterrupt(MOTOR_INTERRUPT_PIN), releaseTriggerISR, LOW);
  interrupts();             // enable global interrupts
  
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

}

void downlinkFrontCamera() {
  if (!frontCam.takePicture()) 
    Serial.println("Failed to take pic.");
  else 
    Serial.println("Picture taken!");
 
  uint16_t downlinkJPGsize = frontCam.frameLength();
  int32_t downlinkTime = millis();
  while(downlinkJPGsize > 0) {
     uint8_t* downlinkBuffer;
     uint8_t downlinkBytesToRead = min(32, downlinkJPGsize);
     downlinkBuffer = frontCam.readPicture(downlinkBytesToRead);
     Serial1.write(downlinkBuffer, downlinkBytesToRead);
     downlinkJPGsize -= downlinkBytesToRead;
  }
  downlinkTime = millis() - downlinkTime;
  Serial.println("downlink of font camera image complete");
  Serial.print(downlinkTime); 
  Serial.println(" ms elapsed");
}

void downlinkRearCamera() {
    if (!rearCam.takePicture()) 
    Serial.println("Failed to take pic.");
  else 
    Serial.println("Picture taken!");
 
  uint16_t downlinkJPGsize = rearCam.frameLength();
  int32_t downlinkTime = millis();
  while(downlinkJPGsize > 0) {
     uint8_t* downlinkBuffer;
     uint8_t downlinkBytesToRead = min(32, downlinkJPGsize);
     downlinkBuffer = rearCam.readPicture(downlinkBytesToRead);
     Serial1.write(downlinkBuffer, downlinkBytesToRead);
     downlinkJPGsize -= downlinkBytesToRead;
  }
  downlinkTime = millis() - downlinkTime;
  Serial.println("downlink of rear camera image complete");
  Serial.print(downlinkTime); 
  Serial.println(" ms elapsed");
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1_TCNT;   // preload timer
  timer1_count++;
  if(timer1_count > rearCamDelay) {
    downlinkRearCamera();
    TIMSK1 = 0; // disable timer1 interrupts
  }
}

ISR(TIMER3_OVF_vect) {
  TCNT3 = timer3_TCNT;
  timer3_count++;
  if (timer3_count > camDelayAfterDeploy) {
    downlinkFrontCamera();
    downlinkRearCamera();
    TIMSK3 = 0; // disable timer3 interrupts
  }
}

void goProTriggerISR() {
  // gopro ISR turns on both gopro cameras and LED
  digitalWrite(GOPRO_1_PWR, HIGH);
  digitalWrite(GOPRO_2_PWR, HIGH);
  digitalWrite(GOPRO_LED_PIN, HIGH);
}

void motorTriggerISR() {
  // Deploy boom and enable timer for front/rear ptc08 pics
  digitalWrite(FRONT_MOTOR_PWR, HIGH);
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = timer3_TCNT;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  TIMSK3 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void releaseTriggerISR() {
  digitalWrite(REAR_MOTOR_PWR, HIGH);
}




