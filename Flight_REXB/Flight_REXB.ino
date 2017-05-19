/*******************************************************************
  RockSat-X REX-B Flight Software
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
const int TEMP_C1 = 4;
const int TEMP_C2 = 6;
const int TEMP_AMBIENT = 7;
const int TEMP_E = 15;

// PRESSURE SENSORS
const int PRESSURE_E = 14;
const int PRESSURE_C1 = 3;
const int PRESSURE_C2 = 5;

// HUMIDITY
const int HUMIDITY = 13;

// SD CARD
const int SD_CHIP_SELECT = 53;
String dataString = ""; // data string to be written to SD card.
char filename[12];
File dataFile;

// STATUS LEDS
const int LED_PWR = 38;
const int LED_TE2_STATUS_HIGH = 40;
const int LED_TE3_STATUS_HIGH = 42;
const int LED_TE3_STATUS_LOW = 44;
const int LED_SENSOR_STATUS = 46;

// Digital interrupt
const int TIMER_EVENT_1 = 20;
const int TIMER_EVENT_2 = 21;
const int DEBOUNCE_DELAY = 20000; // microseconds
volatile boolean timerEvent1 = false; // indicates timer event 1 triggered high.
volatile boolean timerEvent2 = false; // turn on to switch to falling edge ejection interrupt
volatile boolean enableTimer3 = false; // turn true in timer event 2 ISR to turn on timer for ptc08

// gopro and motor pins
const int GOPRO_1_PWR = 22;
const int GOPRO_2_PWR = 24;
const int GOPRO_LED = 26;
const int FRONT_MOTOR_PWR = 28;
const int REAR_MOTOR_PWR = 30;

// PTC08
  // timer1
const int timer1_TCNT = 34286; // (65536 - (16MHz/256)) / 2 -> 1 Hz
const uint8_t rearCamDelay = 100; // rearCamDelay * 500 ms = seconds to delay rear picture
uint8_t timer1_count = 0;         // counts up to rearCamDelay.

  // timer3 - delay after deployment
const uint8_t camDelayAfterDeploy = 10;
const int timer3_TCNT = 34286;
volatile uint8_t timer3_count = 0;
volatile boolean rearCamTrigger = false; // set cam triggers true for cam downlink when available.
volatile boolean frontCamTrigger = false;
char jpgFileName[] = "img_0000000.jpg";
Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial2);
Adafruit_VC0706 rearCam = Adafruit_VC0706(&Serial3);

/*
 * RS232 FRAME HEADERS
 */
 // note: filename and jpg data are seperated by byte 0x00.
const uint8_t JPG_FRAME_START[] = {0xFF, 0xF2};
const uint8_t JPG_FRAME_END[] = {0xFF, 0xF3, 0x0D, 0x0A};

uint16_t testData[] = {245, 19, 2, 52332, 42, 42, 549, 192, 29504, 20495};
/*
 * Function prototypes and ISR
 */
void saveData();  // saves whatever is in dataString to the sd card.
void goProTriggerISR();
void TimerEvent2ISR();
void downlinkImage();
 
/*
 * Power on setup
 */
void setup() {
  // gopro and motors + interrupts
  pinMode(GOPRO_1_PWR, OUTPUT);
  pinMode(GOPRO_2_PWR, OUTPUT);
  pinMode(GOPRO_LED, OUTPUT);
  pinMode(FRONT_MOTOR_PWR, OUTPUT);
  pinMode(REAR_MOTOR_PWR, OUTPUT);
  pinMode(TIMER_EVENT_1, INPUT);
  pinMode(TIMER_EVENT_2, INPUT_PULLUP);
  
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
  else {
    Serial.println("SD card initialized.");
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
    Serial.println("front camera not found");
  }
  if(rearCam.begin()) {
    Serial.println("rear cam initialized");
  }
  else {
    Serial.println("rear camera not found");
  }
  delay(300);
  frontCam.setImageSize(VC0706_640x480);
  rearCam.setImageSize(VC0706_640x480);
  downlinkImage(rearCam); // rear cam downlink on powerup
  
  // setup interrupts
  noInterrupts();           // disable global interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = timer1_TCNT;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();        // enable global interrupts
        // GO PRO ENABLE ON RISING EDGE  
  attachInterrupt(digitalPinToInterrupt(TIMER_EVENT_1), goProTriggerISR, RISING);
      // DEPLOY ON RISING EDGE, RELEASE ON FALLING EDGE
  attachInterrupt(digitalPinToInterrupt(TIMER_EVENT_2), TimerEvent2ISR, CHANGE);


  // create CSV file on SD card.

//  strcpy(filename, "DATA00.CSV");
//  for(int i = 0; i < 100; i++) {
//    filename[4] = '0' + 1/10;
//    filename[5] = '0' + i%10;
//    if (!SD.exists(filename)) {
//      Serial.println("file name: ");
//      Serial.print(filename);
//      break;
//    }
//  }
  
    /*
     *-- SD IMAGE SAVING SECTION -- 
     */
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

/*
 * END SD IMAGE SAVING SECTION
 */

}

void loop() {
  if(rearCamTrigger) {
    downlinkImage(rearCam);
    rearCamTrigger = false;
  }
  if(frontCamTrigger) {
    downlinkImage(frontCam);
    frontCamTrigger = false;
  }
  
  // this part will sample all sensor data and save it to sd card
  //saveData();
  //delay(300);
  
  if(timerEvent2 && enableTimer3) {
    noInterrupts();           // disable all interrupts
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = timer3_TCNT;   // preload timer
    TCCR3B |= (1 << CS12);    // 256 prescaler 
    TIMSK3 |= (1 << TOIE1);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
    enableTimer3 = false;
  }
}

void saveData() {
    dataFile = SD.open(filename, FILE_WRITE);
    //Serial.println(dataString);
    if(dataFile) {
      dataFile.println(dataString);
      dataString = ""; // clear dataString
      Serial.println("Saving data to sd card...");
      dataFile.close(); // close to save data
  }
}

void downlinkImage(Adafruit_VC0706 cam) {
  if(!cam.takePicture()) {
    Serial.println("Failed to take pic.");
  }
  else {
    Serial.println("Picture Taken!");
  }

  uint16_t downlinkJPGsize = cam.frameLength();
  int32_t downlinkTime = millis();
  Serial1.write(JPG_FRAME_START, sizeof(JPG_FRAME_START));
  Serial1.write(jpgFileName, sizeof(jpgFileName));
  Serial1.write(0x00);
  while(downlinkJPGsize > 0) {
    uint8_t * downlinkBuffer;
    uint8_t downlinkBytesToRead = min(32, downlinkJPGsize);
    downlinkBuffer = cam.readPicture(downlinkBytesToRead);
    Serial1.write(downlinkBuffer, downlinkBytesToRead);
    downlinkJPGsize -= downlinkBytesToRead;
  }
  Serial1.write(JPG_FRAME_END, sizeof(JPG_FRAME_END));
  downlinkTime = millis() - downlinkTime;
  Serial.println("downlink of jpg complete");
  Serial.print(downlinkTime);
  Serial.println(" ms elapsed");
}

// timer1: takes interior cam pic 20 seconds before deployment
ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1_TCNT;   // preload timer
  timer1_count++;
  if(timer1_count > rearCamDelay) {
    rearCamTrigger = true;
    timer1_count = 0;
    TIMSK1 = 0; // disable timer1 interrupts
  }
}

// timer3: takes front and interor cam pc after deployment
ISR(TIMER3_OVF_vect) {
  TCNT3 = timer3_TCNT;
  timer3_count++;
  if (timer3_count > camDelayAfterDeploy) {
    frontCamTrigger = true;
    rearCamTrigger = true;
    TIMSK3 = 0; // disable timer3 interrupts
  }
}

void goProTriggerISR() {
  // gopro ISR turns on both gopro cameras and LED
  delayMicroseconds(DEBOUNCE_DELAY);  // debounce delay
  if(!timerEvent1 && digitalRead(TIMER_EVENT_1) == HIGH) {
     digitalWrite(GOPRO_1_PWR, HIGH);  // gopro 1 on
     digitalWrite(GOPRO_2_PWR, HIGH);  // gopro 2 on
     digitalWrite(GOPRO_LED, HIGH);    // led on
     timerEvent1 = true;  // timer event 1 is high
  }
}

void TimerEvent2ISR() {
  delayMicroseconds(DEBOUNCE_DELAY);  // debounce delay
  if(!timerEvent2 && digitalRead(TIMER_EVENT_2) == HIGH) {
    // Deploy boom and enable timer for front/rear ptc08 pics
    digitalWrite(FRONT_MOTOR_PWR, HIGH);
    enableTimer3 = true;
    timerEvent2 = true;
  }
  if(timerEvent2 && digitalRead(TIMER_EVENT_2) == LOW) {
    // eject boom
    digitalWrite(REAR_MOTOR_PWR, HIGH);
    timerEvent2 = false;
  }
  
}




