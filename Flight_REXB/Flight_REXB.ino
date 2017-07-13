/*******************************************************************
  RockSat-X 11: REX-B Flight Software - Avionics Team:
  Nolan Ferguson, Abby Caballero, Leina Hutchinson, Shane Kirkley
  Alex Bertman, and Zachery Toelkes
********************************************************************/

/*******************************************************************
 * TODO:
 * - Test timing of deployment and ejection.
 *******************************************************************/
 
#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_VC0706.h"

// ACCELEROMETER (conversion functions can be found in /resources)
const int ACCL_X = A0;
const int ACCL_Y = A1;
const int ACCL_Z = A2;
const int sampleSize = 10;

// TEMP SENSORS
const int TEMP_C1 = A4;
const int TEMP_C2 = A6;
const int TEMP_AMBIENT = A7;
const int TEMP_E = A15;

// PRESSURE SENSORS
const int PRESSURE_E = A14;
const int PRESSURE_C1 = A3;
const int PRESSURE_C2 = A5;

// HUMIDITY
const int HUMIDITY = A13;

// SENSORS TO SAVE TO SD CARD (accelerometer seperate):
const uint8_t NUM_SENSORS = 8; 
int SENSOR_PINS[] = {TEMP_C1, TEMP_C2, TEMP_E, TEMP_AMBIENT, PRESSURE_E, PRESSURE_C1, PRESSURE_C2, HUMIDITY};

// SD CARD
const int SD_CHIP_SELECT = 53;
String SDdataBuffer = ""; // data string to be written to SD card.
char filename[12];
File dataFile;

// STATUS LEDS
const int LED_TE2_STATUS = 47;
const int LED_TE3_STATUS = 45;
const int LED_DOWNLINK_STATUS = 49;

// Digital interrupt
const int TIMER_EVENT_2 = 20;
const int TIMER_EVENT_3 = 21;
const int DEBOUNCE_DELAY = 20000; // microseconds

// gopro and motor pins
const int GOPRO_1_PWR = 22; // GP1
const int GOPRO_2_PWR = 24; // GP2
const int FRONT_MOTOR_PWR = 28;  // LA1
const int REAR_MOTOR_PWR = 30;   // LA2

// PTC08
volatile boolean rearCamTrigger = false; // set cam triggers true for cam downlink when available.
volatile boolean frontCamTrigger = false;
Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial2); //
Adafruit_VC0706 rearCam = Adafruit_VC0706(&Serial3);  //

// TIMERS: 
  // timer3 - delay EJECTION for ~100s
volatile boolean enableTimer3 = false; // true to enable timer3
const uint8_t ejectDelay = 200;
const int timer3_TCNT = 34286;
volatile uint8_t timer3_count = 0;

  // timer4 - delay DEPLOYMENT after timer event ~5s
volatile boolean enableTimer4 = false; // true to enable timer4
const uint8_t deployDelay = 10;
const int timer4_TCNT = 34286;
volatile uint8_t timer4_count = 0;

  // timer5 - delay LOW RES IMGS after deployment ~
volatile boolean enableTimer5 = false; // true to enable timer5
const uint8_t lowResDelay = 10;
const int timer5_TCNT = 34286;
volatile uint8_t timer5_count = 0;

/* 
 * RS232 FRAMES, DOWNLINK FORMATTING AND DOWNLINKED SENSORS
 */
char jpgFileName[] = "img0.jpg";
const uint8_t JPG_FRAME_START[] = {0xFF, 0xF2};
const uint8_t JPG_FRAME_END[] = {0xFF, 0xF3, 0x0D, 0x0A};

/*
 * Function prototypes and ISR
 */
void saveData();  // saves whatever is in SDdataBuffer to the sd card.
void TimerEvent3ISR();
void TimerEvent2ISR();
void downlinkImage(Adafruit_VC0706 cam);
String getSensorData(); // returns string of all sensor data
void saveData(String data);
int ReadAxis(int axisPin); //Reads 10 samples from axisPin of accelerometer

/*
 * Power on setup
 */
void setup() {
  // gopro and motors + interrupts
  pinMode(GOPRO_1_PWR, OUTPUT);
  pinMode(GOPRO_2_PWR, OUTPUT);
  pinMode(FRONT_MOTOR_PWR, OUTPUT);
  pinMode(REAR_MOTOR_PWR, OUTPUT);
  pinMode(TIMER_EVENT_2, INPUT);
  pinMode(TIMER_EVENT_3, INPUT);
  pinMode(LED_TE2_STATUS, OUTPUT);
  pinMode(LED_TE3_STATUS, OUTPUT);
  pinMode(LED_DOWNLINK_STATUS, OUTPUT);

  // UART setup
  Serial.begin(9600); // usb serial
  Serial1.begin(19200); // RS232 is 19200 baud

  // initialize SD card
  pinMode(SD_CHIP_SELECT, OUTPUT);
  if(!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("ERROR: SD card failure.");
  }
  else {
    Serial.println("SD card initialized.");
  }
  if(frontCam.begin()) {
    Serial.println("front cam intialized");
  }
  else {
    Serial.println("ERROR: front camera not found");
  }
  if(rearCam.begin()) {
    Serial.println("rear cam initialized");
  }
  else {
    Serial.println("ERROR: rear camera not found");
  }
  delay(300);
  rearCam.setImageSize(VC0706_640x480);
  frontCam.setImageSize(VC0706_640x480);
  delay(1200);  
  downlinkImage(rearCam); // rear cam downlink on powerup
  // setup interrupts
      // GO PRO ENABLE ON RISING EDGE, REMAIN ON FOR FLIGHT
  attachInterrupt(digitalPinToInterrupt(TIMER_EVENT_2), TimerEvent2ISR, CHANGE);
      // DEPLOY ON RISING EDGE, ENABLE TIMER3 FOR RELEASE
  attachInterrupt(digitalPinToInterrupt(TIMER_EVENT_3), TimerEvent3ISR, CHANGE);

  // create .csv file on SD card.
  strcpy(filename, "DATA00.CSV");
  for(int i = 0; i < 100; i++) {    
    filename[4] = '0' + i/10;
    filename[5] = '0' + i%10;
    if (!SD.exists(filename)) {
      Serial.println("file name: ");
      Serial.print(filename);
      Serial.println();
      Serial.println();
      break;
    }
  }
}

/*
 * MAIN LOOP: 
 */
void loop() {
  SDdataBuffer = getSensorData(); // fill buffer with all sensor data
  saveData(SDdataBuffer);         // save line to SD card
  
  if(frontCamTrigger) {         // poll camera triggers
    delay(600);
    downlinkImage(frontCam);
    frontCamTrigger = false;
  }
  if(rearCamTrigger) {
    delay(600);
    downlinkImage(rearCam);
    rearCamTrigger = false;
  }
  if(enableTimer3) {   // ENABLE EJECTION DELAY
    noInterrupts();         // disable all interrupts
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = timer3_TCNT;    // preload timer
    TCCR3B |= (1 << CS12);  // 256 prescaler 
    TIMSK3 |= (1 << TOIE1); // enable timer overflow interrupt
    interrupts();           // enable all interrupts
    enableTimer3 = false;
  }
  if (enableTimer4) {  // ENABLE DEPLOYMENT DELAY
    noInterrupts();
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT3 = timer4_TCNT;
    TCCR4B |= (1 << CS12);
    TIMSK4 |= (1 << TOIE1); // timer overflow interrupt enable
    interrupts();
    enableTimer4 = false;
  }
  if (enableTimer5) {  // ENABLE LOW RES IMG DELAY AFTER DEPLOY
    noInterrupts();
    TCCR5A = 0;
    TCCR5B = 0;
    TCNT5 = timer5_TCNT;
    TCCR5B |= (1 << CS12);
    TIMSK5 |= (1 << TOIE1);
    interrupts();
    enableTimer5 = false;
  }
}

/*
 * INTERRUPT AND FUNCTION DEFINITIONS
 */
String getSensorData() {
  // returns all sensor data in string format
  String sensorData = "";
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorData += String(analogRead(SENSOR_PINS[i])) + ",";
  }
  sensorData += String(ReadAxis(ACCL_X)) + ",";
  sensorData += String(ReadAxis(ACCL_Y)) + ",";
  sensorData += String(ReadAxis(ACCL_Z)) + ",";
  return sensorData;
}

void saveData(String data) {
  dataFile = SD.open(filename, FILE_WRITE);
  if(dataFile) {
    dataFile.println(data);
    //Serial.println("Saving data to sd card...");
    dataFile.close(); // close to save data
  }
}

void downlinkImage(Adafruit_VC0706 cam) {
  if(!cam.takePicture()) {
    Serial.println("ERROR: Failed to take pic.");
  }
  else {
    digitalWrite(LED_DOWNLINK_STATUS, HIGH);
    Serial.println("Picture Taken!");
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
    digitalWrite(LED_DOWNLINK_STATUS, LOW);
  }
}

// timer3: boom ejection delay
ISR(TIMER3_OVF_vect) {
  TCNT3 = timer3_TCNT;
  timer3_count++;
  if (timer3_count > ejectDelay) {
    TIMSK3 = 0; // disable timer3 interrupts
    timer3_count = 0;
    digitalWrite(REAR_MOTOR_PWR, HIGH);     // eject boom
    digitalWrite(LED_TE3_STATUS, LOW); // status led low
  }
}

// timer4: deployment delay
ISR(TIMER4_OVF_vect) {
  TCNT4 = timer4_TCNT;
  timer4_count++;
  if (timer4_count > deployDelay) {
    TIMSK4 = 0; // disable timer4 interrupts
    timer4_count = 0;
    digitalWrite(FRONT_MOTOR_PWR, HIGH); // deploy boom
    digitalWrite(LED_TE3_STATUS, HIGH);
    enableTimer3 = true;                 // enable ejection delay (105 s)
    enableTimer5 = true;                 // enable delay low res images (5 s)
  }
}

ISR(TIMER5_OVF_vect) {
  TCNT5 = timer5_TCNT;
  timer5_count++;
  if (timer5_count > lowResDelay) {
    TIMSK5 = 0;
    timer5_count = 0;
    frontCamTrigger = true; // take low res images
    rearCamTrigger = true;
  }
}

void TimerEvent2ISR() {
  // gopro ISR turns on both gopro cameras and LED
  uint8_t i;
  delayMicroseconds(DEBOUNCE_DELAY);  // debounce delay
  if(digitalRead(TIMER_EVENT_2) == HIGH) {
    digitalWrite(GOPRO_1_PWR, HIGH);
    digitalWrite(GOPRO_2_PWR, HIGH);       
    digitalWrite(LED_TE2_STATUS, HIGH); // status led on
    rearCamTrigger = true; // rear image before deployment
    delayMicroseconds(10000);
    digitalWrite(GOPRO_1_PWR, LOW);
    digitalWrite(GOPRO_2_PWR, LOW);       
  }
}

void TimerEvent3ISR() {
  delayMicroseconds(DEBOUNCE_DELAY);  // debounce delay
  if(digitalRead(TIMER_EVENT_3) == HIGH) {
    // Deploy boom and enable timer for front/rear ptc08 pics
    //digitalWrite(LED_TE3_STATUS, HIGH);
    enableTimer4 = true; // deployment delay enable
  }
}

int ReadAxis(int axisPin) {
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
    {
      reading += analogRead(axisPin);
    }
  return reading/sampleSize;
}

void saveImageToSD(Adafruit_VC0706 cam) {
  if (!cam.takePicture()) {
    Serial.println("Failed to take pic.");
    return;
  }
  else {
    Serial.println("Picture taken!");
  }
  char imgFileName[13];
  strcpy(imgFileName, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
   imgFileName[5] = '0' + i/10;
   imgFileName[6] = '0' + i%10;
   // create if does not exist, do not open existing, write, sync after write
   if (! SD.exists(imgFileName)) {
     break;
   }
 }
 Serial.println(imgFileName);
 // Open the file for writing
 File SDimgFile = SD.open(imgFileName, FILE_WRITE);

 // Get the size of the image (frame) taken  
 uint16_t jpglen = cam.frameLength();
 Serial.print("Storing ");
 Serial.print(jpglen, DEC);
 Serial.print(" byte image.");

 int32_t time = millis();
 // Read all the data up to # bytes!
 byte wCount = 0; // For counting # of writes
 while (jpglen > 0) {
   // read 32 bytes at a time;
   uint8_t *buffer;
   uint8_t bytesToRead = min(32, jpglen);
   buffer = cam.readPicture(bytesToRead);
   SDimgFile.write(buffer, bytesToRead);
   if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
     Serial.print('.');
     wCount = 0;
   }
   jpglen -= bytesToRead;
 }
 SDimgFile.close();

 time = millis() - time;
 Serial.println("done!");
 Serial.print(time); 
 Serial.println(" ms elapsed");
}

