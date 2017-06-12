#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>

/*******************************************************************
  RockSat-X REX-B Flight Software
  Avionics Team:
  Nolan Ferguson, Virginia Nystrom, Leina Hutchinson, Shane Kirkley
  Zachery Toelkes, Abby Caballero, with special guest Alex Bertman
********************************************************************/

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
const int LED_PWR = 49;
const int LED_TE2_STATUS_HIGH = 47;
const int LED_TE3_STATUS_HIGH = 45;
const int LED_TE3_STATUS_LOW = 43;
const int LED_SENSOR_STATUS = 41;

// Digital interrupt
const int TIMER_EVENT_2 = 20;
const int TIMER_EVENT_3 = 21;
const int DEBOUNCE_DELAY = 20000; // microseconds
volatile boolean enableTimer3 = false; // turn true in timer event 2 ISR to turn on timer for ptc08

// gopro and motor pins
const int GOPRO_1_PWR = 44;
const int GOPRO_2_PWR = 24;
const int GOPRO_LED = 26;
const int FRONT_MOTOR_PWR = 28;
const int REAR_MOTOR_PWR = 30;

// PTC08
volatile boolean rearCamTrigger = false; // set cam triggers true for cam downlink when available.
volatile boolean frontCamTrigger = false;

// TIMERS: timer3 - safety delay after deployment
const uint8_t ejectDelay = 220;
const int timer3_TCNT = 34286;
volatile uint8_t timer3_count = 0;
volatile bool ejectionSafety = true; // this is set false by timer3 to enable ejection by TE2 LOW

Adafruit_VC0706 frontCam = Adafruit_VC0706(&Serial2);
Adafruit_VC0706 rearCam = Adafruit_VC0706(&Serial3);

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
  pinMode(GOPRO_LED, OUTPUT);
  pinMode(FRONT_MOTOR_PWR, OUTPUT);
  pinMode(REAR_MOTOR_PWR, OUTPUT);
  pinMode(TIMER_EVENT_2, INPUT);
  pinMode(TIMER_EVENT_3, INPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_TE2_STATUS_HIGH, OUTPUT);
  pinMode(LED_TE3_STATUS_HIGH, OUTPUT);
  pinMode(LED_TE3_STATUS_LOW, OUTPUT);
  pinMode(LED_SENSOR_STATUS, OUTPUT);

  digitalWrite(LED_PWR, HIGH);
  // UART setup
  Serial.begin(9600); // usb serial
  Serial1.begin(15360); // RS232 is 15360 baud

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
  downlinkImage(rearCam); // rear cam downlink on powerup
  //downlinkImage(frontCam); // REMOVE
  // setup interrupts
      // GO PRO ENABLE ON RISING EDGE, TURN OFF ON FALLING EDGE
  attachInterrupt(digitalPinToInterrupt(TIMER_EVENT_2), TimerEvent2ISR, CHANGE);
      // DEPLOY ON RISING EDGE, RELEASE ON FALLING EDGE
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
  if(frontCamTrigger) {         // poll camera triggers
    downlinkImage(frontCam);
    frontCamTrigger = false;
  }
  if(rearCamTrigger) {
    downlinkImage(rearCam);
    rearCamTrigger = false;
  }
  SDdataBuffer = getSensorData(); // fill buffer with all sensor data
  saveData(SDdataBuffer);         // save line to SD card
  if(enableTimer3) {
    detachInterrupt(digitalPinToInterrupt(TIMER_EVENT_2));
    detachInterrupt(digitalPinToInterrupt(TIMER_EVENT_3));
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

/*
 * INTERRUPT AND FUNCTION DEFINITIONS
 */
String getSensorData() {
  // returns all sensor data in string format
  // might need to turn off interrupts while this runs?
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
    digitalWrite(LED_SENSOR_STATUS, HIGH);
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
    digitalWrite(LED_SENSOR_STATUS, LOW);
  }
}

// timer3: ejects boom after [ejectDelay * .5 s]
ISR(TIMER3_OVF_vect) {
  TCNT3 = timer3_TCNT;
  timer3_count++;
  if (timer3_count > ejectDelay) {
    TIMSK3 = 0; // disable timer3 interrupts
    digitalWrite(REAR_MOTOR_PWR, HIGH);
    digitalWrite(LED_TE3_STATUS_HIGH, LOW); // status leds
    digitalWrite(LED_TE3_STATUS_LOW, HIGH);
  }
}

void TimerEvent2ISR() {
  // gopro ISR turns on both gopro cameras and LED
  delayMicroseconds(DEBOUNCE_DELAY);  // debounce delay
  if(digitalRead(TIMER_EVENT_2) == HIGH) {
     digitalWrite(GOPRO_1_PWR, HIGH);  // gopro 1 on
     digitalWrite(GOPRO_2_PWR, HIGH);  // gopro 2 on
     delayMicroseconds(200000);
     digitalWrite(GOPRO_1_PWR, LOW);  // gopro 1
     digitalWrite(GOPRO_2_PWR, LOW);  // gopro 2
     digitalWrite(GOPRO_LED, HIGH);    // led on
     digitalWrite(LED_TE2_STATUS_HIGH, HIGH); // status led on
     rearCamTrigger = true; // rear image before deployment
  }
}

void TimerEvent3ISR() {
  delayMicroseconds(DEBOUNCE_DELAY);  // debounce delay
  if(digitalRead(TIMER_EVENT_3) == HIGH) {
    // Deploy boom and enable timer for front/rear ptc08 pics
    digitalWrite(FRONT_MOTOR_PWR, HIGH);
    digitalWrite(LED_TE3_STATUS_HIGH, HIGH);
    enableTimer3 = true;
    frontCamTrigger = true;
    rearCamTrigger = true;
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
 Serial.print(imgFileName);
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
   //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
   jpglen -= bytesToRead;
 }
 SDimgFile.close();

 time = millis() - time;
 Serial.println("done!");
 Serial.print(time); 
 Serial.println(" ms elapsed");
}

