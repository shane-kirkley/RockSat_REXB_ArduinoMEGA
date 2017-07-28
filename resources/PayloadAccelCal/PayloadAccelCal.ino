#include <SD.h>
#include <SPI.h>

//Pin Definitions
const int xInput = A0;
const int yInput = A1;
const int zInput = A2;

const int testingLED = 45;
const int warningLED = 47;


//Raw Ranges
volatile int xRawMin = 512;
volatile int xRawMax = 512;
volatile int yRawMin = 512;
volatile int yRawMax = 512;
volatile int zRawMin = 512;
volatile int zRawMax = 512;

const int sampleSize = 10;
File datafile;
// SD CARD
const int SD_CHIP_SELECT = 53;
String SDdataBuffer = ""; // data string to be written to SD card.
char filename[12];
File dataFile;

void setup() {
  // put your setup code here, to run once:
  analogReference(EXTERNAL);
  pinMode(testingLED,OUTPUT);
  pinMode(warningLED,OUTPUT);
  pinMode(SD_CHIP_SELECT, OUTPUT);
  strcpy(filename, "Calibration.txt");
  SD.begin(SD_CHIP_SELECT);
  
  Serial.begin(9600);
}



void loop() {
  datafile= SD.open(filename,FILE_WRITE);
  int i = 0;
  digitalWrite(testingLED,HIGH);

  int xRaw = ReadAxis(xInput);
  int yRaw = ReadAxis(yInput);
  int zRaw = ReadAxis(zInput);
  
  AutoCalibrate(xRaw, yRaw, zRaw);
  

  delay(1000);
  digitalWrite(testingLED,LOW);
 
  datafile.print("Raw Ranges: X: ");
  datafile.print(xRawMin);
  datafile.print("-");
  datafile.print(xRawMax);
  datafile.print(", Y: ");
  datafile.print(yRawMin);
  datafile.print("-");
  datafile.print(yRawMax);
  datafile.print(", Z: ");
  datafile.print(zRawMin);
  datafile.print("-");
  datafile.print(zRawMax);
  datafile.println();
  datafile.print(xRaw);
  datafile.print(", ");
  datafile.print(yRaw);
  datafile.print(", ");
  datafile.print(zRaw);
  datafile.close();
  while(i < 6){
    digitalWrite(warningLED,HIGH);
    delay(500);
    digitalWrite(warningLED,LOW);
    delay(500);
    i++;
  }

  
  
}

int ReadAxis(int axisPin)
{
  long reading = 0;
  analogRead(axisPin);
  delay(1);
  for (int i = 0; i < sampleSize; i++)
  {
    reading += analogRead(axisPin);
  }
  return reading / sampleSize;
}

void AutoCalibrate(int xRaw, int yRaw, int zRaw)
{
  //Serial.println("Calibrate");
  if (xRaw < xRawMin)
  {
    xRawMin = xRaw;
  }
  if (xRaw > xRawMax)
  {
    xRawMax = xRaw;
  }
  if (yRaw < yRawMin)
  {
    yRawMin = yRaw;
  }
  if (yRaw > yRawMax)
  {
    yRawMax = yRaw;
  }
  if (zRaw < zRawMin)
  {
    zRawMin = zRaw;
  }
  if (zRaw > zRawMax)
  {
    zRawMax = zRaw;
  }
}
