const int ACCL_X = A0;
const int ACCL_Y = A1;
const int ACCL_Z = A2;
int xRawMin = 510;
int xRawMax = 515;
int yRawMin = 512;
int yRawMax = 517;
int zRawMin = 509;
int zRawMax = 514;
const int sampleSize = 10;
int ReadAxis(int axisPin); //Reads 10 samples from 
float AccelCond(int Raw, int RawMin, int RawMax);

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

float AccelCond(int Raw, int RawMin, int RawMax) {
  float Acc = map(Raw,RawMin,RawMax,-1000,1000)/1000.0;
  return Acc;
}

/*--- Example usage: ---
    // reads and converts each axis.
  int xRaw = ReadAxis(ACCL_X);
  float xAcc = AccelCond(xRaw,xRawMin,xRawMax);
  int yRaw = ReadAxis(ACCL_Y);
  float yAcc = AccelCond(yRaw,yRawMin,yRawMax);
  int zRaw = ReadAxis(ACCL_Z);
  float zAcc = AccelCond(zRaw,zRawMin,zRawMax);
*/