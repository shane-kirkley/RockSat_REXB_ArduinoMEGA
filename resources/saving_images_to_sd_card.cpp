   /*
     *-- SD IMAGE SAVING SECTION -- 
     */
 if (!frontCam.takePicture()) 
   Serial.println("Failed to take pic.");
 else 
   Serial.println("Picture taken!");

    // Create an image with the name IMAGExx.JPG
 char filename[13];
 strcpy(filename, "IMAGE00.JPG");
 for (int i = 0; i < 100; i++) {
   filename[5] = '0' + i/10;
   filename[6] = '0' + i%10;
   // create if does not exist, do not open existing, write, sync after write
   if (! SD.exists(filename)) {
     break;
   }
 }
 // Open the file for writing
 File imgFile = SD.open(filename, FILE_WRITE);

 // Get the size of the image (frame) taken  
 uint16_t jpglen = frontCam.frameLength();
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
   buffer = frontCam.readPicture(bytesToRead);
   imgFile.write(buffer, bytesToRead);
   if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
     Serial.print('.');
     wCount = 0;
   }
   //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
   jpglen -= bytesToRead;
 }
 imgFile.close();

 time = millis() - time;
 Serial.println("done!");
 Serial.print(time); 
 Serial.println(" ms elapsed");

/*
 * END SD IMAGE SAVING SECTION
 */