char jpgFileName[] = "img0.jpg";
const uint8_t JPG_FRAME_START[] = {0xFF, 0xF2};
const uint8_t JPG_FRAME_END[] = {0xFF, 0xF3, 0x0D, 0x0A};
const uint8_t CSV_FRAME_START[] = {0xFF, 0xF4};
const uint8_t CSV_DATA_SPACER[] = {0x2C, 0x20}; // comma + space
const uint8_t CSV_FRAME_END[] = {0xFF, 0xF5, 0x0D, 0x0A};
uint16_t downlinkDataBuffer[12] = {};
uint8_t downlinkDataSize = 0;

void downlinkData(); // sends data in downlinkDataBuffer over Serial1 (RS232)

void downlinkData() { // unused right now
 Serial1.write(CSV_FRAME_START, sizeof(CSV_FRAME_START));
 Serial1.write(csvFileName, sizeof(csvFileName));
 Serial1.write(0x00);
 for(int i = 0; i < downlinkDataSize; i++) {
   Serial1.print(downlinkDataBuffer[i], 2);
   Serial1.write(CSV_DATA_SPACER, sizeof(CSV_DATA_SPACER));    
 }
 Serial1.write(CSV_FRAME_END, sizeof(CSV_FRAME_END));
}