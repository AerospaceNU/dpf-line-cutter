#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

const char* CHANGE_LOG_FILENAME = "stateChangeLog.txt";
File changeLogFile(InternalFS);

uint32_t u8_to_u32(const uint8_t* bytes);
float uint32_to_float_bits(uint32_t i);

void memory_setup() {
  Serial.println("opening 1...");
  InternalFS.begin();  
  
  char buff1[1] = { 0 };
  char buff4[4] = { 0 };
  uint32_t temp;

  Serial.println("opening file");
  if ( changeLogFile.open(CHANGE_LOG_FILENAME, FILE_O_READ) ) {
    int fileLength = changeLogFile.size();
    for(int i = 0; i < fileLength - 34; i+=35) {
      for(int j = i; j < i+35; j+=5) {
        changeLogFile.seek(j);
        changeLogFile.read(buff1, 1);
        Serial.print(buff1[0]);
        Serial.print(": ");
        changeLogFile.seek(j + 1);
        changeLogFile.read(buff4, 4);
        temp = u8_to_u32((uint8_t*)buff4);
        if (j%35 < 15) {
          Serial.println(temp);
        } else {
          Serial.println(uint32_to_float_bits(temp));
        }
      }
      Serial.println("----------");
    }
    
    Serial.println("Reached end of file.");
    Serial.print("Total size: ");
    Serial.print(fileLength);
    Serial.println(" bytes.");
  } else {
    Serial.print("Failed to read ");
    Serial.println(CHANGE_LOG_FILENAME);
  }
}

uint32_t u8_to_u32(const uint8_t* bytes) {
  uint32_t u32 = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return u32;
}

float uint32_to_float_bits(uint32_t i) {
  float result = * ( float * ) &i;
  return result;
}
