#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

const char* FILENAME = "stateChangeLog.txt";
File file(InternalFS);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
    Serial.println("opening 1...");
  InternalFS.begin();  
  
  char buff1[1] = { 0 };
  char buff4[4] = { 0 };
  uint32_t temp;

  Serial.println("opening file");
  if ( file.open(FILENAME, FILE_O_READ) ) {
    int fileLength = file.size();
    for(int i = 0; i < fileLength - 34; i+=35) {
      for(int j = i; j < i+35; j+=5) {
        file.seek(j);
        file.read(buff1, 1);
        Serial.print(buff1[0]);
        Serial.print(": ");
        file.seek(j + 1);
        file.read(buff4, 4);
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
    Serial.println(FILENAME);
  }
}

void loop() {}

uint32_t u8_to_u32(const uint8_t* bytes) {
  uint32_t u32 = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return u32;
}

float uint32_to_float_bits(uint32_t i) {
  float result = * ( float * ) &i;
  return result;
}
