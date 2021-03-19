#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

const char* FILENAME = "ID.txt";
const char* VERSION = "0";
const char* LANDED = "0";
const char* ID = "Strawberry";

using namespace Adafruit_LittleFS_Namespace;
File f(InternalFS);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  // Initialize Internal File System
  if(InternalFS.begin()) {
    Serial.println("FS initialized.");
  } else {
    Serial.println("Could not start file system.");
  }

  InternalFS.remove(FILENAME); // uncomment if you want to reset the ID

  f.open(FILENAME, FILE_O_READ);

  if (f) {
    Serial.print(FILENAME);
    Serial.println(" exists.");
    Serial.println();
    
    uint32_t readlen;
    char buffer[64] = { 0 };
    readlen = f.read(buffer, sizeof(buffer));
    buffer[readlen] = 0;
    Serial.println(buffer);
    f.close();
  } else {
    if ( f.open(FILENAME, FILE_O_WRITE) ) {
      Serial.print("Writing data to ");
      Serial.println(FILENAME);
      f.write(VERSION, 1);
      f.write(LANDED, 1);
      f.write(ID, strlen(ID));
      f.close();
      Serial.print("ID is now ");
      Serial.print(VERSION);
      Serial.print(LANDED);
      Serial.println(ID);
    } else {
      Serial.print("Failed to write ");
      Serial.println(FILENAME);
    }
  }
}

void loop() {}
