#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

const char* ID_FILENAME = "ID.txt";

using namespace Adafruit_LittleFS_Namespace;
File idFile(InternalFS);

void identifier_setup(String IDString) {

  char* ID;
  ID = &IDString[0];
 
  // Initialize Internal File System
  if(InternalFS.begin()) {
    Serial.println("FS initialized.");
  } else {
    Serial.println("Could not start file system.");
  }
  if (IDString != "") {
    InternalFS.remove(ID_FILENAME);
  }
  
  idFile.open(ID_FILENAME, FILE_O_READ);
  if (idFile) {
    Serial.print(ID_FILENAME);
    Serial.println(" exists.");
    Serial.println();
    
    uint32_t readlen;
    char buffer[64] = { 0 };
    readlen = idFile.read(buffer, sizeof(buffer));
    buffer[readlen] = 0;
    Serial.println(buffer);
    idFile.close();
  } else {
    if ( idFile.open(ID_FILENAME, FILE_O_WRITE) ) {
      Serial.print("Writing data to ");
      Serial.println(ID_FILENAME);
      idFile.write(ID, strlen(ID));
      idFile.close();
      Serial.print("ID is now ");
      Serial.println(ID);
    } else {
      Serial.print("Failed to write ");
      Serial.println(ID_FILENAME);
    }
  }
}
