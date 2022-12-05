/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

using namespace Adafruit_LittleFS_Namespace;

// the setup function runs once when you press reset or power the board
void fs_setup() 
{
  Serial.println("InternalFS Format Example");
  Serial.println();

  // Wait for user input to run.
  Serial.println("This sketch will destroy all of your data in External Flash! Enter Y to continue.");
  while ( !Serial.available() ) {};
  String input = Serial.readStringUntil('\n');
  if (input != "Y" && input != "y") {
    Serial.println("Not formatting.");
    return; 
  }
  Serial.println();
  Serial.println();

  // Initialize Internal File System
  InternalFS.begin();

  Serial.print("Formatting ... ");
  delay(1); // for message appear on monitor

  // Format 
  InternalFS.format();

  Serial.println("Done");
}
