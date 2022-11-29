#include "bulk_erase.h"
#include "flash_reader.h"
#include "flight_variable_upload.h"
#include "identifier.h"
#include "internal_fs_format.h"
#include "memory_reader.h"

void setup() {
  // put your setup code here, to run once:
  // nonege
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);

  Serial.println("Enter help to see options");
}

void loop() {
  // put your main code here, to run repeatedly:
  // take commands thru serial (or just run in correct sequence?)
  while (Serial.available()==0){}             // wait for user input
  String command = Serial.readStringUntil('\n');
  if (command == "bulk erase") {
    Serial.println("Received bulk erase");
    bulk_erase_setup();
    bulk_erase_loop();
  } else if (command == "flash reader") {
    Serial.println("Enter 0 to read flight data, 1 to read flight variables");
    while (Serial.available()==0){}             // wait for user input
    int readMode = Serial.parseInt();
    command = command + " " + readMode;
    Serial.println(command);
    flash_reader_setup(readMode);
  } else if (command == "flight var upload") {
    Serial.print("Received flight variable upload\n");
    fvu_setup();
  } else if (command == "identifier") {
    Serial.print("Received identifier\n");
    identifier_setup("");
  } else if (command == "identifier -r") {
    Serial.println("RESETTING ID. Enter new ID or just hit enter to keep it the same.");
    while (Serial.available()==0){}             // wait for user input
    String name = Serial.readStringUntil('\n');
    identifier_setup(name);
  } else if (command == "filesystem") {
    Serial.print("Received file system\n");
    fs_setup();
  } else if (command == "memory") {
    Serial.println("Received memory");
    memory_setup();
  } else if (command == "help") {
    Serial.println("bulk erase: clear flash memory");
    Serial.println("flash reader: read flash memory");
    Serial.println("flight var upload: upload flight variables");
    Serial.println("identifier [-r]: show board ID, -r flag will remove identifier");
    Serial.println("filesystem: initialize internal fs (will blow away data!!!)");
    Serial.println("memory: reads changelog (idt this does anything actually)");
  } else {
    Serial.println("Unrecognized command. Enter \"help\" to see valid commands.");
  }
}
