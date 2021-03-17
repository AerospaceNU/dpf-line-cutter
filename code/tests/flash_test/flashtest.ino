#include <Arduino.h>
#include "S25FLx.h"
#include <SPI.h>
#define cs  8   //Chip select pin

void setup() {
  pinMode(cs, OUTPUT);
  Serial.begin(115200);
  while(!Serial);

  while(Serial.available() < 1) { Serial.println("Waiting"); delay(500); }
  Serial.println("Starting");

  Flash flash{};  //starts flash class and initilzes SPI

  Serial.println("Began");

  SPI.setClockDivider(SPI_CLOCK_DIV2); //By default the clock divider is set to 8. 

  Serial.println("Began2");
  flash.waitforit(); // use between each communication to make sure S25FLxx is ready to go.
  flash.read_info(); //will return an error if the chip isn't wired up correctly. 
  Serial.println("Began3");

  // Print the first 10 sectors
  printPage(flash);

  flash.erase_64k(0);

  uint8_t* bank = new uint8_t[255];
  bank = (uint8_t*) "Did you ever hear the tragedy of Darth Plagueis The Wise? I thought not. It’s not a story the Jedi would tell you. It’s a Sith legend. Darth Plagueis was a Dark Lord of the Sith... he could use the Force to influence the midichlorians to create life U w U";
  
  flash.write(0, bank, 255);

  // //void read(unsigned long loc, uint8_t* array, unsigned long length);
  uint8_t* array2 = new uint8_t[32];
//  flash.read(0, array2, 32);
//  Serial.print("Got "); Serial.println((char*)array2);


  printPage(flash);
}

void printPage(Flash& flash) {
  uint8_t* array2 = new uint8_t[16];
  Serial.println(); Serial.println();
  Serial.println("Page 0:");
  Serial.println("===================");
  Serial.println();
  for(uint32_t i = 0; i < 256000 / 16; i+=16) {
    flash.read(i, array2, 16);
//    Serial.print(array2, BIN);
//    Serial.print(" ; ");
    Serial.println((char*)array2);
  }
  Serial.println();
}

void loop() {
//  digitalWrite(11, LOW);
//  digitalWrite(12, HIGH);
//  
//  Serial.print("A4 : "); Serial.println(analogRead(A4) * 2 / 1024.0 * 3.6);
//  delay(100);
}
