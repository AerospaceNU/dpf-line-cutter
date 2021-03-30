#include <SPI.h>
#include "S25FL.h"
#include "main.h"

S25FL flash;  // Starts Flash class and initializes SPI
unsigned long flashLocation = 0;
uint8_t buff[64] = {0};

// struct Data {
//   uint32_t timestamp;
//   uint32_t pressure;
//   float temperature;
//   float accelX;
//   float accelY;
//   float accelZ;
//   uint16_t battSense;
//   uint16_t cutSense1;
//   uint16_t cutSense2;
//   uint16_t currentSense;
//   uint16_t photoresistor;
//   uint8_t state;
// };
Data data;

//float seaLevelFunc2(float pressure, float altitude) { return pressure / pow(1.0 - (altitude / 44330.76), 5.25588); }
//double seaLevel = seaLevelFunc(96595, 500.28); // ISH

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  delay(1000);
  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Divide the clock by 8
  SPI.setDataMode(SPI_MODE0);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  Serial.print("Sea level: "); Serial.println(seaLevelFunc(96549, 502.0));
  Serial.print("Sea level: "); Serial.println(seaLevelFunc(96705, 488.53));
  Serial.print("Sea level: "); Serial.println(seaLevelFunc(97162, 449.19));

  flashLocation = 24000 * 64; // Start on row 25000 I think??
}

bool needsSetup = true;
uint32_t lastTime = 0;

void loop() {
  if (buff[35] == 0 && flashLocation < 25000 * 64) { // Stop at row 27000
    read_flash_packet();

    if(needsSetup) { fakeSetup(data.pressure); needsSetup = false; }
    fakeLoop(data.timestamp, data.pressure, data.photoresistor);
    
    delay(1);
    auto dt = data.timestamp - lastTime;
    if(dt > 100) {
//      Serial.printf("At %u, dt was %u\n", data.timestamp, dt);
    }
    lastTime = data.timestamp;
  }
}

void read_flash_packet() {
  flash.read_start(flashLocation, buff, 64);
  
  data.timestamp = u8_to_u32(&buff[0]);
  data.pressure = u8_to_u32(&buff[4]);
  data.temperature = u8_to_float(&buff[8]);
  data.accelX = u8_to_float(&buff[12]);
  data.accelY = u8_to_float(&buff[16]);
  data.accelZ = u8_to_float(&buff[20]);
  data.battSense = u8_to_u16(&buff[24]);
  data.cutSense1 = u8_to_u16(&buff[26]);
  data.cutSense2 = u8_to_u16(&buff[28]);
  data.currentSense = u8_to_u16(&buff[30]);
  data.photoresistor = u8_to_u16(&buff[32]);
  data.state = buff[34];

//  Serial.printf("%u, %u, %u, %f, %f, %f, %f, %u, %u, %u, %u, %u\n", 
//                data.state, data.timestamp, data.pressure, data.temperature,
//                data.accelX, data.accelY, data.accelZ,
//                data.battSense, data.cutSense1, data.cutSense2,
//                data.currentSense, data.photoresistor);

  flashLocation += 64;
}

float u8_to_float(const uint8_t* bytes) {
  uint32_t u32 = u8_to_u32(bytes);
  float f = * ( float * ) &u32;
  return f;
}

uint32_t u8_to_u32(const uint8_t* bytes) {
  uint32_t u32 = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return u32;
}

uint32_t u8_to_u16(const uint8_t* bytes) {
  uint16_t u16 = (bytes[0] << 8) + bytes[1];
  return u16;
}
