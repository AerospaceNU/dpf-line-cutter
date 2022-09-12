#include "src/common_code/arduino-libs/MS5xxx/MS5xxx.h"
#include "src/common_code/arduino-libs/icm20602/ICM20602.h"
#include <Wire.h>
#include <Arduino.h>

MS5xxx baro{&Wire};
IMU accel{};

#define print(a) Serial.print(a)
#define println(a) Serial.println(a)

void setup() {
  Serial.begin(115200);
  //while(!Serial) {}
  //delay(100); // Wait for the serial monitor
  Wire.begin();
  println("Starting...");

  while (baro.connect() > 0) {
    println("[Baro] Error connecting!");
  }

  while (!accel.isConnected()) {
    println(Read8(REG_WHO_AM_I, accel.address));
  }


  
  while (true) {
    baro.ReadProm();
    baro.Readout();
    const auto a = accel.readout();
    Serial.printf("[Baro] Temp %f Pressure %f\n", baro.GetTemp() / 100.0, baro.GetPres() / 100.0);
    Serial.printf("[Accel] Ax %f Ay %f Az %f T %f Gx %f Gy %f Gz %f\n",
      a.accel_xout, a.accel_yout, a.accel_zout, a.temp_out, a.gyro_xout, a.gyro_yout, a.gyro_zout);
    Serial.printf("BATT [%f] VCAP [%u] PHOTO [%u]",
                  (analogRead(A4)/1023.0)*3.6*2, (analogRead(A7)/1023.0)*3.6*2, analogRead(A6));
    println("");
    delay(1000);
  }
}

void loop() {

}
