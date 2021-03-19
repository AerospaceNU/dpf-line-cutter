#include "src/common_code/arduino-libs/MS5xxx/MS5xxx.h"
#include "src/common_code/arduino-libs/icm20602/ICM20602.h"
#include <Wire.h>
#include <Arduino.h>

MS5xxx baro{&Wire};
IMU accel{};

#define print(a) Serial.print(a)
#define println(a) Serial.println(a)

void setup() {
    Serial.begin(9600);
    while(!Serial) {}
    delay(100); // Wait for the serial monitor
    println("Starting...");

    if(!baro.connect()) { println("[Baro] Error connecting!"); }

    if(!accel.begin()) { println("[Accel] Error connecting!"); }

    baro.ReadProm();
    baro.Readout();
    const auto a = accel.readout();
    Serial.printf("[Baro] Temp %f Pressure %f\n", baro.GetTemp() / 100.0, baro.GetPres() / 100.0);
    Serial.printf("[Accel] Ax %f Ay %f Az %f T %f Gx %f Gy %f Gz %f\n", 
      a.accel_xout, a.accel_yout, a.accel_zout, a.temp_out, a.gyro_xout, a.gyro_yout, a.gyro_zout);
    Serial.printf("BATT [%u] CUT_S1 [%u] CUT_S2 [%u] CURR_SENSE [%u] PHOTO [%u]", 
      analogRead(A4), analogRead(A5), analogRead(A0), analogRead(A1), analogRead(A3));
}

void loop() {

}
