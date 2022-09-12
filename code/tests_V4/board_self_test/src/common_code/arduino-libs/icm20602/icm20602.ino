#include <Wire.h>
#include "ICM20602.h"
#include <Arduino.h>

IMU imu{};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
//  while(!Serial.available());

  Wire.begin(); // THIS BIT IS IMPORTANT!!
  if(!imu.begin()) {
    Serial.println("Error");
  } else {
    Serial.println("Gotem");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.print("A0 "); Serial.print(analogRead(A0)); Serial.print(" A1 "); Serial.println(analogRead(A1));
  delay(100);

  float f = analogRead(A0);

  const auto t = imu.readout();

  Serial.print("accel_xout: "); Serial.println(t.accel_xout);
  Serial.print("accel_yout: "); Serial.println(t.accel_yout);
  Serial.print("accel_zout: "); Serial.println(t.accel_zout);

  Serial.print("temp_out: "); Serial.println(t.temp_out);
//  Serial.print("gyro_xout: "); Serial.println(t.gyro_xout);
//  Serial.print("gyro_yout: "); Serial.println(t.gyro_yout);
//  Serial.print("gyro_zout: "); Serial.println(t.gyro_zout);

  Serial.println();

  imu.setFullscale(IMU::ICM20602_ACCEL_RANGE_8G);
}
