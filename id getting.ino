#include <Wire.h>

#define I2C_ADDRESS 0x42

void setup() {
  Wire.begin(I2C_ADDRESS);
  Serial.begin(9600);
}

void loop() {
  Wire.requestFrom(I2C_ADDRESS, 1);
  while(Wire.available()) {
    int tag_id = Wire.read();
    Serial.println(tag_id);
  }
  delay(100);
}