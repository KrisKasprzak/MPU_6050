/*
  BasicReadings.ino

  Prints acceleration, angular rate and temperature once per half second.

  Wiring, for an Arduino Uno:
    MPU-6050 VCC -> 3.3 V or 5 V, depending on the breakout board
    MPU-6050 GND -> GND
    MPU-6050 SCL -> A5
    MPU-6050 SDA -> A4

  Released under the MIT License, see MPU_6050.h.
*/

#include <Wire.h>
#include <MPU_6050.h>

MPU6050 imuDev;

void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();

  if (!imuDev.begin(400000))
  {
    Serial.println(F("MPU-6050 not found. Check the wiring and the address."));
    while (true) { delay(1000); }
  }

  imuDev.setAccelRange(MPU6050_ACC_4G);
  imuDev.setGyroRange(MPU6050_GYR_500DPS);
  imuDev.setDlpfMode(MPU6050_DLPF_44HZ);

  Serial.println(F("MPU-6050 ready."));
}

void loop()
{
  /* One bus transaction refreshes every value below.                       */
  if (!imuDev.update())
  {
    Serial.println(F("Read failed."));
    delay(500);
    return;
  }

  Serial.print(F("accel g  X "));
  Serial.print(imuDev.getAccelX(), 3);
  Serial.print(F("  Y "));
  Serial.print(imuDev.getAccelY(), 3);
  Serial.print(F("  Z "));
  Serial.print(imuDev.getAccelZ(), 3);

  Serial.print(F("   gyro dps  X "));
  Serial.print(imuDev.getGyroX(), 2);
  Serial.print(F("  Y "));
  Serial.print(imuDev.getGyroY(), 2);
  Serial.print(F("  Z "));
  Serial.print(imuDev.getGyroZ(), 2);

  Serial.print(F("   temp C "));
  Serial.print(imuDev.getTemperatureC(), 2);

  Serial.print(F("   roll "));
  Serial.print(imuDev.getRollDegrees(), 1);
  Serial.print(F("   pitch "));
  Serial.println(imuDev.getPitchDegrees(), 1);

  delay(500);
}
