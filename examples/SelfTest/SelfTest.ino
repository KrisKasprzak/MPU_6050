/*
  SelfTest.ino

  Runs the factory self test and prints the deviation from factory trim for
  each of the six axes. The datasheet allows plus or minus fourteen percent;
  anything wider suggests damaged silicon.

  Keep the board still while the test runs.

  Released under the MIT License, see MPU_6050.h.
*/

#include <Wire.h>
#include <MPU_6050.h>

MPU6050 imuDev;

void printAxis(const __FlashStringHelper *namStr, float devVal)
{
  Serial.print(namStr);
  Serial.print(F("  "));
  Serial.print(devVal, 2);
  Serial.print(F(" percent  "));
  Serial.println((fabs(devVal) <= 14.0f) ? F("pass") : F("FAIL"));
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();

  if (!imuDev.begin(400000))
  {
    Serial.println(F("MPU-6050 not found."));
    while (true) { delay(1000); }
  }

  Serial.println(F("Running the built in self test. Hold still."));

  MPU6050SelfTestResult tstRes;
  imuDev.runSelfTest(tstRes);

  printAxis(F("accel X"), tstRes.accX);
  printAxis(F("accel Y"), tstRes.accY);
  printAxis(F("accel Z"), tstRes.accZ);
  printAxis(F("gyro  X"), tstRes.gyrX);
  printAxis(F("gyro  Y"), tstRes.gyrY);
  printAxis(F("gyro  Z"), tstRes.gyrZ);

  Serial.println(tstRes.allPass ? F("All axes passed.")
                                : F("One or more axes failed."));
}

void loop()
{
  delay(1000);
}
