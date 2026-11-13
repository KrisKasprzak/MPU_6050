/*
  AutoCalibration.ino

  Runs the automatic calibration routine and prints the offsets it settled on,
  so they can be copied into a sketch or saved to EEPROM.

  Place the board flat and still, with the Z axis pointing up, before the
  countdown finishes. The run takes a few seconds.

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
    Serial.println(F("MPU-6050 not found."));
    while (true) { delay(1000); }
  }

  Serial.println(F("Place the board flat and do not touch it."));

  for (uint8_t secIdx = 5; secIdx > 0; secIdx--)
  {
    Serial.print(secIdx);
    Serial.println(F("..."));
    delay(1000);
  }

  Serial.println(F("Calibrating."));

  /* Passing &Serial prints the residual error after every pass so the
     convergence can be watched.                                            */
  bool okFlg = imuDev.autoCalibrate(600, 12, &Serial);

  MPU6050Offsets finOff;
  imuDev.getOffsets(finOff);

  Serial.println();
  Serial.println(okFlg ? F("Calibration succeeded.")
                       : F("Calibration did not fully converge. Was the "
                           "board level and still?"));

  Serial.println(F("Copy these lines into your sketch after begin():"));
  Serial.print(F("  imuDev.setAccelOffsetX("));
  Serial.print(finOff.accX);
  Serial.println(F(");"));
  Serial.print(F("  imuDev.setAccelOffsetY("));
  Serial.print(finOff.accY);
  Serial.println(F(");"));
  Serial.print(F("  imuDev.setAccelOffsetZ("));
  Serial.print(finOff.accZ);
  Serial.println(F(");"));
  Serial.print(F("  imuDev.setGyroOffsetX("));
  Serial.print(finOff.gyrX);
  Serial.println(F(");"));
  Serial.print(F("  imuDev.setGyroOffsetY("));
  Serial.print(finOff.gyrY);
  Serial.println(F(");"));
  Serial.print(F("  imuDev.setGyroOffsetZ("));
  Serial.print(finOff.gyrZ);
  Serial.println(F(");"));
  Serial.println();
}

void loop()
{
  if (imuDev.update())
  {
    Serial.print(F("accel g  "));
    Serial.print(imuDev.getAccelX(), 3);
    Serial.print(F("  "));
    Serial.print(imuDev.getAccelY(), 3);
    Serial.print(F("  "));
    Serial.print(imuDev.getAccelZ(), 3);
    Serial.print(F("   gyro dps  "));
    Serial.print(imuDev.getGyroX(), 2);
    Serial.print(F("  "));
    Serial.print(imuDev.getGyroY(), 2);
    Serial.print(F("  "));
    Serial.println(imuDev.getGyroZ(), 2);
  }

  delay(500);
}
