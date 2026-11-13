/*
  ConnectionMonitor.ino

  Demonstrates the connection health check. The sketch keeps reading the
  sensor and reports the moment the device drops off the bus, then recovers
  automatically once it comes back. Unplug the SDA or SCL wire while this runs
  to see it work.

  Released under the MIT License, see MPU_6050.h.
*/

#include <Wire.h>
#include <MPU_6050.h>

MPU6050 imuDev;

bool          wasUpp  = false;   /* connection state on the previous pass   */
unsigned long lstChk  = 0;       /* time stamp of the last health check     */

void configureSensor(void)
{
  imuDev.setAccelRange(MPU6050_ACC_2G);
  imuDev.setGyroRange(MPU6050_GYR_250DPS);
  imuDev.setDlpfMode(MPU6050_DLPF_44HZ);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();

  wasUpp = imuDev.begin(400000);

  if (wasUpp)
  {
    configureSensor();
    Serial.print(F("Connected. WHO_AM_I reads 0x"));
    Serial.println(imuDev.whoAmI(), HEX);
  }
  else
  {
    Serial.println(F("No sensor at start up. Waiting for it to appear."));
  }
}

void loop()
{
  /* Check the link twice per second. isConnected() confirms the device both
     acknowledges its address and returns a sane identity byte, so it also
     catches a sensor that has browned out and come back confused.          */
  if (millis() - lstChk >= 500)
  {
    lstChk = millis();

    bool nowUpp = imuDev.isConnected();

    if (nowUpp && !wasUpp)
    {
      Serial.println(F("Sensor reconnected. Reconfiguring."));
      imuDev.begin();
      configureSensor();
    }
    else if (!nowUpp && wasUpp)
    {
      Serial.print(F("Sensor lost. Wire error code "));
      Serial.println(imuDev.getLastError());
    }

    wasUpp = nowUpp;
  }

  if (wasUpp && imuDev.update())
  {
    Serial.print(F("magnitude g "));
    Serial.println(imuDev.getAccelMagnitude(), 3);
  }

  delay(250);
}
