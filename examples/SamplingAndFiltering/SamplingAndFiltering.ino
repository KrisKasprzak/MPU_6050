/*
  SamplingAndFiltering.ino

  Shows the three signal conditioning controls together:

    * sample rate      - how often the device produces a new sample
    * oversampling     - how many device samples are averaged per update()
    * high pass filter - removes gravity and slow drift from the acceleration

  The sketch prints the unfiltered and the high pass filtered acceleration
  side by side in a format the Arduino Serial Plotter understands. Lay the
  board still and the filtered traces settle at zero even though Z still
  reads one g. Tap or shake it and only the movement shows up in the
  filtered traces.

  Released under the MIT License, see MPU_6050.h.
*/

#include <Wire.h>
#include <MPU_6050.h>

MPU6050 imuDev;

/* How often loop() calls update(). Keep the high pass cut off well below
   half of this rate.                                                       */
const uint32_t UPD_PERIOD_MS = 10;          /* 100 Hz                       */

void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }

  if (!imuDev.begin(400000))
  {
    Serial.println(F("MPU-6050 not found."));
    while (true) { delay(1000); }
  }

  imuDev.setAccelRange(MPU6050_ACC_4G);
  imuDev.setGyroRange(MPU6050_GYR_500DPS);

  /* Set the low pass filter first or last; either order works, because a
     rate requested in hertz is re-tuned whenever the DLPF changes.         */
  imuDev.setDlpfMode(MPU6050_DLPF_94HZ);
  float actHz = imuDev.setSampleRateHz(500);

  /* Average four device samples per update(): half the noise, about eight
     milliseconds of blocking at 500 Hz.                                    */
  imuDev.setOversampling(MPU6050_OSR_4);

  /* Software high pass filter on the acceleration output, 0.5 Hz cut off.  */
  imuDev.setHighPassCutoff(0.5f);

  Serial.print(F("# sample rate "));
  Serial.print(actHz, 2);
  Serial.print(F(" Hz, oversampling x"));
  Serial.print(imuDev.getOversampling());
  Serial.print(F(", high pass "));
  Serial.print(imuDev.getHighPassCutoff(), 2);
  Serial.println(F(" Hz"));
}

void loop()
{
  static uint32_t lstMs = 0;

  if ((millis() - lstMs) < UPD_PERIOD_MS)
  {
    return;
  }
  lstMs = millis();

  if (!imuDev.update())
  {
    return;
  }

  Serial.print(F("ax:"));
  Serial.print(imuDev.getAccelX(), 3);
  Serial.print(F(" ay:"));
  Serial.print(imuDev.getAccelY(), 3);
  Serial.print(F(" az:"));
  Serial.print(imuDev.getAccelZ(), 3);
  Serial.print(F(" hpx:"));
  Serial.print(imuDev.getAccelHighPassX(), 3);
  Serial.print(F(" hpy:"));
  Serial.print(imuDev.getAccelHighPassY(), 3);
  Serial.print(F(" hpz:"));
  Serial.println(imuDev.getAccelHighPassZ(), 3);
}
