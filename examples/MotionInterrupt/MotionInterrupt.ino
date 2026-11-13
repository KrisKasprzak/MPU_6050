/*
  MotionInterrupt.ino

  Configures the on chip motion detector and reports every time the board is
  tapped or moved. Wire the MPU-6050 INT pin to digital pin 2 on an Uno, or to
  any pin that supports attachInterrupt on other boards.

  Released under the MIT License, see MPU_6050.h.
*/

#include <Wire.h>
#include <MPU_6050.h>

const uint8_t INT_PIN = 2;

MPU6050 imuDev;

volatile bool hitFlg = false;

void motionIsr(void)
{
  hitFlg = true;
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

  imuDev.setAccelRange(MPU6050_ACC_2G);
  imuDev.setDlpfMode(MPU6050_DLPF_260HZ);

  /* The motion detector runs on the high pass filtered signal, so the filter
     has to be on or gravity alone will hold the detector triggered.        */
  imuDev.setAccelHighPassFilter(MPU6050_HPF_5HZ);

  imuDev.setMotionThreshold(20);   /* 20 counts at 2 mg each, so 40 mg      */
  imuDev.setMotionDuration(5);     /* sustained for 5 ms                    */

  /* Active high, push pull, latched until the status register is read.     */
  imuDev.setInterruptPinConfig(false, false, true, false);
  imuDev.setInterruptMask(MPU6050_INT_MOTION);

  pinMode(INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), motionIsr, RISING);

  Serial.println(F("Waiting for motion. Tap the board."));
}

void loop()
{
  if (hitFlg)
  {
    hitFlg = false;

    /* Reading the status register clears the latch and releases the pin.   */
    uint8_t stsVal = imuDev.getInterruptStatus();

    if (stsVal & MPU6050_INT_MOTION)
    {
      Serial.print(F("Motion detected. Axis status 0x"));
      Serial.println(imuDev.getMotionDetectStatus(), HEX);
    }
  }
}
