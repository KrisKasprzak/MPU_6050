/*
  FifoBuffer.ino

  Streams six axis data through the on chip FIFO and drains one complete
  packet at a time. Using the FIFO lets a sketch do other work between reads
  without dropping samples.

  Released under the MIT License, see MPU_6050.h.
*/

#include <Wire.h>
#include <MPU_6050.h>

/* Six axes at two bytes each. Add two more if the temperature sensor is
   added to the FIFO source mask.                                           */
const uint8_t PKT_LEN = 12;

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

  imuDev.setAccelRange(MPU6050_ACC_2G);
  imuDev.setGyroRange(MPU6050_GYR_250DPS);
  imuDev.setDlpfMode(MPU6050_DLPF_44HZ);
  imuDev.setSampleRateHz(50);

  imuDev.setFifoSources(MPU6050_FIF_ACCEL  | MPU6050_FIF_GYRO_X |
                        MPU6050_FIF_GYRO_Y | MPU6050_FIF_GYRO_Z);
  imuDev.setFifoEnabled(true);
  imuDev.resetFifo();

  Serial.println(F("Streaming through the FIFO."));
}

void loop()
{
  uint16_t cntVal = imuDev.getFifoCount();

  /* A full buffer has stopped accepting data, so everything in it is stale
     and the only sane response is to start over.                           */
  if (cntVal >= 1008)
  {
    imuDev.resetFifo();
    Serial.println(F("FIFO overflow, buffer reset."));
    return;
  }

  while (cntVal >= PKT_LEN)
  {
    uint8_t pktBuf[PKT_LEN];

    if (imuDev.readFifoBytes(pktBuf, PKT_LEN) != PKT_LEN)
    {
      break;
    }

    cntVal -= PKT_LEN;

    int16_t accX = (int16_t)(((uint16_t)pktBuf[0]  << 8) | pktBuf[1]);
    int16_t accY = (int16_t)(((uint16_t)pktBuf[2]  << 8) | pktBuf[3]);
    int16_t accZ = (int16_t)(((uint16_t)pktBuf[4]  << 8) | pktBuf[5]);
    int16_t gyrX = (int16_t)(((uint16_t)pktBuf[6]  << 8) | pktBuf[7]);
    int16_t gyrY = (int16_t)(((uint16_t)pktBuf[8]  << 8) | pktBuf[9]);
    int16_t gyrZ = (int16_t)(((uint16_t)pktBuf[10] << 8) | pktBuf[11]);

    float accDiv = imuDev.getAccelScale();
    float gyrDiv = imuDev.getGyroScale();

    Serial.print(accX / accDiv, 3);  Serial.print(F("  "));
    Serial.print(accY / accDiv, 3);  Serial.print(F("  "));
    Serial.print(accZ / accDiv, 3);  Serial.print(F("    "));
    Serial.print(gyrX / gyrDiv, 2);  Serial.print(F("  "));
    Serial.print(gyrY / gyrDiv, 2);  Serial.print(F("  "));
    Serial.println(gyrZ / gyrDiv, 2);
  }

  delay(100);
}
