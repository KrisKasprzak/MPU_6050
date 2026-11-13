/*
  MPU_6050.cpp - Implementation for the InvenSense MPU-6050 six-axis MEMS
                motion tracking device.

  Released under the MIT License. The full license text lives in MPU_6050.h
  together with the revision table for this library.

  Implementation notes
  ---------------------------------------------------------------------------
  The MPU-6050 is a straightforward I2C slave. Every feature is reached by
  reading or writing one of the registers listed in MPU_6050.h, so the bulk of
  this file is thin, well named wrappers over four primitives:

      writeRegister()     - one byte out
      readRegister()      - one byte in
      readRegisters()     - a burst of bytes in, which matters because the
                            device latches all sensor registers at the start
                            of a burst so a single read returns one coherent
                            sample rather than a torn mixture of two
      writeRegisterBits() - a read, modify, write cycle for packed fields

  Sign conventions follow the datasheet: all sensor outputs are sixteen bit
  two's complement values, most significant byte first.
*/

#include "MPU_6050.h"

/* Conversion constant used by the roll and pitch helpers. Declared here so
   the library does not depend on any particular definition of PI.          */
static const float RAD_TO_DEGREES = 57.29577951308232f;

/* Temperature transfer function from the datasheet, section 4.18:
       degrees Celsius = rawCount / 340 + 36.53                             */
static const float TMP_SENSITIVITY = 340.0f;
static const float TMP_OFFSET_DEG  = 36.53f;

/* ========================================================================= */
/* Construction and start up                                                 */
/* ========================================================================= */

/*
  Constructor. Nothing touches the I2C bus here, because a global object is
  built before setup() runs and the Wire peripheral is not ready yet. All the
  constructor does is record which bus and which address to use later.
*/
MPU6050::MPU6050(uint8_t devAdr, TwoWire &busRef)
{
  busPtr = &busRef;
  adrVal = devAdr;
  accRng = MPU6050_ACC_2G;
  gyrRng = MPU6050_GYR_250DPS;
  dlpCfg = MPU6050_DLPF_260HZ;   /* power on value of DLPF_CFG             */
  smpDiv = 0;
  smpReq = 0.0f;
  osrCnt = 1;
  errCod = 0;
  calDne = false;

  hpfCut = 0.0f;
  hpfRdy = false;
  hpfTim = 0;
  for (uint8_t idx = 0; idx < 3; idx++)
  {
    hpfInp[idx] = 0.0f;
    hpfOut[idx] = 0.0f;
  }

  curVal.accX   = 0.0f;
  curVal.accY   = 0.0f;
  curVal.accZ   = 0.0f;
  curVal.tmpDeg = 0.0f;
  curVal.gyrX   = 0.0f;
  curVal.gyrY   = 0.0f;
  curVal.gyrZ   = 0.0f;

  resErr.accX = 0;
  resErr.accY = 0;
  resErr.accZ = 0;
  resErr.gyrX = 0;
  resErr.gyrY = 0;
  resErr.gyrZ = 0;
}

/*
  Start the device with sensible defaults and leave it streaming data.

  Sequence matters here. The MPU-6050 boots into sleep mode with the internal
  eight megahertz relaxation oscillator selected. That oscillator drifts badly
  with temperature, so the recommended practice is to clear the sleep bit and
  then switch the clock to the X axis gyroscope phase locked loop, which is
  far more stable. The gyroscope needs a moment to spin up before the phase
  locked loop will lock, hence the short delay.

  Returns true only when the device answers on the bus afterwards, so the
  caller can branch on a missing or mis-addressed sensor.
*/
bool MPU6050::begin(void)
{
  return begin(0);
}

/*
  Same as begin() but also sets the I2C bus clock. Pass 100000 for standard
  mode or 400000 for fast mode, which the MPU-6050 supports. Passing zero
  leaves whatever clock the sketch already configured untouched.

  Wire.begin() is called here as well. Calling it a second time from the
  sketch is harmless, so either order works.
*/
bool MPU6050::begin(uint32_t busHz)
{
  busPtr->begin();

  if (busHz > 0)
  {
    busPtr->setClock(busHz);
  }

  /* Bail out early if nothing acknowledges the address. Continuing would
     simply write into the void and report success.                         */
  if (!isAlive())
  {
    return false;
  }

  /* Clear the power on sleep bit so the analog front end starts up.        */
  setSleepEnabled(false);
  delay(10);

  /* Prefer the gyroscope phase locked loop over the internal oscillator.   */
  setClockSource(MPU6050_CLK_PLL_XGYR);
  delay(10);

  /* A usable general purpose default: plus or minus two g, plus or minus
     two hundred and fifty degrees per second, a forty four hertz low pass
     filter and a one kilohertz sample rate.                                */
  setAccelRange(MPU6050_ACC_2G);
  setGyroRange(MPU6050_GYR_250DPS);
  setDlpfMode(MPU6050_DLPF_44HZ);
  setSampleRateDivider(0);

  /* Make sure the FIFO and the auxiliary master are quiet until asked for. */
  setFifoEnabled(false);
  setFifoSources(0x00);
  setInterruptMask(0x00);

  /* Any previous filter history belongs to a different configuration.     */
  resetHighPass();

  return isConnected();
}

/*
  Put the device into its lowest power state. The I2C bus itself is left
  alone because other slaves may still be using it.
*/
void MPU6050::end(void)
{
  setSleepEnabled(true);
}

/* ========================================================================= */
/* Health and identity                                                       */
/* ========================================================================= */

/*
  Cheap liveness probe. Addresses the device and looks for an acknowledge
  bit. This costs one byte on the wire and is safe to call inside a fast
  loop, which makes it the right choice for watchdog style polling.
*/
bool MPU6050::isAlive(void)
{
  busPtr->beginTransmission(adrVal);
  errCod = busPtr->endTransmission();
  return (errCod == 0);
}

/*
  Full connection check. Confirms that something acknowledges the address and
  that the identity register reads back a plausible value.

  A floating SDA line pulled up by a resistor reads as 0xFF and a shorted line
  reads as 0x00, so both of those are rejected even if an acknowledge somehow
  appeared. Genuine MPU-6050 silicon reports 0x68; pin compatible clones in
  circulation report other values, so those are accepted rather than refused,
  and whoAmI() is exposed for sketches that need to be strict.
*/
bool MPU6050::isConnected(void)
{
  if (!isAlive())
  {
    return false;
  }

  uint8_t whoVal = whoAmI();

  if (errCod != 0)
  {
    return false;
  }

  return ((whoVal != 0x00) && (whoVal != 0xFF));
}

/*
  Read the identity register. On genuine silicon this returns 0x68, which is
  bits six through one of the slave address and therefore does not change when
  the AD0 pin is pulled high.
*/
uint8_t MPU6050::whoAmI(void)
{
  return readRegister(MPU6050_REG_WHO_AM_I);
}

uint8_t MPU6050::getAddress(void) const
{
  return adrVal;
}

/*
  Retarget the object at a different slave address at run time, which is handy
  when two devices share a bus and the AD0 pins are driven by the host.
*/
void MPU6050::setAddress(uint8_t devAdr)
{
  adrVal = devAdr;
}

/*
  Status code from the most recent Wire transaction. Zero means success; the
  remaining values are defined by the Wire library, where 2 is an address NACK
  and 3 is a data NACK. Useful for reporting why isConnected() failed.
*/
uint8_t MPU6050::getLastError(void) const
{
  return errCod;
}

/* ========================================================================= */
/* Device level control                                                      */
/* ========================================================================= */

/*
  Assert the device reset bit. Every register except WHO_AM_I and the power
  management registers returns to its default, and the part lands back in
  sleep mode, so begin() has to run again afterwards. The hundred millisecond
  delay covers the internal start up time quoted in the datasheet.
*/
void MPU6050::reset(void)
{
  writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 7, true);
  delay(100);
}

/*
  Reset the analog and digital signal paths for all three sensors without
  clearing the configuration registers. Use this to flush a saturated filter
  after a hard mechanical shock.
*/
void MPU6050::resetSignalPaths(void)
{
  writeRegister(MPU6050_REG_SIG_PATH_RST, 0x07);
  delay(2);
}

/*
  Clear the sensor data registers and the FIFO without disturbing the signal
  path or the configuration.
*/
void MPU6050::resetSensorRegisters(void)
{
  writeRegisterBit(MPU6050_REG_USER_CTRL, 0, true);
  delay(2);
}

void MPU6050::sleep(void)
{
  setSleepEnabled(true);
}

void MPU6050::wake(void)
{
  setSleepEnabled(false);
  delay(10);
}

/*
  Sleep mode powers down the sensors but keeps the register contents and the
  I2C interface alive, so settings survive a sleep and wake cycle.
*/
void MPU6050::setSleepEnabled(bool wantSlp)
{
  writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 6, wantSlp);
}

bool MPU6050::getSleepEnabled(void)
{
  return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 6);
}

/*
  Cycle mode is the low power accelerometer mode. The device wakes at the rate
  set by setWakeRate(), takes a single accelerometer sample and goes back to
  sleep. The gyroscope must be in standby for this mode to be meaningful.
*/
void MPU6050::setCycleEnabled(bool wantCyc)
{
  writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 5, wantCyc);
}

bool MPU6050::getCycleEnabled(void)
{
  return readRegisterBit(MPU6050_REG_PWR_MGMT_1, 5);
}

/* LP_WAKE_CTRL occupies bits seven and six of PWR_MGMT_2.                   */
void MPU6050::setWakeRate(uint8_t rateSel)
{
  writeRegisterBits(MPU6050_REG_PWR_MGMT_2, 7, 2, rateSel);
}

uint8_t MPU6050::getWakeRate(void)
{
  return readRegisterBits(MPU6050_REG_PWR_MGMT_2, 7, 2);
}

/*
  The TEMP_DIS bit is inverted with respect to the name used here: setting it
  disables the temperature sensor, so the argument is negated on the way in.
*/
void MPU6050::setTemperatureSensorEnabled(bool wantTmp)
{
  writeRegisterBit(MPU6050_REG_PWR_MGMT_1, 3, !wantTmp);
}

bool MPU6050::getTemperatureSensorEnabled(void)
{
  return !readRegisterBit(MPU6050_REG_PWR_MGMT_1, 3);
}

/*
  Individually park axes to save current. Build the argument from the
  MPU6050_STB_ constants, for example MPU6050_STB_GYRO_X | MPU6050_STB_GYRO_Y
  to leave only the Z axis gyroscope running.
*/
void MPU6050::setStandbyMask(uint8_t stbMsk)
{
  writeRegisterBits(MPU6050_REG_PWR_MGMT_2, 5, 6, stbMsk & 0x3F);
}

uint8_t MPU6050::getStandbyMask(void)
{
  return readRegisterBits(MPU6050_REG_PWR_MGMT_2, 5, 6);
}

/* CLKSEL occupies the lowest three bits of PWR_MGMT_1.                     */
void MPU6050::setClockSource(uint8_t clkSel)
{
  writeRegisterBits(MPU6050_REG_PWR_MGMT_1, 2, 3, clkSel);
}

uint8_t MPU6050::getClockSource(void)
{
  return readRegisterBits(MPU6050_REG_PWR_MGMT_1, 2, 3);
}

/* ========================================================================= */
/* Sampling and filtering                                                    */
/* ========================================================================= */

/*
  True when the gyroscope runs at eight kilohertz. DLPF_CFG values zero and
  seven both bypass the low pass filter; seven is listed as reserved in the
  datasheet but behaves the same way on real silicon.
*/
static bool dlpfBypassed(uint8_t dlpSel)
{
  return ((dlpSel == 0) || (dlpSel == 7));
}

/*
  The sample rate divider produces the output rate from the gyroscope rate:

      sample rate = gyroscope output rate / (1 + divider)

  The gyroscope output rate is eight kilohertz when the digital low pass
  filter is disabled and one kilohertz for every other filter setting, which
  is the single most common source of confusion with this part.

  The accelerometer itself never produces more than one kilohertz. At sample
  rates above that, the same accelerometer value is repeated in consecutive
  samples while the gyroscope keeps updating.

  Setting the divider directly cancels any rate requested earlier through
  setSampleRateHz(), so later DLPF changes will no longer re-tune it.
*/
void MPU6050::setSampleRateDivider(uint8_t divVal)
{
  if (writeRegister(MPU6050_REG_SMPLRT_DIV, divVal))
  {
    smpDiv = divVal;
  }
  smpReq = 0.0f;
}

uint8_t MPU6050::getSampleRateDivider(void)
{
  smpDiv = readRegister(MPU6050_REG_SMPLRT_DIV);
  return smpDiv;
}

/*
  Program the divider that comes closest to the requested rate in hertz and
  return the rate actually achieved.

  The achievable rates are quantized: 8000 / n or 1000 / n for n from 1 to
  256, so the slowest possible rate is 3.90625 Hz. Fractional requests such
  as 12.5 Hz are honoured where the quantization allows.

  The request is remembered. If setDlpfMode() later switches the gyroscope
  between eight kilohertz and one kilohertz, the divider is recomputed so the
  output rate stays where it was asked to be, instead of jumping by a factor
  of eight.
*/
float MPU6050::setSampleRateHz(float reqHz)
{
  if (reqHz <= 0.0f)
  {
    return getSampleRateHz();
  }

  /* Make sure the cached DLPF matches the device before using it.          */
  getDlpfMode();

  smpReq = reqHz;
  applyRequestedRate();

  return getSampleRateHz();
}

/*
  Rounds to the nearest divider rather than truncating, so a request for
  150 Hz lands on 142.9 Hz (divider 6) instead of 166.7 Hz (divider 5).
*/
void MPU6050::applyRequestedRate(void)
{
  float basHz  = dlpfBypassed(dlpCfg) ? 8000.0f : 1000.0f;
  long  divVal = lround(basHz / smpReq) - 1;

  if (divVal < 0)
  {
    divVal = 0;
  }
  if (divVal > 255)
  {
    divVal = 255;
  }

  if (writeRegister(MPU6050_REG_SMPLRT_DIV, (uint8_t)divVal))
  {
    smpDiv = (uint8_t)divVal;
  }
}

/* Achieved output rate, read back from the device.                         */
float MPU6050::getSampleRateHz(void)
{
  return getGyroOutputRateHz() / (1.0f + (float)getSampleRateDivider());
}

float MPU6050::getGyroOutputRateHz(void)
{
  return dlpfBypassed(getDlpfMode()) ? 8000.0f : 1000.0f;
}

/*
  Time between samples in microseconds, from the cached configuration so no
  bus traffic is needed on the oversampling hot path.
*/
uint32_t MPU6050::samplePeriodMicros(void) const
{
  uint32_t basUs = dlpfBypassed(dlpCfg) ? 125UL : 1000UL;
  return basUs * (1UL + (uint32_t)smpDiv);
}

/*
  The digital low pass filter trades bandwidth for noise and adds group delay.
  Heavier filtering is quieter but lags further behind reality, which matters
  in a control loop.

  When a sample rate was requested in hertz, the divider is recomputed here so
  the output rate survives a switch between the eight kilohertz and the one
  kilohertz gyroscope rate.
*/
void MPU6050::setDlpfMode(uint8_t dlpSel)
{
  if (writeRegisterBits(MPU6050_REG_CONFIG, 2, 3, dlpSel & 0x07))
  {
    dlpCfg = dlpSel & 0x07;
  }

  if (smpReq > 0.0f)
  {
    applyRequestedRate();
  }
}

uint8_t MPU6050::getDlpfMode(void)
{
  dlpCfg = readRegisterBits(MPU6050_REG_CONFIG, 2, 3);
  return dlpCfg;
}

/*
  EXT_SYNC_SET routes the FSYNC pin level into the least significant bit of
  one of the sensor outputs so an external event can be time stamped within
  the data stream. Zero disables the feature.
*/
void MPU6050::setExternalSyncMode(uint8_t syncSel)
{
  writeRegisterBits(MPU6050_REG_CONFIG, 5, 3, syncSel);
}

uint8_t MPU6050::getExternalSyncMode(void)
{
  return readRegisterBits(MPU6050_REG_CONFIG, 5, 3);
}

/* ========================================================================= */
/* Software oversampling                                                     */
/* ========================================================================= */

/*
  The MPU-6050 has no oversampling register, so oversampling is done here:
  readAll() and update() take osrVal consecutive device samples, one sample
  period apart, and return their average. White noise drops by the square
  root of the ratio, so 4x halves it and 16x quarters it.

  Each call blocks for roughly osrVal sample periods. At the default one
  kilohertz rate, 16x costs about sixteen milliseconds per update(). At low
  sample rates this adds up quickly: 64x at 10 Hz blocks for 6.4 seconds.

  Raw reads (readRawAll and friends), the FIFO and the calibration routine
  are not affected. Values outside 1 to 64 are clamped.
*/
void MPU6050::setOversampling(uint8_t osrVal)
{
  if (osrVal < 1)
  {
    osrVal = 1;
  }
  if (osrVal > MPU6050_OSR_MAX)
  {
    osrVal = MPU6050_OSR_MAX;
  }
  osrCnt = osrVal;
}

uint8_t MPU6050::getOversampling(void) const
{
  return osrCnt;
}

/*
  Spacing the reads one sample period apart matters: reading faster than the
  device updates just returns the same latched sample several times, which
  costs bus time without reducing any noise.
*/
bool MPU6050::readRawOversampled(MPU6050RawData &outRaw)
{
  if (osrCnt <= 1)
  {
    return readRawAll(outRaw);
  }

  int32_t        sumArr[7] = {0, 0, 0, 0, 0, 0, 0};
  uint32_t       perUs     = samplePeriodMicros();
  uint32_t       lstUs     = 0;
  MPU6050RawData rawRec;

  for (uint8_t idx = 0; idx < osrCnt; idx++)
  {
    if (idx > 0)
    {
      while ((uint32_t)(micros() - lstUs) < perUs)
      {
        yield();
      }
    }
    lstUs = micros();

    if (!readRawAll(rawRec))
    {
      return false;
    }

    sumArr[0] += rawRec.accX;
    sumArr[1] += rawRec.accY;
    sumArr[2] += rawRec.accZ;
    sumArr[3] += rawRec.tmpRaw;
    sumArr[4] += rawRec.gyrX;
    sumArr[5] += rawRec.gyrY;
    sumArr[6] += rawRec.gyrZ;
  }

  /* Round to nearest instead of truncating toward zero, which would bias
     every averaged value by up to one count toward zero.                   */
  int16_t avgArr[7];
  int32_t hlfCnt = (int32_t)osrCnt / 2;

  for (uint8_t idx = 0; idx < 7; idx++)
  {
    int32_t sumVal = sumArr[idx];
    avgArr[idx] = (int16_t)((sumVal >= 0)
                            ? ((sumVal + hlfCnt) / (int32_t)osrCnt)
                            : ((sumVal - hlfCnt) / (int32_t)osrCnt));
  }

  outRaw.accX   = avgArr[0];
  outRaw.accY   = avgArr[1];
  outRaw.accZ   = avgArr[2];
  outRaw.tmpRaw = avgArr[3];
  outRaw.gyrX   = avgArr[4];
  outRaw.gyrY   = avgArr[5];
  outRaw.gyrZ   = avgArr[6];

  return true;
}

/* ========================================================================= */
/* Full scale range                                                          */
/* ========================================================================= */

/*
  Selecting a range rewrites AFS_SEL and caches the choice locally, because
  every scaled reading divides by the matching sensitivity and reading that
  back over I2C on every sample would triple the bus traffic.
*/
void MPU6050::setAccelRange(uint8_t rngSel)
{
  writeRegisterBits(MPU6050_REG_ACCEL_CONFIG, 4, 2, rngSel & 0x03);
  accRng = rngSel & 0x03;
}

/*
  Read the range back from the device rather than from the cache, and refresh
  the cache while doing so. This keeps things correct if another piece of code
  reconfigured the part behind this object's back.
*/
uint8_t MPU6050::getAccelRange(void)
{
  accRng = readRegisterBits(MPU6050_REG_ACCEL_CONFIG, 4, 2);
  return accRng;
}

void MPU6050::setGyroRange(uint8_t rngSel)
{
  writeRegisterBits(MPU6050_REG_GYRO_CONFIG, 4, 2, rngSel & 0x03);
  gyrRng = rngSel & 0x03;
}

uint8_t MPU6050::getGyroRange(void)
{
  gyrRng = readRegisterBits(MPU6050_REG_GYRO_CONFIG, 4, 2);
  return gyrRng;
}

/*
  Accelerometer sensitivity in counts per g. Each step up in range halves the
  resolution, starting from 16384 counts per g at plus or minus two g.
*/
float MPU6050::getAccelScale(void) const
{
  switch (accRng)
  {
    case MPU6050_ACC_4G:  return 8192.0f;
    case MPU6050_ACC_8G:  return 4096.0f;
    case MPU6050_ACC_16G: return 2048.0f;
    default:              return 16384.0f;
  }
}

/*
  Gyroscope sensitivity in counts per degree per second, again halving with
  each range step from 131 counts at plus or minus two hundred and fifty
  degrees per second.
*/
float MPU6050::getGyroScale(void) const
{
  switch (gyrRng)
  {
    case MPU6050_GYR_500DPS:  return 65.5f;
    case MPU6050_GYR_1000DPS: return 32.8f;
    case MPU6050_GYR_2000DPS: return 16.4f;
    default:                  return 131.0f;
  }
}

/* ========================================================================= */
/* High pass filter                                                          */
/* ========================================================================= */

/*
  Hardware filter. The accelerometer high pass filter never appears in the
  data registers. It feeds the motion, zero motion and free fall detectors
  only, and its whole job is to strip gravity out so those detectors respond
  to change rather than to orientation. Pass one of the MPU6050_HPF_ values.
*/
void MPU6050::setAccelHighPassFilter(uint8_t hpfSel)
{
  writeRegisterBits(MPU6050_REG_ACCEL_CONFIG, 2, 3, hpfSel & 0x07);
}

uint8_t MPU6050::getAccelHighPassFilter(void)
{
  return readRegisterBits(MPU6050_REG_ACCEL_CONFIG, 2, 3);
}

/*
  Software filter. Because the hardware filter cannot be read out, the
  library runs its own first order high pass filter on every readAll() or
  update() and serves the result through getAccelHighPassX/Y/Z(). The
  ordinary getAccelX/Y/Z() keep returning the unfiltered values, so both are
  available at the same time.

  Typical uses are removing gravity and slow bias to see vibration, impacts
  or short movements. Choose a cut off well below the frequencies of interest
  and well below half the rate at which update() is called; a cut off above
  that is meaningless because the filter cannot see those frequencies.

  The filter measures the real interval between calls with micros(), so it
  stays correct when update() is called at an irregular rate. Changing the
  cut off restarts the filter. Pass zero to turn it off.
*/
void MPU6050::setHighPassCutoff(float cutHz)
{
  hpfCut = (cutHz > 0.0f) ? cutHz : 0.0f;
  resetHighPass();
}

float MPU6050::getHighPassCutoff(void) const
{
  return hpfCut;
}

bool MPU6050::getHighPassEnabled(void) const
{
  return (hpfCut > 0.0f);
}

/*
  Forget the filter history. The next sample primes the filter and produces a
  zero output, which avoids the large step a cold filter would otherwise show
  as it sees gravity appear out of nowhere. Call this after a long pause in
  sampling, or after changing the range or the offsets.
*/
void MPU6050::resetHighPass(void)
{
  hpfRdy = false;
  for (uint8_t idx = 0; idx < 3; idx++)
  {
    hpfInp[idx] = 0.0f;
    hpfOut[idx] = 0.0f;
  }
}

float MPU6050::getAccelHighPassX(void) const { return hpfOut[0]; }
float MPU6050::getAccelHighPassY(void) const { return hpfOut[1]; }
float MPU6050::getAccelHighPassZ(void) const { return hpfOut[2]; }

/*
  Discrete first order high pass, the standard RC form:

      alpha = RC / (RC + dt),   RC = 1 / (2 pi fc)
      y[n]  = alpha * (y[n-1] + x[n] - x[n-1])

  dt comes from micros(), which wraps every seventy minutes; the unsigned
  subtraction handles the wrap correctly.
*/
void MPU6050::applyHighPass(void)
{
  if (hpfCut <= 0.0f)
  {
    return;
  }

  float    inpArr[3] = {curVal.accX, curVal.accY, curVal.accZ};
  uint32_t nowUs     = micros();

  if (!hpfRdy)
  {
    for (uint8_t idx = 0; idx < 3; idx++)
    {
      hpfInp[idx] = inpArr[idx];
      hpfOut[idx] = 0.0f;
    }
    hpfTim = nowUs;
    hpfRdy = true;
    return;
  }

  float dltSec = (float)(uint32_t)(nowUs - hpfTim) * 1.0e-6f;
  hpfTim = nowUs;

  if (dltSec <= 0.0f)
  {
    dltSec = (float)samplePeriodMicros() * 1.0e-6f;
  }

  const float rcSec = 1.0f / (6.283185307179586f * hpfCut);
  const float alpha = rcSec / (rcSec + dltSec);

  for (uint8_t idx = 0; idx < 3; idx++)
  {
    hpfOut[idx] = alpha * (hpfOut[idx] + inpArr[idx] - hpfInp[idx]);
    hpfInp[idx] = inpArr[idx];
  }
}

/* ========================================================================= */
/* Measurement, raw                                                          */
/* ========================================================================= */

/*
  Read all fourteen sensor bytes in one burst.

  This is the only correct way to sample the device. The sensor registers are
  latched when the burst begins, so accelerometer, temperature and gyroscope
  values in a single burst all belong to the same instant. Three separate
  reads can straddle a sample boundary and hand back a mixture of two samples.
*/
bool MPU6050::readRawAll(MPU6050RawData &outRaw)
{
  uint8_t rawBuf[14];

  if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, rawBuf, 14))
  {
    return false;
  }

  /* Every value arrives most significant byte first, two's complement.     */
  outRaw.accX   = (int16_t)(((uint16_t)rawBuf[0]  << 8) | rawBuf[1]);
  outRaw.accY   = (int16_t)(((uint16_t)rawBuf[2]  << 8) | rawBuf[3]);
  outRaw.accZ   = (int16_t)(((uint16_t)rawBuf[4]  << 8) | rawBuf[5]);
  outRaw.tmpRaw = (int16_t)(((uint16_t)rawBuf[6]  << 8) | rawBuf[7]);
  outRaw.gyrX   = (int16_t)(((uint16_t)rawBuf[8]  << 8) | rawBuf[9]);
  outRaw.gyrY   = (int16_t)(((uint16_t)rawBuf[10] << 8) | rawBuf[11]);
  outRaw.gyrZ   = (int16_t)(((uint16_t)rawBuf[12] << 8) | rawBuf[13]);

  return true;
}

/* Six axis convenience form for callers that do not want the structure.    */
bool MPU6050::readRawMotion(int16_t &accXOut, int16_t &accYOut,
                            int16_t &accZOut, int16_t &gyrXOut,
                            int16_t &gyrYOut, int16_t &gyrZOut)
{
  MPU6050RawData tmpRec;

  if (!readRawAll(tmpRec))
  {
    return false;
  }

  accXOut = tmpRec.accX;
  accYOut = tmpRec.accY;
  accZOut = tmpRec.accZ;
  gyrXOut = tmpRec.gyrX;
  gyrYOut = tmpRec.gyrY;
  gyrZOut = tmpRec.gyrZ;

  return true;
}

bool MPU6050::readRawAccel(int16_t &accXOut, int16_t &accYOut,
                           int16_t &accZOut)
{
  uint8_t rawBuf[6];

  if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, rawBuf, 6))
  {
    return false;
  }

  accXOut = (int16_t)(((uint16_t)rawBuf[0] << 8) | rawBuf[1]);
  accYOut = (int16_t)(((uint16_t)rawBuf[2] << 8) | rawBuf[3]);
  accZOut = (int16_t)(((uint16_t)rawBuf[4] << 8) | rawBuf[5]);

  return true;
}

bool MPU6050::readRawGyro(int16_t &gyrXOut, int16_t &gyrYOut,
                          int16_t &gyrZOut)
{
  uint8_t rawBuf[6];

  if (!readRegisters(MPU6050_REG_GYRO_XOUT_H, rawBuf, 6))
  {
    return false;
  }

  gyrXOut = (int16_t)(((uint16_t)rawBuf[0] << 8) | rawBuf[1]);
  gyrYOut = (int16_t)(((uint16_t)rawBuf[2] << 8) | rawBuf[3]);
  gyrZOut = (int16_t)(((uint16_t)rawBuf[4] << 8) | rawBuf[5]);

  return true;
}

int16_t MPU6050::readRawTemperature(void)
{
  return readRegisterWord(MPU6050_REG_TEMP_OUT_H);
}

/* ========================================================================= */
/* Measurement, engineering units                                            */
/* ========================================================================= */

/*
  Take one coherent sample and convert it into g, degrees per second and
  degrees Celsius. The result is also cached so the individual getters can
  serve it without extra bus traffic.

  When oversampling is enabled the sample is the average of several device
  samples, and when the software high pass filter is enabled its output is
  refreshed from this sample as well.
*/
bool MPU6050::readAll(MPU6050ScaledData &outVal)
{
  MPU6050RawData rawRec;

  if (!readRawOversampled(rawRec))
  {
    return false;
  }

  float accDiv = getAccelScale();
  float gyrDiv = getGyroScale();

  outVal.accX   = (float)rawRec.accX / accDiv;
  outVal.accY   = (float)rawRec.accY / accDiv;
  outVal.accZ   = (float)rawRec.accZ / accDiv;
  outVal.gyrX   = (float)rawRec.gyrX / gyrDiv;
  outVal.gyrY   = (float)rawRec.gyrY / gyrDiv;
  outVal.gyrZ   = (float)rawRec.gyrZ / gyrDiv;
  outVal.tmpDeg = ((float)rawRec.tmpRaw / TMP_SENSITIVITY) + TMP_OFFSET_DEG;

  curVal = outVal;
  applyHighPass();

  return true;
}

/*
  Refresh the internal cache. Call this once per loop, then use the getters
  below as often as needed without paying for another bus transaction.
*/
bool MPU6050::update(void)
{
  MPU6050ScaledData tmpRec;
  return readAll(tmpRec);
}

float MPU6050::getAccelX(void) { return curVal.accX; }
float MPU6050::getAccelY(void) { return curVal.accY; }
float MPU6050::getAccelZ(void) { return curVal.accZ; }
float MPU6050::getGyroX(void)  { return curVal.gyrX; }
float MPU6050::getGyroY(void)  { return curVal.gyrY; }
float MPU6050::getGyroZ(void)  { return curVal.gyrZ; }

float MPU6050::getTemperatureC(void)
{
  return curVal.tmpDeg;
}

float MPU6050::getTemperatureF(void)
{
  return (curVal.tmpDeg * 1.8f) + 32.0f;
}

/*
  Length of the acceleration vector. At rest this sits at one g regardless of
  how the device is oriented, which makes it a quick sanity check on both the
  calibration and the wiring.
*/
float MPU6050::getAccelMagnitude(void)
{
  return sqrt((curVal.accX * curVal.accX) +
               (curVal.accY * curVal.accY) +
               (curVal.accZ * curVal.accZ));
}

/*
  Roll and pitch estimated from gravity alone. These are accurate while the
  device is close to stationary and become meaningless under linear
  acceleration, because the accelerometer cannot tell gravity apart from any
  other acceleration. Fuse with the gyroscope for anything that moves.
*/
float MPU6050::getRollDegrees(void)
{
  return atan2(curVal.accY, curVal.accZ) * RAD_TO_DEGREES;
}

float MPU6050::getPitchDegrees(void)
{
  float denVal = sqrt((curVal.accY * curVal.accY) +
                       (curVal.accZ * curVal.accZ));
  return atan2(-curVal.accX, denVal) * RAD_TO_DEGREES;
}

/* ========================================================================= */
/* Offset trim registers                                                     */
/* ========================================================================= */

/*
  The three accelerometer offset registers hold a value whose least
  significant bit is reserved for the factory temperature compensation trim.
  Overwriting that bit degrades the part's temperature behavior, so every
  write reads the current contents first and preserves bit zero.

  One count of this register equals one part in 2048 of a g, independently of
  the selected full scale range.
*/
void MPU6050::setAccelOffsetX(int16_t offVal)
{
  int16_t oldVal = readRegisterWord(MPU6050_REG_XA_OFFS_H);
  writeRegisterWord(MPU6050_REG_XA_OFFS_H,
                    (int16_t)((offVal & ~0x0001) | (oldVal & 0x0001)));
}

void MPU6050::setAccelOffsetY(int16_t offVal)
{
  int16_t oldVal = readRegisterWord(MPU6050_REG_YA_OFFS_H);
  writeRegisterWord(MPU6050_REG_YA_OFFS_H,
                    (int16_t)((offVal & ~0x0001) | (oldVal & 0x0001)));
}

void MPU6050::setAccelOffsetZ(int16_t offVal)
{
  int16_t oldVal = readRegisterWord(MPU6050_REG_ZA_OFFS_H);
  writeRegisterWord(MPU6050_REG_ZA_OFFS_H,
                    (int16_t)((offVal & ~0x0001) | (oldVal & 0x0001)));
}

/*
  The gyroscope offset registers use all sixteen bits. One count equals one
  part in 32.8 of a degree per second, again independently of the range.
*/
void MPU6050::setGyroOffsetX(int16_t offVal)
{
  writeRegisterWord(MPU6050_REG_XG_OFFS_USRH, offVal);
}

void MPU6050::setGyroOffsetY(int16_t offVal)
{
  writeRegisterWord(MPU6050_REG_YG_OFFS_USRH, offVal);
}

void MPU6050::setGyroOffsetZ(int16_t offVal)
{
  writeRegisterWord(MPU6050_REG_ZG_OFFS_USRH, offVal);
}

int16_t MPU6050::getAccelOffsetX(void)
{
  return readRegisterWord(MPU6050_REG_XA_OFFS_H);
}

int16_t MPU6050::getAccelOffsetY(void)
{
  return readRegisterWord(MPU6050_REG_YA_OFFS_H);
}

int16_t MPU6050::getAccelOffsetZ(void)
{
  return readRegisterWord(MPU6050_REG_ZA_OFFS_H);
}

int16_t MPU6050::getGyroOffsetX(void)
{
  return readRegisterWord(MPU6050_REG_XG_OFFS_USRH);
}

int16_t MPU6050::getGyroOffsetY(void)
{
  return readRegisterWord(MPU6050_REG_YG_OFFS_USRH);
}

int16_t MPU6050::getGyroOffsetZ(void)
{
  return readRegisterWord(MPU6050_REG_ZG_OFFS_USRH);
}

/*
  Load a complete set of offsets in one call. Pair this with getOffsets() and
  a few bytes of EEPROM to skip the calibration wait at every power up.
*/
void MPU6050::setOffsets(const MPU6050Offsets &newOff)
{
  setAccelOffsetX(newOff.accX);
  setAccelOffsetY(newOff.accY);
  setAccelOffsetZ(newOff.accZ);
  setGyroOffsetX(newOff.gyrX);
  setGyroOffsetY(newOff.gyrY);
  setGyroOffsetZ(newOff.gyrZ);
}

void MPU6050::getOffsets(MPU6050Offsets &outOff)
{
  outOff.accX = getAccelOffsetX();
  outOff.accY = getAccelOffsetY();
  outOff.accZ = getAccelOffsetZ();
  outOff.gyrX = getGyroOffsetX();
  outOff.gyrY = getGyroOffsetY();
  outOff.gyrZ = getGyroOffsetZ();
}

/*
  Zero every offset register. Note that this also discards the factory
  accelerometer trim that was programmed at manufacture, so follow it with a
  calibration run rather than leaving the device in this state.
*/
void MPU6050::clearOffsets(void)
{
  MPU6050Offsets zroOff;

  zroOff.accX = 0;
  zroOff.accY = 0;
  zroOff.accZ = 0;
  zroOff.gyrX = 0;
  zroOff.gyrY = 0;
  zroOff.gyrZ = 0;

  setOffsets(zroOff);
  calDne = false;
}

/* ========================================================================= */
/* Automatic calibration                                                     */
/* ========================================================================= */

/*
  Average a block of readings.

  The first hundred samples are thrown away because the digital low pass
  filter needs time to settle after any configuration change, and a settling
  filter biases the mean. Totals accumulate in signed thirty two bit integers,
  which cannot overflow: the worst case is 65535 samples of plus or minus
  32768, well inside the range.

  Results land in avgBuf in the order X, Y and Z acceleration followed by X, Y
  and Z angular rate.
*/
void MPU6050::gatherAverage(uint16_t numSmp, int32_t *avgBuf)
{
  int32_t sumArr[6] = {0, 0, 0, 0, 0, 0};
  uint16_t gotCnt   = 0;
  uint16_t skpCnt   = 0;
  MPU6050RawData rawRec;

  while (gotCnt < numSmp)
  {
    if (!readRawAll(rawRec))
    {
      /* A dropped transaction is not fatal here. Pause and try again so a
         single bus glitch does not abandon a calibration run.              */
      delay(2);
      continue;
    }

    if (skpCnt < 100)
    {
      skpCnt++;
    }
    else
    {
      sumArr[0] += rawRec.accX;
      sumArr[1] += rawRec.accY;
      sumArr[2] += rawRec.accZ;
      sumArr[3] += rawRec.gyrX;
      sumArr[4] += rawRec.gyrY;
      sumArr[5] += rawRec.gyrZ;
      gotCnt++;
    }

    /* One millisecond matches the one kilohertz output rate, so consecutive
       samples are genuinely independent rather than the same latched value
       read several times over.                                             */
    delay(1);
  }

  for (uint8_t idx = 0; idx < 6; idx++)
  {
    avgBuf[idx] = sumArr[idx] / (int32_t)numSmp;
  }
}

/* Calibrate using the compiled in defaults.                                */
bool MPU6050::autoCalibrate(void)
{
  return autoCalibrate(MPU6050_CAL_SAMPLES, MPU6050_CAL_PASSES, NULL);
}

bool MPU6050::autoCalibrate(uint16_t numSmp, uint8_t maxPas)
{
  return autoCalibrate(numSmp, maxPas, NULL);
}

/*
  Automatic calibration.

  Preconditions: the device must be motionless and lying flat with the Z axis
  pointing up, which is the orientation printed on every common breakout
  board. Everything else the routine handles itself.

  Method. At rest the expected readings are zero on both horizontal
  accelerometer axes, exactly one g on the vertical axis and zero on all three
  gyroscope axes. Anything else is bias. The offset registers feed straight
  into the output with a known, fixed gain, so the relationship between an
  offset change and the resulting output change is linear and exactly known:

      one accelerometer offset count moves the output by 1/2048 g
      one gyroscope offset count moves the output by 1/32.8 degrees per second

  Converting the measured error into offset counts therefore gives a
  correction that should land on target in a single step. In practice sensor
  noise, rounding in the offset registers and the reserved temperature
  compensation bit leave a small remainder, so the correction is applied
  repeatedly until every axis sits inside the dead band or the pass budget
  runs out. Convergence normally takes three or four passes.

  The routine temporarily selects the most sensitive range on both sensors,
  because measuring bias at plus or minus sixteen g wastes three bits of
  resolution, and it selects a heavy low pass filter to quiet the average. The
  original configuration is restored before returning, and the resulting
  offsets stay valid at any range since the offset registers are scaled in
  absolute units.

  Pass a Stream, typically &Serial, in logOut to watch the passes converge.

  Returns true when every axis reached its dead band.
*/
bool MPU6050::autoCalibrate(uint16_t numSmp, uint8_t maxPas, Stream *logOut)
{
  /* Preserve whatever the sketch had configured.                           */
  uint8_t savAcc = getAccelRange();
  uint8_t savGyr = getGyroRange();
  uint8_t savDlp = getDlpfMode();
  uint8_t savDiv = getSampleRateDivider();
  float   savReq = smpReq;

  if (numSmp == 0)
  {
    numSmp = MPU6050_CAL_SAMPLES;
  }
  if (maxPas == 0)
  {
    maxPas = MPU6050_CAL_PASSES;
  }

  calDne = false;

  /* Quietest, most sensitive configuration for the measurement.            */
  setAccelRange(MPU6050_ACC_2G);
  setGyroRange(MPU6050_GYR_250DPS);
  setDlpfMode(MPU6050_DLPF_44HZ);
  setSampleRateDivider(0);
  delay(50);

  /* Start from the offsets already programmed, which on a fresh part are
     the factory accelerometer trim values. Keeping them is better than
     zeroing them, because they encode real silicon behavior.               */
  MPU6050Offsets curOff;
  getOffsets(curOff);

  /* Gain from one raw output count to one offset register count.           */
  const float accGai = 2048.0f / getAccelScale();   /* 0.125 at plus/minus 2 g */
  const float gyrGai = 32.8f   / getGyroScale();    /* 0.25 at plus/minus 250  */

  /* Expected vertical reading: exactly one g, expressed in raw counts.     */
  const int32_t accOne = (int32_t)getAccelScale();

  int32_t avgBuf[6] = {0, 0, 0, 0, 0, 0};
  int32_t errArr[6] = {0, 0, 0, 0, 0, 0};
  bool    okFlg = false;

  for (uint8_t pasIdx = 0; pasIdx < maxPas; pasIdx++)
  {
    gatherAverage(numSmp, avgBuf);

    /* Error is measured minus expected, in raw output counts.              */
    errArr[0] = avgBuf[0];
    errArr[1] = avgBuf[1];
    errArr[2] = avgBuf[2] - accOne;
    errArr[3] = avgBuf[3];
    errArr[4] = avgBuf[4];
    errArr[5] = avgBuf[5];

    if (logOut != NULL)
    {
      logOut->print(F("MPU6050 calibration pass "));
      logOut->print(pasIdx + 1);
      logOut->print(F("  residual ax="));
      logOut->print(errArr[0]);
      logOut->print(F(" ay="));
      logOut->print(errArr[1]);
      logOut->print(F(" az="));
      logOut->print(errArr[2]);
      logOut->print(F(" gx="));
      logOut->print(errArr[3]);
      logOut->print(F(" gy="));
      logOut->print(errArr[4]);
      logOut->print(F(" gz="));
      logOut->println(errArr[5]);
    }

    /* Done when every axis is inside its dead band. The bands are widened
       just enough to sit above the noise floor, since chasing the last count
       only makes the loop oscillate.                                       */
    bool inBnd = true;

    for (uint8_t idx = 0; idx < 3; idx++)
    {
      if (abs(errArr[idx]) > MPU6050_CAL_ACC_BAND)
      {
        inBnd = false;
      }
    }
    for (uint8_t idx = 3; idx < 6; idx++)
    {
      if (abs(errArr[idx]) > MPU6050_CAL_GYR_BAND)
      {
        inBnd = false;
      }
    }

    if (inBnd)
    {
      okFlg = true;
      break;
    }

    /* Apply the correction. Raising an offset register raises the reported
       value, so the error is subtracted rather than added. The rounding term
       keeps a half count from being truncated away, which would otherwise
       stall the loop just short of the dead band.                          */
    curOff.accX -= (int16_t)lround((float)errArr[0] * accGai);
    curOff.accY -= (int16_t)lround((float)errArr[1] * accGai);
    curOff.accZ -= (int16_t)lround((float)errArr[2] * accGai);
    curOff.gyrX -= (int16_t)lround((float)errArr[3] * gyrGai);
    curOff.gyrY -= (int16_t)lround((float)errArr[4] * gyrGai);
    curOff.gyrZ -= (int16_t)lround((float)errArr[5] * gyrGai);

    setOffsets(curOff);
    delay(20);
  }

  /* Record what was left over so the sketch can judge the quality of the
     run, then hand the device back exactly as it was found.                */
  resErr.accX = (int16_t)errArr[0];
  resErr.accY = (int16_t)errArr[1];
  resErr.accZ = (int16_t)errArr[2];
  resErr.gyrX = (int16_t)errArr[3];
  resErr.gyrY = (int16_t)errArr[4];
  resErr.gyrZ = (int16_t)errArr[5];

  setAccelRange(savAcc);
  setGyroRange(savGyr);
  setDlpfMode(savDlp);
  setSampleRateDivider(savDiv);
  smpReq = savReq;          /* keep following a rate requested in hertz    */
  delay(20);

  /* The offsets moved, so the old filter history no longer applies.       */
  resetHighPass();

  calDne = okFlg;

  if (logOut != NULL)
  {
    logOut->print(F("MPU6050 calibration "));
    logOut->println(okFlg ? F("converged") : F("did not converge"));
  }

  return okFlg;
}

bool MPU6050::isCalibrated(void) const
{
  return calDne;
}

/*
  Residual error left at the end of the last calibration run, in raw counts.
  Large numbers here usually mean the device was moving or was not level.
*/
void MPU6050::getCalibrationResidual(MPU6050Offsets &outRes) const
{
  outRes = resErr;
}

/* ========================================================================= */
/* Interrupt handling                                                        */
/* ========================================================================= */

/*
  Enable interrupt sources. Build the mask from the MPU6050_INT_ constants.
  Writing zero disables the interrupt pin entirely without changing its
  electrical configuration.
*/
void MPU6050::setInterruptMask(uint8_t intMsk)
{
  writeRegister(MPU6050_REG_INT_ENABLE, intMsk);
}

uint8_t MPU6050::getInterruptMask(void)
{
  return readRegister(MPU6050_REG_INT_ENABLE);
}

/*
  Read and clear the interrupt status. Be aware that reading this register
  clears every latched flag, so capture the returned value once and test all
  the bits of interest against that copy rather than calling again.
*/
uint8_t MPU6050::getInterruptStatus(void)
{
  return readRegister(MPU6050_REG_INT_STATUS);
}

/*
  Poll for a fresh sample. This is the polled alternative to wiring the INT
  pin, and like getInterruptStatus() it clears the latched flags as a side
  effect.
*/
bool MPU6050::isDataReady(void)
{
  return ((getInterruptStatus() & MPU6050_INT_DATA_RDY) != 0);
}

/*
  Electrical and latching behavior of the INT pin.

    actLow  - drive the pin low when asserted instead of high
    opnDrn  - open drain output, needed when several devices share one line
    latEna  - hold the pin asserted until cleared, rather than pulsing it
    clrAny  - clear on any register read, rather than only on INT_STATUS
*/
void MPU6050::setInterruptPinConfig(bool actLow, bool opnDrn,
                                    bool latEna, bool clrAny)
{
  uint8_t cfgVal = readRegister(MPU6050_REG_INT_PIN_CFG);

  cfgVal &= 0x0F;                       /* keep the low nibble untouched    */
  if (actLow) { cfgVal |= 0x80; }
  if (opnDrn) { cfgVal |= 0x40; }
  if (latEna) { cfgVal |= 0x20; }
  if (clrAny) { cfgVal |= 0x10; }

  writeRegister(MPU6050_REG_INT_PIN_CFG, cfgVal);
}

uint8_t MPU6050::getInterruptPinConfig(void)
{
  return readRegister(MPU6050_REG_INT_PIN_CFG);
}

/* ========================================================================= */
/* Motion, zero motion and free fall detection                               */
/* ========================================================================= */

/*
  Motion detection fires when the high pass filtered acceleration on any axis
  exceeds the threshold for longer than the duration. One threshold count is
  two milli g and one duration count is one millisecond.

  Enable MPU6050_INT_MOTION and turn on the high pass filter for this to do
  anything useful.
*/
void MPU6050::setMotionThreshold(uint8_t thrVal)
{
  writeRegister(MPU6050_REG_MOT_THR, thrVal);
}

uint8_t MPU6050::getMotionThreshold(void)
{
  return readRegister(MPU6050_REG_MOT_THR);
}

void MPU6050::setMotionDuration(uint8_t durVal)
{
  writeRegister(MPU6050_REG_MOT_DUR, durVal);
}

uint8_t MPU6050::getMotionDuration(void)
{
  return readRegister(MPU6050_REG_MOT_DUR);
}

/*
  Zero motion is the mirror image: it fires when acceleration stays below the
  threshold for the whole duration. One duration count is sixty four
  milliseconds here, so the detector can span several seconds.
*/
void MPU6050::setZeroMotionThreshold(uint8_t thrVal)
{
  writeRegister(MPU6050_REG_ZRMOT_THR, thrVal);
}

uint8_t MPU6050::getZeroMotionThreshold(void)
{
  return readRegister(MPU6050_REG_ZRMOT_THR);
}

void MPU6050::setZeroMotionDuration(uint8_t durVal)
{
  writeRegister(MPU6050_REG_ZRMOT_DUR, durVal);
}

uint8_t MPU6050::getZeroMotionDuration(void)
{
  return readRegister(MPU6050_REG_ZRMOT_DUR);
}

/*
  Free fall detection requires all three axes to fall below the threshold at
  once, which is what happens when gravity stops being opposed. Threshold
  counts are two milli g and duration counts are one millisecond.
*/
void MPU6050::setFreeFallThreshold(uint8_t thrVal)
{
  writeRegister(MPU6050_REG_FF_THR, thrVal);
}

uint8_t MPU6050::getFreeFallThreshold(void)
{
  return readRegister(MPU6050_REG_FF_THR);
}

void MPU6050::setFreeFallDuration(uint8_t durVal)
{
  writeRegister(MPU6050_REG_FF_DUR, durVal);
}

uint8_t MPU6050::getFreeFallDuration(void)
{
  return readRegister(MPU6050_REG_FF_DUR);
}

/*
  Which axis tripped the motion detector, and in which direction. Bit six and
  five are X positive and negative, four and three are Y, two and one are Z,
  and bit zero reports zero motion.
*/
uint8_t MPU6050::getMotionDetectStatus(void)
{
  return readRegister(MPU6050_REG_MOT_DETECT_STS);
}

/* ========================================================================= */
/* FIFO buffer                                                               */
/* ========================================================================= */

/*
  The FIFO is a 1024 byte buffer that collects samples in the background, so a
  sketch can do other work and drain a batch later instead of servicing every
  sample on time. Enable the buffer, then choose which sensors feed it with
  setFifoSources().
*/
void MPU6050::setFifoEnabled(bool wantFif)
{
  writeRegisterBit(MPU6050_REG_USER_CTRL, 6, wantFif);
}

bool MPU6050::getFifoEnabled(void)
{
  return readRegisterBit(MPU6050_REG_USER_CTRL, 6);
}

/*
  Empty the buffer and clear the overflow flag. Always reset after an overflow
  is seen, because once full the buffer stops accepting data and everything
  read afterwards is stale.

  The register map only honours FIFO_RESET while FIFO_EN is clear, so the
  buffer is disabled for the reset and then put back the way it was. The
  three low bits of USER_CTRL are self clearing reset strobes and are masked
  out so restoring the register cannot fire them again.
*/
void MPU6050::resetFifo(void)
{
  uint8_t ctlVal = readRegister(MPU6050_REG_USER_CTRL);

  if (errCod != 0)
  {
    return;
  }

  ctlVal &= (uint8_t)~0x07;

  writeRegister(MPU6050_REG_USER_CTRL, (uint8_t)(ctlVal & ~0x40));
  writeRegister(MPU6050_REG_USER_CTRL, (uint8_t)((ctlVal & ~0x40) | 0x04));
  writeRegister(MPU6050_REG_USER_CTRL, ctlVal);
}

void MPU6050::setFifoSources(uint8_t srcMsk)
{
  writeRegister(MPU6050_REG_FIFO_EN, srcMsk);
}

uint8_t MPU6050::getFifoSources(void)
{
  return readRegister(MPU6050_REG_FIFO_EN);
}

/*
  Number of bytes waiting, not the number of samples. Divide by the byte count
  of whichever sources are enabled, which is twelve for six axis data or
  fourteen when the temperature sensor is included.
*/
uint16_t MPU6050::getFifoCount(void)
{
  uint8_t rawBuf[2];

  if (!readRegisters(MPU6050_REG_FIFO_COUNTH, rawBuf, 2))
  {
    return 0;
  }

  return (uint16_t)(((uint16_t)rawBuf[0] << 8) | rawBuf[1]);
}

uint8_t MPU6050::readFifoByte(void)
{
  return readRegister(MPU6050_REG_FIFO_R_W);
}

/*
  Drain up to reqLen bytes out of the buffer.

  The transfer is broken into chunks because the Wire library keeps a fixed
  receive buffer, thirty two bytes on most AVR boards, and asking for more
  than that in one call silently truncates the result. Returns the number of
  bytes actually placed in dstBuf.
*/
uint16_t MPU6050::readFifoBytes(uint8_t *dstBuf, uint16_t reqLen)
{
  uint16_t gotLen = 0;

  while (gotLen < reqLen)
  {
    uint16_t remLen = reqLen - gotLen;
    uint8_t  chkLen = (remLen > 28) ? 28 : (uint8_t)remLen;

    if (!readRegisters(MPU6050_REG_FIFO_R_W, dstBuf + gotLen, chkLen))
    {
      break;
    }

    gotLen += chkLen;
  }

  return gotLen;
}

void MPU6050::writeFifoByte(uint8_t datVal)
{
  writeRegister(MPU6050_REG_FIFO_R_W, datVal);
}

/* ========================================================================= */
/* Auxiliary I2C bus                                                         */
/* ========================================================================= */

/*
  Bypass mode connects the auxiliary bus straight through to the main bus, so
  a magnetometer wired to the auxiliary pins becomes visible to the host as an
  ordinary slave. The internal master must be off for bypass to take effect.
*/
void MPU6050::setI2cBypassEnabled(bool wantByp)
{
  if (wantByp)
  {
    setI2cMasterModeEnabled(false);
  }
  writeRegisterBit(MPU6050_REG_INT_PIN_CFG, 1, wantByp);
}

bool MPU6050::getI2cBypassEnabled(void)
{
  return readRegisterBit(MPU6050_REG_INT_PIN_CFG, 1);
}

/*
  Master mode hands the auxiliary bus to the MPU-6050 itself, which then polls
  the attached slaves and stages their data into its own registers.
*/
void MPU6050::setI2cMasterModeEnabled(bool wantMst)
{
  writeRegisterBit(MPU6050_REG_USER_CTRL, 5, wantMst);
}

bool MPU6050::getI2cMasterModeEnabled(void)
{
  return readRegisterBit(MPU6050_REG_USER_CTRL, 5);
}

/* ========================================================================= */
/* Factory self test                                                         */
/* ========================================================================= */

/*
  Average a short burst of samples for the self test. A single sample carries
  enough noise to swing the deviation by several percent, which is a large
  slice of the fourteen percent pass band, so sixteen samples are averaged.
*/
static void selfTestAverage(MPU6050 &imuRef, int16_t *accOut, int16_t *gyrOut)
{
  const uint8_t  SMP_CNT   = 16;
  int32_t        sumArr[6] = {0, 0, 0, 0, 0, 0};
  uint8_t        gotCnt    = 0;
  MPU6050RawData rawRec;

  for (uint8_t idx = 0; idx < SMP_CNT; idx++)
  {
    if (imuRef.readRawAll(rawRec))
    {
      sumArr[0] += rawRec.accX;
      sumArr[1] += rawRec.accY;
      sumArr[2] += rawRec.accZ;
      sumArr[3] += rawRec.gyrX;
      sumArr[4] += rawRec.gyrY;
      sumArr[5] += rawRec.gyrZ;
      gotCnt++;
    }
    delay(2);
  }

  if (gotCnt == 0)
  {
    gotCnt = 1;               /* all reads failed; report zeros, not a trap */
  }

  for (uint8_t idx = 0; idx < 3; idx++)
  {
    accOut[idx] = (int16_t)(sumArr[idx]     / (int32_t)gotCnt);
    gyrOut[idx] = (int16_t)(sumArr[idx + 3] / (int32_t)gotCnt);
  }
}

/*
  Run the built in self test.

  The part contains electrostatic actuators that deflect the proof masses by a
  known amount. The test measures each axis with the actuator off and again
  with it on; the difference is the self test response. Factory trim values
  stored in the SELF_TEST registers say what that response measured at the
  factory, and the ratio between the two is the pass or fail criterion. The
  datasheet allows plus or minus fourteen percent.

  The device is driven at plus or minus eight g and plus or minus two hundred
  and fifty degrees per second during the test, as the datasheet requires, and
  the previous configuration is restored before returning.

  Keep the device still while this runs. The result is written into outRes and
  the function returns the value of the allPass field for convenience.
*/
bool MPU6050::runSelfTest(MPU6050SelfTestResult &outRes)
{
  uint8_t savAcc = readRegister(MPU6050_REG_ACCEL_CONFIG);
  uint8_t savGyr = readRegister(MPU6050_REG_GYRO_CONFIG);

  int16_t offAcc[3];
  int16_t offGyr[3];
  int16_t onnAcc[3];
  int16_t onnGyr[3];

  /* Baseline with the actuators disabled.                                  */
  writeRegister(MPU6050_REG_ACCEL_CONFIG, (uint8_t)(MPU6050_ACC_8G << 3));
  writeRegister(MPU6050_REG_GYRO_CONFIG, (uint8_t)(MPU6050_GYR_250DPS << 3));
  delay(50);
  selfTestAverage(*this, offAcc, offGyr);

  /* Repeat with all six actuators enabled. The upper three bits of each
     configuration register are the per axis self test enables.             */
  writeRegister(MPU6050_REG_ACCEL_CONFIG,
                (uint8_t)(0xE0 | (MPU6050_ACC_8G << 3)));
  writeRegister(MPU6050_REG_GYRO_CONFIG,
                (uint8_t)(0xE0 | (MPU6050_GYR_250DPS << 3)));
  delay(50);
  selfTestAverage(*this, onnAcc, onnGyr);

  /* Restore the caller's configuration before doing any arithmetic, so an
     early return can never leave the part in self test mode.               */
  writeRegister(MPU6050_REG_ACCEL_CONFIG, savAcc);
  writeRegister(MPU6050_REG_GYRO_CONFIG, savGyr);
  delay(50);

  /* Collect the factory trim codes. The accelerometer codes are split: the
     upper three bits live in the per axis registers and the lower two bits
     are packed together in SELF_TEST_A.                                    */
  uint8_t tstBuf[4];

  if (!readRegisters(MPU6050_REG_SELF_TEST_X, tstBuf, 4))
  {
    outRes.allPass = false;
    return false;
  }

  uint8_t codAcc[3];
  uint8_t codGyr[3];

  codAcc[0] = (uint8_t)(((tstBuf[0] >> 3) & 0x1C) | ((tstBuf[3] >> 4) & 0x03));
  codAcc[1] = (uint8_t)(((tstBuf[1] >> 3) & 0x1C) | ((tstBuf[3] >> 2) & 0x03));
  codAcc[2] = (uint8_t)(((tstBuf[2] >> 3) & 0x1C) | (tstBuf[3] & 0x03));
  codGyr[0] = (uint8_t)(tstBuf[0] & 0x1F);
  codGyr[1] = (uint8_t)(tstBuf[1] & 0x1F);
  codGyr[2] = (uint8_t)(tstBuf[2] & 0x1F);

  /* Factory trim equations, datasheet section 4.3. A code of zero means the
     axis was never trimmed, in which case the deviation is undefined and is
     reported as zero rather than as a division by zero.                    */
  float trmAcc[3];
  float trmGyr[3];

  for (uint8_t idx = 0; idx < 3; idx++)
  {
    if (codAcc[idx] == 0)
    {
      trmAcc[idx] = 0.0f;
    }
    else
    {
      trmAcc[idx] = 4096.0f * 0.34f *
                    pow(0.92f / 0.34f, ((float)codAcc[idx] - 1.0f) / 30.0f);
    }

    if (codGyr[idx] == 0)
    {
      trmGyr[idx] = 0.0f;
    }
    else
    {
      trmGyr[idx] = 25.0f * 131.0f * pow(1.046f, (float)codGyr[idx] - 1.0f);
    }
  }

  /* The Y axis gyroscope actuator deflects in the negative direction, so its
     factory trim carries the opposite sign.                                */
  trmGyr[1] = -trmGyr[1];

  float devAcc[3];
  float devGyr[3];

  for (uint8_t idx = 0; idx < 3; idx++)
  {
    float rspAcc = (float)(onnAcc[idx] - offAcc[idx]);
    float rspGyr = (float)(onnGyr[idx] - offGyr[idx]);

    devAcc[idx] = (trmAcc[idx] == 0.0f)
                  ? 0.0f
                  : (100.0f * (rspAcc - trmAcc[idx]) / trmAcc[idx]);
    devGyr[idx] = (trmGyr[idx] == 0.0f)
                  ? 0.0f
                  : (100.0f * (rspGyr - trmGyr[idx]) / trmGyr[idx]);
  }

  outRes.accX = devAcc[0];
  outRes.accY = devAcc[1];
  outRes.accZ = devAcc[2];
  outRes.gyrX = devGyr[0];
  outRes.gyrY = devGyr[1];
  outRes.gyrZ = devGyr[2];

  outRes.allPass = (fabs(devAcc[0]) <= 14.0f) &&
                   (fabs(devAcc[1]) <= 14.0f) &&
                   (fabs(devAcc[2]) <= 14.0f) &&
                   (fabs(devGyr[0]) <= 14.0f) &&
                   (fabs(devGyr[1]) <= 14.0f) &&
                   (fabs(devGyr[2]) <= 14.0f);

  return outRes.allPass;
}

/* ========================================================================= */
/* Low level register access                                                 */
/* ========================================================================= */

/*
  Write one byte to one register. Returns true when the device acknowledged
  every byte of the transaction.
*/
bool MPU6050::writeRegister(uint8_t regAdr, uint8_t datVal)
{
  busPtr->beginTransmission(adrVal);
  busPtr->write(regAdr);
  busPtr->write(datVal);
  errCod = busPtr->endTransmission();

  return (errCod == 0);
}

/*
  Read one byte from one register. Returns zero on a bus failure, which is
  indistinguishable from a genuine zero, so check getLastError() whenever the
  difference matters.
*/
uint8_t MPU6050::readRegister(uint8_t regAdr)
{
  uint8_t datVal = 0;
  readRegisters(regAdr, &datVal, 1);
  return datVal;
}

/*
  Read a block of consecutive registers.

  The repeated start is what makes this work: endTransmission(false) leaves
  the bus held rather than releasing it, so no other master can interleave a
  transaction between setting the register pointer and reading the data.
*/
bool MPU6050::readRegisters(uint8_t regAdr, uint8_t *dstBuf, uint8_t reqLen)
{
  if ((dstBuf == NULL) || (reqLen == 0))
  {
    return false;
  }

  busPtr->beginTransmission(adrVal);
  busPtr->write(regAdr);
  errCod = busPtr->endTransmission(false);

  if (errCod != 0)
  {
    /* Release the bus so a failed read cannot wedge it for other slaves.   */
    busPtr->endTransmission(true);
    return false;
  }

  uint8_t gotLen = busPtr->requestFrom((uint8_t)adrVal, (uint8_t)reqLen);

  if (gotLen != reqLen)
  {
    errCod = 4;             /* Wire defines 4 as an unspecified error       */
    return false;
  }

  for (uint8_t idx = 0; idx < reqLen; idx++)
  {
    dstBuf[idx] = (uint8_t)busPtr->read();
  }

  return true;
}

/*
  Read, modify and write a packed bit field.

    bitPos - index of the most significant bit of the field, 7 down to 0
    bitLen - width of the field in bits
    datVal - right justified value to place in the field

  Bits outside the field keep their current value, which is essential because
  several registers mix unrelated controls into one byte.
*/
bool MPU6050::writeRegisterBits(uint8_t regAdr, uint8_t bitPos,
                                uint8_t bitLen, uint8_t datVal)
{
  uint8_t oldVal = readRegister(regAdr);

  if (errCod != 0)
  {
    return false;
  }

  /* Mask with bitLen ones, shifted so its top bit lands on bitPos.         */
  uint8_t mskVal = (uint8_t)(((1 << bitLen) - 1) << (bitPos - bitLen + 1));
  uint8_t newVal = (uint8_t)(datVal << (bitPos - bitLen + 1));

  newVal &= mskVal;
  oldVal &= (uint8_t)~mskVal;
  oldVal |= newVal;

  return writeRegister(regAdr, oldVal);
}

/* Extract a right justified bit field using the same indexing convention.  */
uint8_t MPU6050::readRegisterBits(uint8_t regAdr, uint8_t bitPos,
                                  uint8_t bitLen)
{
  uint8_t regVal = readRegister(regAdr);
  uint8_t shfCnt = (uint8_t)(bitPos - bitLen + 1);

  return (uint8_t)((regVal >> shfCnt) & ((1 << bitLen) - 1));
}

bool MPU6050::writeRegisterBit(uint8_t regAdr, uint8_t bitPos, bool bitVal)
{
  uint8_t regVal = readRegister(regAdr);

  if (errCod != 0)
  {
    return false;
  }

  if (bitVal)
  {
    regVal |= (uint8_t)(1 << bitPos);
  }
  else
  {
    regVal &= (uint8_t)~(1 << bitPos);
  }

  return writeRegister(regAdr, regVal);
}

bool MPU6050::readRegisterBit(uint8_t regAdr, uint8_t bitPos)
{
  return ((readRegister(regAdr) & (uint8_t)(1 << bitPos)) != 0);
}

/*
  Write a sixteen bit value across two consecutive registers, most significant
  byte first, in a single transaction. Used by the offset registers, where
  splitting the write into two transactions would briefly leave a mismatched
  high and low byte in effect.
*/
bool MPU6050::writeRegisterWord(uint8_t regAdr, int16_t datVal)
{
  busPtr->beginTransmission(adrVal);
  busPtr->write(regAdr);
  busPtr->write((uint8_t)((uint16_t)datVal >> 8));
  busPtr->write((uint8_t)((uint16_t)datVal & 0xFF));
  errCod = busPtr->endTransmission();

  return (errCod == 0);
}

int16_t MPU6050::readRegisterWord(uint8_t regAdr)
{
  uint8_t rawBuf[2];

  if (!readRegisters(regAdr, rawBuf, 2))
  {
    return 0;
  }

  return (int16_t)(((uint16_t)rawBuf[0] << 8) | rawBuf[1]);
}
