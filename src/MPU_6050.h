/*
  MPU_6050.h - Arduino library for the InvenSense MPU-6050 six-axis
              (three-axis gyroscope + three-axis accelerometer) MEMS device.

  --------------------------------------------------------------------------
  MIT License

  Copyright (c) 2026 MPU6050 Arduino Library Contributors

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  --------------------------------------------------------------------------

  Revision table
  --------------------------------------------------------------------------
  | Revision | Date        | Author            | Description                |
  |----------|-------------|-------------------|----------------------------|
  | 1.0.0    | 2026-09-24  | Library Authors   | Initial release. Complete  |
  |          |             |                   | register coverage, auto    |
  |          |             |                   | calibration, connection    |
  |          |             |                   | health check, FIFO, motion |
  |          |             |                   | detection and self test.   |
  | 1.1.0    | 2026-11-13  | Library Authors   | Named hardware high pass   |
  |          |             |                   | filter modes, software     |
  |          |             |                   | high pass filter on the    |
  |          |             |                   | acceleration output,       |
  |          |             |                   | oversampling, fractional   |
  |          |             |                   | sample rates, FIFO reset   |
  |          |             |                   | fix, averaged self test.   |
  --------------------------------------------------------------------------

  Notes
  --------------------------------------------------------------------------
  * All measurements are reported in the customary units for this class of
    device: acceleration in g, angular rate in degrees per second, and
    temperature in degrees Celsius (a Fahrenheit helper is also provided).
  * Every identifier in this library uses a minimum of three characters so the
    code stays readable without a symbol lookup.
*/

#ifndef MPU_6050_H
#define MPU_6050_H

#include <Arduino.h>
#include <Wire.h>

/* ------------------------------------------------------------------------ */
/* Seven bit I2C slave addresses                                            */
/* ------------------------------------------------------------------------ */
#define MPU6050_ADR_LOW        0x68  /* AD0 pin tied low  (default)         */
#define MPU6050_ADR_HIGH       0x69  /* AD0 pin tied high                   */

/* ------------------------------------------------------------------------ */
/* Register map                                                             */
/* ------------------------------------------------------------------------ */
#define MPU6050_REG_XG_OFFS_TC     0x00
#define MPU6050_REG_YG_OFFS_TC     0x01
#define MPU6050_REG_ZG_OFFS_TC     0x02
#define MPU6050_REG_X_FINE_GAIN    0x03
#define MPU6050_REG_Y_FINE_GAIN    0x04
#define MPU6050_REG_Z_FINE_GAIN    0x05
#define MPU6050_REG_XA_OFFS_H      0x06
#define MPU6050_REG_YA_OFFS_H      0x08
#define MPU6050_REG_ZA_OFFS_H      0x0A
#define MPU6050_REG_SELF_TEST_X    0x0D
#define MPU6050_REG_SELF_TEST_Y    0x0E
#define MPU6050_REG_SELF_TEST_Z    0x0F
#define MPU6050_REG_SELF_TEST_A    0x10
#define MPU6050_REG_XG_OFFS_USRH   0x13
#define MPU6050_REG_YG_OFFS_USRH   0x15
#define MPU6050_REG_ZG_OFFS_USRH   0x17
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_FF_THR         0x1D
#define MPU6050_REG_FF_DUR         0x1E
#define MPU6050_REG_MOT_THR        0x1F
#define MPU6050_REG_MOT_DUR        0x20
#define MPU6050_REG_ZRMOT_THR      0x21
#define MPU6050_REG_ZRMOT_DUR      0x22
#define MPU6050_REG_FIFO_EN        0x23
#define MPU6050_REG_I2C_MST_CTRL   0x24
#define MPU6050_REG_I2C_SLV0_ADDR  0x25
#define MPU6050_REG_I2C_MST_STATUS 0x36
#define MPU6050_REG_INT_PIN_CFG    0x37
#define MPU6050_REG_INT_ENABLE     0x38
#define MPU6050_REG_INT_STATUS     0x3A
#define MPU6050_REG_ACCEL_XOUT_H   0x3B
#define MPU6050_REG_TEMP_OUT_H     0x41
#define MPU6050_REG_GYRO_XOUT_H    0x43
#define MPU6050_REG_MOT_DETECT_STS 0x61
#define MPU6050_REG_SIG_PATH_RST   0x68
#define MPU6050_REG_MOT_DETECT_CTL 0x69
#define MPU6050_REG_USER_CTRL      0x6A
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_PWR_MGMT_2     0x6C
#define MPU6050_REG_FIFO_COUNTH    0x72
#define MPU6050_REG_FIFO_COUNTL    0x73
#define MPU6050_REG_FIFO_R_W       0x74
#define MPU6050_REG_WHO_AM_I       0x75

/* Value returned by WHO_AM_I on a genuine MPU-6050 device.                  */
#define MPU6050_WHO_AM_I_VAL       0x68

/* ------------------------------------------------------------------------ */
/* Accelerometer full scale range selections                                */
/* ------------------------------------------------------------------------ */
enum MPU6050AccelRange
{
  MPU6050_ACC_2G  = 0x00,  /* +/-  2 g   16384 LSB per g                    */
  MPU6050_ACC_4G  = 0x01,  /* +/-  4 g    8192 LSB per g                    */
  MPU6050_ACC_8G  = 0x02,  /* +/-  8 g    4096 LSB per g                    */
  MPU6050_ACC_16G = 0x03   /* +/- 16 g    2048 LSB per g                    */
};

/* ------------------------------------------------------------------------ */
/* Gyroscope full scale range selections                                    */
/* ------------------------------------------------------------------------ */
enum MPU6050GyroRange
{
  MPU6050_GYR_250DPS  = 0x00,  /* +/-  250 deg/s  131.0 LSB per deg/s       */
  MPU6050_GYR_500DPS  = 0x01,  /* +/-  500 deg/s   65.5 LSB per deg/s       */
  MPU6050_GYR_1000DPS = 0x02,  /* +/- 1000 deg/s   32.8 LSB per deg/s       */
  MPU6050_GYR_2000DPS = 0x03   /* +/- 2000 deg/s   16.4 LSB per deg/s       */
};

/* ------------------------------------------------------------------------ */
/* Digital low pass filter selections (CONFIG register, DLPF_CFG field)     */
/* Bandwidth figures are accelerometer / gyroscope.                         */
/* ------------------------------------------------------------------------ */
enum MPU6050DlpfMode
{
  MPU6050_DLPF_260HZ = 0x00,  /* gyro output rate 8 kHz                     */
  MPU6050_DLPF_184HZ = 0x01,  /* gyro output rate 1 kHz                     */
  MPU6050_DLPF_94HZ  = 0x02,
  MPU6050_DLPF_44HZ  = 0x03,
  MPU6050_DLPF_21HZ  = 0x04,
  MPU6050_DLPF_10HZ  = 0x05,
  MPU6050_DLPF_5HZ   = 0x06
};

/* ------------------------------------------------------------------------ */
/* Hardware accelerometer high pass filter (ACCEL_CONFIG, ACCEL_HPF field)  */
/*                                                                          */
/* IMPORTANT: this filter only feeds the motion, zero motion and free fall  */
/* detectors. It does NOT change the values in the data registers. For a    */
/* high pass filtered acceleration output use setHighPassCutoff(), which    */
/* runs a software filter on every update().                                */
/* ------------------------------------------------------------------------ */
enum MPU6050HpfMode
{
  MPU6050_HPF_RESET  = 0x00,  /* filter off, output settles to zero         */
  MPU6050_HPF_5HZ    = 0x01,  /* cut off frequency 5 Hz                     */
  MPU6050_HPF_2_5HZ  = 0x02,  /* cut off frequency 2.5 Hz                   */
  MPU6050_HPF_1_25HZ = 0x03,  /* cut off frequency 1.25 Hz                  */
  MPU6050_HPF_0_63HZ = 0x04,  /* cut off frequency 0.63 Hz                  */
  MPU6050_HPF_HOLD   = 0x07   /* freeze the filter at the current sample;   */
                              /* later samples are compared to that value   */
};

/* ------------------------------------------------------------------------ */
/* Software oversampling ratios. The MPU-6050 has no hardware oversampling, */
/* so readAll() and update() average this many consecutive device samples,  */
/* spaced one sample period apart. Any value from 1 to 64 is accepted; the  */
/* named ratios are only for readability.                                   */
/* ------------------------------------------------------------------------ */
enum MPU6050Oversampling
{
  MPU6050_OSR_1  = 1,         /* oversampling off (default)                 */
  MPU6050_OSR_2  = 2,
  MPU6050_OSR_4  = 4,
  MPU6050_OSR_8  = 8,
  MPU6050_OSR_16 = 16,
  MPU6050_OSR_32 = 32,
  MPU6050_OSR_64 = 64
};

#define MPU6050_OSR_MAX        64

/* ------------------------------------------------------------------------ */
/* Clock source selections (PWR_MGMT_1 register, CLKSEL field)              */
/* ------------------------------------------------------------------------ */
enum MPU6050ClockSource
{
  MPU6050_CLK_INTERNAL = 0x00,  /* 8 MHz internal oscillator                */
  MPU6050_CLK_PLL_XGYR = 0x01,  /* PLL with X axis gyroscope reference      */
  MPU6050_CLK_PLL_YGYR = 0x02,  /* PLL with Y axis gyroscope reference      */
  MPU6050_CLK_PLL_ZGYR = 0x03,  /* PLL with Z axis gyroscope reference      */
  MPU6050_CLK_EXT_32K  = 0x04,  /* PLL with external 32.768 kHz reference   */
  MPU6050_CLK_EXT_19M  = 0x05,  /* PLL with external 19.2 MHz reference     */
  MPU6050_CLK_STOP     = 0x07   /* Clock disabled, timing generator reset   */
};

/* ------------------------------------------------------------------------ */
/* Accelerometer only wake up frequency used while cycle mode is active     */
/* ------------------------------------------------------------------------ */
enum MPU6050WakeRate
{
  MPU6050_WAK_1_25HZ = 0x00,
  MPU6050_WAK_5HZ    = 0x01,
  MPU6050_WAK_20HZ   = 0x02,
  MPU6050_WAK_40HZ   = 0x03
};

/* ------------------------------------------------------------------------ */
/* Interrupt source bit masks, valid for INT_ENABLE and INT_STATUS          */
/* ------------------------------------------------------------------------ */
#define MPU6050_INT_FREEFALL   0x80
#define MPU6050_INT_MOTION     0x40
#define MPU6050_INT_ZEROMOTION 0x20
#define MPU6050_INT_FIFO_OFLOW 0x10
#define MPU6050_INT_I2C_MST    0x08
#define MPU6050_INT_DATA_RDY   0x01

/* ------------------------------------------------------------------------ */
/* FIFO source bit masks for the FIFO_EN register                           */
/* ------------------------------------------------------------------------ */
#define MPU6050_FIF_TEMP       0x80
#define MPU6050_FIF_GYRO_X     0x40
#define MPU6050_FIF_GYRO_Y     0x20
#define MPU6050_FIF_GYRO_Z     0x10
#define MPU6050_FIF_ACCEL      0x08
#define MPU6050_FIF_SLV2       0x04
#define MPU6050_FIF_SLV1       0x02
#define MPU6050_FIF_SLV0       0x01
#define MPU6050_FIF_ALL        (MPU6050_FIF_TEMP | MPU6050_FIF_GYRO_X | \
                                MPU6050_FIF_GYRO_Y | MPU6050_FIF_GYRO_Z | \
                                MPU6050_FIF_ACCEL)

/* ------------------------------------------------------------------------ */
/* Standby axis bit masks for the PWR_MGMT_2 register                       */
/* ------------------------------------------------------------------------ */
#define MPU6050_STB_ACCEL_X    0x20
#define MPU6050_STB_ACCEL_Y    0x10
#define MPU6050_STB_ACCEL_Z    0x08
#define MPU6050_STB_GYRO_X     0x04
#define MPU6050_STB_GYRO_Y     0x02
#define MPU6050_STB_GYRO_Z     0x01

/* ------------------------------------------------------------------------ */
/* Compile time defaults for the automatic calibration routine              */
/* ------------------------------------------------------------------------ */
#define MPU6050_CAL_SAMPLES    600   /* readings averaged per pass          */
#define MPU6050_CAL_PASSES     12    /* maximum refinement passes           */
#define MPU6050_CAL_ACC_BAND   8     /* accept accel error, raw counts      */
#define MPU6050_CAL_GYR_BAND   2     /* accept gyro error, raw counts       */

/* ------------------------------------------------------------------------ */
/* Plain data containers                                                    */
/* ------------------------------------------------------------------------ */

/* Raw, unscaled sixteen bit two's complement readings straight from the
   device data registers.                                                   */
struct MPU6050RawData
{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t tmpRaw;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
};

/* Engineering unit readings: g, degrees per second and degrees Celsius.    */
struct MPU6050ScaledData
{
  float accX;
  float accY;
  float accZ;
  float tmpDeg;
  float gyrX;
  float gyrY;
  float gyrZ;
};

/* Hardware offset trim values held in the device offset registers.         */
struct MPU6050Offsets
{
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t gyrX;
  int16_t gyrY;
  int16_t gyrZ;
};

/* Percentage deviation from factory trim reported by the built in self
   test. A healthy part stays inside +/- 14 percent on every axis.          */
struct MPU6050SelfTestResult
{
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
  bool  allPass;
};

/* ======================================================================== */
/* Class MPU6050                                                            */
/* ======================================================================== */
class MPU6050
{
  public:

    /* ---- Construction and start up ---------------------------------- */
    MPU6050(uint8_t devAdr = MPU6050_ADR_LOW, TwoWire &busRef = Wire);

    bool    begin(void);
    bool    begin(uint32_t busHz);
    void    end(void);

    /* ---- Health and identity ---------------------------------------- */
    bool    isConnected(void);
    bool    isAlive(void);
    uint8_t whoAmI(void);
    uint8_t getAddress(void) const;
    void    setAddress(uint8_t devAdr);
    uint8_t getLastError(void) const;

    /* ---- Device level control --------------------------------------- */
    void    reset(void);
    void    resetSignalPaths(void);
    void    resetSensorRegisters(void);
    void    sleep(void);
    void    wake(void);
    void    setSleepEnabled(bool wantSlp);
    bool    getSleepEnabled(void);
    void    setCycleEnabled(bool wantCyc);
    bool    getCycleEnabled(void);
    void    setWakeRate(uint8_t rateSel);
    uint8_t getWakeRate(void);
    void    setTemperatureSensorEnabled(bool wantTmp);
    bool    getTemperatureSensorEnabled(void);
    void    setStandbyMask(uint8_t stbMsk);
    uint8_t getStandbyMask(void);
    void    setClockSource(uint8_t clkSel);
    uint8_t getClockSource(void);

    /* ---- Sampling and filtering ------------------------------------- */
    void     setSampleRateDivider(uint8_t divVal);
    uint8_t  getSampleRateDivider(void);
    float    setSampleRateHz(float reqHz);   /* returns the achieved rate   */
    float    getSampleRateHz(void);
    float    getGyroOutputRateHz(void);      /* 8000 or 1000, per the DLPF  */
    void     setDlpfMode(uint8_t dlpSel);
    uint8_t  getDlpfMode(void);
    void     setExternalSyncMode(uint8_t syncSel);
    uint8_t  getExternalSyncMode(void);

    /* ---- Software oversampling --------------------------------------- */
    void     setOversampling(uint8_t osrVal); /* 1 to 64, 1 disables        */
    uint8_t  getOversampling(void) const;

    /* ---- Full scale range ------------------------------------------- */
    void    setAccelRange(uint8_t rngSel);
    uint8_t getAccelRange(void);
    void    setGyroRange(uint8_t rngSel);
    uint8_t getGyroRange(void);
    float   getAccelScale(void) const;   /* LSB counts per g                */
    float   getGyroScale(void) const;    /* LSB counts per deg/s            */

    /* ---- Hardware high pass filter, motion detectors only ------------ */
    void    setAccelHighPassFilter(uint8_t hpfSel);   /* MPU6050HpfMode     */
    uint8_t getAccelHighPassFilter(void);

    /* ---- Software high pass filter on the acceleration output -------- */
    void    setHighPassCutoff(float cutHz);  /* 0 disables                 */
    float   getHighPassCutoff(void) const;
    bool    getHighPassEnabled(void) const;
    void    resetHighPass(void);             /* restart the filter state   */
    float   getAccelHighPassX(void) const;   /* g, gravity and bias removed */
    float   getAccelHighPassY(void) const;
    float   getAccelHighPassZ(void) const;

    /* ---- Measurement, raw -------------------------------------------- */
    bool    readRawAll(MPU6050RawData &outRaw);
    bool    readRawMotion(int16_t &accXOut, int16_t &accYOut, int16_t &accZOut,
                          int16_t &gyrXOut, int16_t &gyrYOut, int16_t &gyrZOut);
    bool    readRawAccel(int16_t &accXOut, int16_t &accYOut, int16_t &accZOut);
    bool    readRawGyro(int16_t &gyrXOut, int16_t &gyrYOut, int16_t &gyrZOut);
    int16_t readRawTemperature(void);

    /* ---- Measurement, engineering units ------------------------------ */
    bool    readAll(MPU6050ScaledData &outVal);
    bool    update(void);                /* refresh the cached readings     */

    float   getAccelX(void);             /* g                               */
    float   getAccelY(void);
    float   getAccelZ(void);
    float   getGyroX(void);              /* degrees per second              */
    float   getGyroY(void);
    float   getGyroZ(void);
    float   getTemperatureC(void);       /* degrees Celsius                 */
    float   getTemperatureF(void);       /* degrees Fahrenheit              */

    /* Derived helpers computed from the most recent cached sample.        */
    float   getAccelMagnitude(void);     /* vector length, g                */
    float   getRollDegrees(void);        /* rotation about the X axis       */
    float   getPitchDegrees(void);       /* rotation about the Y axis       */

    /* ---- Offset trim registers --------------------------------------- */
    void    setAccelOffsetX(int16_t offVal);
    void    setAccelOffsetY(int16_t offVal);
    void    setAccelOffsetZ(int16_t offVal);
    void    setGyroOffsetX(int16_t offVal);
    void    setGyroOffsetY(int16_t offVal);
    void    setGyroOffsetZ(int16_t offVal);
    int16_t getAccelOffsetX(void);
    int16_t getAccelOffsetY(void);
    int16_t getAccelOffsetZ(void);
    int16_t getGyroOffsetX(void);
    int16_t getGyroOffsetY(void);
    int16_t getGyroOffsetZ(void);
    void    setOffsets(const MPU6050Offsets &newOff);
    void    getOffsets(MPU6050Offsets &outOff);
    void    clearOffsets(void);

    /* ---- Automatic calibration --------------------------------------- */
    bool    autoCalibrate(void);
    bool    autoCalibrate(uint16_t numSmp, uint8_t maxPas);
    bool    autoCalibrate(uint16_t numSmp, uint8_t maxPas, Stream *logOut);
    bool    isCalibrated(void) const;
    void    getCalibrationResidual(MPU6050Offsets &outRes) const;

    /* ---- Interrupt handling ------------------------------------------ */
    void    setInterruptMask(uint8_t intMsk);
    uint8_t getInterruptMask(void);
    uint8_t getInterruptStatus(void);
    bool    isDataReady(void);
    void    setInterruptPinConfig(bool actLow, bool opnDrn,
                                  bool latEna, bool clrAny);
    uint8_t getInterruptPinConfig(void);

    /* ---- Motion, zero motion and free fall detection ------------------ */
    void    setMotionThreshold(uint8_t thrVal);      /* 1 LSB = 2 mg        */
    uint8_t getMotionThreshold(void);
    void    setMotionDuration(uint8_t durVal);       /* 1 LSB = 1 ms        */
    uint8_t getMotionDuration(void);
    void    setZeroMotionThreshold(uint8_t thrVal);
    uint8_t getZeroMotionThreshold(void);
    void    setZeroMotionDuration(uint8_t durVal);   /* 1 LSB = 64 ms       */
    uint8_t getZeroMotionDuration(void);
    void    setFreeFallThreshold(uint8_t thrVal);
    uint8_t getFreeFallThreshold(void);
    void    setFreeFallDuration(uint8_t durVal);
    uint8_t getFreeFallDuration(void);
    uint8_t getMotionDetectStatus(void);

    /* ---- FIFO buffer -------------------------------------------------- */
    void     setFifoEnabled(bool wantFif);
    bool     getFifoEnabled(void);
    void     resetFifo(void);
    void     setFifoSources(uint8_t srcMsk);
    uint8_t  getFifoSources(void);
    uint16_t getFifoCount(void);
    uint8_t  readFifoByte(void);
    uint16_t readFifoBytes(uint8_t *dstBuf, uint16_t reqLen);
    void     writeFifoByte(uint8_t datVal);

    /* ---- Auxiliary I2C bus -------------------------------------------- */
    void    setI2cBypassEnabled(bool wantByp);
    bool    getI2cBypassEnabled(void);
    void    setI2cMasterModeEnabled(bool wantMst);
    bool    getI2cMasterModeEnabled(void);

    /* ---- Factory self test --------------------------------------------- */
    bool    runSelfTest(MPU6050SelfTestResult &outRes);

    /* ---- Low level register access -------------------------------------- */
    bool    writeRegister(uint8_t regAdr, uint8_t datVal);
    uint8_t readRegister(uint8_t regAdr);
    bool    readRegisters(uint8_t regAdr, uint8_t *dstBuf, uint8_t reqLen);
    bool    writeRegisterBits(uint8_t regAdr, uint8_t bitPos,
                              uint8_t bitLen, uint8_t datVal);
    uint8_t readRegisterBits(uint8_t regAdr, uint8_t bitPos, uint8_t bitLen);
    bool    writeRegisterBit(uint8_t regAdr, uint8_t bitPos, bool bitVal);
    bool    readRegisterBit(uint8_t regAdr, uint8_t bitPos);
    bool    writeRegisterWord(uint8_t regAdr, int16_t datVal);
    int16_t readRegisterWord(uint8_t regAdr);

  private:

    /* Averages a block of readings with the device held motionless. Used
       exclusively by the automatic calibration routine.                    */
    void    gatherAverage(uint16_t numSmp, int32_t *avgBuf);

    /* Reads osrCnt samples spaced one sample period apart and averages
       them. Falls through to a single readRawAll() when osrCnt is one.     */
    bool    readRawOversampled(MPU6050RawData &outRaw);

    /* Feeds one acceleration sample through the software high pass filter. */
    void    applyHighPass(void);

    /* Sample period in microseconds, from the cached DLPF and divider.     */
    uint32_t samplePeriodMicros(void) const;

    /* Picks the divider for smpReq against the current DLPF setting.       */
    void    applyRequestedRate(void);

    TwoWire         *busPtr;     /* I2C peripheral the device lives on      */
    uint8_t          adrVal;     /* seven bit slave address                 */
    uint8_t          accRng;     /* cached accelerometer range selection    */
    uint8_t          gyrRng;     /* cached gyroscope range selection        */
    uint8_t          dlpCfg;     /* cached DLPF_CFG field                   */
    uint8_t          smpDiv;     /* cached SMPLRT_DIV register              */
    float            smpReq;     /* rate asked for in setSampleRateHz, or 0 */
    uint8_t          osrCnt;     /* software oversampling ratio, 1 to 64    */
    uint8_t          errCod;     /* last Wire transaction status code       */
    bool             calDne;     /* true once auto calibration converged    */
    MPU6050ScaledData curVal;    /* most recent scaled sample               */
    MPU6050Offsets   resErr;     /* residual error left after calibration   */

    float            hpfCut;     /* software high pass cut off, Hz, 0 = off */
    bool             hpfRdy;     /* filter has seen its first sample        */
    uint32_t         hpfTim;     /* micros() stamp of the previous sample   */
    float            hpfInp[3];  /* previous unfiltered acceleration, g     */
    float            hpfOut[3];  /* current high pass filtered output, g    */
};

#endif /* MPU_6050_H */
