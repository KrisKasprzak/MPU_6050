# MPU_6050 Arduino Library

[![Compile Examples](https://github.com/YOUR_GITHUB_USERNAME/MPU_6050/actions/workflows/compile-examples.yml/badge.svg)](https://github.com/YOUR_GITHUB_USERNAME/MPU_6050/actions/workflows/compile-examples.yml)
[![Arduino Lint](https://github.com/YOUR_GITHUB_USERNAME/MPU_6050/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/YOUR_GITHUB_USERNAME/MPU_6050/actions/workflows/arduino-lint.yml)

A complete Arduino library for the InvenSense MPU-6050 six-axis MEMS device: a
three-axis accelerometer and a three-axis gyroscope with an on-chip
temperature sensor, on a 400 kHz I2C bus.

## Features

- Full register map coverage, from the sample rate divider through to the
  auxiliary I2C master.
- **Automatic calibration** of the hardware offset registers, with a
  measured convergence report.
- **Connection health check** — `isConnected()` for a full verification and
  `isAlive()` for a one-byte probe cheap enough to poll in a loop.
- Readings in engineering units: g, degrees per second, degrees Celsius and
  degrees Fahrenheit.
- Coherent burst reads, so acceleration, temperature and angular rate in one
  sample always belong to the same instant.
- **Sample rate** in Hz, software **oversampling** (1–64×), and both the
  hardware (motion-detector) and a software (data output) **high-pass filter**.
- FIFO buffer, motion / zero-motion / free-fall detection, interrupt pin
  configuration, low-power cycle mode, and the factory self test.
- Low-level register access is public, so nothing in the datasheet is out of
  reach.

## Installation

Download the repository as a ZIP from GitHub (**Code → Download ZIP**) and use
**Sketch → Include Library → Add .ZIP Library** in the Arduino IDE, or clone it
into your Arduino `libraries` directory:

```sh
cd ~/Documents/Arduino/libraries
git clone https://github.com/YOUR_GITHUB_USERNAME/MPU_6050.git
```

PlatformIO users can add the repository URL to `lib_deps`.

## Wiring

| MPU-6050 | Arduino Uno | ESP32 |
|----------|-------------|-------|
| VCC      | 5 V or 3.3 V (check your breakout board) | 3.3 V |
| GND      | GND         | GND   |
| SCL      | A5          | GPIO 22 |
| SDA      | A4          | GPIO 21 |
| AD0      | GND for address 0x68, VCC for 0x69 | same |
| INT      | D2, optional | any input pin, optional |

## Quick start

```cpp
#include <Wire.h>
#include <MPU_6050.h>

MPU6050 imuDev;

void setup()
{
  Serial.begin(115200);

  if (!imuDev.begin(400000))
  {
    Serial.println("MPU-6050 not found.");
    while (true) { delay(1000); }
  }

  imuDev.setAccelRange(MPU6050_ACC_4G);
  imuDev.setGyroRange(MPU6050_GYR_500DPS);
}

void loop()
{
  if (imuDev.update())
  {
    Serial.println(imuDev.getAccelZ(), 3);
  }
  delay(100);
}
```

## Sample rate, oversampling and filtering

### Sample rate

```cpp
imuDev.setDlpfMode(MPU6050_DLPF_44HZ);
float actHz = imuDev.setSampleRateHz(200);   // returns the rate achieved: 200.0
```

The device produces `gyro rate / (1 + divider)` samples per second. The gyro
rate is 8 kHz with the low-pass filter off (`MPU6050_DLPF_260HZ`) and 1 kHz
otherwise, so the achievable rates are 8000/n or 1000/n for n = 1 to 256.
`setSampleRateHz()` picks the nearest one and returns it. The library
remembers the request, so changing the DLPF later keeps the same output rate
and does not jump it by 8×. `setSampleRateDivider()` is still there if you
want direct register control. Note that the accelerometer itself never
updates faster than 1 kHz.

### Oversampling

```cpp
imuDev.setOversampling(MPU6050_OSR_8);   // any value 1..64
```

The MPU-6050 has no hardware oversampling, so the library does it in
software: every `update()` / `readAll()` averages N device samples, spaced one
sample period apart. Noise drops by √N. The trade-off is that the call blocks
for about N sample periods (8× at 1 kHz ≈ 8 ms). Raw reads, the FIFO and
calibration are not affected.

### High-pass filter

The MPU-6050 has two different high-pass filters, and the difference matters:

| | Hardware (`setAccelHighPassFilter`) | Software (`setHighPassCutoff`) |
|---|---|---|
| Affects | Motion, zero-motion and free-fall detectors **only** | Acceleration output |
| Visible in readings | No | Yes, via `getAccelHighPassX/Y/Z()` |
| Cut-off | 5, 2.5, 1.25, 0.63 Hz, or hold | Any frequency in Hz |

```cpp
imuDev.setAccelHighPassFilter(MPU6050_HPF_5HZ);   // for motion interrupts
imuDev.setHighPassCutoff(0.5f);                   // for your data

imuDev.update();
float vibZ = imuDev.getAccelHighPassZ();   // gravity and slow drift removed
float rawZ = imuDev.getAccelZ();           // still ~1 g at rest
```

The software filter is a first-order high-pass filter that measures the real
time between `update()` calls, so it stays correct at irregular call rates.
Keep the cut-off well below half the rate you call `update()` at.

## Automatic calibration

Lay the board flat and still with the Z axis pointing up, then call:

```cpp
imuDev.autoCalibrate(600, 12, &Serial);
```

The routine measures the bias on every axis, converts it into offset-register
counts using the known offset gains (1/2048 g per accelerometer count and
1/32.8 degrees per second per gyroscope count) and repeats until every axis
sits inside the dead band. It temporarily switches to the most sensitive
ranges for the measurement and restores your configuration before it returns.

Calibration takes a few seconds. To skip the wait on every power-up, read the
offsets back once and hard-code them:

```cpp
MPU6050Offsets finOff;
imuDev.getOffsets(finOff);   // save these, then setOffsets() next boot
```

## Checking the connection

```cpp
if (!imuDev.isConnected())
{
  Serial.print("Sensor lost, Wire error ");
  Serial.println(imuDev.getLastError());
}
```

`isConnected()` confirms the device acknowledges its address **and** returns a
plausible identity byte, so it also catches a sensor that has browned out.
`isAlive()` does the address probe alone when you need the cheapest possible
check.

## Examples

| Sketch | What it shows |
|--------|---------------|
| `BasicReadings` | Scaled acceleration, angular rate, temperature, roll and pitch |
| `AutoCalibration` | Automatic calibration with a live convergence log |
| `ConnectionMonitor` | Detecting a sensor that drops off the bus, and recovering |
| `MotionInterrupt` | Hardware motion detection on the INT pin |
| `SelfTest` | Factory self test with per-axis pass or fail |
| `FifoBuffer` | Streaming six-axis data through the on-chip FIFO |
| `SamplingAndFiltering` | Sample rate, oversampling and the software high-pass filter, for the Serial Plotter |

## Notes and limitations

- The Digital Motion Processor is **not** supported. The DMP requires a
  several-kilobyte firmware image that is not part of this library; use the
  raw sensor data with your own filter instead.
- Roll and pitch from `getRollDegrees()` and `getPitchDegrees()` are derived
  from gravity alone. They are accurate at rest and meaningless under linear
  acceleration — fuse them with the gyroscope for anything that moves.
- `clearOffsets()` also discards the factory accelerometer trim, so follow it
  with a calibration run.

## License

MIT. See [LICENSE](LICENSE); the full text and the revision table are also in
[src/MPU_6050.h](src/MPU_6050.h).
