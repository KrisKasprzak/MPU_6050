# Changelog

All notable changes to this library are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [1.1.0] - 2026-11-13

### Added
- `MPU6050HpfMode` enum with named hardware high-pass filter settings
  (`MPU6050_HPF_RESET`, `_5HZ`, `_2_5HZ`, `_1_25HZ`, `_0_63HZ`, `_HOLD`).
- Software high-pass filter on the acceleration output:
  `setHighPassCutoff()`, `getHighPassCutoff()`, `getHighPassEnabled()`,
  `resetHighPass()`, `getAccelHighPassX/Y/Z()`.
- Software oversampling: `setOversampling()` / `getOversampling()` with the
  `MPU6050Oversampling` enum (1 to 64 samples averaged per `update()`).
- `getGyroOutputRateHz()`.
- `SamplingAndFiltering` example.
- GitHub Actions workflows for compiling the examples and for Arduino Lint.

### Changed
- **Library renamed to `MPU_6050`** to avoid clashing with the existing
  Electronic Cats `MPU6050` library. Sketches must now use
  `#include <MPU_6050.h>`. The class name `MPU6050` and all `MPU6050_*`
  constants are unchanged.
- `setSampleRateHz()` now takes a `float`, rounds to the **nearest**
  achievable rate (it used to truncate) and returns the rate achieved.
- `getSampleRateHz()` now returns a `float`, so rates such as 333.3 Hz and
  3.9 Hz are reported exactly.
- A rate requested in hertz is kept when `setDlpfMode()` switches the gyro
  between 8 kHz and 1 kHz, instead of jumping by a factor of eight.
- `runSelfTest()` averages 16 samples per state instead of using one.
- Sources moved into `src/` (Arduino library specification 1.5 layout).

### Fixed
- `resetFifo()` did not reset the FIFO while it was enabled. The register map
  only honours FIFO_RESET while FIFO_EN is clear; the buffer is now disabled
  for the reset and re-enabled afterwards.
- `getSampleRateHz()` treated DLPF_CFG = 7 as 1 kHz; it runs at 8 kHz.

## [1.0.0] - 2026-09-24

### Added
- Initial release: full register coverage, automatic calibration, connection
  health check, FIFO, motion / zero-motion / free-fall detection, self test.
