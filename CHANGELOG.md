# Changelog

## 1.5.3
### Added
- Faster queue handling and smaller default filter window for more responsive detection
- Optional diagnostic sensors for loop time, CPU usage, RAM, and flash statistics (30s averages, memory as % free)
- `sensor_invert` option as an alias for `zones.invert`

## 1.5.2
### Added
- Fixed filter mode constants to compile on C++17
- Improved memory cleanup in zones

## 1.5.1
### Added
- Median or minimum filtering with configurable window size
- Optional persistent storage for zone calibration
- Dual core sensor task with queue messaging
- Fail-safe recalibration when zones stay active too long
- State machine with timeout resets to avoid stuck counts
- Memory leak fixes and general cleanup
## 1.5.0

- Manual ROI configuration fixed
- Sensor initialization fixed
- Fix setup priorities to ensure proper boot up
- Code formatting
- Cleanup

## 1.4.1

- Timing budget test by @Lyr3x in #60
- Restructure configuration by @Lyr3x in #61
- v1.4.0 by @Lyr3x in #55
- Improve roi calibration by @Lyr3x in #64
- Fix presence sensor and wdt crashes by @Lyr3x in #67
- Improve sensor creation, initialization and measurement reads by @Lyr3x in #68
- Use sampling always by @Lyr3x in #71
- Improve error log and fix manual mode by @Lyr3x in #73
- Configure IDE intellisense by @CarsonF in #74
- Fix error handling by @Lyr3x in #75
- Fix manual roi setting by @Lyr3x in #78
