# Changelog

## 1.8.0
- Critical thread-safety fixes for FreeRTOS dual-core operation
- I2C access protected via mutex on ESP32
- Isolated state publishing and flash saves from sensor task core
- Fixed "Out of Range" reboot loop by strictly checking I2C status
- Fixed "Blind" auto-calibration by filtering invalid readings
- Fixed "Ghost Occupancy" bug in path tracking logic
- Fixed shared memory bug for multi-sensor configurations
- Fixed NTP time-jump spurious recalibration by using millis()
- Built Web Portal API backend for calibration and scanning
- Fixed ESP8266 memory compilation errors
- Added Wire library dependency for ESP32 linking
- Updated YAML configurations for ESPHome 2026.2+

## 1.7.0
- Restart sensor after consecutive invalid distance readings
- Configurable `invalid_distance_limit` and `restart_timeout`

## 1.6.0
- Auto restart sensors via XSHUT with multiplexing support
- Startup pin validation with built‑in pull-ups
- Fail-safe recalibration with calibration data stored in flash
- Feature text sensor and diagnostics for XSHUT/INT pin states
- Manual adjustment counter with detailed event logging
- CPU optimizations using a dual-core task with automatic fallback
- Filtering options with adjustable window and median/percentile modes
- Interrupt mode gracefully falls back to polling and logs the reason
- Colored logs for easier troubleshooting

## 1.5.1
- Add diagnostic sensors reporting loop time, CPU usage, and RAM and flash usage percentages

## 1.5.0
- Manual ROI configuration fixed
- Sensor initialization fixed
- Fix setup priorities to ensure proper boot up
- Code formatting
- Cleanup
