# Changelog

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

## 1.6.1
- Automatically create `sensor_name`, `enabled_features` and
  `optional_sensors` text sensors with `entity_category: config`
  to simplify configuration. Default names include the Roode ID to avoid
  duplicates when several sensors are present
- Validation now errors if multiple `platform: roode` text sensor blocks use
  the same id, preventing duplicate entity names
- Fix default text sensor names when the Roode id is omitted
- Apply defaults before validation so unique names are always generated

## 1.5.1
- Add diagnostic sensors reporting loop time, CPU usage, and RAM and flash usage percentages



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
