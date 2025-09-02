# Changelog

## 1.6.6
- Ensure portal server starts reliably on port 8080 and log all web requests
- Drop legacy web server integration and serve API endpoints from the portal server
- Include Arduino `pgmspace.h` directly to fix missing header during compile

## 1.6.5
- Move calibration portal to dedicated web server on port 8080 with HTTP basic auth
- Use `portal_password` token only for securing API endpoints
- Make portal credential settings optional and fix build when web server feature is disabled

## 1.6.4
- Ensure calibration portal registers before the default web page so `/portal` shows the portal and the root URL redirects to it

## 1.6.3
- Always enable web portal; remove portal switch toggle

## 1.6.2
- Add ArduinoJson dependency to resolve missing header during compile

## 1.6.1
- Remove debug logging and cleanup unused code

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
