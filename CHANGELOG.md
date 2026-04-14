# Changelog

## 1.8.0

### Stability

- **Zone dwell-time debounce** — a zone must read as occupied for 150 ms continuously before registering as SOMEONE, and clear for 80 ms before registering as NOBODY. Single-frame noise spikes cannot advance the FSM.
- **Crossing timing guard** — the full entry/exit sequence must span ≥ 300 ms. Sub-300 ms sequences (door swings, reflections) are discarded.
- **Boot-time threshold validation** — calibration data loaded from flash is rejected if the idle distance is outside 200–4000 mm, `threshold_max ≥ idle`, `threshold_min < 2 %` of idle, or `threshold_max − threshold_min < 100 mm`. Corrupt data triggers an immediate fresh calibration.
- **5-layer threshold clamping** — `min_pct` is clamped to 2–49 % and `max_pct` to 51–95 % at every entry point (flash restore, HA slider, calibrateThreshold, boot validation, runtime API). Prevents the "768 false detections in 21 minutes" failure mode from corrupted flash values.
- **Dual-core race fix** — `expected_counter_` is now set after `call.perform()` completes, eliminating the window where a core-0 `update()` could see a stale counter value and log a phantom "manual adjustment".
- **Debounced calibration triggers** — fail-safe and periodic recalibration only fire when both zones have been continuously clear (debounced state). Calibration cannot interrupt a slow-moving person.

### New features

- **Performance Mode switch** — runtime HA switch that suppresses CPU/RAM/loop-time/distance sensor publishing and heap queries during daily operation. Counting, presence, calibration, and status are unaffected. Flip off for setup or troubleshooting.
- **Entity grouping** — HA device page now organised into three sections:
  - *Main* — Last Direction, Presence (day-to-day)
  - *Configuration* — Calibration group (Status, buttons, thresholds, Save Calibration to Flash), Sensor Tuning group (filter, ROI, ranging), Device group (Restart, Performance Mode)
  - *Diagnostic* — all sensor readouts, loop time, CPU, RAM (collapsed by default)
- **"Save Calibration to Flash" switch** — renamed from "Save Settings to Flash" for clarity. This switch controls only calibration threshold persistence; all other settings (filter mode, ROI, etc.) are always saved via ESPHome preferences.

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
