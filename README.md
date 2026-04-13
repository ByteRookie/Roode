# RooDe

[![GitHub release](https://img.shields.io/badge/release-1.8.0--Dev-blue?style=flat-square)](https://github.com/ByteRookie/Roode/tree/1.8.0-Dev)
[![Roode community](https://img.shields.io/discord/879407995837087804.svg?label=Discord&logo=Discord&colorB=7289da&style=for-the-badge)](https://discord.gg/hU9SvSXMHs)

A stable, production-ready people counter built on ESPHome and the VL53L1X time-of-flight sensor. Works with any smart home system that supports ESPHome or MQTT (e.g. Home Assistant). All entities are created automatically via the included package.

**Repo:** [`ByteRookie/Roode`](https://github.com/ByteRookie/Roode) · **Branch:** `1.8.0-Dev`

---

## Table of Contents

- [Quick Start](#quick-start)
- [Stability Features](#stability-features)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Configuration Reference](#configuration-reference)
- [What the Entity Package Provides](#what-the-entity-package-provides)
- [Algorithm](#algorithm)
- [Threshold Distance](#threshold-distance)
- [Sampling and Filtering](#sampling-and-filtering)
- [FAQ / Troubleshoot](#faq--troubleshoot)
- [License](#license)

---

## Quick Start

### Complete YAML (copy, fill in the 9 substitutions, flash)

```yaml
# ── Substitutions — only these lines need editing ─────────────────────────────
substitutions:
  name: "my-door-sensor"        # ESPHome device hostname (no spaces)
  friendly_name: "Doorway"      # Prefix for all HA entity names
  project_name: "my-org.door-counter"   # esphome.project name
  project_version: "1.0.0"      # esphome.project version
  roode_id: "roode_platform"    # links roode: block ↔ entity package (keep default)
  sensor_invert: "false"        # "true" to flip entry/exit direction
  sensor_min: "15%"             # Min detection threshold — clamped to 2–49 %
  sensor_max: "85%"             # Max detection threshold — clamped to 51–95 %
  occupancy_delay: "30s"        # Delay before occupancy clears after last detection

# ── ESPHome core ──────────────────────────────────────────────────────────────
esphome:
  name: $name
  project:
    name: $project_name
    version: $project_version

esp32:
  board: wemos_d1_mini32

# ── Roode external component ──────────────────────────────────────────────────
# Note: source URLs are resolved before substitutions, so this stays hardcoded.
external_components:
  - source: github://ByteRookie/Roode@1.8.0-Dev
    refresh: always

# ── Entity package — creates ALL HA entities automatically ────────────────────
# Buttons, sliders, switches, sensors, text sensors — nothing to define manually.
# Uses $friendly_name and $roode_id from substitutions above.
packages:
  roode_entities: github://ByteRookie/Roode/roode_entities.yaml@1.8.0-Dev

# ── Connectivity ──────────────────────────────────────────────────────────────
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
api:
ota:
  - platform: esphome
logger:

# ── I²C ──────────────────────────────────────────────────────────────────────
i2c:
  sda: GPIO21
  scl: GPIO22

# ── VL53L1X sensor ────────────────────────────────────────────────────────────
vl53l1x:
  calibration:
    ranging: auto
  pins:
    xshut:
      number: GPIO13
      mode: OUTPUT_PULLUP
    interrupt:
      number: GPIO18
      mode: INPUT_PULLUP

# ── People counting algorithm ─────────────────────────────────────────────────
roode:
  id: $roode_id
  sampling: 2
  orientation: parallel
  calibration_persistence: true
  filter_mode: median
  filter_window: 5
  detection_thresholds:
    min: $sensor_min
    max: $sensor_max
  zones:
    invert: $sensor_invert

# ── People counter (adjustable number in HA) ──────────────────────────────────
# This is the only entity not included in the package — add it here.
number:
  - platform: roode
    roode_id: $roode_id
    people_counter:
      name: "$friendly_name People Count"

# ── Occupancy binary sensor (clears after no presence for occupancy_delay) ────
binary_sensor:
  - platform: roode
    roode_id: $roode_id
    presence:
      name: "$friendly_name Presence"
      id: roode_presence

  - platform: template
    name: "$friendly_name Occupancy"
    device_class: occupancy
    lambda: return id(roode_presence).state;
    filters:
      - delayed_off: $occupancy_delay
```

> **First boot:** leave the doorway clear for ~10 s while the sensor measures the idle distance. Walk through once each way — entry and exit should each count +1. If reversed, set `sensor_invert: "true"` and reflash.

---

## Stability Features

Version `1.8.0-Dev` includes a comprehensive set of hardening fixes targeted at long-term, always-on deployments.

### Zone dwell-time debounce
A zone must read as occupied continuously for **150 ms** before it registers. It must read clear for **80 ms** before it unregisters. Single-frame noise spikes cannot advance the crossing state machine (FSM).

### Crossing sequence timing guard
The full entry/exit sequence must span **≥ 300 ms** before a count fires. Sub-300 ms sequences (door swings, reflections) are discarded.

### Threshold validation on boot
Calibration data loaded from flash is rejected if any of these are true:
- Idle distance outside 200–4000 mm
- `threshold_max ≥ idle` (catches the "100% = idle" corruption bug)
- `threshold_min < 2 %` of idle
- `threshold_max − threshold_min < 100 mm`

Bad data triggers a fresh calibration instead of using corrupt values.

### Percentage clamping (5 layers)
`min_pct` is clamped to **2–49 %** and `max_pct` to **51–95 %** at every entry point: flash restore, HA slider calls, `calibrateThreshold()`, boot validation, and the runtime API. This prevents the "768 false detections in 21 minutes" failure mode caused by corrupted flash values (e.g. `max_pct=100%` → `threshold_max = idle` → every sub-idle noise reading triggers a count).

### Dual-core race fix
`expected_counter_` is now set **after** `call.perform()` completes, eliminating the window where a core-0 `update()` could see a stale counter value and log a phantom "manual adjustment".

### Debounced calibration triggers
Fail-safe and periodic recalibration only fire when both zones have been continuously clear (debounced state, not raw distance). Calibration cannot interrupt a slow-moving person.

---

## Hardware

| Part | Recommended | Alternatives |
|------|-------------|--------------|
| MCU | Wemos D1 Mini ESP32 | NodeMCU V2, any ESP32 board |
| ToF sensor | Pololu VL53L1X | GY-53, Pimoroni, Adafruit |
| Power supply | 1 A USB adapter | — |

> Do **not** power from a computer USB port — insufficient current causes sensor brown-outs and false readings.

---

## Wiring

### ESP32

```
              ESP32     VL53L1X
─────────────────────   ───────
              3V3     ─  VIN
              GND     ─  GND
  SDA  GPIO21         ─  SDA
  SCL  GPIO22         ─  SCL
  XSHUT GPIO13        ─  XSHUT
  INT   GPIO18        ─  GPIO1 (interrupt)
```

### ESP8266

```
              ESP8266   VL53L1X
─────────────────────   ───────
              3V3     ─  VIN
              GND     ─  GND
  D2  GPIO4           ─  SDA
  D1  GPIO5           ─  SCL
```

---

## Configuration Reference

### `vl53l1x:`

| Option | Default | Description |
|--------|---------|-------------|
| `calibration.ranging` | `auto` | `auto`, `short`, `medium`, `long` |
| `calibration.offset` | — | Distance offset correction (mm) |
| `calibration.crosstalk` | — | Crosstalk correction (cps) |
| `pins.xshut` | — | GPIO for power-cycling on sensor hang |
| `pins.interrupt` | — | GPIO for data-ready signal (preferred) |
| `timeout` | `2s` | Max wait per measurement |

### `roode:`

| Option | Default | Description |
|--------|---------|-------------|
| `id` | `roode_platform` | Must match `roode_id` in the entity package |
| `sampling` | `2` | Raw readings averaged per update |
| `orientation` | `parallel` | `parallel` or `perpendicular` mounting |
| `roi` | `h16 w6` | Region of interest; `auto` for automatic |
| `detection_thresholds.min` | `15%` | Min distance (% of idle or absolute mm) |
| `detection_thresholds.max` | `85%` | Max distance (% of idle or absolute mm) |
| `calibration_persistence` | `false` | Save calibration to flash across reboots |
| `filter_mode` | `min` | `min`, `median`, or `percentile10` |
| `filter_window` | `5` | Samples in the filter buffer |
| `zones.invert` | `false` | Swap entry/exit zones |
| `force_single_core` | `false` | Disable dual-core tasking |
| `invalid_distance_limit` | `10` | Consecutive bad readings before restart |
| `restart_timeout` | `30s` | Cooldown between sensor restarts |
| `log_fallback_events` | `false` | Log INT fallbacks and XSHUT recoveries |

### Per-zone overrides

```yaml
roode:
  zones:
    entry:
      roi: { height: 16, width: 6 }
      detection_thresholds:
        min: 10%
        max: 80%
    exit:
      roi: { height: 8, width: 6 }
      detection_thresholds:
        min: 10%
        max: 70%
```

---

## What the Entity Package Provides

Including `roode_entities.yaml` automatically creates all of these in Home Assistant — no manual sensor definitions needed.

The package uses two substitutions you can override in your main YAML:

| Substitution | Default | Purpose |
|--------------|---------|---------|
| `friendly_name` | `Roode` | Prefix for all entity names |
| `roode_id` | `roode_platform` | Must match the `id:` in your `roode:` block |

### Entities created by the package

| Entity | Type | Description |
|--------|------|-------------|
| Restart | button | Reboot the device |
| Calibrate Empty Room | button | Re-measure idle distance |
| Calibrate With Person | button | Person-present calibration |
| Filter Window | number | Samples in filter buffer (live, saved to flash) |
| Sampling | number | Readings averaged per update (live, saved) |
| Entry / Exit Max Threshold | number | Upper detection limit per zone (live, saved) |
| Entry / Exit Min Threshold | number | Lower detection limit per zone (live, saved) |
| Auto Calibration Interval | number | Minutes between auto-calibrations |
| Entry / Exit ROI Height & Width | number | ROI size per zone (live, saved) |
| Filter Mode | select | `min` / `median` / `percentile10` (live, saved) |
| Ranging Mode | select | `auto` / `short` / `medium` / `long` (live, saved) |
| Invert Direction | switch | Flip entry/exit (live, saved) |
| Save Settings to Flash | switch | Toggle calibration persistence |
| Status | text_sensor | `ok` / `timeout` / `reinitializing` / `error` / `offline` |
| Version | text_sensor | Firmware version string |
| Last Direction | text_sensor | Most recent entry or exit event |
| Features | text_sensor | Active runtime features summary |
| Distance Zone 0 / 1 | sensor | Measured distance per zone (mm) |
| Max / Min Zone 0 / 1 | sensor | Active thresholds per zone |
| ROI Height / Width Zone 0 / 1 | sensor | Active ROI dimensions |
| Sensor Status Code | sensor | Numeric VL53L1X status (0 = ok) |
| Manual Adjustments | sensor | Total manual people-count corrections |
| Loop Time | sensor | Average sensor loop time |
| CPU Usage | sensor | Estimated MCU CPU % |
| RAM Free | sensor | Free heap % |
| Presence | binary_sensor | True while a crossing zone is active |

The **People Counter** number entity is not in the package (it is user-named). Add it yourself:

```yaml
number:
  - platform: roode
    roode_id: roode_platform
    people_counter:
      name: "$friendly_name People Count"
```

---

## Algorithm

The sensor's 16×16 SPAD grid is split into two Regions of Interest. The PathTrack FSM watches for this crossing sequence:

| Step | Zone 0 | Zone 1 | Meaning |
|------|--------|--------|---------|
| 1 | SOMEONE | NOBODY | Person enters from zone 0 side |
| 2 | SOMEONE | SOMEONE | Person spans both zones |
| 3 | NOBODY | SOMEONE | Person has crossed to zone 1 side |
| 4 | NOBODY | NOBODY | Crossing complete → **Entry +1** |

Reverse sequence → **Exit +1**. Zone states are debounced (150 ms / 80 ms) and the full sequence must take ≥ 300 ms.

### SPAD center reference

```
128,136,144,152,160,168,176,184,  192,200,208,216,224,232,240,248
129,137,145,153,161,169,177,185,  193,201,209,217,225,233,241,249
130,138,146,154,162,170,178,186,  194,202,210,218,226,234,242,250
131,139,147,155,163,171,179,187,  195,203,211,219,227,235,243,251
132,140,148,156,164,172,180,188,  196,204,212,220,228,236,244,252
133,141,149,157,165,173,181,189,  197,205,213,221,229,237,245,253
134,142,150,158,166,174,182,190,  198,206,214,222,230,238,246,254
135,143,151,159,167,175,183,191,  199,207,215,223,231,239,247,255

127,119,111,103, 95, 87, 79, 71,   63, 55, 47, 39, 31, 23, 15,  7
126,118,110,102, 94, 86, 78, 70,   62, 54, 46, 38, 30, 22, 14,  6
125,117,109,101, 93, 85, 77, 69,   61, 53, 45, 37, 29, 21, 13,  5
124,116,108,100, 92, 84, 76, 68,   60, 52, 44, 36, 28, 20, 12,  4
123,115,107, 99, 91, 83, 75, 67,   59, 51, 43, 35, 27, 19, 11,  3
122,114,106, 98, 90, 82, 74, 66,   58, 50, 42, 34, 26, 18, 10,  2
121,113,105, 97, 89, 81, 73, 65,   57, 49, 41, 33, 25, 17,  9,  1
120,112,104, 96, 88, 80, 72, 64,   56, 48, 40, 32, 24, 16,  8,  0  ← Pin 1
```

199 is the default center. The lens inverts the image — to shift the FOV toward the upper-left, pick a center SPAD in the lower-right.

---

## Threshold Distance

On first boot (or after a reset) leave the doorway clear for ~10 s. Roode samples the idle distance and sets:

- `threshold_max = idle × max_pct` (default 85 %)
- `threshold_min = idle × min_pct` (default 15 %)

A crossing registers when the measured distance in a zone is **between** min and max.

With `calibration_persistence: true` these values survive reboots. If they become corrupted (e.g. via an HA slider set to 100 %), the boot-time validator in v1.8.0-Dev automatically discards them and recalibrates.

---

## Sampling and Filtering

Two independent stages smooth raw sensor output:

1. **`sampling`** — averages N consecutive raw measurements into one distance value
2. **Filter** (`filter_mode` + `filter_window`) — applies min / median / percentile10 across the last W values

Total raw reads per reported value = `sampling × filter_window`.

| Mode | Best for | Trade-off |
|------|----------|-----------|
| `min` | Fast response, clean areas | Sensitive to spikes |
| `median` | General use | Ignores outliers, slight lag |
| `percentile10` | Noisy/reflective areas | Balanced responsiveness |

**Recommended starting point:** `sampling: 2`, `filter_mode: median`, `filter_window: 5`. Raise `filter_window` to 7–9 for heavy noise; drop to 3 for faster response.

---

## FAQ / Troubleshoot

**Counter counts backwards.**
Set `sensor_invert: "true"` (or flip the `Invert Direction` switch in HA) and the zones will swap without a reflash.

**False counts in empty room.**
Check boot logs for `Threshold:` lines. `max` must be less than `idle`. If not, press **Calibrate Empty Room** in HA, or clear flash preferences and reboot. With v1.8.0-Dev, corrupted flash thresholds are automatically rejected on boot.

**Sensor not measuring correct distances.**
1. Remove the yellow protective film from the sensor lens.
2. Check I²C wiring — SDA/SCL swaps are the most common mistake.
3. Rule out strong IR sources (sunlight, halogen lamps).
4. Use a dedicated 1 A USB power supply.

**Counts degrade or stop after hours/days.**
Enable `calibration_persistence: true`. Roode auto-recalibrates when zones are clear, keeping thresholds fresh as temperature changes. The debounce and timing guards in v1.8.0-Dev also prevent the FSM from getting stuck.

**"Manual adjustment" logged with nobody touching HA.**
Fixed in v1.8.0-Dev — was a dual-core race condition where `expected_counter_` was written before the HA number update completed.

---

## License

This project is licensed under the terms of the [Unlicense](LICENSE).
