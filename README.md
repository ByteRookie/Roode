# RooDe

[![GitHub release](https://img.shields.io/badge/release-1.8.0-blue?style=flat-square)](https://github.com/ByteRookie/Roode/tree/master)

A stable, production-ready people counter built on ESPHome and the VL53L1X time-of-flight sensor. Works with any smart home system that supports ESPHome or MQTT (e.g. Home Assistant). All entities are created automatically via the included package.

**Repo:** [`ByteRookie/Roode`](https://github.com/ByteRookie/Roode) · **Branch:** `master`

---

## Table of Contents

- [What is Roode?](#what-is-roode)
- [What's New in 1.8.0](#whats-new-in-180)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Getting Started](#getting-started)
- [Configuration Reference](#configuration-reference)
- [What the Entity Package Provides](#what-the-entity-package-provides)
- [Algorithm](#algorithm)
- [Threshold Distance](#threshold-distance)
- [Sampling and Filtering](#sampling-and-filtering)
- [FAQ / Troubleshoot](#faq--troubleshoot)
- [License](#license)

---

## What is Roode?

Roode counts people passing through a doorway or corridor using a single **VL53L1X time-of-flight (ToF) laser sensor** mounted above the opening. No camera, no PIR, no privacy concerns.

The sensor's 16×16 SPAD grid is split into two detection zones (entry and exit). A state machine watches for the zone-entry sequence: zone A active → both active → zone B active → both clear. That sequence means a person crossed in one direction; the reverse sequence means the other. Roode keeps a running count and publishes it to Home Assistant as a live number entity.

### Key abilities

- **Accurate bidirectional counting** — distinguishes entries from exits using the crossing sequence
- **Fully configurable at runtime** — adjust thresholds, filter settings, ROI, and direction via Home Assistant sliders; no reflash needed
- **Auto-calibration** — measures idle distance on first boot and periodically recalibrates as temperature changes
- **Dual-core support** — offloads sensor polling to ESP32 core 1, keeping the main loop free
- **Performance Mode** — suppress diagnostic sensor publishing during daily operation to reduce overhead; flip it off for setup or troubleshooting
- **Flash persistence** — calibration data and runtime settings survive reboots
- **Presence binary sensor** — true while a crossing is in progress; combine with a `delayed_off` filter for occupancy detection
- **ESPHome + Home Assistant native** — all entities auto-created; no custom integrations needed

---

## What's New in 1.8.0

Version 1.8.0 is a stability-focused release targeting long-term, always-on deployments. See [CHANGELOG.md](CHANGELOG.md) for the full list.

- **Zone dwell-time debounce** — 150 ms to register SOMEONE, 80 ms to register NOBODY; single-frame noise cannot advance the FSM
- **Crossing timing guard** — sequences shorter than 300 ms (door swings, reflections) are discarded
- **Boot-time threshold validation** — corrupt flash values are rejected automatically; 5-layer clamping prevents the "100% threshold = idle distance" false-detection loop
- **Dual-core race fix** — `expected_counter_` is now written after `call.perform()` completes, eliminating phantom "manual adjustment" log entries
- **Debounced calibration triggers** — auto-calibration only fires when both zones have been clear for the full debounce period; cannot interrupt a slow-moving person
- **Performance Mode switch** — suppress CPU/RAM/loop-time/distance sensor publishing at runtime; the counting core keeps running unaffected
- **Entity grouping** — HA device page now has three clear sections: Main (daily use), Configuration (calibration + tuning), Diagnostic (setup/troubleshooting)

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

## Getting Started

### Step 1 — Flash the YAML

Copy the template below, fill in the six substitutions at the top, and flash it to your device.

```yaml
# ── Substitutions — only these lines need editing ─────────────────────────────
substitutions:
  name: "my-door-sensor"     # ESPHome device hostname (no spaces)
  friendly_name: "Doorway"   # Prefix for all HA entity names
  sensor_invert: "false"     # "true" to flip entry/exit direction
  sensor_min: "15%"          # Min detection threshold — clamped to 2–49 %
  sensor_max: "85%"          # Max detection threshold — clamped to 51–95 %
  occupancy_delay: "30s"     # Delay before occupancy clears after last detection

# ── ESPHome core ──────────────────────────────────────────────────────────────
esphome:
  name: $name

esp32:
  board: wemos_d1_mini32

# ── Roode external component ──────────────────────────────────────────────────
# Note: source URLs are resolved before substitutions — this line stays hardcoded.
external_components:
  - source: github://ByteRookie/Roode@master
    refresh: always

# ── Entity package — creates ALL HA entities automatically ────────────────────
# Buttons, sliders, switches, sensors, text sensors — nothing to define manually.
# Uses $friendly_name from substitutions above; roode_id defaults to roode_platform.
packages:
  roode_entities: github://ByteRookie/Roode/roode_entities.yaml@master

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
  id: roode_platform          # must match roode_id default in the entity package
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
# The one entity not included in the package — name it here.
number:
  - platform: roode
    roode_id: roode_platform
    people_counter:
      name: "$friendly_name People Count"

# ── Occupancy binary sensor (clears after no presence for occupancy_delay) ────
binary_sensor:
  - platform: roode
    roode_id: roode_platform
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

### Step 2 — First boot calibration

Leave the doorway **completely clear** for about 10 seconds after the device first connects. Roode samples the idle distance from the sensor to the floor (or far wall) and sets detection thresholds automatically.

You will see `Status: ok` in Home Assistant once calibration is complete.

### Step 3 — Verify

Walk through the doorway in both directions. Entry should count +1 and exit should count −1. Check the **People Count** number entity in HA.

If the directions are reversed, flip the **Invert Direction** switch in HA (Configuration section) — no reflash needed.

### Step 4 — Tune

Once counting looks correct, enable **Performance Mode** (Configuration → Device) for daily use. This suppresses the diagnostic sensor updates and reduces publishing overhead. Flip it back off when you need to inspect Distance Zone or threshold readouts.

For further tuning see [Configuration Reference](#configuration-reference) and [Sampling and Filtering](#sampling-and-filtering).

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
| `performance_mode` | `false` | Boot into Performance Mode — suppresses diagnostic publishing (CPU, RAM, loop time, distance zones, features). Toggle off in HA to re-enable diagnostics. |
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

Including `roode_entities.yaml` automatically creates all of these in Home Assistant. No manual sensor definitions are needed.

The package uses two substitutions you can override in your main YAML:

| Substitution | Default | Purpose |
|--------------|---------|---------|
| `friendly_name` | `Roode` | Prefix for all entity names |
| `roode_id` | `roode_platform` | Must match the `id:` in your `roode:` block |

### Entities by HA section

> **Note:** HA's device page has three fixed section labels: Main, Configuration, and Diagnostic. It is not possible to add custom section names like "Calibration" via ESPHome. Within Configuration, HA sorts entities alphabetically — entity names are prefixed `Cal …` (calibration) and `Ctrl …` (controls/settings) so the two groups naturally sort apart.

| Entity name | Type | HA section | Description |
|-------------|------|-----------|-------------|
| Distance Zone 0 / 1 | sensor | Main | Measured distance per zone (mm) — live readout |
| Last Direction | text_sensor | Main | Most recent entry or exit event |
| Presence | binary_sensor | Main | True while a crossing zone is active |
| **— Calibration (sorted first by "Cal" prefix) —** | | Configuration | |
| Cal Status | text_sensor | Configuration | `ok` / `timeout` / `reinitializing` / `error` / `offline` |
| Cal Empty Room | button | Configuration | Re-measure idle distance |
| Cal With Person | button | Configuration | Person-present calibration |
| Cal Low Obstacle | button | Configuration | Calibrate ignoring low objects (e.g. pets) |
| Cal High Obstacle | button | Configuration | Calibrate for door-open scenario |
| Cal Auto Interval | number | Configuration | Hours between auto-calibrations (0 = off) |
| Cal Save to Flash | switch | Configuration | Persist calibration thresholds across reboots |
| Cal Entry Max / Min | number | Configuration | Upper/lower detection limit, entry zone (live, saved) |
| Cal Exit Max / Min | number | Configuration | Upper/lower detection limit, exit zone (live, saved) |
| **— Controls (sorted after by "Ctrl" prefix) —** | | Configuration | |
| Ctrl Filter Mode | select | Configuration | `min` / `median` / `percentile10` (live, saved) |
| Ctrl Filter Window | number | Configuration | Samples in filter buffer (live, saved) |
| Ctrl Invert Direction | switch | Configuration | Flip entry/exit direction (live, saved) |
| Ctrl Performance Mode | switch | Configuration | Suppress diagnostic publishing to reduce overhead |
| Ctrl Ranging Mode | select | Configuration | `auto` / `short` / `medium` / `long` (live, saved) |
| Ctrl Restart | button | Configuration | Reboot the device |
| Ctrl ROI Entry / Exit Height & Width | number | Configuration | ROI dimensions per zone (live, saved) |
| Ctrl Sampling | number | Configuration | Raw readings averaged per update (live, saved) |
| **— Diagnostic (collapsed by default) —** | | Diagnostic | |
| Max / Min Zone 0 / 1 | sensor | Diagnostic | Active threshold values per zone (mm) |
| ROI Height / Width Zone 0 / 1 | sensor | Diagnostic | Active ROI dimensions |
| Sensor Status Code | sensor | Diagnostic | Numeric VL53L1X status (0 = ok) |
| Manual Adjustments | sensor | Diagnostic | Total manual people-count corrections |
| Loop Time | sensor | Diagnostic | Average sensor loop time (ms) |
| CPU Usage | sensor | Diagnostic | Estimated MCU CPU % |
| RAM Free | sensor | Diagnostic | Heap usage % |
| Version | text_sensor | Diagnostic | Firmware version string |
| Features | text_sensor | Diagnostic | Active runtime features summary |

The **People Counter** number entity is not in the package (it is user-named). Add it yourself:

```yaml
number:
  - platform: roode
    roode_id: roode_platform
    people_counter:
      name: "$friendly_name People Count"
```

### About "Cal Save to Flash"

This switch controls whether **calibration threshold data** (idle distance, min/max thresholds) survives reboots. All other settings (filter mode, ROI, ranging mode, etc.) are always saved to flash automatically via ESPHome preferences.

Turn it **off** if you want to test threshold changes in HA without committing them permanently — they will reset on the next reboot. Turn it **on** (or set `calibration_persistence: true` in your YAML) for normal long-term use.

### About "Ctrl Performance Mode"

When **on**, the following are suppressed to reduce overhead during normal operation:
- CPU Usage, RAM Free, Loop Time sensor publishing
- Distance Zone 0/1 sensor publishing
- Features text sensor publishing
- Manual Adjustments counter publishing

Counting, presence, calibration, and Status continue working normally. Turn it **off** when setting up or troubleshooting to see the diagnostic readouts.

---

## Algorithm

The sensor's 16×16 SPAD grid is split into two Regions of Interest. The PathTrack FSM watches for this crossing sequence:

| Step | Zone 0 | Zone 1 | Meaning |
|------|--------|--------|---------|
| 1 | SOMEONE | NOBODY | Person enters from zone 0 side |
| 2 | SOMEONE | SOMEONE | Person spans both zones |
| 3 | NOBODY | SOMEONE | Person has crossed to zone 1 side |
| 4 | NOBODY | NOBODY | Crossing complete → **Entry +1** |

Reverse sequence → **Exit +1**. Zone states are debounced (150 ms enter / 80 ms clear) and the full sequence must span ≥ 300 ms.

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

With `calibration_persistence: true` these values survive reboots. If they become corrupted (e.g. via an HA slider set to 100 %), the boot-time validator in 1.8.0 automatically discards them and recalibrates.

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
Set `sensor_invert: "true"` (or flip the **Invert Direction** switch in HA → Configuration) and the zones will swap without a reflash.

**False counts in empty room.**
Check boot logs for `Threshold:` lines. `max` must be less than `idle`. If not, press **Calibrate Empty Room** in HA. With 1.8.0, corrupted flash thresholds are automatically rejected on boot.

**Sensor not measuring correct distances.**
1. Remove the yellow protective film from the sensor lens.
2. Check I²C wiring — SDA/SCL swaps are the most common mistake.
3. Rule out strong IR sources (sunlight, halogen lamps).
4. Use a dedicated 1 A USB power supply.

**Counts degrade or stop after hours/days.**
Enable `calibration_persistence: true`. Roode auto-recalibrates when zones are clear, keeping thresholds fresh as temperature changes. The debounce and timing guards in 1.8.0 also prevent the FSM from getting stuck.

**"Manual adjustment" logged with nobody touching HA.**
Fixed in 1.8.0 — was a dual-core race condition where `expected_counter_` was written before the HA number update completed.

**Diagnostic sensors stopped updating.**
Performance Mode is probably on. Go to the device in HA → Configuration → **Ctrl Performance Mode** and turn it off. The diagnostic sensors (CPU, RAM, etc.) resume publishing immediately. Distance Zone sensors are in the Main section and are unaffected by Performance Mode.

---

## License

This project is licensed under the terms of the [Unlicense](LICENSE).
