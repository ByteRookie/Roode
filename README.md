# RooDe

[![GitHub release](https://img.shields.io/badge/release-1.8.0--Dev-blue?style=flat-square)](https://github.com/ByteRookie/Roode/tree/1.8.0-Dev)
[![Roode community](https://img.shields.io/discord/879407995837087804.svg?label=Discord&logo=Discord&colorB=7289da&style=for-the-badge)](https://discord.gg/hU9SvSXMHs)

A stable, production-ready people counter built on ESPHome and the VL53L1X time-of-flight sensor. Works with any smart home system that supports ESPHome or MQTT (e.g. Home Assistant). All entities are created automatically.

**Branch: `1.8.0-Dev` — repo: [`ByteRookie/Roode`](https://github.com/ByteRookie/Roode)**

---

## Table of Contents

- [Quick Start](#quick-start)
- [Stability Features](#stability-features)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Configuration Reference](#configuration-reference)
- [Sensors](#sensors)
- [Algorithm](#algorithm)
- [Threshold Distance](#threshold-distance)
- [Sampling and Filtering](#sampling-and-filtering)
- [FAQ / Troubleshoot](#faq--troubleshoot)
- [License](#license)

---

## Quick Start

### 1. Add the external component

```yaml
external_components:
  - source: github://ByteRookie/Roode@1.8.0-Dev
    refresh: always
```

### 2. Pull the standard entity package

```yaml
packages:
  roode: github://ByteRookie/Roode/roode_entities.yaml@1.8.0-Dev
```

### 3. Copy and adapt the example below, flash, and you're done

```yaml
# ─── Substitutions ────────────────────────────────────────────────────────────
substitutions:
  name: "office-door-sensor"
  sensor_name: "Office Entree"
  project_name: "J-Sensor.Doorway-Counter"
  project_version: "1.0.0"
  sensor_invert: "false"   # set "true" to flip entry/exit direction
  sensor_min: "15%"        # minimum detection threshold (clamped to 2–49 %)
  sensor_max: "85%"        # maximum detection threshold (clamped to 51–95 %)
  occupancy_delay: "1min"  # how long after the last detection before "vacant"

# ─── ESPHome core ─────────────────────────────────────────────────────────────
esphome:
  name: $name
  project:
    name: $project_name
    version: $project_version

esp32:
  board: wemos_d1_mini32

# ─── External component ───────────────────────────────────────────────────────
external_components:
  - source: github://ByteRookie/Roode@1.8.0-Dev
    refresh: always

# ─── Standard Roode entity package ───────────────────────────────────────────
packages:
  roode: github://ByteRookie/Roode/roode_entities.yaml@1.8.0-Dev

# ─── Connectivity ─────────────────────────────────────────────────────────────
wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
ota:
  - platform: esphome
logger:

# ─── I²C bus ──────────────────────────────────────────────────────────────────
i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true

# ─── VL53L1X sensor ───────────────────────────────────────────────────────────
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

# ─── People-counting algorithm ────────────────────────────────────────────────
roode:
  sampling: 2
  orientation: parallel
  calibration_persistence: true   # thresholds survive reboots
  filter_mode: median
  filter_window: 5

  detection_thresholds:
    min: $sensor_min
    max: $sensor_max

  zones:
    invert: $sensor_invert

# ─── Occupancy (people present = occupied) ────────────────────────────────────
binary_sensor:
  - platform: roode
    presence:
      name: "$sensor_name Presence"
      id: roode_presence

  - platform: template
    name: "$sensor_name Occupancy"
    device_class: occupancy
    lambda: return id(roode_presence).state;
    filters:
      - delayed_off: $occupancy_delay

# ─── People counter (adjustable in HA) ───────────────────────────────────────
number:
  - platform: roode
    people_counter:
      name: "$sensor_name People Count"

# ─── Extra environmental sensors ──────────────────────────────────────────────
sensor:
  - platform: shtcx
    temperature:
      name: "$sensor_name Temperature"
    humidity:
      name: "$sensor_name Humidity"
    address: 0x70
    update_interval: 60s

  - platform: bh1750
    name: "$sensor_name Illuminance"
    address: 0x23
    update_interval: 60s

  - platform: wifi_signal
    name: "$sensor_name WiFi Signal"
    update_interval: 60s

  - platform: uptime
    name: "$sensor_name Uptime"
    id: uptime_sensor
    update_interval: 60s
    internal: true
    on_value:
      then:
        - text_sensor.template.publish:
            id: uptime_human
            state: !lambda |-
              uint32_t s = (uint32_t)x;
              uint32_t d = s / 86400; s %= 86400;
              uint32_t h = s / 3600;  s %= 3600;
              uint32_t m = s / 60;    s %= 60;
              char buf[32];
              snprintf(buf, sizeof(buf), "%ud %02uh %02um %02us", d, h, m, s);
              return std::string(buf);

text_sensor:
  - platform: template
    name: "$sensor_name Uptime (human)"
    id: uptime_human
    icon: mdi:clock-start
```

**Walk through the door once each way.** Entry and Exit counts should each increment by 1. If they appear reversed, set `sensor_invert: "true"` and reflash.

---

## Stability Features

Version `1.8.0-Dev` includes a comprehensive set of hardening fixes targeted at long-term, always-on deployments.

### Zone dwell-time debounce
A zone must measure a presence continuously for **150 ms** before it registers as occupied. It must read clear for **80 ms** before it unregisters. This eliminates single-frame noise spikes from advancing the crossing state machine (FSM) — the primary cause of false counts in noisy environments.

### Crossing sequence timing guard
The full entry/exit crossing sequence (zone 0 active → both active → zone 1 active → both clear) must span **≥ 300 ms** before a count fires. Sequences faster than a real person walking are discarded.

### Threshold validation on boot
Calibration data loaded from flash is rejected if:
- Idle distance is outside 200–4000 mm
- `threshold_max >= idle` (catches the "100% = idle" corruption bug)
- `threshold_min` is below 2% of idle
- `threshold_max - threshold_min < 100 mm`

Invalid data triggers a fresh calibration instead of using corrupt values.

### Percentage clamping (5 layers)
`min_pct` is clamped to **2–49 %** and `max_pct` to **51–95 %** in:
1. `restore_settings_from_flash()` — flash values outside range are ignored
2. `apply_*_threshold_pct()` — HA slider calls are clamped
3. `calibrateThreshold()` — hard floor applied before computing thresholds
4. `setup()` — calibration rejected if percentages produce bad thresholds
5. `set_threshold_percentages()` — runtime API calls are safe

This prevents the "768 false detections in 21 minutes" failure mode where a corrupted flash value (e.g. `max_pct=100%`) made `threshold_max = idle_distance`, causing any sub-idle noise to trigger a crossing.

### Dual-core race fix
`expected_counter_` is now set **after** `call.perform()` completes (not before), eliminating the window where a core-0 update() could see a stale counter value and log a phantom "manual adjustment".

### Debounced calibration triggers
Fail-safe and periodic recalibration only fire when both zones have been continuously clear (debounced) — not just when the raw distance happens to be above the threshold. This prevents calibration from interrupting a slow-moving person.

---

## Hardware

| Part | Recommended | Alternatives |
|------|-------------|--------------|
| MCU | Wemos D1 Mini ESP32 | NodeMCU V2, any ESP32 |
| Sensor | Pololu VL53L1X | GY-53, Pimoroni, Adafruit |
| Power | 1 A USB supply | — |

> Do **not** power from a computer USB port — insufficient current causes sensor brown-outs and false readings.

---

## Wiring

### ESP32

```
              ESP32   VL53L1X
--------------------- -------
                3V3 - VIN
                GND - GND
  SDA (GPIO21)      - SDA
  SCL (GPIO22)      - SCL
  XSHUT (GPIO13)    - XSHUT
  INT   (GPIO18)    - GPIO1
```

### ESP8266

```
              ESP8266   VL53L1X
----------------------- -------
                  3V3 - VIN
                  GND - GND
  D2 (GPIO4)        - SDA
  D1 (GPIO5)        - SCL
```

---

## Configuration Reference

### `vl53l1x:`

| Option | Default | Description |
|--------|---------|-------------|
| `calibration.ranging` | `auto` | Ranging preset: `auto`, `short`, `medium`, `long` |
| `calibration.offset` | — | Distance offset correction in mm |
| `calibration.crosstalk` | — | Crosstalk correction in cps |
| `pins.xshut` | — | GPIO for power-cycling the sensor on hang |
| `pins.interrupt` | — | GPIO for data-ready signal (preferred over polling) |
| `timeout` | `2s` | Max wait time per measurement |

### `roode:`

| Option | Default | Description |
|--------|---------|-------------|
| `sampling` | `2` | Raw readings averaged per distance update |
| `orientation` | `parallel` | `parallel` or `perpendicular` sensor mounting |
| `roi` | `h16 w6` | Region of interest size; `auto` for automatic |
| `detection_thresholds.min` | `15%` | Minimum distance (% of idle or absolute mm) |
| `detection_thresholds.max` | `85%` | Maximum distance (% of idle or absolute mm) |
| `calibration_persistence` | `false` | Save calibration to flash across reboots |
| `filter_mode` | `min` | `min`, `median`, or `percentile10` |
| `filter_window` | `5` | Number of samples in the filter buffer |
| `zones.invert` | `false` | Swap entry/exit zones |
| `force_single_core` | `false` | Disable dual-core tasking |
| `invalid_distance_limit` | `10` | Consecutive bad readings before sensor restart |
| `restart_timeout` | `30s` | Cooldown between sensor restarts |
| `log_fallback_events` | `false` | Log interrupt fallbacks and XSHUT recoveries |

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
      roi: { height: 8, width: 6, center: 124 }
      detection_thresholds:
        min: 10%
        max: 70%
```

---

## Sensors

### People counter (adjustable in HA)

```yaml
number:
  - platform: roode
    people_counter:
      name: People Count
```

### Full sensor block

```yaml
binary_sensor:
  - platform: roode
    presence:
      name: $sensor_name Presence
    zones:
      entry:
        presence:
          name: $sensor_name Entry Occupied
      exit:
        presence:
          name: $sensor_name Exit Occupied

sensor:
  - platform: roode
    distance_entry:
      name: $sensor_name Distance Entry
    distance_exit:
      name: $sensor_name Distance Exit
    max_threshold_entry:
      name: $sensor_name Max Threshold Entry
    max_threshold_exit:
      name: $sensor_name Max Threshold Exit
    min_threshold_entry:
      name: $sensor_name Min Threshold Entry
    min_threshold_exit:
      name: $sensor_name Min Threshold Exit
    roi_height_entry:
      name: $sensor_name ROI Height Entry
    roi_width_entry:
      name: $sensor_name ROI Width Entry
    loop_time:
      name: $sensor_name Loop Time
    cpu_usage:
      name: $sensor_name CPU Usage
    ram_free:
      name: $sensor_name RAM Free
    sensor_status:
      name: $sensor_name Sensor Status
    manual_adjustment_count:
      name: $sensor_name Manual Adjustments

text_sensor:
  - platform: roode
    version:
      name: $sensor_name Version
  - platform: roode
    entry_exit_event:
      name: $sensor_name Last Direction
  - platform: roode
    sensor_status:
      name: $sensor_name Sensor Status Text
  - platform: roode
    enabled_features:
      name: $sensor_name Enabled Features
```

### Sensor Reference

| Name | Type | Description |
|------|------|-------------|
| `people_counter` | number | Adjustable tally of detected people |
| `presence` | binary_sensor | True while a crossing zone is active |
| `zones.entry.presence` | binary_sensor | Entry zone currently occupied |
| `zones.exit.presence` | binary_sensor | Exit zone currently occupied |
| `distance_entry` | sensor | Measured distance in entry zone (mm) |
| `distance_exit` | sensor | Measured distance in exit zone (mm) |
| `max_threshold_entry/exit` | sensor | Upper detection threshold per zone |
| `min_threshold_entry/exit` | sensor | Lower detection threshold per zone |
| `roi_height/width_entry/exit` | sensor | ROI size per zone in SPAD units |
| `loop_time` | sensor | Average sensor loop time |
| `cpu_usage` | sensor | Estimated MCU CPU usage % |
| `ram_free` | sensor | Free heap % |
| `flash_free` | sensor | Free flash % |
| `sensor_status` | sensor | Numeric VL53L1X status code (0 = ok) |
| `manual_adjustment_count` | sensor | Total manual count corrections |
| `version` | text_sensor | Firmware version string |
| `entry_exit_event` | text_sensor | Last entry or exit direction |
| `sensor_status` (text) | text_sensor | `ok` / `timeout` / `reinitializing` / `error` / `offline` |
| `enabled_features` | text_sensor | Active runtime features summary |

---

## Algorithm

The sensor's 16×16 SPAD grid is split into two Regions of Interest (ROI). The PathTrack FSM looks for this sequence to decide direction:

| Step | Zone 0 | Zone 1 | Meaning |
|------|--------|--------|---------|
| 1 | SOMEONE | NOBODY | Person enters from zone 0 side |
| 2 | SOMEONE | SOMEONE | Person spans both zones |
| 3 | NOBODY | SOMEONE | Person has crossed to zone 1 side |
| 4 | NOBODY | NOBODY | Crossing complete → **Entry counted** |

The reverse sequence (`[0,1,3,2]`) counts an **Exit**.

Zone states are debounced (150 ms active / 80 ms clear) and the full sequence must take ≥ 300 ms, so noise and door movements cannot advance the FSM.

### SPAD center table

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

---

## Threshold Distance

On first boot (or after a reset), leave the doorway clear for ~10 seconds. Roode measures the idle distance to the floor/ceiling and sets:

- `threshold_max = idle × max_pct` (default 85%)
- `threshold_min = idle × min_pct` (default 15%)

A crossing registers when the measured distance in a zone is **between** min and max.

### Threshold protection (v1.8.0-Dev)

The firmware now validates thresholds after loading from flash. If `threshold_max ≥ idle` (e.g. from a corrupted `max_pct = 100%`), the stored data is discarded and a fresh calibration runs. Percentages are also clamped: `min_pct` to 2–49 %, `max_pct` to 51–95 %. This prevents the flash-corruption failure mode where any sub-idle noise reading triggers a false count.

---

## Sampling and Filtering

Two independent stages smooth raw sensor output:

1. **`sampling`** — averages N consecutive raw measurements into one distance reading
2. **Filter** (`filter_mode` + `filter_window`) — applies min/median/percentile10 across the last W readings

Total raw readings per reported value = `sampling × filter_window`.

| Mode | Best for | Tradeoff |
|------|----------|----------|
| `min` | Fast response, low-noise areas | Sensitive to spikes |
| `median` | General use | Ignores outliers, slight lag |
| `percentile10` | Noisy/reflective areas | Balanced responsiveness |

**Recommended starting point:** `sampling: 2`, `filter_mode: median`, `filter_window: 5`.
For very noisy areas raise `filter_window` to 7–9. For maximum responsiveness drop to 3.

---

## FAQ / Troubleshoot

**Counter counts backwards.**
Set `sensor_invert: "true"` (or `zones.invert: true`) and reflash.

**False counts in empty room.**
Check boot logs for `Threshold:` lines. `max` should be less than `idle`. If not, clear flash preferences and let the sensor recalibrate. With v1.8.0-Dev, corrupted flash thresholds are now automatically rejected on boot.

**Sensor not measuring correct distances.**
1. Remove the yellow protective film from the sensor lens.
2. Check I²C wiring — SDA/SCL swaps are common.
3. Rule out strong ambient IR light (sunlight, halogen lamps).
4. Try a different power supply.

**Counts stop after hours/days.**
Enable `calibration_persistence: true`. Roode recalibrates automatically when zones are clear, keeping thresholds fresh as temperature drifts. The debounce and sequence timing guards (v1.8.0-Dev) also prevent the FSM from getting stuck.

**"Manual adjustment" logged with no one touching HA.**
Fixed in v1.8.0-Dev — was a dual-core race condition where `expected_counter_` was set before `call.perform()` completed.

---

## License

This project is licensed under the terms of the [Unlicense](LICENSE).
