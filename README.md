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
- **Fully configurable at runtime** — adjust thresholds, filter settings, ROI, orientation, and direction via Home Assistant sliders and dropdowns; no reflash needed
- **Rich calibration UI** — four calibration modes available from a single HA dropdown: Empty Room, With Person, Low Obstacle, High Obstacle
- **Auto-calibration** — measures idle distance on first boot and periodically recalibrates as temperature changes
- **Dual-core support** — offloads sensor polling to ESP32 core 1, keeping the main loop free
- **Performance Mode** — suppress diagnostic sensor publishing during daily operation to reduce overhead; flip it off for setup or troubleshooting
- **Flash persistence** — calibration data and runtime settings survive reboots
- **Presence binary sensor** — true while a crossing is in progress; combine with a `delayed_off` filter for occupancy detection
- **ESPHome + Home Assistant native** — all entities auto-created; no custom integrations needed

---

## What's New in 1.8.0

Version 1.8.0 is a major stability and usability release targeting long-term, always-on deployments.

### Detection quality fixes

- **PERCENTILE10 filter fixed** — previously used ceiling division that made it identical to FILTER_MIN at default buffer sizes; now correctly picks the 10th percentile value
- **100 mm hard floor no longer kills detection** — for close-range sensors where the hard floor raised `min` above `max`, the max threshold is now automatically raised and logged; no more silent dead zones
- **Runtime threshold guard** — setting min ≥ max via HA number sliders is now rejected with a log warning; zone never silently stops detecting
- **YAML compile-time validation** — `detection_thresholds` with `min ≥ max` now produces a clear compile error; `cpu_optimization` with `activate ≤ deactivate` also errors at compile time
- **`dump_config()` divide-by-zero fixed** — if calibration failed (idle = 0), boot log no longer crashes with undefined behaviour

### FSM and counting stability

- **Zone dwell-time debounce** — a zone must be continuously active for `zone_dwell_ms` (default 100 ms) to register as SOMEONE, and clear for `zone_clear_ms` (default 80 ms) to register as NOBODY; single-frame noise cannot advance the FSM
- **Crossing timing guard** — sequences shorter than `min_sequence_ms` (default 300 ms) are discarded as noise or door swings
- **FSM timeout reset** — stale partial sequences are cleared on timeout, preventing phantom counts when someone stands in the doorway
- **PathTrack buffer and state moved to member variables** — recalibration and sensor restart now properly reset FSM state; stale partial sequences can no longer combine with post-restart readings
- **Dual-core race fix** — `expected_counter_` is written after `call.perform()` completes, eliminating phantom "manual adjustment" log entries
- **NaN guard in people counter** — if the people counter number entity hasn't been set yet (NaN on fresh flash), first crossing now correctly starts from 0 instead of producing a NaN counter

### Calibration improvements

- **Outlier rejection during calibration** — median ± 25% band discards readings from people walking through mid-calibration; falls back to ± 40% for noisy environments before using raw median
- **Debounced auto-calibration trigger** — auto-cal only fires when both zones have been continuously clear for 10 s; cannot interrupt a slow-moving person
- **Fail-safe calibration uses debounced state** — single noisy frames above threshold cannot falsely trigger fail-safe calibration with a person present
- **NTP backward-jump protection** — auto-calibration interval guard now checks `now_epoch >= last_calibration_ts_` before subtracting; NTP corrections or brief clock jumps cannot fire immediate unscheduled calibration
- **Baseline drift guard** — auto-calibration that would shift the idle baseline by more than `max_baseline_drift_pct` (default 15%) is rejected and logged; prevents environmental changes from destabilising long-running deployments

### Runtime UI and HA entity management

- **Rich calibration UI** — "Calibrate" dropdown in HA with four modes: *Empty Room*, *With Person*, *Low Obstacle*, *High Obstacle*; select the mode then press "Run Calibration" to execute; works for both overhead and side-wall mounting positions
- **All runtime settings persisted** — filter mode, filter window, sampling, ROI, ranging mode, orientation, invert direction, and threshold percentages all saved to flash via ESPHome preferences and restored on reboot
- **Entity grouping** — HA device page now has three clear sections: Sensors (Presence, Last Direction, Status), Configuration (calibration + settings), Diagnostic (setup/troubleshooting)
- **Distance Zone sensors always update** — Distance Zone 0/1 are published on every update cycle regardless of Performance Mode
- **Status heartbeat** — Status text sensor re-publishes every 60 s so HA never marks it Unavailable after a reconnect during a long "ok" period

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

Once counting looks correct, enable **Performance Mode** (HA → Configuration section) for daily use. This suppresses the diagnostic sensor updates and reduces publishing overhead. Flip it back off when you need to inspect Distance Zone or threshold readouts.

### Optional: using the ESP-IDF framework

By default ESPHome builds for ESP32 using the Arduino 3.x framework. Roode also compiles cleanly under the native ESP-IDF framework, which can give slightly better memory usage and avoids the Arduino abstraction layer. To use it, add `framework:` to your `esp32:` block:

```yaml
esp32:
  board: wemos_d1_mini32
  framework:
    type: esp-idf
```

Everything else in the YAML stays the same. If you have no reason to switch, the default Arduino 3.x framework is fine.

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
| `orientation` | `parallel` | Always `parallel` — the Above (parallel) ROI centers work correctly for both overhead and side-wall mounting positions |
| `roi` | `h16 w6` | Region of interest; `auto` for automatic |
| `detection_thresholds.min` | `15%` | Min distance (% of idle or absolute mm) |
| `detection_thresholds.max` | `85%` | Max distance (% of idle or absolute mm) |
| `calibration_persistence` | `true` | Save calibration to flash across reboots |
| `performance_mode` | `false` | Boot into Performance Mode — suppresses diagnostic publishing (CPU, RAM, loop time, features). Toggle off in HA to re-enable diagnostics. Distance Zone sensors always publish. |
| `filter_mode` | `min` | `min`, `median`, or `percentile10` |
| `filter_window` | `5` | Samples in the filter buffer |
| `zones.invert` | `false` | Swap entry/exit zones |
| `force_single_core` | `false` | Disable dual-core tasking |
| `invalid_distance_limit` | `10` | Consecutive bad readings before restart |
| `restart_timeout` | `30s` | Cooldown between sensor restarts |
| `log_fallback_events` | `false` | Log INT fallbacks and XSHUT recoveries |
| `zone_dwell_ms` | `100` | ms a zone must stay active before registering as occupied. Lower catches faster crossings but increases noise sensitivity |
| `zone_clear_ms` | `80` | ms a zone must stay inactive before registering as clear |
| `min_sequence_ms` | `300` | Min ms a full crossing sequence must take to be counted. Rejects rapid noise sequences |
| `obstacle_buffer_mm` | `40` | Safety margin (mm) for low/high obstacle calibration modes |
| `max_baseline_drift_pct` | `15` | Max % the idle baseline may shift during auto-calibration. `0` disables the guard. Manual calibration always applies without limit |

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

HA's device page groups entities into sections based on `entity_category`:

| HA section | What appears there |
|------------|-------------------|
| **Sensors** | Passive entities with no `entity_category` — binary sensors, sensors, text sensors |
| **Configuration** | Any entity with `entity_category: config` |
| **Diagnostic** | Any entity with `entity_category: diagnostic` |

**Visible by default (7 entities):**

| Entity | Type | HA section | Description |
|--------|------|-----------|-------------|
| Presence | binary_sensor | Sensors | True while a crossing zone is active |
| Last Direction | text_sensor | Sensors | Most recent crossing event: `Entry` or `Exit` |
| Status | text_sensor | Sensors | `ok` / `timeout` / `error` / `offline` — re-published every 60 s |
| Calibrate | select | Configuration | Pick a mode: *Empty Room*, *With Person*, *Low Obstacle*, *High Obstacle* |
| Run Calibration | button | — (main card) | Execute the currently selected calibration mode |
| Auto Calibration Interval | number | Configuration | Hours between automatic background recalibrations (0 = off) |
| Performance Mode | switch | Configuration | Suppress diagnostic publishing during normal use |
| Restart | button | Configuration | Reboot the device |

**Hidden by default** (enable in HA → Settings → Devices → tap entity count link):

| Entity | Type | HA section | Description |
|--------|------|-----------|-------------|
| Distance Zone 0 / 1 | sensor | Sensors | Measured distance per zone (mm) — live readout |
| Filter Mode | select | Configuration | `min` / `median` / `percentile10` (live, saved) |
| Ranging Mode | select | Configuration | `auto` / `short` / `medium` / `long` / `longer` / `longest` (live, saved) |
| Entry / Exit Max Threshold | number | Configuration | Upper detection limit per zone as % of idle distance (live, saved) |
| Entry / Exit Min Threshold | number | Configuration | Lower detection limit per zone as % of idle distance (live, saved) |
| Entry / Exit ROI Height & Width | number | Configuration | ROI dimensions per zone (live, saved) |
| Filter Window | number | Configuration | Samples in filter buffer (live, saved) |
| Sampling | number | Configuration | Raw readings averaged per update (live, saved) |
| Invert Direction | switch | Configuration | Flip entry/exit direction (live, saved) |
| Save Calibration to Flash | switch | Configuration | Persist calibration thresholds across reboots |
| Max / Min Zone 0 / 1 | sensor | Diagnostic | Active threshold values per zone (mm) |
| ROI Height / Width Zone 0 / 1 | sensor | Diagnostic | Active ROI dimensions |
| Sensor Status Code | sensor | Diagnostic | Numeric VL53L1X status (0 = ok) |
| Manual Adjustments | sensor | Diagnostic | Total manual people-count corrections |
| Loop Time | sensor | Diagnostic | Average sensor loop time (ms) |
| CPU Usage | sensor | Diagnostic | Estimated MCU CPU % |
| RAM Free | sensor | Diagnostic | Heap usage % |
| Flash Used | sensor | Diagnostic | Partition size as % of total flash |
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

### About "Calibrate" dropdown

Select a mode from the dropdown and calibration runs immediately — no separate button needed. The dropdown stays on your last selection so you can see what was last run.

| Mode | When to use |
|------|------------|
| Empty Room | Standard recalibration — clear the doorway and select this |
| With Person | Stand in the doorway to teach the sensor "occupied" distance |
| Low Obstacle | Calibrate so low objects (pets, boxes) are ignored |
| High Obstacle | Calibrate for doors that swing through the detection zone |

### About mounting position

The sensor works correctly in both mounting positions using the same Above/parallel ROI centers:

| Position | Description |
|----------|-------------|
| **Above** | Mounted on the ceiling or door lintel, looking straight down |
| **Side wall** | Mounted on the door frame, looking across the opening |

Both positions use the same parallel ROI zone split. If entry and exit directions are swapped for your mounting, enable **Invert Direction** in HA or set `zones.invert: true` in your YAML.

### About "Save Calibration to Flash"

This switch controls whether **calibration threshold data** (idle distance, min/max thresholds) survives reboots. All other settings (filter mode, ROI, ranging mode, etc.) are always saved to flash automatically via ESPHome preferences.

Turn it **off** to test threshold changes in HA without committing them permanently — they reset on the next reboot. Turn it **on** (or set `calibration_persistence: true` in your YAML) for normal long-term use.

### About "Performance Mode"

When **on**, the following are suppressed to reduce overhead during normal operation:
- CPU Usage, RAM Free, Loop Time, Features sensor publishing

Distance Zone 0/1, counting, presence, calibration, and Status continue working normally regardless of Performance Mode. Turn it **off** when setting up or troubleshooting to see the full diagnostic readouts.

---

## Algorithm

The sensor's 16×16 SPAD grid is split into two Regions of Interest. The PathTrack FSM watches for this crossing sequence:

| Step | Zone 0 | Zone 1 | Meaning |
|------|--------|--------|---------|
| 1 | SOMEONE | NOBODY | Person enters from zone 0 side |
| 2 | SOMEONE | SOMEONE | Person spans both zones |
| 3 | NOBODY | SOMEONE | Person has crossed to zone 1 side |
| 4 | NOBODY | NOBODY | Crossing complete → **Entry +1** |

Reverse sequence → **Exit +1**. Zone states are debounced (`zone_dwell_ms` enter / `zone_clear_ms` clear) and the full sequence must span ≥ `min_sequence_ms`.

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

With `calibration_persistence: true` (the default) these values survive reboots. If they become corrupted (e.g. via an HA slider set to 100 %), the boot-time validator automatically discards them and recalibrates.

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
Check boot logs for `Threshold:` lines. `max` must be less than `idle`. If not, press **Calibrate → Empty Room** in HA. Corrupted flash thresholds are automatically rejected on boot.

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
Performance Mode is probably on. Go to the device in HA → Configuration → **Performance Mode** and turn it off. CPU, RAM, Loop Time, and Features resume publishing immediately. Distance Zone sensors are always active.

**Entities not appearing in the right HA sections after upgrade.**
After flashing new firmware, remove the device from HA (Settings → Devices → three-dot menu → Delete), restart HA, and wait for the device to reconnect and re-register its entities with the correct categories.

---

## License
