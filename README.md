# RooDe

[![GitHub release](https://img.shields.io/badge/release-1.7.0-blue?style=flat-square)](https://github.com/Lyr3x/Roode/releases/tag/1.7.0)
[![Build](https://img.shields.io/github/actions/workflow/status/Lyr3x/Roode/ci.yml?style=flat-square)](https://github.com/Lyr3x/Roode/blob/master/.github/workflows/ci.yml)

[![Roode community](https://img.shields.io/discord/879407995837087804.svg?label=Discord&logo=Discord&colorB=7289da&style=for-the-badge)](https://discord.gg/hU9SvSXMHs)

A people counter that works with any smart home system that supports ESPHome/MQTT (e.g., Home Assistant). All necessary entities are created automatically.


## Table of Contents

- [Quick Start](#quick-start)
- [Hardware Recommendations](#hardware-recommendations)
- [Wiring](#wiring)
  - [ESP32](#esp32)
  - [ESP8266](#esp8266)
- [Configuration](#configuration)
  - [Platform Setup](#platform-setup)
  - [Interrupt vs Polling](#interrupt-vs-polling)
  - [Single vs Dual Core](#single-vs-dual-core)
  - [Sampling and Filtering](#sampling-and-filtering)
- [Configuration Reference](#configuration-reference)
  - [Example Configurations](#example-configurations)
  - [Sensors](#sensors)
- [Threshold distance](#threshold-distance)
- [Algorithm](#algorithm)
- [Features](#features)
- [Logging and Diagnostics](#logging-and-diagnostics)
- [FAQ/Troubleshoot](#faqtroubleshoot)
- [License](#license)

## Quick Start

1. Install [ESPHome](https://esphome.io/) on your computer or Home Assistant.
2. Copy one of the example YAML files to your ESPHome configuration folder (no repository clone required):
   - [peopleCounter32.yaml](peopleCounter32.yaml)
   - [peopleCounter8266.yaml](peopleCounter8266.yaml)
3. Flash it with `esphome run peopleCounter32.yaml` (replace with your file).

## Hardware Recommendations

- ESP8266 or ESP32
  - **Wemos D1 Mini ESP32** <-- Recommended
  - Wemos D1 mini (ESP8266)
  - NodeMCU V2
- 1x VL53L1X
  - **Pololu** <-- Recommended
  - GY-53
  - Black PCB chinese sensor
  - Pimoroni
- 1A Power Supply **Do not use an USB port of your computer!**
- Enclosure (see models in [STL/](STL))
  Pins:
  SDA_PIN 4 (ESP8266) or 21 (ESP32)
  SCL_PIN 5 (ESP8266) or 22 (ESP32)

For setup details with Home Assistant, see the [Home Assistant integration](home_assistant.md).

## Wiring

The sensors from Pololu, Adafruit and the GY-53 can also be connected to the 5v pin (VIN) as they have a voltage regulator.

If you use a GY-53, connect the PS pin to GND (Ps=0).

Ps=1 (default): Serial port UART mode, Pin3 is TX, Pin4 is RX, TTL level, PWM output works.
Ps=0 (when connected to GND): In the IIC mode, the user can operate the chip by himself. The module owns the MCU and does not operate the chip. The PWM output does not work.

### ESP32

```
                    ESP32   VL53L1X board
-------------------------   -------------
                      3V3 - VIN
                      GND - GND
     SDA (pin 42, GPIO21) - SDA
     SCL (pin 39, GPIO22) - SCL
```

### ESP8266

```
                  ESP8266   VL53L1X board
-------------------------   -------------
                      3V3 - VIN
                      GND - GND
              D2 (GPIO 4) - SDA
              D1 (GPIO 5) - SCL
```

## Configuration

### Platform Setup

Roode is provided as an external_component which means it is easy to set up in any ESPHome sensor configuration file.

Other than base ESPHome configuration the only config that's needed for Roode is

```yaml
external_components:
  - source: github://Lyr3x/Roode@master
    refresh: always
vl53l1x:
roode:
```

This uses the recommended default configuration.

However, we offer a lot of flexibility. Here's the full configuration spelled out.

```yml
external_components:
  - source: github://Lyr3x/Roode
    refresh: always
    ref: master

# VL53L1X sensor configuration is separate from Roode people counting algorithm
vl53l1x:
  # ID for this sensor when using multiple VL53L1X modules on the same bus
  sensor_id: 1
  # A non-standard I2C address
  address:
  # How long to wait for boot and measurements before giving up
  timeout: 2s

  # Sensor calibration options
  calibration:
    # The ranging mode is different based on how long the distance is that the sensor need to measure.
    # The longer the distance, the more time the sensor needs to take a measurement.
    # Available options are: auto, shortest, short, medium, long, longer, longest
    ranging: auto
    # The offset correction distance. Run [calibration/OffsetAndXtalkCalibration](calibration/OffsetAndXtalkCalibration)
    # with a 17% grey target 140 mm away and copy the reported value.
    offset: 8mm
    # The corrected photon count in counts per second. Use the same sketch in a
    # dark room to measure crosstalk and copy the result.
    crosstalk: 53406cps

  # Hardware pins
  pins:
    # Shutdown/Enable pin used to change the I2C address and recover the sensor if needed.
    xshut:
      number: GPIO3
      mode: OUTPUT_PULLUP
      ignore_strapping_warning: true
    # Interrupt pin with internal pull-up for the data ready signal
    interrupt:
      number: GPIO1
      mode: INPUT_PULLUP

  # When an xshut pin is provided the library will power cycle the sensor
  # automatically if a measurement times out.
  # On boot the driver checks that the xshut and interrupt pins work and
  # prints the result to the log.

# Roode people counting algorithm
roode:
  # Smooth out measurements by using the minimum distance from this number of readings
  # Increase to 4-5 if jitter is a problem; 1 is fastest but noisier
  sampling: 2

  # The orientation of the two sensor pads in relation to the entryway being tracked.
  # The advised orientation is parallel, but if needed this can be changed to perpendicular.
  orientation: parallel

  # This controls the Region of Interest. Adjust width/height a few steps at a time
  # when the doorway is unusually narrow or wide. The current default is
  roi: { height: 16, width: 6 }
  # We have an experimental automatic mode that can be enabled with
  # roi: auto
  # or only automatic for one dimension
  # roi: { height: 16, width: auto }

  # The detection thresholds for determining whether a measurement should count as a person crossing.
  # A reading must be greater than the minimum and less than the maximum to count as a crossing.
  # These can be given as absolute distances or as percentages.
  # Percentages are based on the automatically determined idle or resting distance.
  detection_thresholds:
    min: 0%  # default minimum is any distance
    # raise by ~5% or 50mm steps if door movements cause counts
    max: 85% # default maximum is 85%
    # an example of absolute units
    # min: 50mm
    # max: 234cm

  # Persist calibration data so thresholds survive restarts
  calibration_persistence: true

  # Jitter reduction options
  filter_mode: median  # min, median or percentile10
  # Increase the window to 7 or 9 for heavy noise, drop to 3 for faster response
  filter_window: 5     # number of samples used by the filter
  # Log interrupt fallback events and XSHUT recoveries
  log_fallback_events: true
  # Disable dual core tasking if needed
  force_single_core: false
  # Event logs show xshut power cycles, interrupt fallbacks and manual adjustments

  # The people counting algorithm works by splitting the sensor's capability reading area into two zones.
  # This allows for detecting whether a crossing is an entry or exit based on which zone was crossed first.
  zones:
    # Flip the entry/exit zones. If Roode seems to be counting backwards, set this to true.
    invert: false

    # Entry/Exit zones can set overrides for individual ROI & detection thresholds here.
    # If omitted, they use the options configured above.
    entry:
      # Entry zone will automatically configure ROI, regardless of ROI above.
      roi: auto
    exit:
      roi:
        # Exit zone height starts at 8. Change by 1-2 if objects are closer on this side
        height: 8
        # Additionally, zones can manually set their center point.
        # Usually though, this is left for Roode to automatically determine.
        center: 124

      detection_thresholds:
        # Exit zone's min detection threshold will be 5% of idle/resting distance, regardless of setting above.
        min: 5%
        # Exit zone's max detection threshold will be 70% of idle/resting distance, regardless of setting above.
        # Adjust these in 5% steps if one side sees false counts
        max: 70%
```

The `entry` and `exit` blocks allow tuning each zone when they behave differently.
For example, an entryway with a shelf on one side might need a smaller ROI or
stricter thresholds only in that zone. Start with small adjustments—change the
ROI height or width by one or two units or nudge thresholds 5 % at a time—and
test before making larger changes.
### Interrupt vs Polling

Roode prefers the interrupt pin for efficient updates. When `interrupt` is defined and validated, the VL53L1X notifies the MCU whenever a new sample is ready. If the INT pin is missing or stops working, Roode falls back to a 10 ms polling loop and tries interrupts again every 30 minutes. Polling also acts as a safety net during startup.

| Situation | Use INT | Use Polling |
| --- | --- | --- |
| Normal operation | ✅ | 🔁 (optional verify) |
| INT not received in time | ⛔️ | ✅ |
| Sensor just booted | ⛔️ | ✅ |
| Interrupt unreliable | ⛔️ | ✅ |
| Low-power mode handling | ⛔️ | ✅ |


### Single vs Dual Core

On ESP32 targets Roode tries to run the sensor loop on the second CPU core so
Wi‑Fi and other ESPHome tasks stay responsive.  If the task fails to start or
when running on an ESP8266 the code automatically falls back to a single‑core
loop.  You can force single‑core mode with `force_single_core: true`.

### Sampling and Filtering

Roode smooths distance readings in two stages. The driver first averages
multiple raw measurements using the `sampling` option. Each zone then applies a
filter across the last few averaged values controlled by `filter_mode` and
`filter_window`.

Raising `sampling` makes each reading steadier while `filter_window` dictates
how many of those readings must agree before an event fires. Because the filter
operates on averaged data, the total number of raw readings considered is
`sampling` multiplied by `filter_window`. This gives better noise rejection at
the cost of reaction speed. Start with `sampling: 2` and `filter_window: 3` and
increase them together if your environment is unstable. See the table below for
how the available filter modes behave.

| Mode | When to use | Pros | Cons |
| --- | --- | --- | --- |
| `min` | Very clean environments or quick response needed | Reacts instantly to changes | Sensitive to noise and outliers |
| `median` | General use when noise is moderate | Ignores spikes for stable readings | Can lag behind fast motion |
| `percentile10` | Noisy locations where some jitter must be ignored | Balances responsiveness and noise rejection | Slightly less stable than median |

#### `sampling`

*Averages consecutive raw measurements before filtering.* Increase above `2` only when noise causes flickering.

**Recommended values** moved to the Quick Tips section below.

#### `filter_window`

*Number of past measurements considered by the filter.* `3` is responsive, while `5+` helps in harsh lighting or reflective areas.

**Recommended values** moved to the Quick Tips section below.

Filter mode tips: use `median` to ignore spikes or `percentile10` for gradual noise.

### Quick Tips Summary

The two settings work together: a window of `3` with `sampling: 2` means each
reported value reflects six raw readings. Raise both when sunlight or
reflections cause false triggers.

#### Sampling

| `sampling` | When to use | Tradeoff |
| ---------- | ---------- | -------- |
| `1` | Fastest response, low noise | Higher noise |
| `2–3` | Balanced stability and speed | Slight delay |
| `4+` | Very noisy or unstable areas | Noticeable lag |

#### Filter Window

| `filter_window` | When to use | Tradeoff |
| --------------- | ---------- | -------- |
| `3` | General smoothing | Slightly slower response |
| `5+` | Suppress false triggers | Laggy detection |
| `1` | Maximum responsiveness | No noise rejection |

### Configuration Reference

| Option(s) | Required? | Default | Purpose | When to change | Strategy | Conservative Example | Aggressive Example |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `vl53l1x.sensor_id` | Optional | `1` | Distinguish multiple sensors on one bus | Using multiple VL53L1X modules | Assign unique ID per sensor | `sensor_id: 1` | `sensor_id: 2` |
| `vl53l1x.address` & `vl53l1x.pins.xshut` | Optional together | `0x29` | Change the sensor I²C address | Address conflict or multi-sensor setup | Provide an XSHUT pin and new address | *(not set)* | `address: 0x31`<br>`pins:`<br>`  xshut: GPIO3` |
| `vl53l1x.timeout` | Optional | `2s` | How long to wait for a measurement | Long ranges may need more time | Increase in 500&nbsp;ms steps until errors stop | `timeout: 2s` | `timeout: 3s` |
| `vl53l1x.pins.interrupt` | Optional | none | GPIO for data ready signal | Efficient updates | Use if you can spare a pin | *(not set)* | `interrupt: GPIO32` |
| `vl53l1x.calibration.ranging` | Optional | `auto` | Measurement range preset | Known distance extremes | Pick the shortest range that works | `ranging: auto` | `ranging: long` |
| `vl53l1x.calibration.offset` | Optional | none | Distance offset correction | Sensor mounted behind glass | Set the measured mm offset after calibration | *(not set)* | `offset: 20mm` |
| `vl53l1x.calibration.crosstalk` | Optional | none | Photon count correction | Strong reflections | Only adjust with ST's calibration output | *(not set)* | `crosstalk: 100000cps` |
| `roode.sampling` | Optional | `2` | Number of readings averaged | Smoother or faster response | Try 3–5 for noisy areas; above 5 adds lag | `sampling: 2` | `sampling: 5` |
| `roode.orientation` | Optional | `parallel` | Sensor pad orientation | Sensor rotated 90° | Set to `perpendicular` | `orientation: parallel` | `orientation: perpendicular` |
| `roode.roi` | Optional | `h16 w6` | Size of measurement window | Narrow doorway or wide hall | Change by 2–4 units or use `auto` to learn | `roi: { height: 16, width: 6 }` | `roi: auto` |
| `roode.detection_thresholds` | Optional | `min:0% max:85%` | Distance limits for detecting people | Sensor too close or far from traffic | Raise `min` ~5% (or ~50 mm) each time | `detection_thresholds: { min: 5%, max: 85% }` | `detection_thresholds: { min: 50mm, max: 234cm }` |
| `roode.calibration_persistence` | Optional | `false` | Save thresholds in flash | Sensor reboots often | Enable to keep tuning | `calibration_persistence: false` | `calibration_persistence: true` |
| `roode.filter_mode` & `roode.filter_window` | Optional | `min` / `5` | How samples are combined and window size | Noisy environment | Use `median`/`percentile10` with larger windows | `filter_mode: min`<br>`filter_window: 5` | `filter_mode: percentile10`<br>`filter_window: 9` |
| `roode.log_fallback_events` | Optional | `false` | Record INT/XSHUT fallback events | Debugging unexpected counts | Enable while testing | `log_fallback_events: false` | `log_fallback_events: true` |
| `roode.force_single_core` | Optional | `false` | Disable dual-core optimization | ESP32 issues with multi-core | Set true if crashes occur | `force_single_core: false` | `force_single_core: true` |
| `roode.sun_entity_id` | Optional | `sun.sun` | Use Home Assistant sun entity for sunrise/sunset | No coordinates available | Change if using a custom sun entity | *(not set)* | `sun_entity_id: sun.custom` |
| `roode.zones.invert` | Optional | `false` | Swap entry and exit zones | Counts appear reversed | Set true then recalibrate | `zones: { invert: false }` | `zones: { invert: true }` |
| `roode.zones.entry/exit` | Optional | none | Per-zone ROI and thresholds | Uneven hallway or obstacles | Tweak each zone separately as needed | *(not set)* | `zones:`<br>`  exit:`<br>`    roi:`<br>`      height: 8` |


### Example Configurations

| File | Description |
| --- | --- |
| [peopleCounter32.yaml](peopleCounter32.yaml) | Minimal setup for ESP32 |
| [peopleCounter32Dev.yaml](peopleCounter32Dev.yaml) | Most advanced ESP32 configuration |
| [peopleCounter8266.yaml](peopleCounter8266.yaml) | Minimal setup for ESP8266 |
| [peopleCounter8266Dev.yaml](peopleCounter8266Dev.yaml) | Most advanced ESP8266 configuration |
| [extra_sensors_example.yaml](extra_sensors_example.yaml) | Additional diagnostic sensors |
| [ambient_light_example.yaml](ambient_light_example.yaml) | Ambient light and temperature sensors |

### Sensors

#### People Counter

The most important one is the people counter.

```yaml
number:
  - platform: roode
    people_counter:
      name: People Count
```

Regardless of how close we can get, people counting will never be perfect.
This allows the current people count to be adjusted easily via Home Assistant.

#### Other sensors available

```yaml
binary_sensor:
  - platform: roode
    presence_sensor:
      name: $friendly_name presence
    sensor_xshut_state:
      name: $friendly_name xshut state

sensor:
  - platform: roode
    id: hallway
    distance_entry:
      name: $friendly_name distance zone 0
      filters:
        - delta: 100.0
    distance_exit:
      name: $friendly_name distance zone 1
      filters:
        - delta: 100.0
    max_threshold_entry:
      name: $friendly_name max zone 0
    max_threshold_exit:
      name: $friendly_name max zone 1
    min_threshold_entry:
      name: $friendly_name min zone 0
    min_threshold_exit:
      name: $friendly_name min zone 1
    roi_height_entry:
      name: $friendly_name ROI height zone 0
    roi_width_entry:
      name: $friendly_name ROI width zone 0
    roi_height_exit:
      name: $friendly_name ROI height zone 1
    roi_width_exit:
      name: $friendly_name ROI width zone 1
    loop_time:
      name: $friendly_name loop time
    cpu_usage:
      name: $friendly_name CPU usage
    ram_free:
      name: $friendly_name RAM usage
    flash_free:
      name: $friendly_name flash usage
    sensor_status:
      name: $friendly_name sensor status
    interrupt_status:
      name: $friendly_name interrupt status
    manual_adjustment_count:
      name: $friendly_name manual adjusts

text_sensor:
  - platform: roode
    version:
      name: $friendly_name version
  - platform: roode
    entry_exit_event:
      name: $friendly_name last direction
  - platform: roode
    enabled_features:
      name: $friendly_name enabled features
      ## This sensor is a text_sensor that lists all enabled features
```

#### Light and temperature sensors

To use ambient light suppression and scheduled recalibration, define a lux and temperature sensor with IDs that Roode can reference:

```yaml
sensor:
  - platform: bh1750
    name: "Illuminance"
    id: illuminance
    address: 0x23
    update_interval: 30s

  - platform: shtcx
    temperature:
      name: "Temperature"
      id: temperature
    humidity:
      name: "Humidity"
    address: 0x70
    update_interval: 60s

roode:
  use_light_sensor: true
  lux_sensor: illuminance
  temperature_sensor: temperature
```
The features string lists items as `name:value` pairs separated by new lines.
The current output includes: `xshut`, `refresh`, `cpu_mode`, `cpu`,
`cpu_cores`, `ram`, `flash`, `calibration_value`, `calibration`,
`schedule_calibration`, `buffer` and `sun_event`.
Memory values are printed with **KB**, **MB** or **GB** units. Calibration time
uses the device clock in `h:MMAM/PM` format or displays `error` if the clock
has not been initialised. When the device falls back to UTC because no time
zone was provided by Home Assistant, the time is followed by `(UTC)`.

Example output:

```
cpu_mode:dual
cpu:ESP32-D0WDQ5
cpu_cores:2
xshut:enabled
refresh:interrupt
ram:309KB
flash:16MB
calibration_value:1399
calibration:6:01PM (UTC)
schedule_calibration:8:01PM (UTC)
buffer:75% (18/24)
sun_event:sunset 7:45PM
```

### Threshold distance

A crossing is detected when the measured distance for a zone falls between its
configured minimum and maximum values. Roode determines starting thresholds
automatically: after powering up, leave the area clear for about 10&nbsp;seconds so
the idle distance can be measured. The default maximum threshold is 80&nbsp;% of this
resting value.



To fine-tune detection, adjust the `detection_thresholds` option in your YAML or call the `recalibrate` service to re-measure the idle distance.

By default, the sensor calculates thresholds after startup by sampling the idle distance for about 10 seconds. The maximum threshold is set to 80% of this distance and the minimum to 15%. These can be changed at runtime using the `set_entry_threshold_percentages()` and `set_exit_threshold_percentages()` methods.

If you install the sensor \~20 cm above a door and want to ignore door movements, you might lower the minimum threshold:

```yaml
detection_thresholds:
  min: 10%
  max: 80%
```

Or in code:

```cpp
set_entry_threshold_percentages(10, 80);
```

This ensures movements too close to the sensor (like door leaf motion) are filtered out while still detecting people passing underneath.




See the [calibration instructions](calibration/) for further details.

## Algorithm

The implemented algorithm is an improved version of my own implementation which checks the direction of a movement through two defined zones. ST implemented a nice and efficient way to track the path from one to the other direction. I migrated the algorithm with some changes into the Roode project.
The concept of path tracking is the detection of a human:

- In the first zone only
- In both zones
- In the second zone only
- In no zone

That way we can ensure the direction of movement.

The sensor creates a 16x16 grid and computes the final distance by averaging all the values in that grid.
We are defining two different Region of Interest (ROI) inside this grid. Then the sensor will measure the two distances in the two zones and will detect any presence and tracks the path to receive the direction.

However, the algorithm is very sensitive to the slightest modification of the ROI, regarding both its size and its positioning inside the grid.

STMicroelectronics defines default values for these parameters as follows:

The center of the ROI you set is based on the table below and the optical center has to be set as the pad above and to the right of your exact center:

Set the center SPAD of the region of interest (ROI)
based on VL53L1X_SetROICenter() from STSW-IMG009 Ultra Lite Driver

ST user manual [UM2555](https://www.st.com/resource/en/user_manual/um2555-ultralite-driver-for-vl53l1x.pdf) explains ROI selection in detail, so we recommend
reading that document carefully. Here is a table of SPAD locations from
UM2555 (199 is the default/center):

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
120,112,104, 96, 88, 80, 72, 64,   56, 48, 40, 32, 24, 16,  8,  0 <- Pin 1
```

This table is oriented as if looking into the front of the sensor (or top of
the chip). SPAD 0 is closest to pin 1 of the VL53L1X, which is the corner
closest to the VDD pin on the Pololu VL53L1X carrier board:

```
  +--------------+
  |             O| GPIO1
  |              |
  |             O|
  | 128    248   |
  |+----------+ O|
  ||+--+  +--+|  |
  |||  |  |  || O|
  ||+--+  +--+|  |
  |+----------+ O|
  | 120      0   |
  |             O|
  |              |
  |             O| VDD
  +--------------+
```

However, note that the lens inside the VL53L1X inverts the image it sees
(like the way a camera works). So for example, to shift the sensor's FOV to
sense objects toward the upper left, you should pick a center SPAD in the lower right.

## Features

| Feature | Description |
| --- | --- |
| Path tracking algorithm | Distinguishes entry vs exit by tracking the order of zone crossings |
| Auto restart via XSHUT | Sensor restarts automatically if a measurement times out |
| Clean shutdown | Memory and sensor power managed on reboot |
| Startup pin test | Logs and disables features if xshut or interrupt pins fail |
| Built-in pull-ups | XSHUT and interrupt pins use internal pull-ups, no resistors needed |
| Metrics sensors | Optional sensors report loop time, CPU usage, RAM and flash usage |
| Fail-safe recalibration | Triggers recalibration if a zone stays active too long |
| Persistent calibration | Calibration data can persist in flash across reboots |
| Scheduled recalibration | Periodically or temperature-based recalibration of thresholds |
| Ambient light suppression | Learns lux levels with optional sunrise prediction |
| Home Assistant sun fallback | Uses `sun.sun` when latitude/longitude omitted |
| Dual-core tasking | Keeps polling responsive on ESP32 with automatic retry/fallback |
| Filtering options | Median/percentile filters smooth jitter with adjustable window |
| FSM timeouts | Resets the state machine when a transition stalls |
| CPU optimizations | Automatic optimizations when CPU usage exceeds 90% |
| Interrupt fallback | Interrupt mode with graceful fallback to polling and logs |
| XSHUT multiplexing | Supports multiple sensors sharing I²C bus |
| Feature text sensor | Reports enabled and fallback features for diagnostics |
| Manual adjustment counter | Tracks user corrections to the people count |
| Diagnostic sensors | Report INT/XSHUT pin states and other metrics |
| Event logging | Logs sensor power cycles, fallback reasons, and manual adjustments |
| Colored logs | Normal info in green, details in yellow, failures in red |

## Logging and Diagnostics

Roode prints key events to the ESPHome logger. Set `log_fallback_events: true`
in the `roode:` section to include interrupt fallbacks and XSHUT recovery
details. Event logs cover power cycles of the sensor, automatic changes between
interrupt and polling mode, and manual adjustments to the people count.

### Feature text sensor

The `enabled_features` text sensor summarizes which runtime features are active.
Typical values include `dual_core` or `single_core`, `xshut` or `no_xshut`, and
`interrupt` or `polling`. This helps verify that the hardware pins and options
are detected correctly.

### Diagnostic sensors

Optional sensors provide insight into Roode's operation:

- `loop_time`, `cpu_usage`, `ram_free` and `flash_free` report resource usage.
- `sensor_status` and `interrupt_status` show the current hardware state.
- ROI size and threshold sensors allow live tuning of each zone.
- `manual_adjustment_count` records people-count corrections.

See [extra_sensors_example.yaml](extra_sensors_example.yaml) for how to enable
these sensors.


## FAQ/Troubleshoot

**Question:** Why is the Sensor not measuring the correct distances?

**Answer:** This can happen in various scenarios. I try to list causes sorted by likelihood

1. You did not remove the protection film (most times its yellow)
2. You did not connect the Sensor properly
3. Light interference (You will see a lot of noise)
4. Bad connections

**Question:** The counter counts backwards.

**Answer:** Set `zones.invert: true` or rotate the sensor so entry and exit zones match your doorway, then recalibrate.

## License

This project is licensed under the terms of the [Unlicense](LICENSE).

