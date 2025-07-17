# RooDe

[![GitHub release](https://img.shields.io/badge/release-1.6.0-blue?style=flat-square)](https://github.com/Lyr3x/Roode/releases/tag/1.6.0)
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
- [Configuration Reference](#configuration-reference)
  - [Example Configurations](#example-configurations)
  - [Sensors](#sensors)
- [Threshold distance](#threshold-distance)
- [Algorithm](#algorithm)
- [Features](#features)
  - [Interrupt vs Polling](#interrupt-vs-polling)
  - [Single vs Dual Core](#single-vs-dual-core)
  - [Sampling and Filtering](#sampling-and-filtering)
  - [Scheduled Recalibration](#scheduled-recalibration)
  - [Ambient Light Learning & Sunlight Suppression](#ambient-light-learning--sunlight-suppression)
  - [CPU Resilience & Multicore](#cpu-resilience--multicore)
  - [INT Pin Robustness](#int-pin-robustness)
  - [Context-Aware Calibration](#context-aware-calibration)
  - [Adaptive Filtering](#adaptive-filtering)
  - [Light Control](#light-control)
  - [Temp Control](#temp-control)
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
  # Scheduled sensor recalibration and ambient light suppression
  auto_recalibrate_interval: 6h
  recalibrate_on_temp_change: false
  max_temp_delta_for_recalib: 8
  recalibrate_cooldown: 30min
  use_light_sensor: false
  lux_learning_window: 24h
  lux_sample_interval: 1min
  use_sunrise_prediction: false
  latitude: 37.7749
  longitude: -122.4194
  alpha: 0.5
  base_multiplier: 1.0
  max_multiplier: 4.0
  time_multiplier: 1.5
  combined_multiplier: 3.0
  suppression_window: 30min
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

### Configuration Reference

| Section | Description |
| --- | --- |
| [VL53L1X Options](#vl53l1x-options) | Sensor hardware settings |
| [General Options](#general-options) | Core behavior tuning |
| [Recalibration Options](#recalibration-options) | Automatic recalibration triggers |
| [Ambient Light Learning & Light Control](#ambient-light-learning--light-control) | Light suppression features |
| [Zone Options](#zone-options) | Per-zone overrides |


#### VL53L1X Options · [Back to list](#configuration-reference)

Minimal example

```yaml
vl53l1x:
  pins:
    xshut: GPIO3
    interrupt: GPIO1
```

Full example

```yaml
vl53l1x:
  sensor_id: 1
  address: 0x31
  timeout: 2s
  calibration:
    ranging: auto
    offset: 20mm
    crosstalk: 100000cps
  pins:
    xshut: GPIO3
    interrupt: GPIO32
```

| Option(s) | Required? | Default | Purpose | When to change | Strategy |
| --- | --- | --- | --- | --- | --- |
| `vl53l1x.sensor_id` | Optional | `1` | Distinguish multiple sensors on one bus | Using multiple VL53L1X modules | Assign unique ID per sensor |
| `vl53l1x.address` & `vl53l1x.pins.xshut` | Optional together | `0x29` | Change the sensor I²C address | Address conflict or multi-sensor setup | Provide an XSHUT pin and new address |
| `vl53l1x.timeout` | Optional | `2s` | How long to wait for a measurement | Long ranges may need more time | Increase in 500 ms steps until errors stop |
| `vl53l1x.pins.interrupt` | Optional | none | GPIO for data ready signal | Efficient updates | Use if you can spare a pin |
| `vl53l1x.calibration.ranging` | Optional | `auto` | Measurement range preset | Known distance extremes | Pick the shortest range that works |
| `vl53l1x.calibration.offset` | Optional | none | Distance offset correction | Sensor mounted behind glass | Set the measured mm offset after calibration |
| `vl53l1x.calibration.crosstalk` | Optional | none | Photon count correction | Strong reflections | Only adjust with ST's calibration output |

#### General Options · [Back to list](#configuration-reference)

Minimal example

```yaml
roode:
  sampling: 2
```

Full example

```yaml
roode:
  sampling: 2
  orientation: perpendicular
  roi: { height: 16, width: 6 }
  detection_thresholds: { min: 5%, max: 85% }
  calibration_persistence: true
  filter_mode: median
  filter_window: 5
  log_fallback_events: true
  force_single_core: false
```

| Option(s) | Required? | Default | Purpose | When to change | Strategy |
| --- | --- | --- | --- | --- | --- |
| `roode.sampling` | Optional | `2` | Number of readings averaged | Smoother or faster response | Try 3–5 for noisy areas; above 5 adds lag |
| `roode.orientation` | Optional | `parallel` | Sensor pad orientation | Sensor rotated 90° | Set to `perpendicular` |
| `roode.roi` | Optional | `h16 w6` | Size of measurement window | Narrow doorway or wide hall | Change by 2–4 units or use `auto` to learn |
| `roode.detection_thresholds` | Optional | `min:0% max:85%` | Distance limits for detecting people | Sensor too close or far from traffic | Raise `min` ~5% (or ~50 mm) each time |
| `roode.calibration_persistence` | Optional | `false` | Save thresholds in flash | Sensor reboots often | Enable to keep tuning |
| `roode.filter_mode` & `roode.filter_window` | Optional | `min` / `5` | How samples are combined and window size | Noisy environment | Use `median`/`percentile10` with larger windows |
| `roode.log_fallback_events` | Optional | `false` | Record INT/XSHUT fallback events | Debugging unexpected counts | Enable while testing |
| `roode.force_single_core` | Optional | `false` | Disable dual-core optimization | ESP32 issues with multi-core | Set true if crashes occur |

#### Recalibration Options · [Back to list](#configuration-reference)

Minimal example

```yaml
roode:
  auto_recalibrate_interval: 6h
```

Full example

```yaml
roode:
  auto_recalibrate_interval: 6h
  recalibrate_on_temp_change: true
  max_temp_delta_for_recalib: 8
  recalibrate_cooldown: 30min
  idle_recalibrate_interval: 12h
```

| Option(s) | Required? | Default | Purpose | When to change | Strategy |
| --- | --- | --- | --- | --- | --- |
| `roode.auto_recalibrate_interval` | Optional | `6h` | Time between automatic recalibrations | Sensor drifts gradually | Increase for stable temps or set to 0 to disable |
| `roode.recalibrate_on_temp_change` | Optional | `false` | Recalibrate when temperature shifts | Use with a temperature sensor | Enable only if temps vary |
| `roode.max_temp_delta_for_recalib` | Optional | `8` | Temperature change in °C that triggers recalibration | Rapid heat/cool cycles | Lower if drift occurs quickly, raise to avoid noise |
| `roode.recalibrate_cooldown` | Optional | `30min` | Minimum time between automatic recalibrations | Multiple triggers in short time | Extend to prevent loops or shorten for responsiveness |
| `roode.idle_recalibrate_interval` | Optional | `0` | Recalibrate after long idle period | Sensor rarely triggered | Set long interval like 12h to combat slow drift |

#### Ambient Light Learning & Light Control · [Back to list](#configuration-reference)

Minimal example

```yaml
roode:
  use_light_sensor: true
```

Full example

```yaml
roode:
  use_light_sensor: true
  lux_learning_window: 24h
  lux_sample_interval: 1min
  use_sunrise_prediction: true
  latitude: 37.7749
  longitude: -122.4194
  alpha: 0.5
  base_multiplier: 1.0
  max_multiplier: 4.0
  time_multiplier: 1.5
  combined_multiplier: 3.0
  suppression_window: 30min
```

| Option(s) | Required? | Default | Purpose | When to change | Strategy |
| --- | --- | --- | --- | --- | --- |
| `roode.use_light_sensor` | Optional | `false` | Enable lux learning to suppress sunlight events | Sensor near windows | Enable with a light sensor |
| `roode.lux_learning_window` | Optional | `24h` | Time range for lux history | Slow lighting changes | Shorten for seasonal shifts |
| `roode.lux_sample_interval` | Optional | `1min` | How often to sample lux | Battery savings | Increase for low-power setups |
| `roode.use_sunrise_prediction` | Optional | `false` | Use sunrise & sunset times for lux suppression | Outdoors or bright windows | Enable with timezone/location data |
| `roode.latitude` & `roode.longitude` | Optional | none | Manual override for location when predicting sunrise | No Home Assistant location or custom site | Supply decimal degrees for your location |
| `roode.alpha` | Optional | `0.5` | Sensitivity of lux spike detection | Too many or too few suppressions | Increase for aggressive suppression, lower for gentle |
| `roode.base_multiplier` | Optional | `1.0` | Minimum lux multiplier | Keep events with small spikes | Raise if suppression triggers too often |
| `roode.max_multiplier` | Optional | `4.0` | Maximum lux multiplier | Hard limit on sunlight suppression | Lower for less effect |
| `roode.time_multiplier` | Optional | `1.5` | Extra weighting near sunrise/sunset | Bright horizon light at dawn/dusk | Adjust if events still occur |
| `roode.combined_multiplier` | Optional | `3.0` | Limit when both time & lux match | Balances lux and schedule inputs | Increase to enforce stricter suppression |
| `roode.suppression_window` | Optional | `30min` | Ignore repeated light spikes | Sudden sunlight bursts | Reduce for indoor lights |

#### Zone Options · [Back to list](#configuration-reference)

Minimal example

```yaml
roode:
  zones:
    invert: false
```

Full example

```yaml
roode:
  zones:
    invert: true
    entry:
      roi: auto
    exit:
      roi:
        height: 8
        center: 124
      detection_thresholds:
        min: 5%
        max: 70%
```

| Option(s) | Required? | Default | Purpose | When to change | Strategy |
| --- | --- | --- | --- | --- | --- |
| `roode.zones.invert` | Optional | `false` | Swap entry and exit zones | Counts appear reversed | Set true then recalibrate |
| `roode.zones.entry/exit` | Optional | none | Per-zone ROI and thresholds | Uneven hallway or obstacles | Tweak each zone separately as needed |


### Example Configurations

| File | Description |
| --- | --- |
| [peopleCounter32.yaml](peopleCounter32.yaml) | Minimal setup for ESP32 |
| [peopleCounter32Dev.yaml](peopleCounter32Dev.yaml) | Most advanced ESP32 configuration |
| [peopleCounter8266.yaml](peopleCounter8266.yaml) | Minimal setup for ESP8266 |
| [peopleCounter8266Dev.yaml](peopleCounter8266Dev.yaml) | Most advanced ESP8266 configuration |
| [extra_sensors_example.yaml](extra_sensors_example.yaml) | Additional diagnostic sensors |

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
      ## The state publishes on startup so Home Assistant never shows "unknown"
```
The features string lists items as `name:value` pairs separated by new lines.
The current output includes: `xshut`, `refresh`, `cpu_mode`, `cpu`,
`cpu_cores`, `ram`, `flash`, `calibration_value`, `calibration`,
`scheduled_recalibration`, `ambient_light_learning`, `light_control`,
`temp_control`, `light_control_status`, `cpu_resilience`,
`int_pin_robustness`, `context_calibration` and `adaptive_filtering`.
Features that rely on sensors report `disabled` if a sensor error occurs and
automatically switch back to `enabled` after the sensor recovers.
Memory values are printed with **KB**, **MB** or **GB** units. Calibration time
uses the device clock in `h:MMAM/PM` format or displays `unknown` if the clock
has not been initialised.

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
calibration:6:01PM
scheduled_recalibration:supported
ambient_light_learning:supported
light_control:lux
temp_control:disabled
light_control_status:0
cpu_resilience:supported
int_pin_robustness:supported
context_calibration:supported
adaptive_filtering:supported
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
| [Dual-core tasking](#single-vs-dual-core) | Keeps polling responsive on ESP32 with automatic retry/fallback |
| [Filtering options](#sampling-and-filtering) | Median/percentile filters smooth jitter with adjustable window |
| FSM timeouts | Resets the state machine when a transition stalls |
| CPU optimizations | Automatic optimizations when CPU usage exceeds 90% |
| [Interrupt fallback](#interrupt-vs-polling) | Interrupt mode with graceful fallback to polling and logs |
| XSHUT multiplexing | Supports multiple sensors sharing I²C bus |
| Feature text sensor | Reports enabled and fallback features for diagnostics |
| Manual adjustment counter | Tracks user corrections to the people count |
| Diagnostic sensors | Report INT/XSHUT pin states and other metrics |
| Event logging | Logs sensor power cycles, fallback reasons, and manual adjustments |
| Colored logs | Normal info in green, details in yellow, failures in red |
| [Scheduled recalibration](#scheduled-recalibration) | Periodically recalibrates to prevent sensor drift |
| [Ambient light learning](#ambient-light-learning--sunlight-suppression) | Learns lux patterns and suppresses sunlight spikes |
| [CPU resilience & multicore](#cpu-resilience--multicore) | Retries dual-core mode and recovers after failures |
| [INT pin robustness](#int-pin-robustness) | Monitors missed interrupts and recovers via XSHUT |
| [Context-aware calibration](#context-aware-calibration) | Suggests recalibration after repeated manual changes |
| [Adaptive filtering](#adaptive-filtering) | Adjusts filter window based on motion & lighting |
| [Light control](#light-control) | Uses lux or location data to suppress sunlight spikes |
| [Temp control](#temp-control) | Enables recalibration on significant temperature change |

### Interrupt vs Polling · [Back to list](#features)

Roode prefers the interrupt pin for efficient updates. When `interrupt` is defined and validated, the VL53L1X notifies the MCU whenever a new sample is ready. If the INT pin is missing or stops working, Roode falls back to a 10&nbsp;ms polling loop and tries interrupts again every 30&nbsp;min. Polling also acts as a safety net during startup.

| Situation | Use INT | Use Polling |
| --- | --- | --- |
| Normal operation | ✅ | 🔁 (optional verify) |
| INT not received in time | ⛔️ | ✅ |
| Sensor just booted | ⛔️ | ✅ |
| Interrupt unreliable | ⛔️ | ✅ |
| Low-power mode handling | ⛔️ | ✅ |

### Single vs Dual Core · [Back to list](#features)

On ESP32 targets Roode tries to run the sensor loop on the second CPU core so Wi‑Fi and other ESPHome tasks stay responsive. If the task fails to start or when running on an ESP8266 the code automatically falls back to a single‑core loop. You can force single‑core mode with `force_single_core: true`.

### Sampling and Filtering · [Back to list](#features)

Roode smooths distance readings in two stages. The driver first averages multiple raw measurements using the `sampling` option. Each zone then applies a filter across the last few averaged values controlled by `filter_mode` and `filter_window`.

Raising `sampling` makes each reading steadier while `filter_window` dictates how many of those readings must agree before an event fires. Because the filter operates on averaged data, the total number of raw readings considered is `sampling` multiplied by `filter_window`. This gives better noise rejection at the cost of reaction speed. Start with `sampling: 2` and `filter_window: 3` and increase them together if your environment is unstable. See the table below for how the available filter modes behave.

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

### Quick Tips Summary · [Back to list](#features)

The two settings work together: a window of `3` with `sampling: 2` means each reported value reflects six raw readings. Raise both when sunlight or reflections cause false triggers.

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

### Scheduled Recalibration · [Back to list](#features)

Roode automatically runs the calibration routine on a schedule and when
certain conditions are met.  The default interval is every **6&nbsp;h** and
the recalibration can also be triggered by temperature shifts or long idle
periods.  Automatic recalibrations respect a cooldown window (default
30&nbsp;min) so multiple triggers only perform a single calibration. Manual
recalibration via the API or button ignores this cooldown and executes
immediately. If the temperature sensor stops reporting valid data the
temperature-based trigger disables itself and retries after
30&nbsp;min.
The time-based schedule works on its own—you can disable temperature or idle
triggers while still using the interval timer.

### Ambient Light Learning & Sunlight Suppression · [Back to list](#features)

When a light sensor is provided Roode learns the typical lux pattern over a
24&nbsp;h window.  Sudden spikes beyond the learned 95&percnt; percentile are
considered outliers.  If a lux spike occurs around sunrise or sunset it will be
treated more aggressively to avoid false counts from direct sunlight.
If the configured light sensor fails to report valid lux values, the
learning and suppression logic disables itself and automatically retries
after 30&nbsp;min.
Sunrise prediction and lux learning operate independently—you can enable either
one or both depending on available sensors.

> **Note**: Storing a full 24&nbsp;h lux history at one‑minute intervals keeps
> roughly 1,440 samples in memory (about 6&nbsp;kB). Extended windows or faster
> sampling require even more RAM, so an ESP32 is recommended when using a large
> buffer.

| Condition          | Suppression?  | Multiplier                                 | Log Event                   |
| ------------------ | ------------- | ------------------------------------------ | --------------------------- |
| Lux spike only     | ✅             | `dynamic_multiplier`                       | `lux_outlier_detected`      |
| Time-only (no lux) | ❌             | –                                          | –                           |
| Lux + time window  | ✅ (strongest) | `max(dynamic_multiplier, time_multiplier)` | `sunlight_suppressed_event` |

### CPU Resilience & Multicore · [Back to list](#features)

On multi‑core ESP32s Roode attempts to run the sensor loop on the second core
for improved responsiveness.  If initialization fails it logs
`dual_core_fallback` and retries every 5&nbsp;min until it succeeds, logging
`dual_core_recovered` when normal operation resumes.

### INT Pin Robustness · [Back to list](#features)

Roode monitors the interrupt pin for missed events.  After three interrupt
fallbacks within 30&nbsp;min it temporarily switches back to polling and resets
the sensor via the XSHUT pin.

### Context-Aware Calibration · [Back to list](#features)

Repeated manual adjustments are tracked.  If more than five adjustments occur
within an hour while the lux level is high Roode performs a soft recalibration
and logs `manual_recalibrate_triggered`.

### Adaptive Filtering · [Back to list](#features)

The filter window dynamically expands when lux spikes four to six times above
the learned maximum and returns to the default size once lighting stabilises.
This change is recorded with the `filter_window_changed` log event.

### Light Control · [Back to list](#features)

Lux readings or sunrise predictions can suppress events when direct light would
cause false counts. `light_control` reports the active mode as `lux`,
`location` or `both` depending on which inputs are enabled. If neither the light
sensor nor sunrise prediction is configured the offset value stays `0`.

### Temp Control · [Back to list](#features)

The counter can recalibrate when the ambient temperature changes drastically.
Temperature-based recalibration is optional—scheduled recalibration works even
without a temperature sensor. The feature disables itself if the sensor reports
errors and automatically retries after 30&nbsp;min.

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

### Logging Events Overview

| Category | Events |
| --- | --- |
| **Lux/Light** | `lux_learning_complete`, `lux_outlier_detected`, `sunlight_suppressed_event`, `lux_model_bootstrapping`, `lux_sensor_error` |
| **Recalibration** | `auto_recalibrate_interval`, `temp_triggered_recalibration`, `idle_triggered_recalibration`, `recalibrate_cooldown_active`, `manual_recalibrate_triggered`, `temperature_sensor_error` |
| **CPU/Core** | `dual_core_fallback`, `dual_core_recovered` |
| **Interrupt** | `interrupt_fallback`, `int_pin_missed` |
| **Filtering** | `filter_window_changed` |


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

