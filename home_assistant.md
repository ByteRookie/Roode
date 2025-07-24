To make all functions of Roode work with home assistant you need to set up a few entities and automations. 
Roode has endpoints to set the count value, reset the counter to 0 and to recalibrate. Unfortunately its not possible to expose buttons via ESPHome that do just that.

```
# This automation script runs when the counter has changed.
# It sets the value slider on the GUI. This slides also had its own automation when the value is changed.
- alias: "Set people32 slider"
  trigger:
    platform: state
    entity_id: sensor.roode32_people_counter_2
  action:
    service: input_number.set_value
    target:
      entity_id: input_number.set_people32
    data:
      value: "{{ states('sensor.roode32_people_counter_2') }}"
- alias: "people32 slider moved"
  trigger:
    platform: state
    entity_id: input_number.set_people32
  action:
    service: esphome.roode32_set_counter
    data:
      newCount: "{{ states('input_number.set_people32') | int }}"
```

## Adjustable Sensor Settings

Add the following `input_number` helpers to Home Assistant so each setting can
be adjusted at runtime. Home Assistant stores these values persistently, so
changes survive reboots. The table below lists each helper and its purpose.

| Setting                    | Entity ID                                    | Range / Step             | Purpose                                   |
| -------------------------- | -------------------------------------------- | ------------------------ | ----------------------------------------- |
| `invalid_distance_limit`   | `input_number.roode_invalid_distance_limit`  | 1–100, step: 1          | Number of bad reads before restart        |
| `restart_timeout`          | `input_number.roode_restart_timeout`         | 1–120 sec, step: 1      | Cooldown before sensor restarts           |
| `sampling`                 | `input_number.roode_sampling`                | 1–6, step: 1            | Smoothing level (raw data averaging)      |
| `filter_window`            | `input_number.roode_filter_window`           | 3–9, step: 2            | Window size for smoothing filtered values |
| `calibration.offset`       | `input_number.roode_calibration_offset`      | -50 to 50 mm, step: 1    | Offset adjustment after calibration       |
| `calibration.crosstalk`    | `input_number.roode_calibration_crosstalk`   | 0–100000 cps, step: 1000 | Crosstalk calibration value               |
| `detection_thresholds.min` | `input_number.roode_detection_min_threshold` | 0–100%, step: 1         | Minimum threshold (global)                |
| `detection_thresholds.max` | `input_number.roode_detection_max_threshold` | 0–100%, step: 1         | Maximum threshold (global)                |
| `roi.height`               | `input_number.roode_roi_height`              | 4–16, step: 1           | Region of Interest (height)               |
| `roi.width`                | `input_number.roode_roi_width`               | 4–16, step: 1           | Region of Interest (width)                |

The configuration below defines every helper with its default values. You can
customise the names or units if desired.

```yaml
input_number:
  roode_invalid_distance_limit:
    name: Invalid Distance Limit
    initial: 10
    min: 1
    max: 100
    step: 1
    mode: box
    # Number of bad reads before restart

  roode_restart_timeout:
    name: Restart Timeout
    unit_of_measurement: s
    initial: 30
    min: 1
    max: 120
    step: 1
    mode: box
    # Cooldown before sensor restarts

  roode_sampling:
    name: Sampling
    initial: 2
    min: 1
    max: 6
    step: 1
    mode: box
    # Smoothing level (raw data averaging)

  roode_filter_window:
    name: Filter Window
    initial: 5
    min: 3
    max: 9
    step: 2
    mode: box
    # Window size for smoothing filtered values

  roode_calibration_offset:
    name: Calibration Offset
    unit_of_measurement: mm
    initial: 0
    min: -50
    max: 50
    step: 1
    mode: box
    # Offset adjustment after calibration

  roode_calibration_crosstalk:
    name: Calibration Crosstalk
    unit_of_measurement: cps
    initial: 0
    min: 0
    max: 100000
    step: 1000
    mode: box
    # Crosstalk calibration value

  roode_detection_min_threshold:
    name: Detection Min Threshold
    unit_of_measurement: "%"
    initial: 0
    min: 0
    max: 100
    step: 1
    mode: box
    # Minimum threshold (global)

  roode_detection_max_threshold:
    name: Detection Max Threshold
    unit_of_measurement: "%"
    initial: 85
    min: 0
    max: 100
    step: 1
    mode: box
    # Maximum threshold (global)

  roode_roi_height:
    name: ROI Height
    unit_of_measurement: px
    initial: 16
    min: 4
    max: 16
    step: 1
    mode: box
    # Region of Interest (height)

  roode_roi_width:
    name: ROI Width
    unit_of_measurement: px
    initial: 6
    min: 4
    max: 16
    step: 1
    mode: box
    # Region of Interest (width)
```
