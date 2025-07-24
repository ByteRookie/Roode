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

Roode automatically exposes the following sensor settings as persistent
`input_number` helpers. They appear the first time the device boots and retain
their values through reboots, so no extra configuration is required. Each helper
starts at the same default value as the corresponding setting and can be changed
in real time. The table below lists every helper, its default range and its
purpose.

| Setting                    | Entity ID                                    | Range / Step             | Default | Purpose                                   |
| -------------------------- | -------------------------------------------- | ------------------------ | ------- | ----------------------------------------- |
| `invalid_distance_limit`   | `input_number.roode_invalid_distance_limit`  | 1–100, step: 1           | 10      | Number of bad reads before restart        |
| `restart_timeout`          | `input_number.roode_restart_timeout`         | 1–120 sec, step: 1       | 30      | Cooldown before sensor restarts           |
| `sampling`                 | `input_number.roode_sampling`                | 1–6, step: 1             | 2       | Smoothing level (raw data averaging)      |
| `filter_window`            | `input_number.roode_filter_window`           | 3–9, step: 2             | 5       | Window size for smoothing filtered values |
| `calibration.offset`       | `input_number.roode_calibration_offset`      | -50 to 50 mm, step: 1    | 0       | Offset adjustment after calibration       |
| `calibration.crosstalk`    | `input_number.roode_calibration_crosstalk`   | 0–100000 cps, step: 1000 | 0       | Crosstalk calibration value               |
| `detection_thresholds.min` | `input_number.roode_detection_min_threshold` | 0–100%, step: 1          | 15      | Minimum threshold (global)                |
| `detection_thresholds.max` | `input_number.roode_detection_max_threshold` | 0–100%, step: 1          | 80      | Maximum threshold (global)                |
| `roi.height`               | `input_number.roode_roi_height`              | 4–16, step: 1            | 16      | Region of Interest (height)               |
| `roi.width`                | `input_number.roode_roi_width`               | 4–16, step: 1            | 6       | Region of Interest (width)                |

