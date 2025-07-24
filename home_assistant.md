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

## Runtime configuration service

Roode exposes a `config` API service automatically. Call the service using the
device name with `_config` appended, e.g. `esphome.roode32_config`. Send a JSON
string in the `json` field with any combination of settings to update. Omit a
key to keep its current value. Updated settings are stored in flash so they
survive reboots.

### Parameters

| Name | Purpose | Recommended Value |
| --- | --- | --- |
| `orientation` | Sensor orientation | `parallel` or `perpendicular` |
| `sampling` | Number of raw readings averaged | `2` (1–10 readings) |
| `filter_mode` | Ranging data filter | `min`, `median`, or `percentile10` |
| `filter_window` | Size of filter window | `5` (1–20 samples) |
| `log_fallback_events` | Enable extra event logging | `false` |
| `calibration_persistence` | Save calibration data in flash | `false` |
| `force_single_core` | Disable ESP32 dual core optimizations | `false` |
| `invalid_distance_limit` | Allowed suspect readings before restart | `10` reads |
| `restart_timeout` | Cooldown between restarts | `30` seconds |
| `invert_zones` | Swap entry and exit zones | `false` |
| `entry_min` / `exit_min` | Minimum threshold | `200` mm |
| `entry_max` / `exit_max` | Maximum threshold | `85%` of idle distance |
| `entry_roi_height` / `exit_roi_height` | ROI height | `16` px |
| `entry_roi_width` / `exit_roi_width` | ROI width | `6` px |
| `entry_roi_center` / `exit_roi_center` | ROI center | `167` / `231` (parallel) |

### Example

```yaml
- service: esphome.roode32_config
  data:
    json: |
      {
        "orientation": "perpendicular",
        "sampling": 2,

        "filter_mode": "median",
        "filter_window": 5,

        "log_fallback_events": false,
        "calibration_persistence": false,
        "force_single_core": false,
        "invalid_distance_limit": 10,
        "restart_timeout": 30,

        "invert_zones": false,
        "entry_min": 200,
        "entry_max": 85,
        "exit_min": 200,
        "exit_max": 85,
        "entry_roi_height": 16,
        "entry_roi_width": 6,
        "entry_roi_center": 195,
        "exit_roi_height": 16,
        "exit_roi_width": 6,
        "exit_roi_center": 60
      }
```
