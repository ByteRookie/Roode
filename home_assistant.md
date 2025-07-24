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
device name with `_config` appended, e.g. `esphome.roode32_config`. All settings
below can be changed at runtime without reflashing the device.

### Parameters

| Name | Purpose |
| --- | --- |
| `orientation` | `parallel` or `perpendicular` sensor orientation |
| `sampling` | Number of raw readings averaged |
| `filter_mode` | `min`, `median`, or `percentile10` filtering |
| `filter_window` | Size of the filter window |
| `log_fallback_events` | Enable extra event logging |
| `calibration_persistence` | Save calibration data in flash |
| `force_single_core` | Disable ESP32 dual core optimizations |
| `invalid_distance_limit` | Allowed suspect readings before restart |
| `restart_timeout` | Cooldown in seconds between restarts |
| `invert_zones` | Swap entry and exit zones |
| `entry_min` / `exit_min` | Minimum threshold in mm for each zone |
| `entry_max` / `exit_max` | Maximum threshold in mm for each zone |
| `entry_roi_height` / `exit_roi_height` | ROI height in pixels for each zone |
| `entry_roi_width` / `exit_roi_width` | ROI width in pixels for each zone |
