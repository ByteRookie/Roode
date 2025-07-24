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

Add the `config` service under the `api:` section of your ESPHome YAML to change
any Roode option without reflashing. Call the service using the device name with
`_config` appended, e.g. `esphome.roode32_config`.

```yaml
api:
  services:
    - service: config
      variables:
        orientation: string
        sampling: int
        filter_mode: string
        filter_window: int
        log_fallback_events: bool
        calibration_persistence: bool
        force_single_core: bool
        invalid_distance_limit: int
        restart_timeout: int
        invert_zones: bool
        entry_min: int
        entry_max: int
        exit_min: int
        exit_max: int
        entry_roi_height: int
        entry_roi_width: int
        exit_roi_height: int
        exit_roi_width: int
      then:
        - lambda: |
            auto r = id(roode_platform);
            r->set_orientation(orientation == "perpendicular" ? esphome::roode::Perpendicular : esphome::roode::Parallel);
            r->set_sampling_size(sampling);
            if (filter_mode == "median")
              r->set_filter_mode(esphome::roode::FILTER_MEDIAN);
            else if (filter_mode == "percentile10")
              r->set_filter_mode(esphome::roode::FILTER_PERCENTILE10);
            else
              r->set_filter_mode(esphome::roode::FILTER_MIN);
            r->set_filter_window(filter_window);
            r->set_log_fallback_events(log_fallback_events);
            r->set_calibration_persistence(calibration_persistence);
            r->set_force_single_core(force_single_core);
            r->set_invalid_distance_limit(invalid_distance_limit);
            r->set_restart_timeout(restart_timeout * 1000);
            r->set_invert_direction(invert_zones);
            r->entry->threshold->min = entry_min;
            r->entry->threshold->max = entry_max;
            r->exit->threshold->min = exit_min;
            r->exit->threshold->max = exit_max;
            r->entry->roi_override->height = entry_roi_height;
            r->entry->roi_override->width = entry_roi_width;
            r->exit->roi_override->height = exit_roi_height;
            r->exit->roi_override->width = exit_roi_width;
```

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
