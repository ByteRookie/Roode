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

To control which sensors are visible in Home Assistant, call the `exposed_sensors`
service (requires API services to be enabled) with one of three options:

* `ALL` – expose every Roode sensor and the `enabled_features` text sensor
* `NONE` – hide all optional sensors. Only Occupancy, Presence and Uptime
  sensors stay visible and the `enabled_features` text sensor is hidden
* A comma-separated list of specific sensor names. Names are matched
  case-insensitively and may omit the friendly name prefix. The `enabled_features`
  text sensor remains visible

The repository includes `custom_components/roode/services.yaml` so the service
UI in Home Assistant lists `ALL` and `NONE` while allowing custom values.

Only real sensors can be selected. Configuration parameters like `sampling` or
`force_single_core` are not sensors and will be ignored. Check the names shown
by the `exposed_sensors` text sensor before calling the service.

Example calls:

```yaml
# The Home Assistant service UI lists ALL and NONE as options
# and allows typing a comma-separated list thanks to
# custom_components/roode/services.yaml
# Expose all sensors
- service: esphome.roode32_exposed_sensors
  data:
    sensors: ALL

# Hide all sensors
- service: esphome.roode32_exposed_sensors
  data:
    sensors: NONE

# Expose only selected sensors
- service: esphome.roode32_exposed_sensors
  data:
    sensors: "cpu_usage, interrupt_status"

The firmware restarts after changes so newly exposed sensors are discovered by
Home Assistant.
```
