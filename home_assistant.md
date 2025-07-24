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

* `ALL` – expose every Roode sensor
* `NONE` – hide all sensors
* A comma-separated list of specific sensor names

Example calls:

```yaml
# Expose all sensors
- service: esphome.roode32_exposed_sensors
  data:
    sensors: ALL

# Hide all sensors
- service: esphome.roode32_exposed_sensors
  data:
    sensors: NONE

# Expose just the entry and exit distance sensors
- service: esphome.roode32_exposed_sensors
  data:
    sensors: "distance_entry, distance_exit"

The firmware restarts after changes so newly exposed sensors are discovered by
Home Assistant.
```
