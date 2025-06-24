To make all functions of Roode work with Home Assistant you can now use ESPHome
buttons for resetting the counter and starting a recalibration.
The examples below still demonstrate how to wire automations if you prefer that
approach.

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

If using ESPHome 2022.11 or newer you can create buttons directly in the Roode
configuration:

```yaml
button:
  - platform: template
    name: Reset counter
    on_press:
      - lambda: |-
          id(roode_platform)->reset_counter();
  - platform: template
    name: Recalibrate
    on_press:
      - lambda: |-
          id(roode_platform)->recalibration();
```