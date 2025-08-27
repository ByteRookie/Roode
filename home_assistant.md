To make all functions of Roode work with home assistant you need to set up a few entities and automations.
Roode now exposes button entities to start a passive scan or force a recalibration, so no manual template buttons are required.
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

## Portal Switch and REST API

Roode ships with an optional web portal that serves a small HTML page and a
REST API for configuration and calibration tasks. The web server is disabled
on boot to save memory; enable it only when needed.

### Enabling the portal

The example firmware defines a `portal_switch` template switch. Turning this
switch on starts the web server and registers the portal; turning it off stops
both the server and the portal logic:

```yaml
globals:
  - id: portal_on
    type: bool
    restore_value: yes
    initial_value: 'false'

switch:
  - platform: template
    id: portal_switch
    name: Portal
    lambda: |-
      return id(portal_on);
    turn_on_action:
      - lambda: |-
          id(portal_on) = true;
          id(roode_platform).start_portal();
      - web_server.start:
    turn_off_action:
      - lambda: |-
          id(portal_on) = false;
          id(roode_platform).stop_portal();
      - web_server.stop:
```

If the ESPHome `api:` component is enabled, this `Portal` switch is exposed to
Home Assistant and can be toggled from the UI or via automations.

### Endpoints

When the portal is active the device serves:

- `/portal` or `/` – simple HTML configuration page.
- `/api/settings/current` – current firmware and ROI settings.
- `/api/scan/start` *(POST)* – start a passive scan.
- `/api/scan/status` – progress of the active scan.
- `/api/scan/cancel` *(POST)* – cancel the running scan.
- `/api/roi/preview` – preview recommended ROI/threshold values.
- `/api/roi/apply` *(POST)* – apply the recommended settings.
- `/api/scan/sessions` – list stored calibration sessions.
- `/api/scan/session/<id>` – retrieve a specific session.
- `/api/scan/delete` *(POST)* – delete all stored sessions.
- `/api/export/all` – export all session data as JSON.

Leave the `portal_switch` off during normal operation to keep resource usage
low. The portal and endpoints are only loaded while the switch is on.

## Passive Scan Trigger

Roode exposes `Start Passive Scan` and `Recalibrate` button entities that
request a scan session or rerun the calibration directly from the device.
Both buttons are enabled by default, so Home Assistant will automatically
surface them in the interface. Pressing the scan button creates a
timestamped `passive_scan_YYYY-MM-DD_HH-MM/` directory with an empty
`session.json` and dispatches an `esphome.<node>_start_passive_scan`
command to the ESPHome node. The `Recalibrate` button immediately invokes
the device's calibration routine.

## Calibration Dashboard

Import the Lovelace view in `homeassistant/roode_dashboard.yaml` to add a
dedicated panel for running scans and reviewing results. The view provides
buttons to start a passive scan, accept the computed ROI or rerun the
calibration.

To expose advanced tuning options, add an `input_boolean` that acts as a
"Show expert parameters" toggle:

```
input_boolean:
  roode_show_expert:
    name: Show expert parameters
```

Place `homeassistant/python_scripts/apply_roi_result.py` alongside the scan
script. When the *Accept* button is pressed, the script copies the most
recent `roi_result.json` into the Home Assistant configuration directory so
it can be used for an OTA update. After a scan finishes, the panel displays
ROI dimensions and detection thresholds; enabling *Show expert parameters*
reveals additional tunable defaults.

## Calibration Session Data

Calibration measurements are written to timestamped folders using the
pattern `calibration_YYYY-MM-DD_HH-MM/`.  Each session directory
initially contains a `session.json` file holding the raw records defined
in `components/roode/schema.py`.  After post-analysis, an additional
`roi_result.json` file may be stored in the same directory.

Example layout:

```
calibration_2024-04-23_14-30/
├── session.json
└── roi_result.json
```

Within `session.json` every entry uses the following fields:

- `type` – record type identifier
- `grid` – ROI grid or positional reference
- `trial` – sequential trial number
- `ranging` – raw ranging value in millimetres
- `timestamp` – ISO 8601 timestamp when recorded
- `data` – arbitrary metadata

Timestamps use a `YYYY-MM-DD_HH-MM` prefix for the directory and ISO 8601
format within the JSON entries.  Using this naming convention allows
other components to discover and share session data reliably.

### Collecting Calibration Sessions

A helper script `session_recorder.py` listens for the device's WebSocket
messages and stores them in `session.json`:

```bash
python session_recorder.py ws://device.local/session
```

Each JSON payload from the device is appended as a line to the session file.
When a message with `type` set to `scan_complete` is received, the script
closes the file and optionally runs a follow-up analysis command supplied via
`--analysis`.
