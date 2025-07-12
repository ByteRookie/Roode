# RooDe – Smart People Counter for ESPHome + Home Assistant

[![GitHub release](https://img.shields.io/github/v/tag/Lyr3x/Roode?style=flat-square)](https://GitHub.com/Lyr3x/Roode/releases/)
[![Build](https://img.shields.io/github/workflow/status/Lyr3x/Roode/CI?style=flat-square)](https://github.com/Lyr3x/Roode/blob/master/.github/workflows/ci.yml)
[![Maintenance](https://img.shields.io/maintenance/yes/2025?style=flat-square)](https://GitHub.com/Lyr3x/Roode/graphs/commit-activity)
[![Roode community](https://img.shields.io/discord/879407995837087804.svg?label=Discord&logo=Discord&colorB=7289da&style=for-the-badge)](https://discord.gg/hU9SvSXMHs)
![ESPHome Compatible](https://img.shields.io/badge/ESPHome-supported-blue?style=flat-square)
![Home Assistant](https://img.shields.io/badge/Home%20Assistant-Ready-brightgreen?style=flat-square)

---

## 📚 Table of Contents
- [📌 What is RooDe?](#-what-is-roode)
- [🚀 Quick Start](#-quick-start)
- [🔧 Features Overview](#-features-overview)
- [🛠️ Hardware Setup](#-hardware-setup)
- [🧬 ROI Zones & Grid Details](#-roi-zones--grid-details)
- [📈 Detection Algorithm & State Machine](#-detection-algorithm--state-machine)
- [🛡️ Filtering, Protection & Safeguards](#-filtering-protection--safeguards)
- [🧪 Sensors, Diagnostics & Output](#-sensors-diagnostics--output)
- [📘 Configuration References](#-configuration-references)
- [❓ FAQ](#-faq)
- [💬 Support](#-support)
- [📜 License](#-license)

---

## 📌 What is RooDe?

**RooDe** is a smart occupancy and directional people counter that uses a time-of-flight sensor (VL53L1X) to determine whether someone is entering or leaving a room. It's fully compatible with ESPHome and Home Assistant and uses zone-based analysis and an internal algorithm to make accurate real-time decisions.

---

## 🚀 Quick Start

Add this to your ESPHome YAML:

```yaml
external_components:
  - source: github://Lyr3x/Roode@master

vl53l1x:
roode:
```

Supports:
- ESP32 (fully supported)
- ESP8266 (limited performance, polling only)

---

## 🔧 Features Overview

| Feature                          | Description |
|----------------------------------|-------------|
| Directional Counting             | Entry/Exit detection with bidirectional logic |
| ROI Zone Customization           | You define the Regions of Interest on the sensor grid |
| XSHUT Pin Support                | Reset the sensor on boot or on fault |
| Interrupt Pin Support            | Data-ready signal improves efficiency |
| Boot-Time Pin Verification       | Gracefully disables features if wiring is incomplete |
| State Persistence                | Occupancy persists after reboots |
| Agraerthiom Debug View           | Live exposure of zone distance and logic status |
| Adaptive ROI Logic               | Internal mapping supports dynamic zone reshaping |

---

## 🛠️ Hardware Setup

| Pin       | ESP32 GPIO | Description               |
|-----------|------------|---------------------------|
| SDA       | 21         | I²C Data                   |
| SCL       | 22         | I²C Clock                  |
| XSHUT     | 5          | Shutdown pin (optional)    |
| INTERRUPT | 4          | Data ready pin (optional)  |

---

## 🧬 ROI Zones & Grid Details

### VL53L1X ROI System

The VL53L1X sensor has a 16×16 SPAD matrix, which RooDe divides into **2 or more vertical rectangular ROIs**. You define these zones via the `roode.zones` config.

Example split:

```yaml
roode:
  zones:
    - roi: [8, 0, 15, 15]
      name: top_zone
    - roi: [0, 0, 7, 15]
      name: bottom_zone
```

This configuration divides the full width into **top (right)** and **bottom (left)** segments. People crossing from one zone to the other are tracked.

> Each ROI works like a motion detector that records distance in mm over time.

---

## 📈 Detection Algorithm & State Machine

### Entry Flow:
1. `top_zone` triggers first
2. `bottom_zone` triggers next (within time window)
3. → Counts as entry

### Exit Flow:
1. `bottom_zone` triggers first
2. `top_zone` triggers next (within time window)
3. → Counts as exit

### State Machine Highlights

- Internal state machine tracks transition sequences.
- Valid sequences result in counter adjustments.
- Invalid or incomplete transitions are discarded.

---

## 🛡️ Filtering, Protection & Safeguards

RooDe includes built-in stability checks and filters:

| Type               | Description |
|--------------------|-------------|
| Timeout Filter     | Prevents stale triggers from affecting counts |
| Cooldown Window    | Avoids double-counting from slow walkers |
| Invalid Distance   | Ignores reflections or erroneous sensor spikes |
| Pin Test at Boot   | Disables features if pins are unset or unstable |
| Async Safety       | Ensures state resets don’t crash ESP during loops |

---

## 🧪 Sensors, Diagnostics & Output

RooDe exposes the following sensors in ESPHome:

| Entity                        | Description |
|-------------------------------|-------------|
| `sensor.roode_entry_count`    | Count of detected entries |
| `sensor.roode_exit_count`     | Count of detected exits |
| `sensor.roode_occupancy`      | Entry - Exit total |
| `sensor.roode_zone_top`       | Distance from top zone (mm) |
| `sensor.roode_zone_bottom`    | Distance from bottom zone (mm) |
| `text_sensor.roode_status`    | Algorithm state (e.g., idle, triggering, blocked) |

---

## 📘 Configuration References

- [`peopleCounter32.yaml`](peopleCounter32.yaml)
- [`extra_sensors_example.yaml`](extra_sensors_example.yaml)
- [`home_assistant.md`](home_assistant.md)

---

## ❓ FAQ

**Q: Can I use only one zone?**  
No, directional tracking requires at least two zones for comparison.

**Q: Why does my sensor sometimes read `0mm`?**  
This may indicate an invalid reflection. Increase the distance threshold in the zone config.

**Q: How does it recover from a freeze?**  
The sensor auto-resets using the XSHUT pin if available.

**Q: Can I reset counters manually?**  
Yes, using ESPHome's `number` control entities or through Home Assistant.

---

## 💬 Support

Questions or improvements?  
Join the community: [Discord](https://discord.gg/hU9SvSXMHs)

---

## 📜 License

MIT – see [LICENSE](LICENSE)
