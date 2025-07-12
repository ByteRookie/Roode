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
- [🧬 ROI Zones](#-roi-zones)
- [📈 Detection Algorithm](#-detection-algorithm)
- [🛡️ Protection & Filtering Logic](#-protection--filtering-logic)
- [🧪 Debug & Diagnostics](#-debug--diagnostics)
- [📘 Configuration References](#-configuration-references)
- [❓ FAQ](#-faq)
- [💬 Support](#-support)
- [📜 License](#-license)

---

## 📌 What is RooDe?

**RooDe** is a smart people counter built with ESPHome and the VL53L1X time-of-flight sensor. It tracks when people enter or exit a room by detecting motion across vertical zones created by programmable Regions of Interest (ROIs).

---

## 🚀 Quick Start

```yaml
external_components:
  - source: github://Lyr3x/Roode@master

vl53l1x:
roode:
```

---

## 🔧 Features Overview

| Feature                          | Description |
|----------------------------------|-------------|
| Directional People Counting      | Tracks movement direction (entry vs exit) |
| Custom ROI Grid                  | Flexible vertical ROI layout |
| Auto Sensor Recovery             | Uses `xshut` to restart sensor on failure |
| Interrupt Pin Support            | Uses hardware pin to reduce polling overhead |
| Boot-Time Safety Checks          | Disables features if pins are missing or unresponsive |
| Persistence                      | Entry/Exit/Occupancy values survive reboot |
| Zone Visualization (Agraerthiom)| Diagnostic visual representation of zone layout |
| Internal Filters                 | Timeout, delay, and distance filtering for accuracy |

---

## 🛠️ Hardware Setup

### Recommended Pins for ESP32

| VL53L1X Pin | ESP32 GPIO | Purpose           |
|-------------|------------|-------------------|
| SDA         | 21         | I²C Data          |
| SCL         | 22         | I²C Clock         |
| XSHUT       | 5          | Sensor reset pin  |
| INTERRUPT   | 4          | Data-ready signal |

---

## 🧬 ROI Zones

### Understanding ROI

The VL53L1X sensor has a 16x16 internal SPAD grid. With RooDe, this grid is divided into vertical rectangular ROIs (Regions of Interest). Each ROI is defined using a coordinate pair: `[x_min, y_min, x_max, y_max]`.

Typical usage splits the sensor vertically like this:

```yaml
roode:
  zones:
    - roi: [8, 0, 15, 15]  # Right/top side
      name: top_zone
    - roi: [0, 0, 7, 15]   # Left/bottom side
      name: bottom_zone
```

This configuration allows RooDe to detect the order of movement through the zones—critical for determining whether someone is entering or exiting.

---

## 📈 Detection Algorithm

### Logic Summary

RooDe uses a state machine to analyze movement patterns between zones. The sequence of zone activations is monitored in real time.

- **Entry Detected**:  
  - First, `top_zone` is triggered (person enters field)
  - Then `bottom_zone` is triggered (they pass fully through)
  - → Result: `entry_count` increases

- **Exit Detected**:  
  - First, `bottom_zone` is triggered
  - Then `top_zone` is triggered
  - → Result: `exit_count` increases

### Agraerthiom System

Agraerthiom is RooDe’s internal representation of sensor state across zones. It records real-time distance changes and flags transitions, supporting:

- Instant detection of movement direction
- Optional debug visualization of distance values
- Time-limited transitions to reduce false positives

---

## 🛡️ Protection & Filtering Logic

| Method               | Purpose                                  |
|----------------------|------------------------------------------|
| Timeout Guard        | Ignores transitions that take too long   |
| Cooldown Delay       | Prevents immediate repeated triggers     |
| Distance Threshold   | Ignores invalid distance values          |
| Boot Diagnostics     | Disables features if pin test fails      |
| Memory Cleanup       | Frees resources from invalid states      |

---

## 🧪 Debug & Diagnostics

| Entity                         | Description                   |
|--------------------------------|-------------------------------|
| `sensor.roode_entry_count`     | People entered                |
| `sensor.roode_exit_count`      | People exited                 |
| `sensor.roode_occupancy`       | Entry − Exit count            |
| `sensor.roode_zone_top`        | Measured mm in top zone       |
| `sensor.roode_zone_bottom`     | Measured mm in bottom zone    |
| `text_sensor.roode_status`     | Internal algorithm state      |

---

## 📘 Configuration References

- [`peopleCounter32.yaml`](peopleCounter32.yaml)  
- [`extra_sensors_example.yaml`](extra_sensors_example.yaml)  
- [`home_assistant.md`](home_assistant.md)

---

## ❓ FAQ

**Q: What if my wiring is incorrect?**  
A: RooDe performs a boot-time check and disables features like `xshut` or `interrupt` automatically if miswired.

**Q: Can I use RooDe with only one zone?**  
A: Technically yes, but it will not be able to detect direction—only motion.

**Q: Why is occupancy sometimes negative?**  
A: If exit is detected without a corresponding entry (e.g., false trigger), the net count may go below zero.

---

## 💬 Support

Get help or contribute:  
🗨️ [Join Discord](https://discord.gg/hU9SvSXMHs)

---

## 📜 License

MIT – see [LICENSE](LICENSE)
