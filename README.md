# RooDe – Smart People Counter for ESPHome + Home Assistant

[![GitHub release](https://img.shields.io/github/v/tag/Lyr3x/Roode?style=flat-square)](https://GitHub.com/Lyr3x/Roode/releases/)
[![Build](https://img.shields.io/github/workflow/status/Lyr3x/Roode/CI?style=flat-square)](https://github.com/Lyr3x/Roode/blob/master/.github/workflows/ci.yml)
[![Maintenance](https://img.shields.io/maintenance/yes/2025?style=flat-square)](https://GitHub.com/Lyr3x/Roode/graphs/commit-activity)
[![Roode community](https://img.shields.io/discord/879407995837087804.svg?label=Discord&logo=Discord&colorB=7289da&style=for-the-badge)](https://discord.gg/hU9SvSXMHs)
![ESPHome Compatible](https://img.shields.io/badge/ESPHome-supported-blue?style=flat-square)
![Home Assistant](https://img.shields.io/badge/Home%20Assistant-Ready-brightgreen?style=flat-square)

---

![RooDe Entry/Exit Flow](images/roode-entry-exit-flow.png)  
*Above: People entering and exiting a doorway tracked by the RooDe sensor.*

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

**RooDe** is a smart people counter built with ESPHome and the VL53L1X time-of-flight sensor. It tracks when people enter or exit a room based on movement patterns through vertical detection zones.

**Use cases**:
- Room occupancy automation  
- Energy-saving HVAC control  
- Crowd flow monitoring  
- General-purpose foot traffic analytics  

---

## 🚀 Quick Start

```yaml
external_components:
  - source: github://Lyr3x/Roode@master

vl53l1x:
roode:
```

> ✅ ESP32 and ESP8266 compatible  
> ✅ Works with MQTT + ESPHome  
> ✅ YAML examples included

---

## 🔧 Features Overview

| Feature                          | Description |
|----------------------------------|-------------|
| Directional People Counting      | Distinguishes entry and exit through a defined zone |
| Multi-Zone ROI Tracking          | Configurable vertical grid zones to track motion paths |
| Automatic Sensor Recovery        | Detects and restarts hung sensors using `xshut` |
| Boot-Time Self-Test              | Verifies pin connections and disables unstable features |
| Persistent Storage               | Maintains count values through device reboots |
| Interrupt Pin Support            | Reduces CPU usage by waiting for data-ready signal |
| Visual Debug Grid (Agraerthiom)  | Helps you align and visualize ROI zone positions |
| Built-in Diagnostics             | Exposes sensors for live debugging in Home Assistant |
| Low False Positive Rate          | Protection logic filters invalid movement patterns |
| Async-safe Memory Usage          | Frees memory and gracefully shuts down unused processes |

---

## 🛠️ Hardware Setup

| Component      | Recommended Part                                |
|----------------|--------------------------------------------------|
| Sensor         | VL53L1X (Pololu, Adafruit, etc.)                 |
| Microcontroller | ESP32 (preferred) or ESP8266 (limited support) |
| Optional       | 3D enclosure (`STL/` folder) for alignment/stability |

### Basic Wiring (ESP32)

| Sensor Pin | ESP32 GPIO | Purpose       |
|------------|------------|---------------|
| SDA        | 21         | I²C Data      |
| SCL        | 22         | I²C Clock     |
| XSHUT      | 5          | Shutdown control (required) |
| INTERRUPT  | 4          | Data ready (optional but recommended) |

---

## 🧬 ROI Zones

![ROI Zones Explained](images/roode-roi-grid.png)  
*Above: RooDe’s sensor splits a doorway into top and bottom detection zones.*

```yaml
roode:
  zones:
    - roi: [8, 0, 15, 15]  # Top half of doorway
      name: top_zone
    - roi: [0, 0, 7, 15]   # Bottom half
      name: bottom_zone
```

```
+----------------+
| top_zone       |
|                |
|----------------|
| bottom_zone    |
+----------------+
```

---

## 📈 Detection Algorithm – How It Works

![RooDe Detection Flow](images/roode-algorithm-flow.png)  
*Above: RooDe determines direction by tracking movement across zones.*

### Entry:
1. Person triggers `top_zone`
2. Then triggers `bottom_zone` (within time window)
3. → Count increases (+1)

### Exit:
1. Person triggers `bottom_zone`
2. Then triggers `top_zone`
3. → Count decreases (−1)

---

## 🛡️ Protection & Filtering Logic

| Safeguard Type        | Description |
|-----------------------|-------------|
| Timeout Guard         | Rejects incomplete crossings |
| Movement Delay        | Prevents double-counting during slow passage |
| Distance Thresholds   | Ignores false reflections or pets |
| Startup Failsafe      | Disables xshut/interrupt if pin test fails |
| Cooldown Window       | Buffer time between valid transitions |

---

## 🧪 Debug & Diagnostics

| Entity                        | Description |
|-------------------------------|-------------|
| `sensor.roode_entry_count`    | People entered |
| `sensor.roode_exit_count`     | People exited |
| `sensor.roode_occupancy`      | Net count |
| `sensor.roode_zone_top`       | Distance for top ROI |
| `sensor.roode_zone_bottom`    | Distance for bottom ROI |
| `text_sensor.roode_status`    | Current detection status |

---

## 📘 Configuration References

- [`peopleCounter32.yaml`](peopleCounter32.yaml)  
- [`extra_sensors_example.yaml`](extra_sensors_example.yaml)  
- [`home_assistant.md`](home_assistant.md)

---

## ❓ FAQ

**Q: What happens if the xshut or interrupt pin isn't connected properly?**  
A: RooDe automatically disables features relying on that pin and logs a warning so the sensor can still function.

**Q: Can I use it without the interrupt pin?**  
A: Yes, but using interrupts reduces CPU usage by avoiding constant polling.

**Q: I’m getting unexpected counts. What should I check?**  
A: Increase the ROI threshold or adjust the transition timing. Also verify that your zones are correctly aligned with the physical doorway.

**Q: Is this accurate for fast movements or groups?**  
A: It works best for one person at a time. Rapid or overlapping movement may cause miscounts without tuning.

---

## 💬 Support

Need help or want to contribute?  
👉 [Join our Discord](https://discord.gg/hU9SvSXMHs)

---

## 📜 License

MIT – see [LICENSE](LICENSE)
