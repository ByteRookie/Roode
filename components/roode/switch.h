#pragma once
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace roode {

class Roode;  // forward declaration avoids circular include

/**
 * ESPHome switch that inverts the entry/exit counting direction at runtime.
 * Persisted to flash (key 0xBD) when calibration_persistence is enabled.
 */
class InvertDirectionSwitch : public esphome::switch_::Switch {
 public:
  void set_roode_hub(Roode *hub) { hub_ = hub; }

 protected:
  void write_state(bool state) override;

 private:
  Roode *hub_{nullptr};
};

/**
 * ESPHome switch that enables or disables saving calibration data to flash.
 * When turned off, settings survive only until the next reboot.
 */
class CalibrationPersistenceSwitch : public esphome::switch_::Switch {
 public:
  void set_roode_hub(Roode *hub) { hub_ = hub; }

 protected:
  void write_state(bool state) override;

 private:
  Roode *hub_{nullptr};
};

}  // namespace roode
}  // namespace esphome
