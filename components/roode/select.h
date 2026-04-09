#pragma once
#include "esphome/components/select/select.h"

namespace esphome {
namespace roode {

class Roode;  // forward declaration avoids circular include

/**
 * ESPHome select entity that exposes the active filter mode at runtime.
 * Options: "min", "median", "percentile10"
 * The selection is persisted to flash when calibration_persistence is enabled.
 */
class FilterModeSelect : public esphome::select::Select {
 public:
  void set_roode_hub(Roode *hub) { hub_ = hub; }

 protected:
  void control(const std::string &value) override;

 private:
  Roode *hub_{nullptr};
};

}  // namespace roode
}  // namespace esphome
