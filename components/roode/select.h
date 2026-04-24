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

/**
 * ESPHome select entity that exposes the VL53L1X ranging mode at runtime.
 * Options: "auto", "short", "medium", "long", "longer", "longest"
 * "auto" means Roode will pick the best mode based on measured idle distance.
 * The selection is persisted to flash when calibration_persistence is enabled.
 */
class RangingModeSelect : public esphome::select::Select {
 public:
  void set_roode_hub(Roode *hub) { hub_ = hub; }

 protected:
  void control(const std::string &value) override;

 private:
  Roode *hub_{nullptr};
};

/**
 * ESPHome select entity that exposes the sensor orientation at runtime.
 * Options: "parallel", "perpendicular"
 * Changing the orientation resets ROI centers and triggers a full recalibration.
 * The selection is persisted to flash when calibration_persistence is enabled.
 */
class OrientationSelect : public esphome::select::Select {
 public:
  void set_roode_hub(Roode *hub) { hub_ = hub; }

 protected:
  void control(const std::string &value) override;

 private:
  Roode *hub_{nullptr};
};

}  // namespace roode
}  // namespace esphome
