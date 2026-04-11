#pragma once
#include "esphome/components/number/number.h"

namespace esphome {
namespace roode {

class Roode;  // forward declaration avoids circular include

/**
 * ESPHome number entity that maps to a Roode runtime setting.
 * One class covers all 11 number entities via the Setting enum.
 * control() dispatches to the matching apply_* method on Roode.
 */
class RoodeSettingNumber : public esphome::number::Number {
 public:
  enum Setting {
    FILTER_WINDOW,
    SAMPLING,
    ENTRY_MAX_PCT,
    ENTRY_MIN_PCT,
    EXIT_MAX_PCT,
    EXIT_MIN_PCT,
    AUTO_CAL_INTERVAL_HOURS,
    ENTRY_ROI_HEIGHT,
    ENTRY_ROI_WIDTH,
    EXIT_ROI_HEIGHT,
    EXIT_ROI_WIDTH,
  };

  void set_roode_hub(Roode *hub) { hub_ = hub; }
  void set_setting(Setting s) { setting_ = s; }

 protected:
  void control(float value) override;

 private:
  Roode *hub_{nullptr};
  Setting setting_{FILTER_WINDOW};
};

}  // namespace roode
}  // namespace esphome
