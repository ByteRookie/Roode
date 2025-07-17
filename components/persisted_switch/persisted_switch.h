#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace switch_ {

class PersistedSwitch : public switch_::Switch, public Component {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_restore_value(bool restore) { this->restore_value_ = restore; }
  void setup() override;

 protected:
  void write_state(bool state) override;
  bool restore_value_{false};
  ESPPreferenceObject pref_;
};

}  // namespace switch_
}  // namespace esphome
