#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include <string>

namespace esphome {
namespace persisted_select {

class PersistedSelect : public select::Select, public Component {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_restore_value(bool restore) { this->restore_value_ = restore; }
  void setup() override;

 protected:
  void control(const std::string &value) override;
  bool restore_value_{false};
  ESPPreferenceObject pref_;
};

}  // namespace persisted_select
}  // namespace esphome
