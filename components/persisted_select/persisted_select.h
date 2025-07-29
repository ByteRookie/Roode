#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include <algorithm>

namespace esphome {
namespace select {

class PersistedSelect : public select::Select, public Component {
 public:
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void setup() override;

 protected:
  void control(const std::string &value) override;
  ESPPreferenceObject pref_;
};

}  // namespace select
}  // namespace esphome
