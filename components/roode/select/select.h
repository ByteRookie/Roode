#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "../roode.h"

namespace esphome {
namespace roode {

class SensorModeSelect : public Component, public select::Select, public Parented<Roode> {
 public:
  void setup() override;
  void control(const std::string &value) override;
};

}  // namespace roode
}  // namespace esphome
