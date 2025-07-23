#include "select.h"

namespace esphome {
namespace roode {

void SensorModeSelect::setup() {
  if (this->parent_ == nullptr)
    return;
  // Publish initial state based on parent's mode
  auto mode = this->parent_->get_sensor_mode();
  auto opt = this->at(static_cast<size_t>(mode));
  if (opt.has_value())
    this->publish_state(opt.value());
}

void SensorModeSelect::control(const std::string &value) {
  if (this->parent_ == nullptr)
    return;
  if (value == "no_sensors")
    this->parent_->set_sensor_mode(Roode::SENSOR_MODE_NONE);
  else
    this->parent_->set_sensor_mode(Roode::SENSOR_MODE_ALL);
}

}  // namespace roode
}  // namespace esphome
