#include "persisted_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace switch_ {

void PersistedSwitch::write_state(bool state) {
  this->publish_state(state);
  if (this->restore_value_) {
    this->pref_.save(&state);
  }
}

void PersistedSwitch::setup() {
  bool value{false};
  if (this->restore_value_) {
    this->pref_ = global_preferences->make_preference<bool>(this->get_object_id_hash());
    if (this->pref_.load(&value)) {
      ESP_LOGI("switch", "'%s': Restored state %s", this->get_name().c_str(), value ? "ON" : "OFF");
    } else {
      ESP_LOGI("switch", "'%s': No previous state found", this->get_name().c_str());
    }
  }
  this->publish_state(value);
}

}  // namespace switch_
}  // namespace esphome
