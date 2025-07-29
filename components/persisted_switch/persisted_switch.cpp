#include "persisted_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace switch_ {

void PersistedSwitch::write_state(bool state) {
  this->publish_state(state);
  this->pref_.save(&state);
}

void PersistedSwitch::setup() {
  this->pref_ = global_preferences->make_preference<bool>(this->get_object_id_hash());
  bool value{false};
  if (this->pref_.load(&value)) {
    ESP_LOGI("switch", "'%s': Restored state %s", this->get_name().c_str(), ONOFF(value));
  }
  this->publish_state(value);
}

}  // namespace switch_
}  // namespace esphome
