#include "persisted_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace select {

void PersistedSelect::control(const std::string &value) {
  this->publish_state(value);
  this->pref_.save(&value);
}

void PersistedSelect::setup() {
  this->pref_ = global_preferences->make_preference<std::string>(this->get_object_id_hash());
  std::string value;
  if (this->pref_.load(&value)) {
    ESP_LOGI("select", "'%s': Restored state %s", this->get_name().c_str(), value.c_str());
  } else if (!this->traits.get_options().empty()) {
    value = this->traits.get_options().front();
  }
  this->publish_state(value);
}

}  // namespace select
}  // namespace esphome
