#include "persisted_select.h"
#include "esphome/core/log.h"
#include <string>

namespace esphome {
namespace select {

void PersistedSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->restore_value_) {
    this->pref_.save(&value);
  }
}

void PersistedSelect::setup() {
  std::string value;
  if (this->restore_value_) {
    this->pref_ = global_preferences->make_preference<std::string>(this->get_object_id_hash());
    if (this->pref_.load(&value)) {
      ESP_LOGI("select", "'%s': Restored state %s", this->get_name().c_str(), value.c_str());
    } else if (!this->traits.get_options().empty()) {
      ESP_LOGI("select", "'%s': No previous state found", this->get_name().c_str());
      value = this->traits.get_options().front();
    }
  } else if (!this->traits.get_options().empty()) {
    value = this->traits.get_options().front();
  }
  if (!value.empty())
    this->publish_state(value);
}

}  // namespace select
}  // namespace esphome
