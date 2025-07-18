#include "persisted_select.h"
#include "esphome/core/log.h"
#include <string>

namespace esphome {
namespace persisted_select {

void PersistedSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->restore_value_) {
    const auto &options = this->traits.get_options();
    uint8_t idx = 0;
    for (size_t i = 0; i < options.size(); i++) {
      if (options[i] == value) {
        idx = i;
        break;
      }
    }
    this->pref_.save(&idx);
  }
}

void PersistedSelect::setup() {
  std::string value;
  if (this->restore_value_) {
    this->pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash());
    uint8_t idx;
    if (this->pref_.load(&idx)) {
      const auto &options = this->traits.get_options();
      if (idx < options.size())
        value = options[idx];
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

}  // namespace persisted_select
}  // namespace esphome
