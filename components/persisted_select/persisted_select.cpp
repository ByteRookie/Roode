#include "persisted_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace select {

void PersistedSelect::control(const std::string &value) {
  this->publish_state(value);
  const auto &options = this->traits.get_options();
  auto it = std::find(options.begin(), options.end(), value);
  uint8_t idx = it == options.end() ? 0 : static_cast<uint8_t>(it - options.begin());
  this->pref_.save(&idx);
}

void PersistedSelect::setup() {
  const auto &options = this->traits.get_options();
  this->pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash());
  uint8_t idx = 0;
  if (this->pref_.load(&idx) && idx < options.size()) {
    ESP_LOGI("select", "'%s': Restored option %u", this->get_name().c_str(), idx);
  } else {
    idx = 0;
  }
  if (!options.empty())
    this->publish_state(options[idx]);
}

}  // namespace select
}  // namespace esphome
