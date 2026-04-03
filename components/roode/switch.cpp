#include "roode.h"
#include "esphome/core/log.h"

namespace esphome {
namespace roode {

void PortalSwitch::setup() {
  if (parent_ != nullptr) {
    this->publish_state(parent_->is_portal_enabled());
  }
}

void PortalSwitch::write_state(bool state) {
  this->publish_state(state);
  if (parent_ != nullptr) {
    parent_->set_portal_enabled(state);
  }
}

void PortalSwitch::dump_config() {
  LOG_SWITCH("", "Roode Portal Switch", this);
}

}  // namespace roode
}  // namespace esphome
