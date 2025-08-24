#include "configuration.h"

namespace esphome {
namespace roode {

static const char *const TAG = "Configuration";

Zone Configuration::getZoneConfiguration(uint8_t zone) {
  // Create a zone object with default settings.
  Zone cfg(zone, this);
  return cfg;
}

void Configuration::setCorrectDistanceSettings(float average_entry_zone_distance,
                                               float average_exit_zone_distance) {
  float avg = (average_entry_zone_distance + average_exit_zone_distance) / 2.0f;
  // Choose a ranging profile based on average distance.
  // Values are approximations that map distance to one of the predefined modes.
  if (avg < 1300.0f) {
    setSensorMode(1);  // Shortest range
  } else if (avg < 2000.0f) {
    setSensorMode(2);  // Short
  } else if (avg < 3000.0f) {
    setSensorMode(3);  // Medium
  } else if (avg < 4000.0f) {
    setSensorMode(4);  // Long
  } else {
    setSensorMode(5);  // Longer distances
  }
}

void Configuration::publishSensorConfiguration(int dist_threshold_arr[2], bool isMax) {
  ESP_LOGD(TAG, "publishSensorConfiguration %s entry=%d exit=%d", isMax ? "max" : "min",
           dist_threshold_arr[0], dist_threshold_arr[1]);
}

void Configuration::setSensorMode(int sensor_mode, int timing_budget) {
  using namespace esphome::vl53l1x;
  const RangingMode *mode = nullptr;
  switch (sensor_mode) {
    case 1:
      mode = Ranging::Shortest;
      break;
    case 2:
      mode = Ranging::Short;
      break;
    case 3:
      mode = Ranging::Medium;
      break;
    case 4:
      mode = Ranging::Long;
      break;
    case 5:
      mode = Ranging::Longer;
      break;
    case 6:
      mode = Ranging::Longest;
      break;
    default:
      mode = Ranging::Long;
      break;
  }

  if (timing_budget > 0 && timing_budget != mode->timing_budget) {
    // Create a custom ranging mode with the requested timing budget.
    mode = new RangingMode("custom", static_cast<uint16_t>(timing_budget), mode->mode);
  }

  if (sensor_ != nullptr && mode != nullptr) {
    sensor_->set_ranging_mode(mode);
    timing_budget_ = mode->timing_budget;
  }
}

}  // namespace roode
}  // namespace esphome

