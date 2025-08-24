#pragma once

#include "zone.h"
#include "../vl53l1x/vl53l1x.h"
#include "esphome/core/log.h"

namespace esphome {
namespace roode {

using TofSensor = esphome::vl53l1x::VL53L1X;

class Configuration {
 public:
  explicit Configuration(TofSensor *sensor) : sensor_(sensor) {}

  Zone getZoneConfiguration(uint8_t zone);
  int getTimingBudget() const { return timing_budget_; }

 private:
  void setCorrectDistanceSettings(float average_entry_zone_distance, float average_exit_zone_distance);
  void publishSensorConfiguration(int dist_threshold_arr[2], bool isMax);
  void setSensorMode(int sensor_mode, int timing_budget = 0);

  TofSensor *sensor_{nullptr};
  int timing_budget_{0};
};

}  // namespace roode
}  // namespace esphome

