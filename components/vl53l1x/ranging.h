#pragma once
#include "VL53L1X_ULD.h"

namespace esphome {
namespace vl53l1x {

struct RangingMode {
  explicit RangingMode(const char * name, uint16_t timing_budget, EDistanceMode mode = EDistanceMode::Long)
      : name{name}, timing_budget{timing_budget}, mode{mode} {}

  const char *name;
  uint16_t const timing_budget;
  uint16_t const delay_between_measurements = timing_budget + 5;
  EDistanceMode const mode;
};

namespace Ranging {
inline const RangingMode kShortest{"Shortest", 15, Short};
inline const RangingMode kShort{"Short", 20};
inline const RangingMode kMedium{"Medium", 33};
inline const RangingMode kLong{"Long", 50};
inline const RangingMode kLonger{"Longer", 100};
inline const RangingMode kLongest{"Longest", 200};

__attribute__((unused)) inline const RangingMode *Shortest = &kShortest;
__attribute__((unused)) inline const RangingMode *Short = &kShort;
__attribute__((unused)) inline const RangingMode *Medium = &kMedium;
__attribute__((unused)) inline const RangingMode *Long = &kLong;
__attribute__((unused)) inline const RangingMode *Longer = &kLonger;
__attribute__((unused)) inline const RangingMode *Longest = &kLongest;
}  // namespace Ranging

}  // namespace vl53l1x
}  // namespace esphome
