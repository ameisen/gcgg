#pragma once

#include <unordered_map>

namespace gcgg::output
{
  struct state
  {
    real feedrate = 0.0;
    real print_accel = 0.0;
    real travel_accel = 0.0;
    real retract_accel = 0.0;
    vector3<> jerk;
    real extrude_jerk;
    std::unordered_map<uint, uint> extruder_temp;
    std::unordered_map<uint, uint> bed_temp;
    std::unordered_map<uint, uint> fan_speeds;

    vector3<> position;
    vector3<> prev_position;
  };
}
