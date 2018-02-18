#pragma once

#include "gcgg.hpp"
#include "config.hpp"

namespace gcgg::motion
{
  class trapezoid final
  {
  public:
    struct data final
    {
      vector3<> start_position_;
      vector3<> end_position_;
      real start_speed_;
      real speed_;
      real end_speed_;
      vector3<> acceleration_;
      vector3<> jerk_;
    };

  public:
    vector3<> start_position_;
    vector3<> end_position_;
    real plateau_speed_;
    real start_speed_;
    real end_speed_;
    real ramp_time_[2];
    real ramp_distance_[2];
    real plateau_time_;
    real plateau_distance_;

    trapezoid(const data & __restrict init);
  };
}
