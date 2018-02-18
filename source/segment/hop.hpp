#pragma once

#include "movement.hpp"

namespace gcgg::segments
{
  // Z-only movement without extrusion
  class hop final : public movement
  {
  public:
    static constexpr const uint64 type = hash("hop");

  protected:
  public:
    hop() : movement(type)
    {
      is_travel_ = true;
    }
    virtual ~hop() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "hop";

      out += movement::dump();

      return out;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      if (acceleration_hint_ != state.travel_accel && acceleration_hint_ != 0)
      {
        state.travel_accel = acceleration_hint_;

        out += "M204";

        char buffer[512];
        sprintf(buffer, "%.8f", acceleration_hint_);
        out += " T";
        out += trim_float(buffer);
        out += "\n";
      }

      if (jerk_hint_.z != state.jerk.z && start_position_.z != end_position_.z && jerk_hint_.z != 0)
      {
        out += "M205";
        char buffer[512];
        state.jerk.z = jerk_hint_.z;
        sprintf(buffer, "%.8f", jerk_hint_.z);
        out += " Z";
        out += trim_float(buffer);
        out += "\n";
      }

      out += "G0";

      char buffer[512];

      if (start_position_.z != end_position_.z)
      {
        state.position.z = end_position_.z;
        sprintf(buffer, "%.8f", state.position.z);
        out += " Z";
        out += trim_float(buffer);
      }

      if (cfg.output.format == config::format::gcode)
      {
        if (feedrate_ != state.feedrate)
        {
          state.feedrate = feedrate_;

          sprintf(buffer, "%.8f", feedrate_);
          out += " F";
          out += trim_float(buffer);
        }
      }
      else
      {
        if (motion_data_.plateau_feedrate_ != state.feedrate)
        {
          state.feedrate = motion_data_.plateau_feedrate_;

          sprintf(buffer, "%.8f", motion_data_.plateau_feedrate_);
          out += " F";
          out += trim_float(buffer);
        }

        if (motion_data_.exit_feedrate_ != state.feedrate)
        {
          sprintf(buffer, "%.8f", motion_data_.exit_feedrate_);
          out += " A";
          out += trim_float(buffer);
        }
      }

      out += "\n";
    }
  };
}
