#pragma once

#include "movement.hpp"

namespace gcgg::segments
{
  // Extrusion-only move with no XYZ movement
  // It inherits from movement so it can use accel/jerk hints.
  class extrusion final : public movement
  {
  public:
    static constexpr const uint64 type = hash("extrusion");
  protected:

    real extrude_ = 0.0;
    real feedrate_ = 0.0;

  public:
    extrusion() : movement(type) {}
    virtual ~extrusion() {}

    void set_extrude(real extrude) __restrict
    {
      extrude_ = extrude;
    }

    void set_feedrate(real feedrate) __restrict
    {
      feedrate_ = feedrate;
    }

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "extrusion";
      char buffer[512];
      sprintf(buffer, "%f", extrude_);
      out += " E";
      out += trim_float(buffer);
      sprintf(buffer, "%f", feedrate_);
      out += " F";
      out += trim_float(buffer);

      return out;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      const bool run_M205 =
        (acceleration_hint_ != state.retract_accel && acceleration_hint_ != 0) ||
        (jerk_extrude_hint_ != state.extrude_jerk && jerk_extrude_hint_ != 0);

      if (run_M205)
      {
        out += "M205";
        char buffer[512];

        if (acceleration_hint_ != state.retract_accel && acceleration_hint_ != 0)
        {
          state.retract_accel = acceleration_hint_;

          sprintf(buffer, "%.8f", acceleration_hint_);
          out += " R";
          out += trim_float(buffer);
        }

        if (jerk_extrude_hint_ != state.extrude_jerk && jerk_extrude_hint_ != 0)
        {
          state.extrude_jerk = jerk_extrude_hint_;

          char buffer[512];
          sprintf(buffer, "%.8f", jerk_extrude_hint_);
          out += " E";
          out += trim_float(buffer);
        }
        out += "\n";
      }

      out += "G1";

      char buffer[512];
      sprintf(buffer, "%.8f", extrude_);
      out += " E";
      out += trim_float(buffer);

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
