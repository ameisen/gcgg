#pragma once

#include "movement.hpp"

namespace gcgg::segments
{
  // Linear movement with extrusion
  class extrusion_move final : public movement
  {
  public:
    static constexpr const uint64 type = hash("extrusion_move");

  protected:
    real extrude_ = 0.0;

  public:
    extrusion_move() : movement(type) {}
    virtual ~extrusion_move() {}

    void set_extrude(real extrude) __restrict
    {
      extrude_ = extrude;
    }

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "extrusion_move";
      char buffer[512];
      sprintf(buffer, "%f", extrude_);
      out += " E";
      out += buffer;

      out += movement::dump();

      return out;
    }

    real get_extrusion() const __restrict
    {
      return extrude_;
    }

    void set_extrusion(real extrusion) __restrict
    {
      extrude_ = extrusion;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      if (acceleration_hint_ != state.print_accel && acceleration_hint_ != 0)
      {
        state.print_accel = acceleration_hint_;

        out += "M204";

        char buffer[512];
        sprintf(buffer, "%.8f", acceleration_hint_);
        out += " P";
        out += trim_float(buffer);
        out += "\n";
      }

      bool emit_jerk_hint = (jerk_extrude_hint_ != state.extrude_jerk) && jerk_extrude_hint_ != 0;
      if (start_position_.x != end_position_.x && jerk_hint_.x != state.jerk.x && jerk_hint_.x != 0)
      {
        emit_jerk_hint = true;
      }
      if (start_position_.y != end_position_.y && jerk_hint_.y != state.jerk.y && jerk_hint_.y != 0)
      {
        emit_jerk_hint = true;
      }
      if (start_position_.z != end_position_.z && jerk_hint_.z != state.jerk.z && jerk_hint_.z != 0)
      {
        emit_jerk_hint = true;
      }

      if (emit_jerk_hint)
      {
        out += "M205";
        char buffer[512];
        if (jerk_extrude_hint_ != state.extrude_jerk && jerk_extrude_hint_ != 0)
        {
          state.extrude_jerk = jerk_extrude_hint_;
          sprintf(buffer, "%.8f", jerk_extrude_hint_);
          out += " E";
          out += trim_float(buffer);
        }
        if (jerk_hint_ != state.jerk)
        {
          if (start_position_.x != end_position_.x && jerk_hint_.x != state.jerk.x && jerk_hint_.x != 0)
          {
            state.jerk.x = jerk_hint_.x;
            sprintf(buffer, "%.8f", jerk_hint_.x);
            out += " X";
            out += trim_float(buffer);
          }
          if (start_position_.y != end_position_.y && jerk_hint_.y != state.jerk.y && jerk_hint_.y != 0)
          {
            state.jerk.y = jerk_hint_.y;
            sprintf(buffer, "%.8f", jerk_hint_.y);
            out += " Y";
            out += trim_float(buffer);
          }
          if (start_position_.z != end_position_.z && jerk_hint_.z != state.jerk.z && jerk_hint_.z != 0)
          {
            state.jerk.z = jerk_hint_.z;
            sprintf(buffer, "%.8f", jerk_hint_.z);
            out += " Z";
            out += trim_float(buffer);
          }
        }
        out += "\n";
      }

      out += "G1";

      char buffer[512];
      sprintf(buffer, "%.8f", extrude_);
      out += " E";
      out += trim_float(buffer);

      if (start_position_.x != end_position_.x)
      {
        state.position.x = end_position_.x;
        sprintf(buffer, "%.8f", state.position.x);
        out += " X";
        out += trim_float(buffer);
      }
      if (start_position_.y != end_position_.y)
      {
        state.position.y = end_position_.y;
        sprintf(buffer, "%.8f", state.position.y);
        out += " Y";
        out += trim_float(buffer);
      }
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
