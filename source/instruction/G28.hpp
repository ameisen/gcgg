#pragma once

#include "delay_instruction.hpp"

namespace gcgg::instructions
{
  // Set extruder to absolute mode. This can and will be elided in non-gcode output.
  class G28 final : public delay_instruction
  {
  public:
    static constexpr const uint64 type = hash("G28");

  protected:
    vector3<bool> home_axis_;

  public:
    G28(const gc::command & __restrict cmd) : delay_instruction(type),
      home_axis_(
        cmd.has_argument("X"),
        cmd.has_argument("Y"),
        cmd.has_argument("Z")
      )
    {
      if (!home_axis_.x && !home_axis_.y && !home_axis_.z)
      {
        home_axis_ = { true, true, true };
      }
    }
    virtual ~G28() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "Home";

      if (home_axis_.x)
      {
        out += " X";
      }
      if (home_axis_.y)
      {
        out += " Y";
      }
      if (home_axis_.z)
      {
        out += " Z";
      }

      return out;
    }

    const vector3<bool> & axis() const __restrict
    {
      return home_axis_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      out += "G28";
      if (!home_axis_.x || !home_axis_.y || !home_axis_.z)
      {
        if (home_axis_.x)
        {
          out += " X";
        }
        if (home_axis_.y)
        {
          out += " Y";
        }
        if (home_axis_.z)
        {
          out += " Z";
        }
      }
      if (home_axis_.x)
      {
        state.position.x = 0.0;
      }
      if (home_axis_.y)
      {
        state.position.y = 0.0;
      }
      if (home_axis_.z)
      {
        state.position.z = 0.0;
      }
      out += "\n";
    }
  };
}
