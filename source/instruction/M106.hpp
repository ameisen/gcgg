#pragma once

#include "instruction.hpp"
#include "gcode/command.hpp"

namespace gcgg::instructions
{
  // Set Fan On
  class M106 final : public instruction
  {
  public:
    static constexpr const uint64 type = hash("M106");

  protected:
    uint number_ = 0;
    uint speed_ = 0;

  public:
    M106(const gc::command & __restrict cmd) : instruction(type),
      number_(cmd.get_argument("P", 0)),
      speed_(cmd.get_argument("S", 255))
    {
    }
    virtual ~M106() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "fan_on";

      char buffer[512];
      sprintf(buffer, "%u", number_);
      out += " P";
      out += buffer;
      sprintf(buffer, "%u", speed_);
      out += " S";
      out += buffer;

      return out;
    }

    uint get_speed() const __restrict
    {
      return speed_;
    }

    uint get_number() const __restrict
    {
      return number_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      state.fan_speeds[number_] = speed_;

      out += "M106";

      char buffer[512];
      if (number_ != 0)
      {
        sprintf(buffer, "%u", number_);
        out += " P";
        out += buffer;
      }
      if (speed_ != 255)
      {
        sprintf(buffer, "%u", speed_);
        out += " S";
        out += buffer;
      }

      out += "\n";
    }
  };
}
