#pragma once

#include "instruction.hpp"
#include "gcode/command.hpp"

namespace gcgg::instructions
{
  // Set Fan Off
  class M107 final : public instruction
  {
  public:
    static constexpr const uint64 type = hash("M107");

  protected:
    uint number_ = 0;

  public:
    M107(const gc::command & __restrict cmd) : instruction(type),
      number_(cmd.get_argument("P", 0))
    {
    }
    virtual ~M107() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "fan_off";

      char buffer[512];
      sprintf(buffer, "%u", number_);
      out += " P";
      out += buffer;

      return out;
    }

    uint get_number() const __restrict
    {
      return number_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      state.fan_speeds[number_] = 0;

      out += "M107";

      char buffer[512];
      if (number_ != 0)
      {
        sprintf(buffer, "%u", number_);
        out += " P";
        out += buffer;
      }

      out += "\n";
    }
  };
}
