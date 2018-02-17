#pragma once

#include "instruction.hpp"
#include "gcode/command.hpp"

namespace gcgg::instructions
{
  class M104 final : public instruction
  {
  public:
    static constexpr const uint64 type = hash("M104");

  protected:
    uint number_;
    uint temperature_;

  public:
    M104(const gc::command & __restrict cmd) : instruction(type),
      number_(cmd.get_argument<uint>("P", uint(0))),
      temperature_(cmd.get_argument<uint>("S", uint(-1)))
    {
      if (!cmd.has_argument("S"))
      {
        printf("M104 command is missing S argument\n");
      }
    }
    virtual ~M104() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "set_extruder_temperature";

      char buffer[512];
      sprintf(buffer, "%u", number_);
      out += " P";
      out += buffer;
      sprintf(buffer, "%u", temperature_);
      out += " S";
      out += buffer;

      return out;
    }

    uint get_temperature() const __restrict
    {
      return temperature_;
    }

    uint get_number() const __restrict
    {
      return number_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      state.extruder_temp[number_] = get_temperature();

      out += "M104";

      char buffer[512];
      if (number_ != 0)
      {
        sprintf(buffer, "%u", number_);
        out += " P";
        out += buffer;
      }
      sprintf(buffer, "%u", temperature_);
      out += " S";
      out += buffer;

      out += "\n";
    }
  };
}
