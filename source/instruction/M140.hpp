#pragma once

#include "delay_instruction.hpp"
#include "gcode/command.hpp"

namespace gcgg::instructions
{
  // Set Bed Temperature
  class M140 final : public delay_instruction
  {
  public:
    static constexpr const uint64 type = hash("M140");

  protected:
    // TODO handle R (bed standby temperature)
    uint number_ = 0;
    uint temperature_ = uint(-1);

  public:
    M140(const gc::command & __restrict cmd) : delay_instruction(type),
      number_(cmd.get_argument("H", 0)),
      temperature_(cmd.get_argument("S", uint(-1)))
    {
      // TODO throw error for invalid input.
    }
    virtual ~M140() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "set_bed_temperature";

      char buffer[512];
      sprintf(buffer, "%u", number_);
      out += " P";
      out += buffer;
      if (temperature_ == uint(-1))
      {
        sprintf(buffer, "%u", temperature_);
        out += " S";
        out += buffer;
      }

      return out;
    }

    uint get_number() const __restrict
    {
      return number_;
    }

    uint get_temperature() const __restrict
    {
      return temperature_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      state.bed_temp[number_] = get_temperature();

      out += "M140";

      char buffer[512];
      if (number_ != 0)
      {
        sprintf(buffer, "%u", number_);
        out += " P";
        out += buffer;
      }
      if (temperature_ != uint(-1))
      {
        sprintf(buffer, "%u", temperature_);
        out += " S";
        out += buffer;
      }

      out += "\n";
    }
  };
}
