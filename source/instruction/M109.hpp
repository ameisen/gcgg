#pragma once

#include "delay_instruction.hpp"
#include "gcode/command.hpp"

namespace gcgg::instructions
{
  // Set Extruder Temperature and Wait
  class M109 final : public delay_instruction
  {
  public:
    static constexpr const uint64 type = hash("M109");

  protected:
    uint number_ = 0;
    uint minimum_target_ = uint(-1);
    uint accurate_target_ = uint(-1);

  public:
    M109(const gc::command & __restrict cmd) : delay_instruction(type),
      number_(cmd.get_argument("P", 0)),
      minimum_target_(cmd.get_argument("S", uint(-1))),
      accurate_target_(cmd.get_argument("R", uint(-1)))
    {
      // TODO throw error for invalid input.
    }
    virtual ~M109() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "set_extruder_temp_wait";

      char buffer[512];
      sprintf(buffer, "%u", number_);
      out += " P";
      out += buffer;
      if (minimum_target_ == uint(-1))
      {
        sprintf(buffer, "%u", minimum_target_);
        out += " S";
        out += buffer;
      }
      if (accurate_target_ == uint(-1))
      {
        sprintf(buffer, "%u", accurate_target_);
        out += " R";
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
      if (accurate_target_ != uint(-1))
      {
        return accurate_target_;
      }
      return minimum_target_;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      state.extruder_temp[number_] = get_temperature();

      out += "M109";

      char buffer[512];
      if (number_ != 0)
      {
        sprintf(buffer, "%u", number_);
        out += " P";
        out += buffer;
      }
      if (minimum_target_ != uint(-1))
      {
        sprintf(buffer, "%u", minimum_target_);
        out += " S";
        out += buffer;
      }
      if (accurate_target_ != uint(-1))
      {
        sprintf(buffer, "%u", accurate_target_);
        out += " R";
        out += buffer;
      }

      out += "\n";
    }
  };
}
