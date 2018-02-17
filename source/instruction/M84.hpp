#pragma once

#include "instruction.hpp"

namespace gcgg::instructions
{
  // Stop Idle Hold
  class M84 final : public instruction
  {
  public:
    static constexpr const uint64 type = hash("M84");

  protected:
    uint delay_ = 0;

  public:
    M84(const gc::command & __restrict cmd) : instruction(type),
      delay_(cmd.get_argument<uint>("S", uint(0)))
    {}
    virtual ~M84() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "disable_steppers";
      char buffer[512];
      sprintf(buffer, "%u", delay_);
      out += " S";
      out += buffer;
      return out;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      out += "M84";
      char buffer[512];
      if (delay_ != 0)
      {
        sprintf(buffer, "%u", delay_);
        out += " S";
        out += buffer;
      }
      out += "\n";
    }
  };
}
