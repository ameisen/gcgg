#pragma once

#include "command.hpp"

namespace gcgg::instructions
{
  class instruction : public command
  {
  public:
    instruction(uint64 type, bool delay = false) : command(type, delay) {}
    virtual ~instruction() {}

    virtual bool is_segment() const __restrict override final { return false; }
    virtual bool is_instruction() const __restrict override final { return true; }
  };
}
