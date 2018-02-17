#pragma once

#include "instruction.hpp"

namespace gcgg::instructions
{
  // Instruction that can incur a delay, and thus must halt movement.
  class delay_instruction : public instruction
  {
  public:
    delay_instruction(uint64 type) : instruction(type, true) {}
    virtual ~delay_instruction() {}
  };
}
