#pragma once

#include "command.hpp"
#include "config.hpp"

namespace gcgg::output
{
  extern bool write_gcode(const std::string & __restrict filename, const std::vector<gcgg::command *> & __restrict commands, const config & __restrict cfg);
}
