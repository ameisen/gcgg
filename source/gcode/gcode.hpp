#pragma once

#include <unordered_map>
#include <command.hpp>
#include "gcode/command.hpp"
#include "config.hpp"

namespace gcgg
{
  class gcode final
  {
    using command_token = std::vector<std::string>;
    using token_vector = std::vector<command_token>;

    static token_vector tokenize(const std::vector<char> &__restrict data);
    static std::vector<gc::command> parse(const token_vector & __restrict tokens);

    std::vector<gc::command> commands_;

  public:
    gcode(const std::string & __restrict filename);
    ~gcode();

    std::vector<gcgg::command *> process(const config & __restrict cfg) const __restrict;
  };
}
