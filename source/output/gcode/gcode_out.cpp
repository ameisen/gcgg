#include "gcgg.hpp"
#include "gcode_out.hpp"
#include "output/state.hpp"

bool gcgg::output::write_gcode(const std::string & __restrict filename, const std::vector<gcgg::command *> & __restrict commands, const config & __restrict cfg)
{
  std::string output;
  output::state state;

  // We start by making usre that the printer is in the correct state.
  output += "G21\n"; // Set units to millimeters
  output += "G90\n"; // Absolute Positioning
  output += "M83\n"; // Relative Extrusion



  for (auto * __restrict cmd : commands)
  {
    cmd->out_gcode(output, state, cfg);
  }

  FILE *fp = fopen(filename.c_str(), "wb");
  if (!fp)
  {
    assert(false);
  }

  fwrite(output.c_str(), 1, output.length(), fp);

  fclose(fp);

  return true;
}
