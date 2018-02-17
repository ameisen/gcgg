#include "gcgg.hpp"
#include "gcode/gcode.hpp"
#include "output/gcode/gcode_out.hpp"

int main(int argc, const char * const __restrict * const __restrict argv)
{
  //static const char dummy_file[] = "C:\\Users\\mkuklinski\\Documents\\MSP_1-100 Tiger II-1.gcode";
  //static const char out_file[] = "C:\\Users\\mkuklinski\\Documents\\OPT_1-100 Tiger II-1.gcode";
  static const char dummy_file[] = "H:\\MSP_test_cube_holes.gcode";
  static const char out_file[] = "H:\\OPT_test_cube_holes.gcode";

  gcode _gc = { dummy_file };

  config cfg;

  auto commands = _gc.process(cfg);

  //for (const auto &cmd : commands)
  //{
  //  printf("%s\n", cmd->dump().c_str());
  //}

  printf("Outputing...\n");
  output::write_gcode(out_file, commands, cfg);

  return 0;
}
