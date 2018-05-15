#include "gcgg.hpp"
#include "gcode/gcode.hpp"
#include "output/gcode/gcode_out.hpp"

int main(int argc, const char * const __restrict * const __restrict argv)
{
  //static const char dummy_file[] = "C:\\Users\\mkuklinski\\Documents\\MSP_bed_carriage.gcode";
  //static const char out_file[] = "C:\\Users\\mkuklinski\\Documents\\OPT_bed_carriage.gcode";
  //static const char dummy_file[] = "H:\\sphere_zhop.gcode";
  //static const char out_file[] = "H:\\sphere_zhop_opt.gcode";
  static const char dummy_file[] = R"(C:\Users\mkuklinski\Documents\test.gcode)";
  static const char out_file[] = R"(C:\Users\mkuklinski\Documents\o_test.gcode)";

  gcode _gc = { dummy_file };

  config cfg;
  //cfg.arc.generate = false;
  //cfg.smoothing.enable = false;

  auto commands = _gc.process(cfg);

  //for (const auto &cmd : commands)
  //{
  //  printf("%s\n", cmd->dump().c_str());
  //}

  printf("Outputing...\n");
  output::write_gcode(out_file, commands, cfg);

  return 0;
}
