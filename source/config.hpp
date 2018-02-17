#pragma once

namespace gcgg
{
  struct config final
  {
    struct
    {
      real epsilon = 0.1; // 0.02;
    } extrusion;

    struct
    {
      bool generate = false;
      real min_angle = 10.0; // 45 degrees
      real radius = 0.1; // The radius of the circle of the arc. Also equal to how much of a linear segment is 'cut off' from the corner.
      real min_radius = 0.05; // might need to be smaller... or larger.
      real gcode_segment_modulus = 45.0; // per angle difference, how many subdivisions to generate. // Values > 180 guarantee that all arcs will generate a single linear segment.
    } arc;

    enum class format
    {
      gcode = 0,
      gcode2
    };

    struct
    {
      format format = format::gcode;
      bool subdivide_arcs = true; // Should we turn arcs into sets of segments?
    } output;
  };
}
