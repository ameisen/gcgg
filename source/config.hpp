#pragma once

namespace gcgg
{
  struct config final
  {
    struct
    {
      bool all_no_extrude_as_travel = true;
      bool brute_force_feedrate = true;
    } options;

    struct
    {
      real epsilon = 0.1; // 0.02;
    } extrusion;

    struct
    {
      bool generate = true;
      bool constant_speed = true; // Should the arc only allow a constant speed across it?
      usize max_segments = 1000;
      real max_angle = 150.0;
      real min_angle = 30.0; // 45 degrees
      real radius = 0.1; // The radius of the circle of the arc. Also equal to how much of a linear segment is 'cut off' from the corner.
      real travel_radius = 0.4; // Travels can have a much larger radius.
      bool halve_travels = false;
      real min_radius = 0.05; // might need to be smaller... or larger.
      bool constrain_radius = true; // Should we constrain the radius to the original vertex position?
    } arc;

    struct
    {
      bool enable = true; // Should we try to smooth certain angles no matter what?
      real min_angle = 20.0;;
      real new_angle = 5.0;
    } smoothing;

    enum class format
    {
      gcode = 0,
      gcode2
    };

    struct
    {
      format format = format::gcode;
      bool subdivide_arcs = true; // Should we turn arcs into sets of segments?

      bool generate_G15 = false; // G15 is a custom instruction that generates a movement arc. Not the same as a controlled arc.
    } output;

    struct
    {
      vector3<> acceleration = { 2000, 1500, 400 };
      real extrusion_acceleration = 4000;
      vector3<> feedrate = vector3<>{200.0, 150.0, 80.0} * 60.0;
      real extrusion_feedrate = 200 * 60;
      vector3<> jerk = { 20, 20, 20 };
      real extrusion_jerk = 20;
    } defaults;
  };
}

/*
echo:Filament settings: Disabled
echo:  M200 D1.75
echo:  M200 D0
echo:Steps per unit:
echo:  M92 X160.00 Y200.00 Z400.00 E88.78
echo:Maximum feedrates (units/s):
echo:  M203 X200.00 Y150.00 Z80.00 E200.00
echo:Maximum Acceleration (units/s2):
echo:  M201 X2000 Y1500 Z400 E4000
echo:Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>
echo:  M204 P4000.00 R1200.00 T4000.00
echo:Advanced: S<min_feedrate> T<min_travel_feedrate> B<min_segment_time_ms> X<max_xy_jerk> Z<max_z_jerk> E<max_e_jerk>
echo:  M205 S0.00 T0.00 B20000 X20.00 Y20.00 Z0.00 E0.00
echo:Home offset:
echo:  M206 X0.00 Y0.00 Z0.00
echo:PID settings:
echo:
echo:Retract: S<length> F<units/m> Z<lift>
echo:  M207 S2.00 F12000.00 Z0.00
echo:Recover: S<length> F<units/m>
echo:  M208 S0.00 F12000.00
echo:Auto-Retract: S=0 to disable, 1 to interpret extrude-only moves as retracts or recoveries
echo:  M209 S0
echo:Linear Advance:
echo:  M900 K42.50 R0.00
*/
