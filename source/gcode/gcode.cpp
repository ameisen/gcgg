#include "gcgg.hpp"
#include "gcode.hpp"

#include "segment/extrusion.hpp"
#include "segment/extrusion_move.hpp"
#include "segment/hop.hpp"
#include "segment/linear.hpp"
#include "segment/travel.hpp"
#include "segment/arc.hpp"
#include "segment/arc_accumulator.hpp"

#include "instruction/M84.hpp"
#include "instruction/M104.hpp"
#include "instruction/M106.hpp"
#include "instruction/M107.hpp"
#include "instruction/M109.hpp"
#include "instruction/M140.hpp"
#include "instruction/M190.hpp"

#include "instruction/G28.hpp"

#include "config.hpp"

#include <cstdio>
#include <cctype>

#include <list>

gcode::gcode(const std::string &__restrict filename)
{
  FILE *fp = fopen(filename.c_str(), "rb");
  if (!fp)
  {
    xassert(false); // TODO
  }
  fseek(fp, 0, SEEK_END);
  usize file_size = ftell(fp);
  rewind(fp);

  std::vector<char> file_data;
  file_data.resize(file_size);
  fread(file_data.data(), 1, file_size, fp);
  fclose(fp);

  const auto tokens = tokenize(file_data);
  auto parsed_cmds = parse(tokens);
  commands_ = std::move(parsed_cmds);
}

gcode::~gcode()
{
}

namespace
{
  static bool is_newline(char c)
  {
    return c == '\r' || c == '\n';
  }

  static bool is_space(char c)
  {
    return std::isspace(c) != 0;
  }
}

gcode::token_vector gcode::tokenize(const std::vector<char> &__restrict data)
{
  printf("Tokenizing gcode\n");

  static constexpr const char comment_char = ';';

  // Derived by testing with my files. Probably needs to be different for other real world files.
  static constexpr const usize mean_token_length = 24;

  gcode::token_vector out;
  out.reserve(data.size() / mean_token_length);

  command_token cmd_token;
  cmd_token.reserve(10);
  std::string token;
  const auto push_token = [&]()
  {
    if (token.length())
    {
      cmd_token.push_back(token);
      token.clear();
    }
  };

  const auto push_cmd_token = [&]()
  {
    if (cmd_token.size())
    {
      out.push_back(cmd_token);
      cmd_token.clear();
    }
  };

  bool in_comment = false;

  for (const char c : data)
  {
    if (c == comment_char)
    {
      in_comment = true;
    }
    else if (is_newline(c))
    {
      in_comment = false;
      push_token();
      push_cmd_token();
    }
    else if (is_space(c))
    {
      push_token();
    }
    else if (!in_comment)
    {
      token += c;
    }
  }
  push_token();
  push_cmd_token();

  return out;
}

namespace
{
  static bool isalpha(char c)
  {
    return std::isalpha(c) != 0;
  }
}

std::vector<gc::command> gcode::parse(const gcode::token_vector & __restrict tokens)
{
  printf("Parsing tokens\n");

  std::vector<gc::command> out;
  out.reserve(tokens.size());

  for (const auto & __restrict tv : tokens)
  {
    out.emplace_back();

    gc::command &cmd = out.back();
    cmd._cmd_string = tv[0];
    for (usize i = 1; i < tv.size(); ++i)
    {
      const std::string & __restrict token = tv[i];
      std::string key;
      std::string value;
      
      bool in_key = true;
      for (char c : token)
      {
        if (in_key && isalpha(c))
        {
          key += c;
        }
        else
        {
          in_key = false;
          value += c;
        }
      }

      real rvalue = 0.0;
      if (value.length())
      {
        rvalue = strtod(value.c_str(), nullptr);
      }
      cmd._arguments[key] = rvalue;
    }
  }

  return out;
}

std::vector<gcgg::command *> gcode::process(const config & __restrict cfg) const __restrict
{
  printf("Processing...\n");

  // Using a custom vector class would let use implement it using realloc which would likely be substantially faster.
  // This, however, would add another mess of maintenence, so I'm not doing it, at least right now.
  std::vector<gcgg::command *> out;
  // Reserve
  out.reserve(commands_.size() * 20);

  real feedrate = cfg.defaults.feedrate.z;
  real print_accel = cfg.defaults.acceleration.max_element();
  real travel_accel = cfg.defaults.acceleration.max_element();
  real retract_accel = cfg.defaults.extrusion_acceleration;
  vector3<> acceleration = cfg.defaults.acceleration;
  vector3<> jerk = cfg.defaults.jerk;
  real extrude_jerk = cfg.defaults.extrusion_jerk;
  std::unordered_map<uint, uint> extruder_temp;
  std::unordered_map<uint, uint> bed_temp;
  std::unordered_map<uint, uint> fan_speeds;

  vector3<> position;
  bool prev_move_cmd = false;

  bool absolute_mode = true;
  bool relative_extrusion = false;

  bool steppers_enabled = true;

  // We should handle the potential for absolute extrusion values.
  real current_extrusion = 0.0;

  const auto extract_position = [&](const gc::command & __restrict command)
  {
    if (absolute_mode)
    {
      position.x = command.get_argument("X", position.x);
      position.y = command.get_argument("Y", position.y);
      position.z = command.get_argument("Z", position.z);
    }
    else
    {
      position.x += command.get_argument("X", 0.0);
      position.y += command.get_argument("Y", 0.0);
      position.z += command.get_argument("Z", 0.0);
    }
  };

  // Iterate over the commands and generate movement and operation sequences.
  __pragma(warning(disable:4307));
  for (const auto & __restrict command : commands_)
  {
    bool move_cmd = false;

    const vector3<> start_position = position;

    switch (hash(command._cmd_string))
    {
      // Movement Commands
    case hash("G0"): {
      move_cmd = true;

      const bool has_z = command.has_argument_not("Z", position.z);
      const bool has_xy = command.has_argument_not("X", position.x) || command.has_argument_not("Y", position.y);

      feedrate = command.get_argument("F", feedrate);
      extract_position(command);
      
      if (has_xy)
      {
        auto * __restrict cmd = new segments::travel;
        cmd->set_positions(start_position, position);
        cmd->set_feedrate(feedrate);
        cmd->acceleration_hint_ = travel_accel;
        cmd->acceleration_ = acceleration.limit({ travel_accel, travel_accel, travel_accel });
        cmd->jerk_hint_ = jerk;
        cmd->jerk_extrude_hint_ = extrude_jerk;
        out.push_back(cmd);
      }
      else if (has_z)
      {
        auto * __restrict cmd = new segments::hop;
        cmd->set_positions(start_position, position);
        cmd->set_feedrate(feedrate);
        cmd->acceleration_hint_ = travel_accel;
        cmd->acceleration_ = acceleration.limit({ travel_accel, travel_accel, travel_accel });
        cmd->jerk_hint_ = jerk;
        cmd->jerk_extrude_hint_ = extrude_jerk;
        out.push_back(cmd);
      }
      else
      {
        // This is a meaningless command? Might have a feedrate associated, but for movement it won't generate an opcode.
      }
    } break;
    case hash("G1"): {
      move_cmd = true;

      const bool has_extrude = command.has_argument("E") && (command.get_argument("E", 0.0) != 0.0);
      const bool has_z = command.has_argument_not("Z", position.z);
      const bool has_xy = command.has_argument_not("X", position.x) || command.has_argument_not("Y", position.y);
      const bool has_xyz = has_xy || has_z;
      
      feedrate = command.get_argument("F", feedrate);
      extract_position(command);

      if (has_extrude)
      {
        const real extrude = [&]() -> real
        {
          real E;
          if (relative_extrusion)
          {
            E = command.get_argument("E", 0.0);
            current_extrusion += E;
          }
          else
          {
            const real new_E = command.get_argument("E", 0.0);
            E = new_E - current_extrusion;
            current_extrusion = new_E;
          }
          return E;
        }();

        // Z movement-only with extrusion seems... unlikely. Just treat it as a normal move.
        if (has_xyz)
        {
          auto * __restrict cmd = new segments::extrusion_move;
          cmd->set_positions(start_position, position);
          cmd->set_extrude(extrude);
          cmd->set_feedrate(feedrate);
          cmd->acceleration_hint_ = print_accel;
          cmd->acceleration_ = acceleration.limit({ print_accel, print_accel, print_accel });
          cmd->jerk_hint_ = jerk;
          cmd->jerk_extrude_hint_ = extrude_jerk;
          out.push_back(cmd);
        }
        else
        {
          // Extrude-only
          auto * __restrict cmd = new segments::extrusion;
          cmd->set_extrude(extrude);
          cmd->set_feedrate(feedrate);
          cmd->acceleration_hint_ = retract_accel;
          cmd->acceleration_ = vector3<>(cfg.defaults.extrusion_acceleration).limit({ retract_accel, retract_accel, retract_accel }); // TODO extrusion acceleration?
          cmd->jerk_hint_ = jerk;
          cmd->jerk_extrude_hint_ = extrude_jerk;
          out.push_back(cmd);
        }
      }
      else
      {
        if (has_xy)
        {
          if (cfg.options.all_no_extrude_as_travel)
          {
            auto * __restrict cmd = new segments::travel;
            cmd->set_positions(start_position, position);
            cmd->set_feedrate(feedrate);
            cmd->acceleration_hint_ = print_accel;
            cmd->acceleration_ = acceleration.limit({ print_accel, print_accel, print_accel });
            cmd->jerk_hint_ = jerk;
            cmd->jerk_extrude_hint_ = extrude_jerk;
            out.push_back(cmd);
          }
          else
          {
            auto * __restrict cmd = new segments::linear;
            cmd->set_positions(start_position, position);
            cmd->set_feedrate(feedrate);
            cmd->acceleration_hint_ = print_accel;
            cmd->acceleration_ = acceleration.limit({ print_accel, print_accel, print_accel });
            cmd->jerk_hint_ = jerk;
            cmd->jerk_extrude_hint_ = extrude_jerk;
            out.push_back(cmd);
          }
        }
        else if (has_z)
        {
          auto * __restrict cmd = new segments::hop;
          cmd->set_positions(start_position, position);
          cmd->set_feedrate(feedrate);
          cmd->acceleration_hint_ = travel_accel;
          cmd->acceleration_ = acceleration.limit({ travel_accel, travel_accel, travel_accel });
          cmd->jerk_hint_ = jerk;
          cmd->jerk_extrude_hint_ = extrude_jerk;
          out.push_back(cmd);
        }
        else
        {
          // Not a move, so don't generate a command.
        }
      }

    } break;

      // Non-Movement Commands
    case hash("M82"): {
      // Set Absolute Extrusion
      relative_extrusion = false;
    } break;
    case hash("M83"): {
      // Set Relative Extrusion
      relative_extrusion = true;
    } break;

    case hash("M84"): {
      // Disable Steppers
      auto * __restrict cmd = new instructions::M84(command);
      out.push_back(cmd);
    } break;

    case hash("M104"): {
      // Set Extruder Temperature, no wait
      auto * __restrict cmd = new instructions::M104(command);
      // Eliminate redundant/invalid commands
      uint temperature = cmd->get_temperature();
      uint extruder = cmd->get_number();

      if (temperature == uint(-1) || temperature == extruder_temp[extruder])
      {
        delete cmd;
      }
      else
      {
        extruder_temp[extruder] = temperature;
        out.push_back(cmd);
      }
    } break;

    case hash("M106"): {
      // Fan On
      auto * __restrict cmd = new instructions::M106(command);
      // Eliminate redundant/invalid commands
      uint speed = cmd->get_speed();
      uint fan = cmd->get_number();

      if (speed == fan_speeds[fan])
      {
        delete cmd;
      }
      else
      {
        fan_speeds[fan] = speed;
        out.push_back(cmd);
      }
    } break;

    case hash("M107"): {
      // Fan Off
      auto * __restrict cmd = new instructions::M107(command);
      // Eliminate redundant/invalid commands
      uint fan = cmd->get_number();

      if (0 == fan_speeds[fan])
      {
        delete cmd;
      }
      else
      {
        fan_speeds[fan] = 0;
        out.push_back(cmd);
      }
    } break;

    case hash("M109"): {
      // Set Extruder Temperature and wait
      auto * __restrict cmd = new instructions::M109(command);
      // Eliminate redundant/invalid commands
      uint temperature = cmd->get_temperature();
      uint extruder = cmd->get_number();

      if (temperature == uint(-1) || temperature == extruder_temp[extruder])
      {
        delete cmd;
      }
      else
      {
        extruder_temp[extruder] = temperature;
        out.push_back(cmd);
      }
    } break;

    case hash("M140"): {
      // Set Bed Temperature, no wait
      auto * __restrict cmd = new instructions::M140(command);
      // Eliminate redundant/invalid commands
      uint temperature = cmd->get_temperature();
      uint heater = cmd->get_number();

      if (temperature == uint(-1) || temperature == bed_temp[heater])
      {
        delete cmd;
      }
      else
      {
        bed_temp[heater] = temperature;
        out.push_back(cmd);
      }
    } break;

    case hash("M190"): {
      // Set Bed Temperature and wait
      auto * __restrict cmd = new instructions::M190(command);
      // Eliminate redundant/invalid commands
      uint temperature = cmd->get_temperature();
      uint heater = cmd->get_number();

      if (temperature == uint(-1) || temperature == bed_temp[heater])
      {
        delete cmd;
      }
      else
      {
        bed_temp[heater] = temperature;
        out.push_back(cmd);
      }
    } break;

    case hash("G28"): {
      // Home
      auto * __restrict cmd = new instructions::G28(command);

      if (cmd->axis().x)
      {
        position.x = 0.0;
      }
      if (cmd->axis().y)
      {
        position.y = 0.0;
      }
      if (cmd->axis().z)
      {
        position.z = 0.0;
      }

      out.push_back(cmd);
    } break;

    case hash("G90"): {
      // Set Absolute Positioning
      absolute_mode = true;
    } break;

    case hash("G91"): {
      // Set Relative Positioning
      absolute_mode = false;
    } break;

      // State Commands (these do not generate opcodes, and instead are used for calculating motion or other things)
    case hash("M204"): {
      // SET DEFAULT ACCELERATION
      if (command.has_argument("S")) // Legacy, but still used by some slicers like Cura
      {
        print_accel = command.get_argument("S", print_accel);
        travel_accel = command.get_argument("S", travel_accel);
      }

      print_accel = command.get_argument("P", print_accel);
      travel_accel = command.get_argument("T", travel_accel);
      retract_accel = command.get_argument("R", retract_accel);
    } break;
    case hash("M205"): {
      // ADVANCED SETTINGS (jerk)
      jerk.x = command.get_argument("X", jerk.x);
      jerk.y = command.get_argument("Y", jerk.y);
      jerk.z = command.get_argument("Z", jerk.z);
      extrude_jerk = command.get_argument("E", extrude_jerk);
    } break;

    default: {
      printf("Unknown Command: %s\n", command._cmd_string.c_str());
    } break;
    }

    prev_move_cmd = move_cmd;
  }
  __pragma(warning(default:4307));

  usize contiguous_segment_count = 0;
  usize move_commands_orig = 0;

  for (const auto * __restrict cmd : out)
  {
    switch (cmd->get_type())
    {
    case segments::extrusion_move::type:
    case segments::hop::type:
    case segments::linear::type:
    case segments::travel::type: {
      ++move_commands_orig;
    }
    }
  }

  if (out.size() >= 2)
  {
    printf("Eliminating redundant movements...\n");
    // Now we compact the commands by finding ones that can be merged.
    auto prev_iter = out.begin();
    for (auto iter = prev_iter + 1; iter != out.end();)
    {
      auto * __restrict prev_cmd = *prev_iter;
      const auto * __restrict cur_cmd = *iter;

      // Only valid if they're the same command type.
      if (prev_cmd->get_type() == cur_cmd->get_type())
      {
        // Also only valid if they're movement types.
        switch (cur_cmd->get_type())
        {
        case segments::extrusion_move::type:
        case segments::hop::type:
        case segments::linear::type:
        case segments::travel::type: {
          segments::movement * __restrict prev_move_cmd = static_cast<segments::movement * __restrict>(prev_cmd);
          const segments::movement * __restrict cur_move_cmd = static_cast<const segments::movement * __restrict>(cur_cmd);

          // They must have the same feedrate as well.
          if (prev_move_cmd->get_feedrate() != cur_move_cmd->get_feedrate())
          {
            break;
          }

          const vector3<> prev_vector = prev_move_cmd->get_vector().normalized();
          const vector3<> cur_vector = cur_move_cmd->get_vector().normalized();

          const real dot_product = prev_vector.dot(cur_vector);

          if (!is_equal(dot_product, 1.0))
          {
            break;
          }

          static constexpr const bool compare_hints = true;

          // If this is an extrusion, make sure the extrusion rate is the same.
          if (cur_cmd->get_type() == segments::extrusion_move::type)
          {
            const real prev_time = prev_move_cmd->get_vector().length() / prev_move_cmd->get_feedrate();
            const real cur_time = cur_move_cmd->get_vector().length() / cur_move_cmd->get_feedrate();

            const segments::extrusion_move * __restrict prev_extrusion_cmd = static_cast<const segments::extrusion_move * __restrict>(prev_cmd);
            const segments::extrusion_move * __restrict cur_extrusion_cmd = static_cast<const segments::extrusion_move * __restrict>(cur_cmd);
          
            const real prev_extrude = prev_extrusion_cmd->get_extrusion();
            const real cur_extrude = cur_extrusion_cmd->get_extrusion();

            const real prev_extrusion_rate = prev_extrude / prev_time;
            const real cur_extrusion_rate = cur_extrude / cur_time;

            if (!is_equal(prev_extrusion_rate, cur_extrusion_rate, cfg.extrusion.epsilon))
            {
              break;
            }

            if constexpr (compare_hints)
            {
              if (!is_equal(prev_move_cmd->jerk_extrude_hint_, cur_move_cmd->jerk_extrude_hint_))
              {
                break;
              }
            }
          }

          if constexpr (compare_hints)
          {
            // Validate that acceleration/jerk are similar.
            const real prev_acceleration_hint = prev_move_cmd->acceleration_hint_;
            const real cur_acceleration_hint = cur_move_cmd->acceleration_hint_;

            const vector3<> prev_jerk_hint = prev_move_cmd->jerk_hint_;
            const vector3<> cur_jerk_hint = cur_move_cmd->jerk_hint_;

            if (!is_equal(prev_acceleration_hint, cur_acceleration_hint))
            {
              break;
            }

            if (!is_equal(prev_jerk_hint, cur_jerk_hint))
            {
              break;
            }
          }

          // Otherwise, these appear to be contiguous segments.
          ++contiguous_segment_count;

          prev_move_cmd->set_end_position(cur_move_cmd->get_end_position());
          if (cur_cmd->get_type() == segments::extrusion_move::type)
          {
            segments::extrusion_move * __restrict prev_extrusion_cmd = static_cast<segments::extrusion_move * __restrict>(prev_cmd);
            const segments::extrusion_move * __restrict cur_extrusion_cmd = static_cast<const segments::extrusion_move * __restrict>(cur_cmd);

            prev_extrusion_cmd->set_extrusion(prev_extrusion_cmd->get_extrusion() + cur_extrusion_cmd->get_extrusion());
          }

          iter = out.erase(iter);
          continue;

        } break;
        }
      }

      prev_iter = iter++;
    }
  }

  if (contiguous_segment_count)
  {
    const double reduction = 100.0 * (double(move_commands_orig - contiguous_segment_count) / double(move_commands_orig));
    printf(
      "Merged %llu contiguous segments (%.2f%% original count - %llu -> %llu)\n",
      contiguous_segment_count,
      reduction,
      move_commands_orig,
      move_commands_orig - contiguous_segment_count
    );
  }

  printf("Linking motion segments\n");
  {
    gcgg::segments::segment * __restrict prev_seg = nullptr;
    for (gcgg::command * __restrict cmd : out)
    {
      if (!cmd->is_segment())
      {
        if (cmd->is_delay())
        {
          prev_seg = nullptr;
        }
        continue;
      }

      gcgg::segments::segment * __restrict cur_seg = static_cast<gcgg::segments::segment * __restrict>(cmd);
      if (prev_seg)
      {
        prev_seg->next_segment_ = cur_seg;
        cur_seg->prev_segment_ = prev_seg;
      }

      prev_seg = cur_seg;
    }
  }

  // Calculate trapezoid values. We need to do this more than once.
  printf("Calculating Motion\n");
  for (auto * __restrict seg : out)
  {
    seg->compute_motion(cfg, false);
  }

  if (cfg.smoothing.enable && out.size() >= 2)
  {
    printf("Smoothing surface...\n");

  }

  if (cfg.arc.generate && out.size() >= 2)
  {
    usize generated_arcs = 0;

    printf("Generating arc segments...\n");
    out.reserve(out.size() * 2); // To prevent iterators from being invalidated. This is hacky and non-standard, but should work.

    auto prev_iter = out.begin();
    for (auto iter = prev_iter + 1; iter != out.end();)
    {
      auto * __restrict prev_cmd = *prev_iter;
      auto * __restrict cur_cmd = *iter;

      // TODO we should really handle this case. I don't know how, yet.
      if (prev_cmd->get_type() == segments::arc::type)
      {
        prev_iter = iter++;
        continue;
      }

      const auto is_move = [](const auto * __restrict cmd)->bool
      {
        switch (cmd->get_type())
        {
        case segments::extrusion_move::type:
        case segments::hop::type:
        case segments::linear::type:
        case segments::travel::type:
          return true;
        }
        return false;
      };

      if (!is_move(prev_cmd) || cur_cmd->is_delay())
      {
        prev_iter = iter++;
        continue;
      }

      if (!is_move(cur_cmd))
      {
        // If the current command is not a movement command, we might need to just increment iter until
        // we either find one, or we hit a delay instruction, as there might be non-moves between our moves
        // that are inconsequential.
        do
        {
          ++iter;

          if (iter == out.end())
          {
            break;
          }

          cur_cmd = *iter;
          if (is_move(cur_cmd))
          {
            goto continue_exec;
          }

          // If it's a delay instruction, we need to just keep moving, since it interrupts motion.
          if (cur_cmd->is_delay())
          {
            goto continue_exec;
          }

        } while (iter != out.end());
        break;
      continue_exec:;

        if (!is_move(cur_cmd) || cur_cmd->is_delay())
        {
          prev_iter = iter++;
          continue;
        }
      }

      segments::movement * __restrict prev_segment_cmd = static_cast<segments::movement * __restrict>(prev_cmd);
      segments::movement * __restrict cur_segment_cmd = static_cast<segments::movement * __restrict>(cur_cmd);

      // Calculate the angle between the two segments.
      const vector3<> segment_vectors[2] = {
        prev_segment_cmd->get_vector(),
        cur_segment_cmd->get_vector(),
      };

      const vector3<> segment_norm_vectors[2] = {
        segment_vectors[0].normalized(),
        segment_vectors[1].normalized(),
      };

      const double angle = segment_norm_vectors[0].angle_between(segment_norm_vectors[1]);

      if (angle <= cfg.arc.min_angle)
      {
        prev_iter = iter++;
        continue;
      }

      const bool is_travel = prev_segment_cmd->is_travel_ && cur_segment_cmd->is_travel_;

      double arc_radius;
      if (is_travel && cfg.arc.halve_travels)
      {
        arc_radius = min(
          prev_segment_cmd->get_vector().length(),
          cur_segment_cmd->get_vector().length() / 2
        );
      }
      else
      {
        arc_radius = is_travel ? cfg.arc.travel_radius : cfg.arc.radius;
      }

      // TODO in reality we should be generating ovaloid arcs, to handle differences in velocity.

      if (prev_segment_cmd->get_vector().length() < arc_radius || (cur_segment_cmd->get_vector().length() * 0.5) < arc_radius)
      {
        // TODO handle this situation by reducing arc length better than this. This is nasty.
        if (prev_segment_cmd->get_vector().length() < arc_radius)
        {
          arc_radius = prev_segment_cmd->get_vector().length();
        }
        if ((cur_segment_cmd->get_vector().length() * 0.5) < arc_radius)
        {
          arc_radius = (cur_segment_cmd->get_vector().length() * 0.5);
        }
        if (arc_radius <= cfg.arc.min_radius)
        {
          prev_iter = iter++;
          continue;
        }
      }

      ++generated_arcs;

      // Generate a split point for both segments
      // Reduce any extrusion (and apply said extrusion to the arc)
      const real segment_orig_lengths[2] = {
        segment_vectors[0].length(),
        segment_vectors[1].length()
      };

      const auto get_extrusion = [](const segments::segment *seg)->real
      {
        if (seg->get_type() != segments::extrusion_move::type)
        {
          return 0.0;
        }
        const segments::extrusion_move * __restrict extrusion_cmd = static_cast<const segments::extrusion_move * __restrict>(seg);
        return extrusion_cmd->get_extrusion();
      };

      const auto set_extrusion = [](segments::segment *seg, real extrude)
      {
        if (seg->get_type() != segments::extrusion_move::type)
        {
          return;
        }
        segments::extrusion_move * __restrict extrusion_cmd = static_cast<segments::extrusion_move * __restrict>(seg);
        return extrusion_cmd->set_extrude(extrude);
      };

      const real segment_orig_extrude[2] = {
        get_extrusion(prev_segment_cmd),
        get_extrusion(cur_segment_cmd)
      };

      const real segment_new_fraction[2] = {
        (segment_orig_lengths[0] - arc_radius) / segment_orig_lengths[0],
        (segment_orig_lengths[1] - arc_radius) / segment_orig_lengths[1]
      };
      
      const vector3<> corner = prev_segment_cmd->get_end_position();
      const vector3<> prev_seg_new_end = prev_segment_cmd->get_start_position() + (segment_vectors[0] * segment_new_fraction[0]);
      const vector3<> cur_seg_new_start = cur_segment_cmd->get_end_position() - (segment_vectors[1] * segment_new_fraction[1]);

      const real segment_new_extrude[2] = {
        segment_orig_extrude[0] * segment_new_fraction[0],
        segment_orig_extrude[1] * segment_new_fraction[1]
      };

      const real segment_extrude_remainder[2] = {
        segment_orig_extrude[0] - segment_new_extrude[0],
        segment_orig_extrude[1] - segment_new_extrude[1]
      };

      if (is_equal(prev_seg_new_end.distance(cur_seg_new_start), 0.0))
      {
        prev_iter = iter++;
        continue;
      }

      set_extrusion(prev_segment_cmd, segment_new_extrude[0]);
      set_extrusion(cur_segment_cmd, segment_new_extrude[1]);
      prev_segment_cmd->set_end_position(prev_seg_new_end);
      cur_segment_cmd->set_start_position(cur_seg_new_start);

      real start_feedrate = prev_segment_cmd->get_feedrate();
      real end_feedrate = cur_segment_cmd->get_feedrate();
      if (cfg.arc.constant_speed)
      {
        // If we are a constant speed arc, average out the two feedrates.
        start_feedrate = end_feedrate = (start_feedrate + end_feedrate) * 0.5;
      }

      auto * __restrict new_arc = new segments::arc(
        segment_extrude_remainder,
        { start_feedrate, end_feedrate },
        { prev_segment_cmd->acceleration_hint_,  cur_segment_cmd->acceleration_hint_ },
        { prev_segment_cmd->jerk_hint_,  cur_segment_cmd->jerk_hint_ },
        { prev_segment_cmd->jerk_extrude_hint_,  cur_segment_cmd->jerk_extrude_hint_ },
        corner,
        prev_seg_new_end,
        cur_seg_new_start,
        arc_radius,
        angle
      );

      new_arc->parent_velocities_[0] = (prev_segment_cmd->get_end_position() - prev_segment_cmd->get_start_position()).normalized(prev_segment_cmd->get_feedrate());
      new_arc->parent_velocities_[1] = (cur_segment_cmd->get_end_position() - cur_segment_cmd->get_start_position()).normalized(cur_segment_cmd->get_feedrate());

      new_arc->is_travel_ = is_travel;

      // Do we need to delete the previous segment (has it been completely replaced with arcs?
      // TODO currently we never destroy the current segment as we check against half-lengths. We should revisit that.
      if (is_equal(prev_segment_cmd->get_vector().length(), 0.0))
      {
        delete *prev_iter;
        *prev_iter = new_arc;
      }
      else
      {
        iter = (out.insert(iter, new_arc)) + 1; // insert before cur_seg, then get the iterator again due to invalidation rules.
      }

      prev_iter = iter++;
    }

    printf("Generated Corner Arcs: %llu\n", generated_arcs);
  }

  // Generate arcs where possible.
  if (cfg.reg_arc_gen.enable)
  {
    std::vector<gcgg::command * __restrict> erase_set;

    uint64_t generated_arcs = 0;
    segments::arc_accumulator accumulator;

    printf("Generating Arcs from curved segment sets\n");

    const auto flush_accumulator = [&](auto &iterator) -> bool
    {
      if (!accumulator.conditional_reset())
      {
        // If the accumulator is actually valid, it means we've generated an arc.
        // TODO optimize this bit.
        erase_set.insert(erase_set.end(), accumulator.get_segments().begin(), accumulator.get_segments().end());
        iterator = out.insert(iterator, new segments::arc_accumulator(std::move(accumulator)));
        ++generated_arcs;
        accumulator.reset();
        return true;
      }
      return false;
    };

    for (auto i = out.begin(); i != out.end();)
    {
      gcgg::command * __restrict cmd = *i;

      const auto is_move = [](const auto * __restrict cmd)->bool
      {
        switch (cmd->get_type())
        {
        case segments::extrusion_move::type:
        case segments::hop::type:
        case segments::linear::type:
        case segments::travel::type:
          return true;
        }
        return false;
      };

      if (!is_move(cmd))
      {
        // We don't consume this one.
        if (!flush_accumulator(i))
        {
          ++i;
        }
        continue;
      }

      segments::movement * __restrict segment_cmd = static_cast<segments::movement * __restrict>(cmd);
      
      if (!accumulator.consume_segment(*segment_cmd, cfg))
      {
        // If we don't consume this, this arc is finished, if it exists at all.
        if (!flush_accumulator(i))
        {
          ++i;
        }
        continue;
      }

      // If we did consume it... yay. Continue to the next one.
      ++i;
    }
    if (!accumulator.conditional_reset())
    {
      // If the accumulator is actually valid, it means we've generated an arc.
      // TODO optimize this bit.
      erase_set.insert(erase_set.end(), accumulator.get_segments().begin(), accumulator.get_segments().end());
      out.push_back(new segments::arc_accumulator(std::move(accumulator)));
      ++generated_arcs;
      accumulator.reset();
    }

    // Clear the erase set. This is messy and slow.
    if (erase_set.size())
    {
      printf("Performing segment garbage collection... (%llu segments to delete)\n", uint64(erase_set.size()));
      for (auto i = out.begin(); i != out.end();)
      {
        auto ei = std::find(erase_set.begin(), erase_set.end(), *i);
        if (ei != erase_set.end())
        {
          // No need to perform a normal erase which requires a copy. Do a fast erase.
          fast_erase_iterator(erase_set, ei);
          i = out.erase(i);
          if (erase_set.size() == 0)
          {
            break;
          }
        }
        else
        {
          ++i;
        }
      }
    }

    printf("Generated Arcs: %llu\n", generated_arcs);
  }

  if (cfg.arc.generate && cfg.output.subdivide_arcs)
  {
    printf("Subdividing Arcs\n");
    for (auto i = out.begin(); i != out.end();)
    {
      gcgg::command * __restrict cmd = *i;

      if (cmd->get_type() != segments::arc::type)
      {
        ++i;
        continue;
      }

      gcgg::segments::arc * __restrict arc_seg = static_cast<gcgg::segments::arc * __restrict>(cmd);

      if (arc_seg->should_subdivide(cfg))
      {
        i = out.erase(i);

        auto new_segments = arc_seg->generate_segments(cfg);

        delete arc_seg;

        i = out.insert(i, new_segments.begin(), new_segments.end());
      }
      else
      {
        ++i;
      }
    }
  }

  printf("Linking motion segments\n");
  {
    gcgg::segments::segment * __restrict prev_seg = nullptr;
    for (gcgg::command * __restrict cmd : out)
    {
      if (!cmd->is_segment())
      {
        if (cmd->is_delay())
        {
          prev_seg = nullptr;
        }
        continue;
      }

      gcgg::segments::segment * __restrict cur_seg = static_cast<gcgg::segments::segment * __restrict>(cmd);
      if (prev_seg)
      {
        prev_seg->next_segment_ = cur_seg;
        cur_seg->prev_segment_ = prev_seg;
      }

      prev_seg = cur_seg;
    }
  }

  printf("Calculating Motion\n");
  for (auto * __restrict seg : out)
  {
    seg->compute_motion(cfg, true);
  }

  return out;
}
