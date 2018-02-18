#pragma once

#include "movement.hpp"
#include "extrusion_move.hpp"
#include "travel.hpp"
#include <algorithm>

namespace gcgg::segments
{
  // Linear movement with extrusion
  class arc final : public movement
  {
  public:
    static constexpr const uint64 type = hash("arc");

  protected:
    real extrude_[2];
    real seg_feedrate_[2];
    real acceleration_[2];
    vector3<> jerk_[2];
    real extrude_jerk_[2];
    vector3<> corner_;
    real radius_;
    real angle_;

  public:
    vector3<> parent_velocities_[2];

  public:
    real get_feedrate_element(usize i) const __restrict
    {
      xassert(i < 2);
      return seg_feedrate_[i];
    }

    arc(
      const real(&__restrict extrude)[2],
      const real(&__restrict feedrate)[2],
      const real(&__restrict acceleration)[2],
      const vector3<>(&__restrict jerk)[2],
      const real(&__restrict extrude_jerk)[2],
      const vector3<> & __restrict corner,
      const vector3<> & __restrict start,
      const vector3<> & __restrict end,
      real radius,
      real angle
    ) : movement(type)
    {
      start_position_ = start;
      end_position_ = end;
      feedrate_ = (feedrate[0] + feedrate[1]) * 0.5; // TODO adjust this for ovaloid arcs

      extrude_[0] = extrude[0];
      extrude_[1] = extrude[1];
      seg_feedrate_[0] = feedrate[0];
      seg_feedrate_[1] = feedrate[1];
      acceleration_[0] = acceleration[0];
      acceleration_[1] = acceleration[1];
      jerk_[0] = jerk[0];
      jerk_[1] = jerk[1];
      extrude_jerk_[0] = extrude_jerk[0];
      extrude_jerk_[1] = extrude_jerk[1];
      corner_ = corner;
      radius_ = radius;
      angle_ = angle;

      if (seg_feedrate_[0] == 0.0)
      {
        seg_feedrate_[0] = seg_feedrate_[1];
      }
      if (acceleration_[0] == 0.0)
      {
        acceleration_[0] = acceleration_[1];
      }
      if (extrude_jerk_[0] == 0.0)
      {
        extrude_jerk_[0] = extrude_jerk_[1];
      }
      if (jerk_[0].x == 0.0)
      {
        jerk_[0].x = jerk_[1].x;
      }
      if (jerk_[0].y == 0.0)
      {
        jerk_[0].y = jerk_[1].y;
      }
      if (jerk_[0].z == 0.0)
      {
        jerk_[0].z = jerk_[1].z;
      }

      acceleration_hint_ = (acceleration_[0] + acceleration_[1]) * 0.5;
      jerk_hint_ = (jerk_[0] + jerk_[1]) * 0.5;
      jerk_extrude_hint_ = (extrude_jerk_[0] + extrude_jerk_[1]) * 0.5;
    }
    arc() : movement(type) {}
    virtual ~arc() {}

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "arc";

      out += movement::dump();

      return out;
    }

    real get_extrusion() const __restrict
    {
      return extrude_[0] + extrude_[1];
    }

    std::vector<gcgg::command *> generate_segments(const config & __restrict cfg) const __restrict
    {
      // TODO validate segments against a jerk test, and subdivide further if the jerk test fails.

      std::vector<gcgg::command *> out;

      const vector3<> center_point = (start_position_ + end_position_) * 0.5;
      const vector3<> arc_origin = corner_ + ((center_point - corner_) * 2.0); // TODO needs to be adjusted for ovaloid arcs.

      //const uint subdivisions = std::max(uint(std::round((angle_ / cfg.arc.gcode_segment_modulus)) + 0.5), 0u);

      struct segment final
      {
        vector3<> start;
        vector3<> end;
      };

      std::vector<segment> segments = { { start_position_, end_position_ } };

      const vector3<> start_vector = (corner_ - start_position_).normalized();
      const vector3<> end_vector = (end_position_ - corner_).normalized();

      const auto get_current_angle = [&]() -> real
      {
        const real angle = std::acos(start_vector.dot((segments[0].end - segments[0].start).normalized())) * 57.2958;
        return angle;
      };

      const auto angle_between = [](const segment & __restrict a, const segment & __restrict b) -> real
      {
        const vector3<> vec_a = (a.end - a.start).normalized();
        const vector3<> vec_b = (b.end - b.start).normalized();
        return std::acos(vec_a.dot(vec_b)) * 57.2958;
      };

      const real min_segment_angle = cfg.arc.min_angle;

      while (angle_ < cfg.arc.max_angle && get_current_angle() >= cfg.arc.min_angle/* && segments.size() < cfg.arc.max_segments*/)
      {
        std::vector<segment> new_segments;
        new_segments.reserve(segments.size() * 2);

        bool added_segments = false;

        for (const segment & __restrict seg : segments)
        {
          // Every segment here will get split in twain.
          vector3<> segment_center = (seg.start + seg.end) * 0.5;
          // We need to renormalize this center position around the arc origin.

          if (is_equal(segment_center.distance(arc_origin), 0.0))
          {
            new_segments.push_back(seg);
          }
          else
          {
            vector3<> segment_vector = (segment_center - arc_origin).normalized() * radius_;
            vector3<> arc_position = arc_origin + segment_vector;

            // Generate two segments from this.
            bool valid_segments = !is_equal(seg.start.distance(arc_position), 0.0) && !is_equal(seg.end.distance(arc_position), 0.0);
            valid_segments = valid_segments && (angle_between({ seg.start, arc_position }, { arc_position, seg.end }) >= min_segment_angle);

            if (!valid_segments)
            {
              new_segments.push_back(seg);
            }
            else
            {
              new_segments.push_back({ seg.start, arc_position });
              new_segments.push_back({ arc_position, seg.end });
              added_segments = true;
            }
          }
        }

        if (!added_segments)
        {
          break;
        }

        segments = std::move(new_segments);
      }

      real total_arc_length = 0.0;
      const real original_length[2] = {
        start_position_.distance(corner_),
        end_position_.distance(corner_)
      };

      for (const segment & __restrict seg : segments)
      {
        total_arc_length += seg.start.distance(seg.end);
      }

      const real total_arc_length_half = total_arc_length * 0.5;

      const real adusted_extrusions[2] = {
        extrude_[0] * (total_arc_length_half / original_length[0]),
        extrude_[1] * (total_arc_length_half / original_length[1])
      };

      out.reserve(segments.size());

      // Segments is now all of the subdivided segments.
      const real segments_divisor = real(segments.size() + 1);
      usize iter = 1; // 1 because we want to start in the middle of the interpolation.
      for (const segment & __restrict seg : segments)
      {
        const real interpoland = real(iter++) / segments_divisor;

        const auto lerp = [](auto x, auto y, real s) -> auto
        {
          return x + s * (y - x);
        };

        const real feedrate = lerp(seg_feedrate_[0], seg_feedrate_[1], interpoland);
        real extrusion;
        if (adusted_extrusions[0] == 0.0 && adusted_extrusions[1] != 0.0)
        {
          // travel to extrude
          if (interpoland >= 0.5)
          {
            extrusion = ((adusted_extrusions[0] + adusted_extrusions[1]) / (real(segments.size()) / 2));
          }
          else
          {
            extrusion = 0.0;
          }
        }
        else if (adusted_extrusions[0] != 0.0 && adusted_extrusions[1] == 0.0)
        {
          // extrude to travel
          if (interpoland <= 0.5)
          {
            extrusion = ((adusted_extrusions[0] + adusted_extrusions[1]) / (real(segments.size()) / 2));
          }
          else
          {
            extrusion = 0.0;
          }
        }
        else
        {
          // all else
          extrusion = ((adusted_extrusions[0] + adusted_extrusions[1]) / segments.size());
        }
        const real acceleration = lerp(acceleration_[0], acceleration_[1], interpoland);
        const real extrude_jerk = lerp(extrude_jerk_[0], extrude_jerk_[1], interpoland);
        const vector3<> jerk = lerp(jerk_[0], jerk_[1], interpoland);

        segments::segment * __restrict new_seg;

        if (extrusion != 0.0)
        {
          // This is an extrusion move.
          auto s = new segments::extrusion_move;
          s->set_positions(seg.start, seg.end);
          s->acceleration_hint_ = acceleration;
          s->set_feedrate(feedrate);
          s->jerk_extrude_hint_ = extrude_jerk;
          s->jerk_hint_ = jerk;
          s->set_extrude(extrusion);
          new_seg = s;
        }
        else
        {
          auto s = new segments::travel;
          s->set_positions(seg.start, seg.end);
          s->acceleration_hint_ = acceleration;
          s->set_feedrate(feedrate);
          s->jerk_extrude_hint_ = extrude_jerk;
          s->jerk_hint_ = jerk;
          new_seg = s;
        }

        new_seg->from_arc_ = true;

        out.push_back(new_seg);
      }

      return out;
    }

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict override final
    {
      const vector3<> center_point = (start_position_ + end_position_) * 0.5;
      const vector3<> arc_origin = corner_ + ((center_point - corner_) * 2.0); // TODO needs to be adjusted for ovaloid arcs.

      if (cfg.output.generate_G15)
      {
        // G15 specifies the input feedrate and output feedrate for each axis. The firmware linearly interpolates between these based on time.
        vector3<> in_feedrate = parent_velocities_[0];
        vector3<> out_feedrate = parent_velocities_[1];

        real extrusion;
        {
          extrusion = extrude_[0] + extrude_[1];
          // TODO extrude value needs to be adjusted for arc length,
          // as an arc is shorter than the original length it replaces.
        }

        // TODO process acceleration and jerk

        out += "G15";
        char buffer[512];

        if (start_position_.x != end_position_.x)
        {
          state.position.x = end_position_.x;
          sprintf(buffer, "%.8f", state.position.x);
          out += " X";
          out += trim_float(buffer);
        }
        if (start_position_.y != end_position_.y)
        {
          state.position.y = end_position_.y;
          sprintf(buffer, "%.8f", state.position.y);
          out += " Y";
          out += trim_float(buffer);
        }
        if (start_position_.z != end_position_.z)
        {
          state.position.z = end_position_.z;
          sprintf(buffer, "%.8f", state.position.z);
          out += " Z";
          out += trim_float(buffer);
        }

        sprintf(buffer, "%.8f", in_feedrate.x);
        out += " A";
        out += trim_float(buffer);
        sprintf(buffer, "%.8f", in_feedrate.y);
        out += " B";
        out += trim_float(buffer);
        sprintf(buffer, "%.8f", in_feedrate.z);
        out += " C";
        out += trim_float(buffer);

        sprintf(buffer, "%.8f", out_feedrate.x);
        out += " D";
        out += trim_float(buffer);
        sprintf(buffer, "%.8f", out_feedrate.y);
        out += " E";
        out += trim_float(buffer);
        sprintf(buffer, "%.8f", out_feedrate.z);
        out += " F";
        out += trim_float(buffer);

        // TODO need to output a time value. This requires the arc length, which is also required to adjust extrusion.

        return;
      }

      const uint subdivisions = std::max(uint(std::round((angle_ / cfg.arc.min_angle)) + 0.5), 0u);

      struct segment final
      {
        vector3<> start;
        vector3<> end;
      };

      std::vector<segment> segments = { {start_position_, end_position_} };

      for (uint i = 0; i < subdivisions; ++i)
      {
        std::vector<segment> new_segments;
        new_segments.reserve(segments.size() * 2);

        for (const segment & __restrict seg : segments)
        {
          // Every segment here will get split in twain.
          vector3<> segment_center = (seg.start + seg.end) * 0.5;
          // We need to renormalize this center position around the arc origin.

          if (is_equal(segment_center.distance(arc_origin), 0.0))
          {
            new_segments.push_back(seg);
          }
          else
          {
            vector3<> segment_vector = (segment_center - arc_origin).normalized() * radius_;
            vector3<> arc_position = arc_origin + segment_vector;

            // Generate two segments from this.
            if (is_equal(seg.start.distance(arc_position), 0.0) || is_equal(seg.end.distance(arc_position), 0.0))
            {
              new_segments.push_back(seg);
            }
            else
            {
              new_segments.push_back({ seg.start, arc_position });
              new_segments.push_back({ arc_position, seg.end });
            }
          }
        }

        segments = std::move(new_segments);
      }

      real total_arc_length = 0.0;
      const real original_length[2] = {
        start_position_.distance(corner_),
        end_position_.distance(corner_)
      };

      for (const segment & __restrict seg : segments)
      {
        total_arc_length += seg.start.distance(seg.end);
      }

      const real total_arc_length_half = total_arc_length * 0.5;

      const real adusted_extrusions[2] = {
        extrude_[0] * (total_arc_length_half / original_length[0]),
        extrude_[1] * (total_arc_length_half / original_length[1])
      };

      // Segments is now all of the subdivided segments.
      const real segments_divisor = real(segments.size() + 1);
      usize iter = 1; // 1 because we want to start in the middle of the interpolation.
      for (const segment & __restrict seg : segments)
      {
        const real interpoland = real(iter++) / segments_divisor;
        
        const auto lerp = [](auto x, auto y, real s) -> auto
        {
          return x + s * (y - x);
        };

        const real feedrate = lerp(seg_feedrate_[0], seg_feedrate_[1], interpoland);
        real extrusion;
        if (adusted_extrusions[0] == 0.0 && adusted_extrusions[1] != 0.0)
        {
          // travel to extrude
          if (interpoland >= 0.5)
          {
            extrusion = ((adusted_extrusions[0] + adusted_extrusions[1]) / (real(segments.size()) / 2));
          }
          else
          {
            extrusion = 0.0;
          }
        }
        else if (adusted_extrusions[0] != 0.0 && adusted_extrusions[1] == 0.0)
        {
          // extrude to travel
          if (interpoland <= 0.5)
          {
            extrusion = ((adusted_extrusions[0] + adusted_extrusions[1]) / (real(segments.size()) / 2));
          }
          else
          {
            extrusion = 0.0;
          }
        }
        else
        {
          // all else
          extrusion = ((adusted_extrusions[0] + adusted_extrusions[1]) / segments.size());
        }
        const real acceleration = lerp(acceleration_[0], acceleration_[1], interpoland);
        const real extrude_jerk = lerp(extrude_jerk_[0], extrude_jerk_[1], interpoland);
        const vector3<> jerk = lerp(jerk_[0], jerk_[1], interpoland);

        out_gcode_segment(
          out,
          state,
          feedrate,
          extrusion,
          acceleration,
          extrude_jerk,
          jerk,
          seg.start,
          seg.end
        );
      }
    }

  private:
    void out_gcode_segment(
      std::string & __restrict out,
      output::state & __restrict state,
      real feedrate,
      real extrusion,
      real acceleration,
      real extrude_jerk,
      const vector3<> & __restrict jerk,
      const vector3<> & __restrict start_position,
      const vector3<> & __restrict end_position
    ) const __restrict
    {
      if (extrusion == 0.0)
      {
        if (acceleration != state.travel_accel && acceleration != 0)
        {
          state.travel_accel = acceleration;

          out += "M204";

          char buffer[512];
          sprintf(buffer, "%.8f", acceleration);
          out += " T";
          out += trim_float(buffer);
          out += "\n";
        }
      }
      else
      {
        if (acceleration != state.print_accel && acceleration != 0)
        {
          state.print_accel = acceleration;

          out += "M204";

          char buffer[512];
          sprintf(buffer, "%.8f", acceleration);
          out += " P";
          out += trim_float(buffer);
          out += "\n";
        }
      }

      bool emit_jerk_hint = (extrude_jerk != state.extrude_jerk) && extrude_jerk != 0 && extrusion != 0.0;
      if (start_position.x != end_position.x && jerk.x != state.jerk.x && jerk.x != 0)
      {
        emit_jerk_hint = true;
      }
      if (start_position.y != end_position.y && jerk.y != state.jerk.y && jerk.y != 0)
      {
        emit_jerk_hint = true;
      }
      if (start_position.z != end_position.z && jerk.z != state.jerk.z && jerk.z != 0)
      {
        emit_jerk_hint = true;
      }

      if (emit_jerk_hint)
      {
        out += "M205";
        char buffer[512];
        if (extrude_jerk != state.extrude_jerk && extrude_jerk != 0 && extrusion != 0.0)
        {
          state.extrude_jerk = extrude_jerk;
          sprintf(buffer, "%.8f", extrude_jerk);
          out += " E";
          out += trim_float(buffer);
        }
        if (jerk != state.jerk)
        {
          if (start_position.x != end_position.x && jerk.x != state.jerk.x && jerk.x != 0)
          {
            state.jerk.x = jerk.x;
            sprintf(buffer, "%.8f", jerk.x);
            out += " X";
            out += trim_float(buffer);
          }
          if (start_position.y != end_position.y && jerk.y != state.jerk.y && jerk.y != 0)
          {
            state.jerk.y = jerk.y;
            sprintf(buffer, "%.8f", jerk.y);
            out += " Y";
            out += trim_float(buffer);
          }
          if (start_position.z != end_position.z && jerk.z != state.jerk.z && jerk.z != 0)
          {
            state.jerk.z = jerk.z;
            sprintf(buffer, "%.8f", jerk.z);
            out += " Z";
            out += trim_float(buffer);
          }
        }
        out += "\n";
      }

      out += (extrusion == 0.0) ? "G0" : "G1";

      char buffer[512];
      if (extrusion != 0.0)
      {
        sprintf(buffer, "%.8f", extrusion);
        out += " E";
        out += trim_float(buffer);
      }

      if (start_position.x != end_position.x)
      {
        state.position.x = end_position.x;
        sprintf(buffer, "%.8f", state.position.x);
        out += " X";
        out += trim_float(buffer);
      }
      if (start_position.y != end_position.y)
      {
        state.position.y = end_position.y;
        sprintf(buffer, "%.8f", state.position.y);
        out += " Y";
        out += trim_float(buffer);
      }
      if (start_position.z != end_position.z)
      {
        state.position.z = end_position.z;
        sprintf(buffer, "%.8f", state.position.z);
        out += " Z";
        out += trim_float(buffer);
      }

      if (feedrate != state.feedrate)
      {
        state.feedrate = feedrate;

        sprintf(buffer, "%.8f", feedrate);
        out += " F";
        out += trim_float(buffer);
      }

      out += " ; arc\n";
    }

    virtual void compute_motion(const config & __restrict cfg, bool require_jerk) __restrict override final
    {
    }
  };
}
