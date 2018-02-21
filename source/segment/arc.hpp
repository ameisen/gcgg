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
    vector3<> arc_origin_;
    
  public:
    vector3<> parent_velocities_[2];

  private:
    struct segment final
    {
      vector3<> start;
      vector3<> end;

      real linear_offset = 1.0;

      vector3<> get_vector(real magnitude = 1.0) const __restrict
      {
        return (end - start).normalized(magnitude);
      }

      real angle_between(const segment & __restrict seg) const __restrict
      {
        const vector3<> vec_a = (end - start).normalized();
        const vector3<> vec_b = (seg.end - seg.start).normalized();
        return vec_a.angle_between(vec_b);
      }
    };

    struct segment_data final
    {
      std::vector<segment> segments_;
      real feedrate_;
      real length_;
      array<real, 2> extrusion_;
      real acceleration_;
      vector3<> jerk_;
      real extrude_jerk_;
    };

    // high_precision forces very small subsegments, for calculation of arcs for arc instruction emission.
    // Should not be used when subdividing arcs.
    segment_data get_segments_ (const config & __restrict cfg, bool high_precision = false) const __restrict
    {
      // TODO validate segments against a jerk test, and subdivide further if the jerk test fails.

      const vector3<> center_point = mean(start_position_ + end_position_);
      const vector3<> arc_origin = arc_origin_;

      // TODO something here still isn't right, as the radius' we get aren't correct for our angles all the time.

      const real arc_radius = radius_;
      real arc_constrain = corner_.distance(arc_origin);

      if (angle_ <= 90.0)
      {
        const real angle_temp = angle_ / 90.0;
        arc_constrain = slerp(0.0, arc_radius, angle_temp);
      }
      else
      {
        const real angle_temp = (angle_ - 90) / 90.0;
        arc_constrain = lerp(arc_radius, arc_constrain, pow(angle_temp, 3.0));
      }

      const vector3<> arc_center_point = arc_origin + (corner_ - arc_origin).normalized(arc_constrain);

      std::vector<segment> segments = { { start_position_, end_position_ } };

      const vector3<> start_vector = (corner_ - start_position_).normalized();
      const vector3<> end_vector = (end_position_ - corner_).normalized();

      const auto get_current_angle = [&]() -> real
      {
        // Calculate the max angle of the segments, which may not be equivalent.
        real largest_angle = 0.0;

        vector3<> cur_vector = (corner_ - start_position_).normalized();
        for (const segment & __restrict seg : segments)
        {
          const vector3<> seg_vector = (seg.end - seg.start).normalized();
          const real angle = cur_vector.angle_between(seg_vector);
          largest_angle = max(largest_angle, angle);

          cur_vector = seg_vector;
        }
        const vector3<> seg_vector = (end_position_ - corner_).normalized();
        const real angle = cur_vector.angle_between(seg_vector);
        largest_angle = max(largest_angle, angle);

        return largest_angle;
      };

      // For high precision, let's just use 1 degree as the target angle for now. Probably more precise than needed,
      // might cause slowdowns in the compiler. Can adjust later.
      static constexpr const real hp_angle = 1.0;
      const real min_angle = high_precision ? hp_angle : cfg.arc.min_angle;

      std::vector<segment> new_segments;
      while (angle_ < cfg.arc.max_angle && get_current_angle() >= min_angle && segments.size() < cfg.arc.max_segments)
      {
        new_segments.clear();
        new_segments.reserve(segments.size() * 2);

        bool added_segments = false;

        real current_segment_offset = 0.0;

        usize i = 0;
        segment prev_segment = { start_position_, corner_ };
        for (const segment & __restrict seg : segments)
        {
          const real test_segment_offset = (segments.size() == 1) ? 1.0 : (current_segment_offset + (seg.linear_offset * 0.5));
          current_segment_offset += seg.linear_offset;
          assert(current_segment_offset <= 1.0);
          assert(test_segment_offset <= 1.0);

          segment next_segment = (i == segments.size() - 1) ?
            segment{ corner_, end_position_ } :
            segments[i + 1];

          // Every segment here will get split in twain.
          vector3<> segment_center = mean(seg.start + seg.end);
          // We need to renormalize this center position around the arc origin.

          // Get the angle to and from this segment, to see if it needs to be subdivided.
          const real max_angle = max(
            prev_segment.angle_between(seg),
            seg.angle_between(next_segment)
          );

          const bool valid_angle = max_angle < min_angle;

          if (valid_angle || is_zero(segment_center.distance(arc_origin)))
          {
            new_segments.push_back(seg);
            prev_segment = seg;
          }
          else
          {
            real radius = arc_radius;

            if (cfg.arc.constrain_radius)
            {
              radius = slerp(arc_radius, arc_constrain, test_segment_offset);
            }

            const vector3<> arc_position = arc_origin + (segment_center - arc_origin).normalized(radius);

            // Generate two segments from this.
            bool valid_segments = !is_zero(seg.start.distance(arc_position)) && !is_zero(seg.end.distance(arc_position));
            //valid_segments = valid_segments && (angle_between({ seg.start, arc_position }, { arc_position, seg.end }) >= min_segment_angle);

            if (!valid_segments)
            {
              new_segments.push_back(seg);
              prev_segment = seg;
            }
            else
            {
              if (segments.size() != 1)
              {
                new_segments.push_back({ seg.start, arc_position });
                new_segments.back().linear_offset = seg.linear_offset * 0.5;
                new_segments.push_back({ arc_position, seg.end });
                new_segments.back().linear_offset = seg.linear_offset * 0.5;
              }
              else
              {
                new_segments.push_back({ seg.start, arc_position });
                new_segments.back().linear_offset = 1.0;
                new_segments.push_back({ arc_position, seg.end });
                new_segments.back().linear_offset = -1.0;
              }
              added_segments = true;
              prev_segment = new_segments.back();
            }
          }
          ++i;
        }

        if (!added_segments)
        {
          break;
        }

        std::swap(segments, new_segments); // Swapping instead of scoping-out and recreating can help limit allocations a bit.
      }

      real total_arc_length = 0.0;
      vector3<> arc_length_elements;
      const real original_length[2] = {
        start_position_.distance(corner_),
        end_position_.distance(corner_)
      };

      for (const segment & __restrict seg : segments)
      {
        total_arc_length += seg.start.distance(seg.end);
        arc_length_elements += (seg.end - seg.start).abs();
      }

      // Calculate new feedrate.
      const real mean_feedrate = mean(seg_feedrate_[0] + seg_feedrate_[1]);
      const real mean_acceleration = mean(acceleration_[0] + acceleration_[1]);
      const vector3<> mean_jerk = mean(jerk_[0] + jerk_[1]);

      const vector3<> in_velocity = (corner_ - start_position_).normalized(seg_feedrate_[0]);
      const vector3<> out_velocity = (end_position_ - corner_).normalized(seg_feedrate_[1]);
      //const vector3<> velocity_diff = (out_velocity - in_velocity).abs();

      // We need to solve v = at and d = vt for t... and v.
      // Two Equations:
      // T = -(sqrt(d) / sqrt(a))  V = -(sqrt(a) * sqrt(d))
      // T = (sqrt(d) / sqrt(a))  V = (sqrt(a) * sqrt(d))
      // The one that is positive is correct.
      const vector3<> element_times = {
        std::abs(std::sqrt(arc_length_elements.x) / std::sqrt(mean_acceleration)),
        std::abs(std::sqrt(arc_length_elements.y) / std::sqrt(mean_acceleration)),
        std::abs(std::sqrt(arc_length_elements.z) / std::sqrt(mean_acceleration))
      };

      const vector3<> element_velocities = {
        std::abs(std::sqrt(mean_acceleration) * std::sqrt(arc_length_elements.x)),
        std::abs(std::sqrt(mean_acceleration) * std::sqrt(arc_length_elements.y)),
        std::abs(std::sqrt(mean_acceleration) * std::sqrt(arc_length_elements.z))
      };

      // TODO apply jerk
      // TODO other calculations need to happen for other segments first.

      // Feedrate is per minute, so result must be multiplied by 60 as the result is per second.
      const real new_feedrate = min(mean_feedrate, element_velocities.length() * 60.0); // mean_feedrate;//  std::min(mean_feedrate, (total_arc_length / accel_time) * 60);

      const real adusted_extrusions[2] = {
        extrude_[0] * ((total_arc_length * 0.5) / original_length[0]),
        extrude_[1] * ((total_arc_length * 0.5) / original_length[1])
      };

      return {
        segments,
        mean_feedrate,
        total_arc_length,
        { adusted_extrusions[0], adusted_extrusions[1] },
        mean_acceleration,
        mean_jerk,
        mean(extrude_jerk_[0] + extrude_jerk_[1])
      };
    }

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

      acceleration_hint_ = mean(acceleration_[0] + acceleration_[1]);
      jerk_hint_ = mean(jerk_[0] + jerk_[1]);
      jerk_extrude_hint_ = mean(extrude_jerk_[0] + extrude_jerk_[1]);

      const vector3<> center_point = mean(start_position_ + end_position_);
      arc_origin_ = corner_ + ((center_point - corner_) * 2.0); // TODO needs to be adjusted for ovaloid arcs.
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
      std::vector<gcgg::command *> out;

      const segment_data segdata = get_segments_(cfg);
      const std::vector<segment> & __restrict segments = segdata.segments_;

      const auto adusted_extrusions = segdata.extrusion_;

      out.reserve(segments.size());

      // Segments is now all of the subdivided segments.
      const real segments_divisor = real(segments.size() + 1);
      usize iter = 1; // 1 because we want to start in the middle of the interpolation.
      for (const segment & __restrict seg : segments)
      {
        const real interpoland = real(iter++) / segments_divisor;

        const real segment_mult = seg.start.distance(seg.end) / segdata.length_;

        const real feedrate = segdata.feedrate_;
        real extrusion;
        if (adusted_extrusions[0] == 0.0 && adusted_extrusions[1] != 0.0)
        {
          // travel to extrude
          if (interpoland >= 0.5)
          {
            extrusion = (adusted_extrusions[0] + adusted_extrusions[1]) * segment_mult * 2.0;
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
            extrusion = (adusted_extrusions[0] + adusted_extrusions[1]) * segment_mult * 2.0;
          }
          else
          {
            extrusion = 0.0;
          }
        }
        else
        {
          // all else
          extrusion = (adusted_extrusions[0] + adusted_extrusions[1]) * segment_mult;
        }
        const real acceleration = segdata.acceleration_;
        const real extrude_jerk = segdata.extrude_jerk_;
        const vector3<> jerk = segdata.jerk_;

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

      const real arc_radius = radius_;
      const real arc_constrain = min(arc_radius, center_point.distance(arc_origin));

      const segment_data segdata = get_segments_(cfg);
      const std::vector<segment> & __restrict segments = segdata.segments_;

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
      else
      {
        assert(false); // what exactly are we outputting? TODO add better error system.
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
