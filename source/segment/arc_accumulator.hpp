#pragma once

#include "movement.hpp"

namespace gcgg::segments
{
  // Linear movement without extrusion
  class arc_accumulator final : public movement
  {
    // As described elsewhere, iif the vertices of the segments lie on the sphere, we need two segments to define an arc.
    // If they do _not_ (and thus the vertices alternate between being in and out of the sphere), we need four.
    static constexpr const bool intersecting_vertices = true;
    static constexpr const size_t min_segment_count = intersecting_vertices ? 4 : 2;

    struct chord final
    {
      const movement * __restrict segments[2] = { nullptr, nullptr };

      vector3<> get_vector() const __restrict
      {
        return segments[1]->get_mean_position() - segments[0]->get_mean_position();
      }

      vector3<> get_mean_position() const __restrict
      {
        return mean(segments[0]->get_mean_position(), segments[1]->get_mean_position());
      }

      void clear() __restrict
      {
        segments[0] = nullptr;
        segments[1] = nullptr;
      }

      operator bool() const __restrict
      {
        return segments[0] != nullptr && segments[1] != nullptr;
      }

      void insert(const movement * __restrict segment) __restrict
      {
        if (!segments[0])
        {
          segments[0] = segment;
          return;
        }
        segments[1] = segment;
      }

      void shuffle() __restrict
      {
        segments[0] = segments[1];
        segments[1] = nullptr;
      }
    };

    struct direction final
    {
      enum class plane
      {
        xy = 0,
        xz,
        yz,
      } plane_;
      bool handedness_;
      vector3<> left_vector;

      direction() = default;

      direction(const chord & __restrict chord1, const chord & __restrict chord2)
      {
        vector3<> in_direction = chord1.get_vector().normalized();
        vector3<> move_direction = chord2.get_vector().normalized();
        vector3<> move_direction_abs = move_direction.abs();

        if (move_direction_abs.x >= move_direction_abs.z && move_direction_abs.y >= move_direction_abs.z)
        {
          // XY
          plane_ = plane::xy;
          const vector3<> up = { 0.0, 0.0, 1.0 };
          left_vector = in_direction.cross(up).normalized();
        }
        else if (move_direction_abs.x >= move_direction_abs.y && move_direction_abs.z >= move_direction_abs.y)
        {
          // XZ
          plane_ = plane::xz;
          const vector3<> up = { 0.0, 1.0, 0.0 };
          left_vector = in_direction.cross(up).normalized();
        }
        else if (move_direction_abs.y >= move_direction_abs.z && move_direction_abs.y >= move_direction_abs.z)
        {
          // YZ
          plane_ = plane::yz;
          const vector3<> up = { 1.0, 0.0, 0.0 };
          left_vector = in_direction.cross(up).normalized();
        }
        else
        {
          exit(1);
        }

        const real dot_product = move_direction.dot(left_vector);

        handedness_ = (dot_product <= 0.0);
      }

      bool operator == (const direction & __restrict dir) const __restrict
      {
        return plane_ == dir.plane_ && handedness_ == dir.handedness_;
      }

      bool operator != (const direction & __restrict dir) const __restrict
      {
        return plane_ != dir.plane_ || handedness_ != dir.handedness_;
      }
    };

    std::vector<movement *> m_Segments;
    real                   m_AccumAngle = 0.0;
    real                   m_MeanAngle = 0.0;
    direction              m_Direction;

  public:
    static constexpr const uint64 type = hash("arc_accumulator");

  protected:
  public:
    arc_accumulator() : movement(type) {}
    arc_accumulator(arc_accumulator && __restrict accum) :
      movement(type),
      m_Segments(std::move(accum.m_Segments)),
      m_AccumAngle(accum.m_AccumAngle),
      m_Direction(accum.m_Direction),
      m_MeanAngle(accum.m_MeanAngle)
    {}

    virtual ~arc_accumulator()
    {
      // Any segments in the accumulator have had ownership taken by the accumulator.
      for (segment * __restrict seg : m_Segments)
      {
        delete seg;
      }
    }

    arc_accumulator & operator = (arc_accumulator && __restrict accum) __restrict
    {
      m_Segments = std::move(accum.m_Segments);
      m_AccumAngle = accum.m_AccumAngle;
      m_Direction = accum.m_Direction;
      m_MeanAngle = accum.m_MeanAngle;

      return *this;
    }

    virtual std::string dump() const __restrict override final
    {
      std::string out;
      out += "arc_accumulator";

      out += movement::dump();

      return out;
    }

    const auto & get_segments() const __restrict
    {
      return m_Segments;
    }

    size_t get_segment_count() const __restrict
    {
      return m_Segments.size();
    }

    void reset() __restrict
    {
      m_Segments.clear();
      m_AccumAngle = 0.0;
      m_MeanAngle = 0.0;
    }

    bool conditional_reset() __restrict
    {
      if (m_Segments.size() < min_segment_count)
      {
        reset();
        return true;
      }
      return false;
    }

    bool consume_segment(movement & __restrict seg, const config & __restrict cfg) __restrict
    {
      // TODO needs more validation, mainly about acceleration, jerk, extrude _rate_ (extrusion amount over distance)...

      const bool result = [&]() {
        // Working on presently:
        // At four segments, we can calculate the direction and such. We want to avoid some calculations until then
        // as two segments represents a chord, not one.

        if (seg.get_vector().length() >= cfg.reg_arc_gen.max_segment_length)
        {
          return false;
        }

        if (!cfg.output.arcs_support_Z && seg.get_vector().z != 0.0)
        {
          return false;
        }

        if (m_Segments.size() == 0)
        {
          m_Segments.push_back(&seg);
          return true;
        }

        const segment * __restrict back_seg = m_Segments.back();
        if (back_seg->get_type() != seg.get_type())
        {
          // Only accept the same type, ever.
          return false;
        }

        vector3<> cur_vec = seg.get_vector().normalized();
        real angle;
        // If we have more than two points, we should calculate the test angle by chord rather than by segment. Two segments = one chord (segment.center -> segment.center).
        if (m_Segments.size() >= 2)
        {
          const movement * __restrict chord_segments[2] = {
            m_Segments[m_Segments.size() - 2],
            m_Segments[m_Segments.size() - 1]
          };
          const vector3<> chord_points[2] = {
            chord_segments[0]->get_mean_position(),
            chord_segments[1]->get_mean_position()
          };
          const vector3<> back_vec = chord_points[0].vector_to(chord_points[1]);
          angle = back_vec.angle_between(cur_vec);
        }
        else
        {
          const vector3<> back_vec = back_seg->get_vector().normalized();
          angle = back_vec.angle_between(cur_vec);
        }
        if (angle >= cfg.reg_arc_gen.max_angle)
        {
          return false;
        }

        // We shoudl scale this based upon segment size. Tiny segments will have much larger angles.
        if (m_Segments.size() >= min_segment_count)
        {
          // Is the angle within allowable limits?
          const real angle_diff = abs(m_MeanAngle - angle);
          if (angle_diff >= cfg.reg_arc_gen.max_angle_divergence)
          {
            return false;
          }
        }

        //static constexpr const real max_accumulated_angle = 360.0; // one circle
        //if ((angle + m_AccumAngle) >= max_accumulated_angle) // 360.0 - must be < 180, due to weirdness in how we calculate the radius. TODO
        //{
        //  return false;
        //}

        // TODO perform a Z test to see if we are increasing on Z at a consistent rate.

        if (m_Segments.size() >= min_segment_count)
        {
          // calculate the canonical direction value.
          // TODO recalculate potentially using segment pairs. Cannot necessarily use one segment to determine this.
          const chord chords[2] = {
            { m_Segments[m_Segments.size() - (1 + 2)], m_Segments[m_Segments.size() - (1 + 1)] },
            { m_Segments[m_Segments.size() - (1 + 0)], &seg }
          };
          const direction seg_direction = direction{ chords[0], chords[1] };

          if (m_Segments.size() == min_segment_count)
          {
            m_Direction = direction{ chords[0], chords[1] };
          }
          else
          {
            if (m_Direction != seg_direction)
            {
              return false;
            }
          }
        }

        m_Segments.push_back(&seg);

        size_t divisor = 0;
        if (m_Segments.size() >= min_segment_count)
        {
          // Calculate the accumulated angle.
          // Unlike for the mean angle, here we don't blend the chords.
          chord chords[2];

          m_AccumAngle = 0.0;
          for (const movement * __restrict cur_seg : m_Segments)
          {
            if (!chords[0])
            {
              chords[0].insert(cur_seg);
            }
            else
            {
              chords[1].insert(cur_seg);
            }

            if (chords[0] && chords[1])
            {
              const vector3<> chord_vectors[2] = {
                chords[0].get_vector().normalized(),
                chords[1].get_vector().normalized()
              };

              const real cur_angle = chord_vectors[0].angle_between(chord_vectors[1]);
              m_AccumAngle += cur_angle;

              chords[0].clear();
              chords[1].clear();
            }
          }

          chords[0].clear();
          chords[1].clear();

          // Recalculate the mean angle.
          real mean_angle = 0.0;
          for (const movement * __restrict cur_seg : m_Segments)
          {
            if (!chords[0])
            {
              chords[0].insert(cur_seg);
            }
            else
            {
              chords[1].insert(cur_seg);
            }

            if (chords[0] && chords[1])
            {
              // get_vector cur_chord

              const vector3<> chord_vectors[2] = {
                chords[0].get_vector().normalized(),
                chords[1].get_vector().normalized()
              };

              const real cur_angle = chord_vectors[0].angle_between(chord_vectors[1]);
              mean_angle += cur_angle;

              chords[0] = chords[1];
              chords[1].clear();
              ++divisor;
            }
          }
          m_MeanAngle = mean_angle / real(divisor);
        }

        return true;
      }();

      if (result == false)
      {
        // flush the accumulator.
        conditional_reset();
      }

      return result;
    }

    private:

    std::tuple<real, vector3<>> solve (const config & __restrict cfg) const __restrict
    {
      real radius = 0.0;
      vector3<> origin = { 0,0,0 };

      // Resolve the segments into on-circle subsegments.
      // We still need to resolve the start and end vertices.

      struct subsegment final
      {
        const movement * __restrict segment;
        vector3<> start_;
        vector3<> end_;
        real weight = 1.0;

        real get_length() const __restrict
        {
          return start_.distance(end_);
        }

        vector3<> get_vector() const __restrict
        {
          return end_ - start_;
        }
      };

      std::vector<subsegment> subsegments;
      real accumulated_length = 0.0;
      real weight_accum = 1.0;

      for (const auto *this_segment : m_Segments)
      {
        const vector3<> vector_along = this_segment->get_end_position() - this_segment->get_start_position();
        const real magnitude_along = vector_along.length();

        const vector3<> start = subsegments.size() ? subsegments.back().end_ : this_segment->get_start_position();
        const vector3<> vec1 = this_segment->get_start_position() + vector_along.normalized(magnitude_along * 0.25);
        const vector3<> vec2 = this_segment->get_start_position() + vector_along.normalized(magnitude_along * 0.75);

        subsegments.emplace_back(subsegment{ this_segment, start, vec1 });
        accumulated_length += subsegments.back().get_length();
        subsegments.emplace_back(subsegment{ this_segment, vec1, vec2 });
        accumulated_length += subsegments.back().get_length();
      }
      subsegments.emplace_back(subsegment{ m_Segments.back(), subsegments.back().end_, m_Segments.back()->get_end_position() });
      accumulated_length += subsegments.back().get_length();

      const real mean_length = accumulated_length / subsegments.size();

      // add weights
      for (subsegment & seg : subsegments)
      {
        const real length = seg.get_length();
        seg.weight = length / mean_length;
      }

      // Calculate origin.
      const subsegment *prev_subseg = nullptr;
      for (size_t i = 1; i < subsegments.size() - 1; ++i)
      {
        const auto &subseg = subsegments[i];

        if (!prev_subseg)
        {
          prev_subseg = &subseg;
          continue;
        }

        vector3<> vectors[2] = {
          prev_subseg->get_vector().normalized(),
          subseg.get_vector().normalized()
        };
      
        const auto plane = m_Direction.plane_;
        vector3<> up;
        switch (plane)
        {
        case direction::plane::xy:
          up = { 0.0, 0.0, 1.0 }; break;
        case direction::plane::xz:
          up = { 0.0, 1.0, 0.0 }; break;
        case direction::plane::yz:
          up = { 1.0, 0.0, 0.0 }; break;
        }

        const vector3<> points[2] = {
          mean(prev_subseg->start_, prev_subseg->end_),
          mean(subseg.start_, subseg.end_)
        };

        const vector3<> cross_vectors[2] = {
          vectors[0].cross(up).normalized(),
          vectors[1].cross(up).normalized(),
        };

        const vector3<> cross_a = (points[1] - points[0]).cross(cross_vectors[1]);

        const vector3<> cross_b = cross_vectors[0].cross(cross_vectors[1]);
        if (cross_b.length() == 0.0)
        {
          exit(1);
        }

        const real a = cross_a.length() / cross_b.length();

        vector3<> origin_vector = (a * cross_vectors[0]);
        if (origin_vector.normalized().dot(vectors[1]) < 0.0)
        {
          origin_vector = -origin_vector;
        }
        const vector3<> _orig = points[0] + origin_vector;

        origin += _orig;// *prev_subseg->weight;
        weight_accum *= prev_subseg->weight;

        prev_subseg = &subseg;
      }



      origin /= real(subsegments.size() - 2);

      for (size_t i = 1; i < subsegments.size() - 1; ++i)
      {
        const auto &subseg = subsegments[i];
        radius += origin.distance(subseg.start_) * subseg.weight;
        radius += origin.distance(subseg.end_) * subseg.weight;
      }

      radius /= real((subsegments.size() - 2) * 2);

      radius = std::max(radius, (m_Segments.front()->get_start_position().distance(m_Segments.back()->get_end_position())) / 2.0);

      return {radius, origin};
    }

    public:

    virtual void out_gcode(std::string & __restrict out, output::state & __restrict state, const config & __restrict cfg) const __restrict
    {
      real mean_radius;
      vector3<> origin;
      std::tie(mean_radius, origin) = solve(cfg);

      vector3<> start_position_ = m_Segments.front()->get_start_position();
      vector3<> end_position_ = m_Segments.back()->get_end_position();

      if (acceleration_hint_ != state.print_accel && acceleration_hint_ != 0)
      {
        state.print_accel = acceleration_hint_;

        out += "M204";

        char buffer[512];
        sprintf(buffer, "%.8f", acceleration_hint_);
        out += " P";
        out += trim_float(buffer);
        out += "\n";
      }

      bool emit_jerk_hint = false;
      if (start_position_.x != end_position_.x && jerk_hint_.x != state.jerk.x && jerk_hint_.x != 0)
      {
        emit_jerk_hint = true;
      }
      if (start_position_.y != end_position_.y && jerk_hint_.y != state.jerk.y && jerk_hint_.y != 0)
      {
        emit_jerk_hint = true;
      }
      if (start_position_.z != end_position_.z && jerk_hint_.z != state.jerk.z && jerk_hint_.z != 0)
      {
        emit_jerk_hint = true;
      }

      if (emit_jerk_hint)
      {
        out += "M205";
        char buffer[512];
        if (start_position_.x != end_position_.x && jerk_hint_.x != state.jerk.x && jerk_hint_.x != 0)
        {
          state.jerk.x = jerk_hint_.x;
          sprintf(buffer, "%.8f", jerk_hint_.x);
          out += " X";
          out += trim_float(buffer);
        }
        if (start_position_.y != end_position_.y && jerk_hint_.y != state.jerk.y && jerk_hint_.y != 0)
        {
          state.jerk.y = jerk_hint_.y;
          sprintf(buffer, "%.8f", jerk_hint_.y);
          out += " Y";
          out += trim_float(buffer);
        }
        if (start_position_.z != end_position_.z && jerk_hint_.z != state.jerk.z && jerk_hint_.z != 0)
        {
          state.jerk.z = jerk_hint_.z;
          sprintf(buffer, "%.8f", jerk_hint_.z);
          out += " Z";
          out += trim_float(buffer);
        }
        out += "\n";
      }

      std::string cmd;

      bool debug = false;

      if (is_equal(mean_radius, 47.5209018, 0.000001))
      {
        debug = true;
        out += "##############DEBUG###############\n";
      }

      if (m_Direction.handedness_)
      {
        // Counter
        out += "G3";
      }
      else
      {
        // Clock
        out += "G2";
      }

      char buffer[512];

      // Determine if we have more than half of a circle.
      bool more_than_half_circle = false;
      if (m_Segments.size() >= 3)
      {
        //const vector3<> start_position = (m_Segments.front()->get_start_position() - origin).normalized();
        //const vector3<> end_position = (m_Segments.back()->get_end_position() - origin).normalized();
        //real angle1 = atan2(start_position.y, start_position.x);
        //real angle2 = atan2(start_position.y, start_position.x);
        //angle2 = angle2;

        //const vector3<> first_vector = m_Segments.front()->get_start_position().vector_to(origin);
        //const vector3<> second_vector = m_Segments[1]->get_start_position().vector_to(origin);
        //const vector3<> last_vector = m_Segments.back()->get_start_position().vector_to(origin);

        const vector3<> first_normal = m_Segments.front()->get_start_position().vector_to(origin);
        const vector3<> second_normal = m_Segments[1]->get_start_position().vector_to(origin);
        const vector3<> last_normal = m_Segments.back()->get_end_position().vector_to(origin);

        const auto plane = m_Direction.plane_;
        vector3<> up;
        switch (plane)
        {
        case direction::plane::xy:
          up = { 0.0, 0.0, 1.0 }; break;
        case direction::plane::xz:
          up = { 0.0, 1.0, 0.0 }; break;
        case direction::plane::yz:
          up = { 1.0, 0.0, 0.0 }; break;
        }

        static constexpr const bool use_90 = false; // otherwise uses 180.
        static constexpr const bool use_135 = false;
        if constexpr (use_90)
        {
          if (last_normal.dot(first_normal) <= 0.0)
          {
            more_than_half_circle = true;
          }
        }
        if constexpr (use_135)
        {
          vector3<> binormal = first_normal.cross(up).normalized();
          if (binormal.dot(second_normal) < 0.0)
          {
            binormal = -binormal;
          }

          binormal = mean(binormal + first_normal).normalized();

          if (last_normal.dot(binormal) <= 0.0)
          {
            more_than_half_circle = true;
          }
        }
        else
        {
          vector3<> binormal = first_normal.cross(up).normalized();
          if (binormal.dot(second_normal) <= 0.0)
          {
            binormal = -binormal;
          }

          //printf("start: %f\n", (atan2(first_normal.y, first_normal.x) + constants<real>::pi) * constants<real>::rad_to_angle);
          //printf("second: %f\n", (atan2(second_normal.y, second_normal.x) + constants<real>::pi) * constants<real>::rad_to_angle);
          //printf("binormal: %f\n", (atan2(binormal.y, binormal.x) + constants<real>::pi) * constants<real>::rad_to_angle);
          //printf("last: %f\n", (atan2(last_normal.y, last_normal.x) + constants<real>::pi) * constants<real>::rad_to_angle);

          if (last_normal.dot(binormal) < 0.0)
          {
            more_than_half_circle = true;
          }
          //if (debug)
          //  more_than_half_circle = !more_than_half_circle;
        }

        //more_than_half_circle = false;

        // the above isn't really right. It _should_ work, but for some reason it does not.
        //more_than_half_circle = false;

        // We should calculate the center point of the arc for both cases (wide and narrow arc).
        // We then should choose the one that has the closest radius to the expectation.

        // We have a radius and two points. From that, we can generate two points that are R apart from our two vertices.
        // These represent the centers of two circles. We want the one closest to our origin.
        // The one that makes the largest arc is the one representing -R, the other is R.
        // We can determine that using dot products or some other magical-ness. Either-or.
#if 0

        vector3<> first_tangent = first_normal.cross(up).normalized();
        vector3<> last_tangent = last_normal.cross(up).normalized();

        chord last_end_chord;
        chord chords[2];

        real accumulated_angle = 0.0;
        for (const movement * __restrict cur_seg : m_Segments)
        {
          if (!chords[0])
          {
            chords[0].insert(cur_seg);
          }
          else
          {
            chords[1].insert(cur_seg);
          }

          if (chords[0] && chords[1])
          {
            const vector3<> chord_vectors[2] = {
              chords[0].get_vector().normalized(),
              chords[1].get_vector().normalized()
            };

            if (accumulated_angle == 0.0)
            {
              // If this is the first angle, we should add in the tangent angle as well.
              if (first_tangent.dot(chord_vectors[0]) <= 0.0)
              {
                first_tangent = -first_tangent;
              }
              const real cur_angle = first_tangent.angle_between(chord_vectors[0]);
              accumulated_angle += cur_angle;
            }

            const real cur_angle = chord_vectors[0].angle_between(chord_vectors[1]);
            accumulated_angle += cur_angle;

            chords[0].clear();
            last_end_chord = chords[1];
            chords[1].clear();
          }
        }

        // Do we add the last tangents angle diff?
        if (last_end_chord)
        {
          if (last_tangent.dot(last_end_chord.get_vector()) <= 0.0)
          {
            last_tangent = -last_tangent;
          }
          const real cur_angle = last_end_chord.get_vector().normalized().angle_between(last_tangent);
          accumulated_angle += cur_angle;
        }

        if (accumulated_angle >= 180.0)
        {
          more_than_half_circle = true;
        }
#endif
      }

      {
        sprintf(buffer, "%.8f", (more_than_half_circle) ? -mean_radius : mean_radius);
        out += " R";
        out += trim_float(buffer);
      }
      //if (start_position_.x != end_position_.x)
      {
        state.prev_position.x = state.position.x;
        state.position.x = end_position_.x;
        sprintf(buffer, "%.8f", state.position.x);
        out += " X";
        out += trim_float(buffer);
      }
      //if (start_position_.y != end_position_.y)
      {
        state.prev_position.y = state.position.y;
        state.position.y = end_position_.y;
        sprintf(buffer, "%.8f", state.position.y);
        out += " Y";
        out += trim_float(buffer);
      }
      if (start_position_.z != end_position_.z)
      {
        state.prev_position.z = state.position.z;
        state.position.z = end_position_.z;
        sprintf(buffer, "%.8f", state.position.z);
        out += " Z";
        out += trim_float(buffer);
      }

      if (feedrate_ != state.feedrate)
      {
        state.feedrate = feedrate_;

        sprintf(buffer, "%.8f", feedrate_);
        out += " F";
        out += trim_float(buffer);
      }

      out += "\n";
    }
  };
}
