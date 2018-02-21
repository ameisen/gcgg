#include "gcgg.hpp"
#include "movement.hpp"

extern usize failed_jerk_tests;

void segments::movement::compute_motion(const config & __restrict cfg, bool require_jerk) __restrict
{
  //motion::trapezoid::data trap_data;
  //trap_data.acceleration_ = acceleration_;
  //trap_data.end_position_ = end_position_;
  //trap_data.start_position_ = start_position_;
  //trap_data.jerk_ = jerk_hint_;
  //trap_data.speed_ = feedrate_;
  //trap_data.end_speed_ = (next_segment_) ? (next_segment_->get_velocity().length()) : 0;
  //trap_data.start_speed_ = (prev_segment_) ? prev_segment_->motion_data_.exit_feedrate_ : 0;
  //
  //trapezoid_ = new motion::trapezoid{ trap_data };

  // Calculate feedrates and trapezoidal motion data.
  const vector3<> in_velocity = (prev_segment_) ? (prev_segment_->get_vector().normalized(prev_segment_->motion_data_.exit_feedrate_)) : vector3<>::zero;
  const vector3<> velocity = get_velocity();
  const vector3<> direction = get_vector().normalized();
  const vector3<> out_velocity = (next_segment_) ? (next_segment_->get_velocity()) : vector3<>::zero;
  const vector3<> out_direction = (next_segment_) ? (next_segment_->get_vector().normalized()) : direction;

  const vector3<> jerk = jerk_hint_;
  const vector3<> acceleration = acceleration_;

  real in_feedrate;
  real out_feedrate;

  // TODO do we need to detect and handle if _extrusion_ axis-inverts? I don't know if that really happens along segments, but
  // I imagine it would cause problems otherwise.

  // in velocity is not controlled by this segment, and is just the out velocity of the previous segment, adjusted for our vector.
  if (prev_segment_)
  {
    // We need to compute a linear feedrate from the input velocity, relative to our direction.
    // Because a velocity is independent of direction, we should just be able to get the vector length.
    in_feedrate = in_velocity.length();
  }
  else
  {
    // TODO apply jerk only
    in_feedrate = 0;
  }

  // Out velocity is more difficult. We need to calculate an out velocity that is scaled in terms of the next segment.
  if (next_segment_)
  {
    // Jerk is used to make up for scaling differences since the dot product of the two segments is unlikely to be the same.
    // We can check anyways, though.
    if (direction.dot(out_direction) == 1.0)
    {
      // If they're the same direction, we can just use the out velocity, jerk-adjusted of course.
      // TODO jerk adjust
      out_feedrate = out_velocity.length();
    }
    else
    {
      const auto calculate_divisor = [](real from, real to) -> real
      {
        if (from && !to)
        {
          return 0.0;
        }
        if ((from > 0 && to < 0) || (from < 0 && to > 0))
        {
          return 0.0;
        }
        if (from && to)
        {
          return from / to;
        }
        return -1.0;
      };

      vector3<> divisors = {
        calculate_divisor(velocity.x, out_velocity.x),
        calculate_divisor(velocity.y, out_velocity.y),
        calculate_divisor(velocity.z, out_velocity.z)
      };

      // Otherwise we need to scale by component.
      if (
        direction.is_inverted(out_direction) ||
        (divisors.x == 0.0 && divisors.y == 0.0 && divisors.z == 0.0) ||
        (divisors.x == -1.0 && divisors.y == -1.0 && divisors.z == -1.0)
        )
      {
        // If there is a direction change on an axis, scaling will never work.
        // Instead, we technically have to drop speed to zero, but we jerk adjust
        // to try to limit the speed loss.
        out_feedrate = 0.0;
        // TODO jerk adjust
      }
      else
      {
        static constexpr const bool brute_force_integrate = true;

        vector3<> new_velocity;
        bool jerkable = false;

        // Calculate divisors for each axis. Then we will divide by the mean.
        // We need to handle elements that are zero, otherwise we will get ungood behavior.

        // Ignore divisors which are zero.
        real mean_div = 3.0;
        if (divisors.x == -1.0)
        {
          divisors.x = 0.0;
          mean_div -= 1.0;
        }
        if (divisors.y == -1.0)
        {
          divisors.y = 0.0;
          mean_div -= 1.0;
        }
        if (divisors.z == -1.0)
        {
          divisors.z = 0.0;
          mean_div -= 1.0;
        }

        const real mean_divisor = divisors.linear_sum() / mean_div;

        enum class integrate_step
        {
          base = 0,
          up,
          down
        };

        integrate_step step = integrate_step::base;
        vector3<> last_difference;
        real divisor = mean_divisor;

        do
        {

          // Divide all elements by the mean_divisor. This should get us a centroid velocity that can be jerked in either direction to accomodate
          // the differences.

          // If brute force integration is on, we will keep trying to change the divisor until we find something that works well.

          real prev_divisor = divisor;
          if (step == integrate_step::up)
          {
            divisor *= 1.001; // 1.0000001;
          }
          else if (step == integrate_step::down)
          {
            divisor *= 0.999; // 0.9999999;
            if (is_equal(divisor, 0.0))
            {
              break;
            }
          }

          vector3<> prev_new_velocity = new_velocity;
          new_velocity = velocity / divisor;

          const vector3<> difference = (new_velocity - out_velocity);
          // Sanity check to make sure we can accomodate the difference with jerk.
          // TODO try to handle this more gracefully.
          bool was_jerkable = jerkable;
          jerkable = (difference.abs().x <= jerk.x) && (difference.abs().y <= jerk.y) && (difference.abs().z <= jerk.z);
          
          bool jerk_valid = jerkable || !was_jerkable;

          if (step == integrate_step::base)
          {
            step = integrate_step::up;
          }
          else if (step == integrate_step::up)
          {
            // Has the result improved?
            if (!jerk_valid || last_difference.length() < difference.length())
            {
              // The previous result was better. Revert to it.
              divisor = prev_divisor;
              new_velocity = prev_new_velocity;
              jerkable = was_jerkable;
              step = integrate_step::down;
              continue;
            }
          }
          else if (step == integrate_step::down)
          {
            // Has the result improved?
            if (!jerk_valid || last_difference.length() < difference.length())
            {
              // The previous result was better. Revert to it.
              divisor = prev_divisor;
              new_velocity = prev_new_velocity;
              jerkable = was_jerkable;
              break;
            }
          }

          last_difference = difference;
        } while (cfg.options.brute_force_feedrate && require_jerk);

        if (jerkable)
        {
          out_feedrate = new_velocity.length();
        }
        else
        {
          out_feedrate = 0;
          if (require_jerk)
          {
            ++failed_jerk_tests;
          }
        }
      }
    }
  }
  else
  {
    out_feedrate = 0.0;
    // TODO apply jerk only
  }

  // Apply jerk to in/out.
  const auto calculate_jerked_feedrate = [&direction](const vector3<> & __restrict jerk) -> real
  {
    vector3<> jerk_velocity = (direction / direction.max_element()) * jerk.max_element();
    real divisor = 1.0;
    if (jerk_velocity.x > jerk.x)
    {
      divisor = min(divisor, jerk_velocity.x / jerk.x);
    }
    if (jerk_velocity.y > jerk.y)
    {
      divisor = min(divisor, jerk_velocity.y / jerk.y);
    }
    if (jerk_velocity.z > jerk.z)
    {
      divisor = min(divisor, jerk_velocity.z / jerk.z);
    }

    jerk_velocity /= divisor;

    return jerk_velocity.length();
  };
  if (in_feedrate == 0)
  {
    in_feedrate = calculate_jerked_feedrate(jerk);
  }
  if (out_feedrate == 0)
  {
    vector3<> half_jerk = jerk * 0.5;
    out_feedrate = calculate_jerked_feedrate(half_jerk);
  }

  motion_data_.calculated_ = true;
  motion_data_.entry_feedrate_ = in_feedrate;
  motion_data_.plateau_feedrate_ = feedrate_;
  motion_data_.exit_feedrate_ = out_feedrate;
}
