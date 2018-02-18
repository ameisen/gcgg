#include "gcgg.hpp"
#include "movement.hpp"

void segments::movement::compute_motion() __restrict
{
  // Calculate feedrates and trapezoidal motion data.
  const vector3<> in_velocity = (prev_segment_) ? (prev_segment_->get_velocity()) : vector3<>::zero;
  const vector3<> velocity = get_velocity();
  const vector3<> direction = (end_position_ - start_position_).normalized();
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
      // Otherwise we need to scale by component.
      if (direction.is_inverted(out_direction))
      {
        // If there is a direction change on an axis, scaling will never work.
        // Instead, we technically have to drop speed to zero, but we jerk adjust
        // to try to limit the speed loss.
        out_feedrate = 0.0;
        // TODO jerk adjust
      }
      else
      {
#if 0
        // We start by scaling by the greatest element to match it in the output.
        real cur_major;
        real out_major;
        if (velocity.x >= velocity.y && velocity.x >= velocity.z)
        {
          // X is the major velocity
          cur_major = velocity.x;
          out_major = out_velocity.x;
        }
        else if (velocity.y >= velocity.x && velocity.y >= velocity.z)
        {
          // Y is the major velocity
          cur_major = velocity.y;
          out_major = out_velocity.y;
        }
        else if (velocity.z >= velocity.x && velocity.z >= velocity.y)
        {
          // Z is the major velocity
          cur_major = velocity.z;
          out_major = out_velocity.z;
        }

        const real divisor = out_major / cur_major;
        const vector3<> new_velocity = velocity / divisor;
        // Now we have to further adjust new_velocity to accomodate jerk.
#else
        // Calculate divisors for each axis. Then we will divide by the mean.
        // We need to handle elements that are zero, otherwise we will get ungood behavior.

        const auto calculate_divisor = [](real from, real to) -> real
        {
          if (from && to)
          {
            return to / from;
          }
          return 0.0;
        };

        const vector3<> divisors = {
          calculate_divisor(velocity.x, out_velocity.x),
          calculate_divisor(velocity.y, out_velocity.y),
          calculate_divisor(velocity.z, out_velocity.z)
        };

        // TODO should we be using is_equal?
        assert(divisors.x != 0.0 || divisors.y != 0.0 || divisors.z != 0.0);

        // Ignore divisors which are zero.
        real mean_div = 3.0;
        if (divisors.x == 0.0)
        {
          mean_div -= 1.0;
        }
        if (divisors.y == 0.0)
        {
          mean_div -= 1.0;
        }
        if (divisors.z == 0.0)
        {
          mean_div -= 1.0;
        }

        const real mean_divisor = divisors.linear_sum() / mean_div;

        // Divide all elements by the mean_divisor. This should get us a centroid velocity that can be jerked in either direction to accomodate
        // the differences.

        vector3<> new_velocity = velocity / mean_divisor;
#endif

        const vector3<> difference = (new_velocity - out_velocity);
        // Sanity check to make sure we can accomodate the difference with jerk.
        // TODO try to handle this more gracefully.
        assert(difference.abs().x <= jerk.x);
        assert(difference.abs().y <= jerk.y);
        assert(difference.abs().z <= jerk.z);

        out_feedrate = new_velocity.length();
      }
    }
  }
  else
  {
    out_feedrate = 0.0;
    // TODO apply jerk only
  }

  motion_data_.calculated_ = true;
  motion_data_.entry_feedrate_ = in_feedrate;
  motion_data_.plateau_feedrate_ = feedrate_;
  motion_data_.exit_feedrate_ = out_feedrate;
}
