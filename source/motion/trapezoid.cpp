#include "gcgg.hpp"
#include "trapezoid.hpp"

motion::trapezoid::trapezoid(const data & __restrict init) :
  start_position_(init.start_position_), end_position_(init.end_position_)
{
  // From this, we need to calculate the trapezoidal movement data.
  // This is used implicitly in gcode2 and ggc, but is also used internally
  // for calculating the proper parameters for things like arcs.

  const real distance = start_position_.distance(end_position_);

  // TODO handle jerk?
  const real ramp_speed_diff[2] = {
    init.speed_ - init.start_speed_,
    init.end_speed_ - init.speed_
  };

  // Calculate acceleration.
  vector3<> axial_acceleration = (init.end_position_ - init.start_position_).normalized();
  axial_acceleration /= axial_acceleration.max_element();
  axial_acceleration *= init.acceleration_;

  const real linear_acceleration = axial_acceleration.length();

  // Calculate how long to accelerate/decelerate to the appropriate speeds.
  const real ramp_times[2] = {
    std::abs(ramp_speed_diff[0]) / linear_acceleration,
    std::abs(ramp_speed_diff[1]) / linear_acceleration,
  };

  // Calculate the ramp distances
  const real ramp_distance[2] = {
    ramp_times[0] ? 
      (init.start_speed_ * ramp_times[0]) + (0.5 * linear_acceleration * (ramp_times[0] * ramp_times[0])) :
      0,
    ramp_times[1] ?
      (init.speed_ * ramp_times[1]) + (0.5 * linear_acceleration * (ramp_times[1] * ramp_times[1])) :
      0
  };

  const real total_ramp_distance = ramp_distance[0] + ramp_distance[1];
  // If the total ramp distance is greater than the distance of the segment, we cannot use trapezoidal motion. Instead, calculate a triangle.
  const bool calculate_triangle = total_ramp_distance > distance;

  if (!calculate_triangle)
  {
    // Trapezoid
    const real plateau_distance = distance - total_ramp_distance;
    const real plateau_time = plateau_distance / init.speed_;

    plateau_speed_ = init.speed_;
    start_speed_ = init.start_speed_;
    end_speed_ = init.end_speed_;
    ramp_time_[0] = ramp_times[0];
    ramp_time_[1] = ramp_times[1];
    ramp_distance_[0] = ramp_distance[0];
    ramp_distance_[1] = ramp_distance[1];
    plateau_time_ = plateau_time;
    plateau_distance_ = plateau_distance;
  }
  else
  {
    const real plateau_distance = 0;
    const real plateau_time = 0;

    plateau_time_ = plateau_time;
    plateau_distance_ = plateau_distance;

    // Triangle
    // If they're equal, the triangle is already solved.
    if (total_ramp_distance != distance)
    {
      // Otherwise we need to calculate shorter triangles that don't reach peak speed.

      // distance = rdistance0 + rdistance1
      // rdistance0 = (speed * time0) + (0.5 * accel * (time0^2))
      // rdistance1 = (speed * time1) + (0.5 * accel * (time1^2))
      // time0 = speed_diff0 / accel
      // time1 = speed/diff1 / accel
      // speed_diff0 = speed - start_speed
      // speed_diff1 = end_speed - speed
      // variable is speed
      //
      // time0 = ((speed - start_speed) / accel)
      // time1 = ((end_speed - speed) / accel)
      //
      // rdistance0 = (speed * ((speed - start_speed) / accel)) + (0.5 * accel * (((speed - start_speed) / accel)^2))
      // rdistance1 = (speed * ((end_speed - speed) / accel)) + (0.5 * accel * (((end_speed - speed) / accel)^2))
      //
      // distance = (speed * ((speed - start_speed) / accel)) + (0.5 * accel * (((speed - start_speed) / accel)^2)) +
      // (speed * ((end_speed - speed) / accel)) + (0.5 * accel * (((end_speed - speed) / accel)^2))
      // 
      // https://www.wolframalpha.com/input/?i=b+%3D+(a+*+((a+-+d)+%2F+g))+%2B+(0.5+*+g+*+(((a+-+d)+%2F+g)%5E2))+%2B+(a+*+((f+-+a)+%2F+g))+%2B+(0.5+*+g+*+(((f+-+a)+%2F+g)%5E2))+solve+for+a
      //
      // We get two solutions out of this:
      // speed = 0.5 * (2 * start_speed - sqrt(2) * sqrt(2 * distance * accel + start_speed ^ 2 - end_speed ^ 2))
      // speed = 0.5 * (sqrt(2) * sqrt(2 * distance * accel + start_speed ^ 2 - end_speed ^ 2) + 2 * start_speed)

      const auto sq = [](real value)->real
      {
        return value * value;
      };

      const real speed_solutions[2] = {
        0.5 * (2.0 * init.start_speed_ - sqrt(2.0) * sqrt(2.0 * distance * linear_acceleration + sq(init.start_speed_) - sq(init.end_speed_))),
        0.5 * (sqrt(2.0) * sqrt(2.0 * distance * linear_acceleration + sq(init.start_speed_) - sq(init.end_speed_)) + 2.0 * init.start_speed_)
      };

      const real best_speed = max(speed_solutions[0], speed_solutions[1]);

      const real new_ramp_speed_diff[2] = {
        best_speed - init.start_speed_,
        init.end_speed_ - best_speed
      };

      const real new_ramp_times[2] = {
        new_ramp_speed_diff[0] / linear_acceleration,
        new_ramp_speed_diff[1] / linear_acceleration,
      };

      const real new_ramp_distance[2] = {
        (init.start_speed_ * new_ramp_times[0]) + (0.5 * linear_acceleration * (new_ramp_times[0] * new_ramp_times[0])),
        (best_speed * new_ramp_times[1]) + (0.5 * linear_acceleration * (new_ramp_times[1] * new_ramp_times[1]))
      };

      plateau_speed_ = best_speed; // sort of irrelevant since there is no plateau. It's the target speed for the triangle.
      start_speed_ = init.start_speed_;
      end_speed_ = init.end_speed_;
      ramp_time_[0] = new_ramp_times[0];
      ramp_time_[1] = new_ramp_times[1];
      ramp_distance_[0] = new_ramp_distance[0];
      ramp_distance_[1] = new_ramp_distance[1];
    }
  }
}
