#pragma once

namespace gcgg
{
  template <>
  struct constants<vector3<>> final
  {
    constants() = delete;

    static constexpr const vector3<> epsilon = { constants<double>::epsilon, constants<double>::epsilon, constants<double>::epsilon };
  };

  template <>
  static bool is_equal<vector3<>>(const vector3<> & __restrict A, const vector3<> & __restrict B, const vector3<> & __restrict epsilon)
  {
    const vector3<> result = (A - B).abs();
    return result.x < epsilon.x && result.y < epsilon.y && result.z < epsilon.z;
  }
}
