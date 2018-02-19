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

  template <>
  static real delerp<vector3<real>>(const vector3<real> & __restrict x, const vector3<real> & __restrict y, const vector3<real> & __restrict v)
  {
    vector3<> a = (x - v);
    vector3<> b = (x - y);
    if (b.x == 0.0)
    {
      if (b.y != 0.0)
      {
        a.x = a.y;
        b.x = b.y;
      }
      else
      {
        a.x = a.z;
        b.x = b.z;
      }
    }
    if (b.y == 0.0)
    {
      a.y = a.x;
      b.y = b.x;
    }
    if (b.z == 0.0)
    {
      a.z = a.x;
      b.z = b.x;
    }

    return (a / b).min_element();
  }
}
