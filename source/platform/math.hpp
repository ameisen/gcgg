#pragma once

#include <cmath>

namespace gcgg
{
  template <typename T>
  class vector3;

  template <typename T>
  struct constants;

  template <>
  struct constants<float> final
  {
    constants() = delete;

    static constexpr const float epsilon = 0.000000001f;
    static constexpr const float pi = 3.141592653589793238f;
    static constexpr const float pi2 = pi * 2.0f;
    static constexpr const float rad_to_angle = 360.0f / pi2;
    static constexpr const float angle_to_rad = pi2 / 360.0f;
  };

  template <>
  struct constants<double> final
  {
    constants() = delete;

    static constexpr const double epsilon = 0.00000000001;
    static constexpr const double pi = 3.141592653589793238;
    static constexpr const double pi2 = pi * 2.0;
    static constexpr const double rad_to_angle = 360.0 / pi2;
    static constexpr const double angle_to_rad = pi2 / 360.0;
  };

  template <typename T>
  static constexpr T square(const T & __restrict value)
  {
    return value * value;
  }

  template <typename T>
  static constexpr T cube(const T & __restrict value)
  {
    return value * value;
  }

  static constexpr float abs(float v)
  {
    if (v < 0.0f)
    {
      return -v;
    }
    return v;
  }

  static constexpr double abs(double v)
  {
    if (v < 0.0)
    {
      return -v;
    }
    return v;
  }

  template <typename T>
  static constexpr bool is_equal(const T & __restrict A, const T & __restrict B, const T & __restrict epsilon = constants<T>::epsilon);

  template <>
  static constexpr bool is_equal<float>(const float & __restrict A, const float & __restrict B, const float & __restrict epsilon)
  {
    return abs(A - B) < epsilon;
  }

  template <>
  static constexpr bool is_equal<double>(const double & __restrict A, const double & __restrict B, const double & __restrict epsilon)
  {
    return abs(A - B) < epsilon;
  }

  template <typename T>
  static constexpr bool is_zero(const T & __restrict A, const T & __restrict epsilon = constants<T>::epsilon);

  template <>
  static constexpr bool is_zero<float>(const float & __restrict A, const float & __restrict epsilon)
  {
    return abs(A) < epsilon;
  }

  template <>
  static constexpr bool is_zero<double>(const double & __restrict A, const double & __restrict epsilon)
  {
    return abs(A) < epsilon;
  }


  template <typename T>
  static constexpr T lerp(const T & __restrict x, const T & __restrict y, real s)
  {
    return x + s * (y - x);
  }

  template <typename T>
  static constexpr T slerp(const T & __restrict x, const T & __restrict y, real s)
  {
    return x + sqrt(s) * (y - x);
  }

  template <typename T>
  static constexpr real delerp(const T & __restrict x, const T & __restrict y, const T & __restrict v)
  {
    return (x - v) / (x - y);
  }

  // TODO cannot be constexpr because of acos.
  static float dot_to_angle(float dot)
  {
    return std::acos(dot) * constants<float>::rad_to_angle;
  }

  // TODO cannot be constexpr because of acos.
  static double dot_to_angle(double dot)
  {
    return std::acos(dot) * constants<double>::rad_to_angle;
  }

  // TODO add type validation
  
  template <typename T>
  static constexpr T sum(const T & __restrict val)
  {
    return val;
  }

  template <typename T, typename... Args>
  static constexpr T sum(const T & __restrict val, Args... args)
  {
    if constexpr (sizeof...(args) == 0)
    {
      return val;
    }

    return val + sum(args...);
  }

  // TODO add type validation
  template <typename T, typename... Args>
  static constexpr T mean(const T & __restrict val, Args... args)
  {
    return sum(val, args...) / T(1 + sizeof...(args));
  }

  // TODO add type validation
  template <typename T>
  static constexpr T min(const T & __restrict val)
  {
    return val;
  }

  // TODO add type validation
  template <typename T, typename... Args>
  static constexpr T min(const T & __restrict val, Args... args)
  {
    return std::min(val, min(args...));
  }

  // TODO add type validation
  template <typename T>
  static constexpr T max(const T & __restrict val)
  {
    return val;
  }

  // TODO add type validation
  template <typename T, typename... Args>
  static constexpr T max(const T & __restrict val, Args... args)
  {
    return std::max(val, max(args...));
  }

  template <typename T>
  static constexpr T clamp(const T & __restrict val, const T & __restrict _min, const T & __restrict _max)
  {
    return std::max(std::min(val, _max), _min);
  }

  static const char * trim_float(char *buffer)
  {
    usize len = strlen(buffer);
    while (buffer[--len] == '0')
    {
      buffer[len] = '\0';
    }
    if (buffer[len] == '.')
    {
      buffer[len] = '\0';
    }

    return buffer;
  }
}
