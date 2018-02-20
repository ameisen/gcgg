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
  };

  template <>
  struct constants<double> final
  {
    constants() = delete;

    static constexpr const double epsilon = 0.00000000001;
  };

  template <typename T>
  static bool is_equal(const T & __restrict A, const T & __restrict B, const T & __restrict epsilon = constants<T>::epsilon);

  template <>
  static bool is_equal<float>(const float & __restrict A, const float & __restrict B, const float & __restrict epsilon)
  {
    return std::abs(A - B) < epsilon;
  }

  template <>
  static bool is_equal<double>(const double & __restrict A, const double & __restrict B, const double & __restrict epsilon)
  {
    return std::abs(A - B) < epsilon;
  }

  template <typename T>
  static T lerp(const T & __restrict x, const T & __restrict y, real s)
  {
    return x + s * (y - x);
  }

  template <typename T>
  static T slerp(const T & __restrict x, const T & __restrict y, real s)
  {
    return x + sqrt(s) * (y - x);
  }

  template <typename T>
  static real delerp(const T & __restrict x, const T & __restrict y, const T & __restrict v)
  {
    return (x - v) / (x - y);
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
