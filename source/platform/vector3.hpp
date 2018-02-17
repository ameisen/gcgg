#pragma once

#include <cmath>

namespace gcgg
{
  template <typename T = real>
  class vector3 final
  {
  public:
    union
    {
      T values_[3] = { 0.0, 0.0, 0.0 };
      struct
      {
        T x;
        T y;
        T z;
      };
    };

  public:
    constexpr vector3() = default;
    constexpr vector3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    constexpr vector3(T _x, T _y) : vector3(_x, _y, 0.0) {}
    constexpr vector3(T val) : vector3(val, val, val) {}
    constexpr vector3(const T(&__restrict data)[2]) : vector3(data[0], data[1]) {}
    constexpr vector3(const T(&__restrict data)[3]) : vector3(data[0], data[1], data[2]) {}
    constexpr vector3(const vector3 & __restrict data) : vector3(data.x, data.y, data.z) {}

    constexpr T length_sq() const __restrict
    {
      return x * x + y * y + z * z;
    }

    constexpr T length() const __restrict
    {
      return std::sqrt(length_sq());
    }

    constexpr void normalize(T magnitude = 1.0) __restrict
    {
      const T current_magnitude_recip = magnitude / length();

      x *= current_magnitude_recip;
      y *= current_magnitude_recip;
      z *= current_magnitude_recip;
    }

    constexpr vector3 normalized(T magnitude = 1.0) const __restrict
    {
      const T current_magnitude_recip = magnitude / length();

      return *this * current_magnitude_recip;
    }

    constexpr vector3 operator + (const vector3 & __restrict vec) const __restrict
    {
      return { x + vec.x, y + vec.y, z + vec.z };
    }

    constexpr vector3 operator - (const vector3 & __restrict vec) const __restrict
    {
      return { x - vec.x, y - vec.y, z - vec.z };
    }

    constexpr vector3 operator * (const vector3 & __restrict vec) const __restrict
    {
      return { x * vec.x, y * vec.y, z * vec.z };
    }

    constexpr vector3 operator / (const vector3 & __restrict vec) const __restrict
    {
      return { x / vec.x, y / vec.y, z / vec.z };
    }

    constexpr vector3 operator % (const vector3 & __restrict vec) const __restrict
    {
      return { std::fmod(x, vec.x), std::fmod(y, vec.y), std::fmod(z, vec.z) };
    }

    constexpr vector3 operator * (real val) const __restrict
    {
      return { x * val, y * val, z * val };
    }

    constexpr vector3 operator / (real va) const __restrict
    {
      return { x / val, y / val, z / val };
    }

    constexpr vector3 operator % (real va) const __restrict
    {
      return { std::fmod(x, val), std::fmod(y, val), std::fmod(z, val) };
    }

    constexpr vector3 & operator += (const vector3 & __restrict vec) __restrict
    {
      x += vec.x;
      y += vec.y;
      z += vec.z;
      return *this;
    }

    constexpr vector3 & operator -= (const vector3 & __restrict vec) __restrict
    {
      x -= vec.x;
      y -= vec.y;
      z -= vec.z;
      return *this;
    }

    constexpr vector3 & operator *= (const vector3 & __restrict vec) __restrict
    {
      x *= vec.x;
      y *= vec.y;
      z *= vec.z;
      return *this;
    }

    constexpr vector3 & operator /= (const vector3 & __restrict vec) __restrict
    {
      x /= vec.x;
      y /= vec.y;
      z /= vec.z;
      return *this;
    }

    constexpr vector3 & operator %= (const vector3 & __restrict vec) __restrict
    {
      x = std::fmod(x, vec.x);
      y = std::fmod(y, vec.y);
      z = std::fmod(z, vec.z);
      return *this;
    }

    constexpr vector3 & operator *= (real val) __restrict
    {
      x *= val;
      y *= val;
      z *= val;
    }

    constexpr vector3 & operator /= (real va) __restrict
    {
      x /= val;
      y /= val;
      z /= val;
    }

    constexpr vector3 & operator %= (real va) __restrict
    {
      x = std::fmod(x, val);
      y = std::fmod(y, val);
      z = std::fmod(z, val);
      return *this;
    }

    constexpr vector3 operator + () const __restrict
    {
      return *this;
    }

    constexpr vector3 operator - () const __restrict
    {
      return { -x, -y, -z };
    }

    constexpr T dot(const vector3 & __restrict vec) const __restrict
    {
      return x * vec.x + y * vec.y + z * vec.z;
    }

    constexpr T distance_sq(const vector3 & __restrict vec) const __restrict
    {
      return (*this - vec).length_sq();
    }

    constexpr T distance(const vector3 & __restrict vec) const __restrict
    {
      return (*this - vec).length();
    }

    constexpr friend vector3 operator * (real val, const vector3 & __restrict vec)
    {
      return { val * vec.x, val * vec.y, val * vec.z };
    }

    constexpr friend vector3 operator / (real val, const vector3 & __restrict vec)
    {
      return { val / vec.x, val / vec.y, val / vec.z };
    }

    constexpr friend vector3 operator % (real val, const vector3 & __restrict vec)
    {
      return { fmod(val, vec.x), fmod(val, vec.y), fmod(val, vec.z) };
    }

    constexpr bool operator == (const vector3 & __restrict vec) const __restrict
    {
      // TODO
      return x == vec.x && y == vec.y && z == vec.z;
    }

    constexpr bool operator != (const vector3 & __restrict vec) const __restrict
    {
      // TODO
      return x != vec.x || y != vec.y || z != vec.z;
    }

    constexpr vector3 abs() const __restrict
    {
      return { std::abs(x), std::abs(y), std::abs(z) };
    }
  };
}
