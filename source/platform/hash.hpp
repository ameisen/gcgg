#pragma once

namespace gcgg
{
  namespace _fnv
  {
    static constexpr const uint64 offset_basis = 0XCBF29CE484222325;
    static constexpr const uint64 prime = 0X100000001B3;
  }

  static uint64 hash(const std::string & __restrict str)
  {
    uint64 out = _fnv::offset_basis;
    for (char c : str)
    {
      out ^= uint8(c);
      out *= _fnv::prime;
    }
    return out;
  }

  template <usize N>
  static constexpr uint64 hash(const char(&str)[N])
  {
    uint64 out = _fnv::offset_basis;
    for (usize i = 0; i < (N - 1); ++i)
    {
      out ^= uint8(str[i]);
      out = (out * _fnv::prime);
    }
    return out;
  }
}
