#pragma once

namespace gcgg::gc
{
  struct command
  {
    std::string _cmd_string;
    std::unordered_map<std::string, real> _arguments;

    bool has_argument(const std::string & __restrict key) const __restrict
    {
      return _arguments.find(key) != _arguments.end();
    }

    bool has_argument_not(const std::string & __restrict key, real comparand) const __restrict
    {
      const auto iter = _arguments.find(key);
      if (iter == _arguments.end())
      {
        return false;
      }
      return iter->second != comparand;
    }

    template <typename T = real>
    T get_argument(const std::string & __restrict key, T default_value = T(0)) const __restrict;

    template <>
    real get_argument<real>(const std::string & __restrict key, real default_value) const __restrict
    {
      const auto iter = _arguments.find(key);
      if (iter == _arguments.end())
      {
        return default_value;
      }
      return iter->second;
    }

    template <>
    uint get_argument<uint>(const std::string & __restrict key, uint default_value) const __restrict
    {
      const auto iter = _arguments.find(key);
      if (iter == _arguments.end())
      {
        return default_value;
      }
      if (iter->second < 0.0)
      {
        printf("unsigned integer argument is less than 0. Aborting.");
        exit(1);
      }
      return uint(llround(iter->second));
    }

    template <>
    int get_argument<int>(const std::string & __restrict key, int default_value) const __restrict
    {
      const auto iter = _arguments.find(key);
      if (iter == _arguments.end())
      {
        return default_value;
      }
      return int(llround(iter->second));
    }

    template <>
    uint64 get_argument<uint64>(const std::string & __restrict key, uint64 default_value) const __restrict
    {
      const auto iter = _arguments.find(key);
      if (iter == _arguments.end())
      {
        return default_value;
      }
      if (iter->second < 0.0)
      {
        printf("unsigned integer argument is less than 0. Aborting.");
        exit(1);
      }
      return uint64(llround(iter->second));
    }

    template <>
    int64 get_argument<int64>(const std::string & __restrict key, int64 default_value) const __restrict
    {
      const auto iter = _arguments.find(key);
      if (iter == _arguments.end())
      {
        return default_value;
      }
      return int64(llround(iter->second));
    }
  };
}
