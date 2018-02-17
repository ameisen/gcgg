#pragma once

#if defined(_WIN32)
# include "platform/windows/windows.hpp"
#else
# error Unsupported Platform
#endif

#include "math.hpp"
#include "hash.hpp"
#include "vector3.hpp"

#include "math_post.hpp"
