#pragma once

#include <cassert>

#define OS Windows

#define __likely(c) (c)
#define __unlikely(c) (c)
#define __unreachable __assume(0)
#define nodefault default: { __unreachable; }

#define xassert(c) assert(c)
