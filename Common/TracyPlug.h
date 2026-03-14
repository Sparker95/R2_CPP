#pragma once

// Must be included instead of Tracy.hpp

#ifdef TRACY_ENABLE
#include "Tracy/tracy/Tracy.hpp"
#else

#ifndef ZoneScopedN
#define ZoneScopedN(a)
#endif

#ifndef FrameMark
#define FrameMark
#endif

#endif // if TRACY_ENABLE


