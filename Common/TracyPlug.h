#pragma once

// Empty defines for Tracy macros, can be used if Tracy is not available.

/*
#ifdef TRACY_ENABLE
#include "Tracy/tracy/Tracy.hpp"
#else
#include "TracyPlug.h"
#endif
*/

#ifndef ZoneScopedN
#define ZoneScopedN(a)
#endif

#ifndef FrameMark
#define FrameMark
#endif
