#pragma once
#include "ansidecl.h"
#include <stdlib.h>
static inline void *xmalloc (size_t n) { return malloc (n); }
