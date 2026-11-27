#pragma once
#define ATTRIBUTE_NORETURN __attribute__((noreturn))
#define ATTRIBUTE_UNUSED __attribute__((unused))
#define ATTRIBUTE_PRINTF(a,b)
#define ATTRIBUTE_PRINTF_1
#define ATTRIBUTE_PRINTF_2
#define ATTRIBUTE_FPTR_PRINTF_2
#define ATTRIBUTE_FPTR_PRINTF_3
#define ATTRIBUTE_PRINTF_3
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))
#endif
#define ATTRIBUTE_FALLTHROUGH __attribute__((fallthrough))
