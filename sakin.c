/* SPDX-License-Identifier: 0BSD
 *
 * sakin.c - optional convenience translation unit.
 *
 * sakin is STB-style: the implementation lives in sakin.h and is compiled when
 * SAKIN_IMPLEMENTATION is defined in EXACTLY ONE translation unit. If you don't
 * want to add that #define to one of your own .c files, just add this file to
 * your build instead.
 *
 * Use ONE of these, never both (both = duplicate symbols at link time):
 *
 *   a) add sakin.c to your build system, or
 *   b) #define SAKIN_IMPLEMENTATION before #include "sakin.h" in one .c file.
 *
 * Configuration macros (SAKIN_MAX_LEGS, SAKIN_USE_DOUBLE, SAKIN_NO_GAIT,
 * SAKIN_NO_JUMP, ...) must be identical in every translation unit. When you use
 * this file, pass them on the compiler command line (they are applied before
 * this file is read), for example:
 *
 *     cc -DSAKIN_MAX_LEGS=4 -DSAKIN_USE_DOUBLE sakin.c ...
 *
 * Do not rely on a #define in another header to configure sakin.c unless that
 * header is force-included ahead of it.
 */

#ifndef SAKIN_IMPLEMENTATION
#define SAKIN_IMPLEMENTATION
#endif

#include "sakin.h"
