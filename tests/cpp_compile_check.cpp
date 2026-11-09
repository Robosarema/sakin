// SPDX-License-Identifier: 0BSD
// cpp_compile_check.cpp - sakin.h must compile and link as C++ (Arduino/ESP32
// builds are C++). Also exercises the config matrix at compile time.
//
// Build variants (all must pass):
//   c++ -std=c++11 -Wall -Wextra -Werror cpp_compile_check.cpp
//   cc  -std=c99   -Wall -Wextra -Werror -DSAKIN_NO_GAIT -DSAKIN_NO_JUMP
//       -DSAKIN_USE_DOUBLE -DSAKIN_STATIC -DSAKIN_MAX_LEGS=4 config_matrix.c

#define SAKIN_IMPLEMENTATION
#include "../sakin.h"

int main() {
  sakin_t bot;
  sakin_init(&bot, 0); // NULL config
  sakin_gait_select(&bot, SAKIN_GAIT_RIPPLE);
  sakin_command(&bot, 50.0f, 0.0f, 0.0f);
  sakin_body_height(&bot, 10.0f); // body-Z channel must exist in C++

  sakin_leg_angles_t angles[SAKIN_MAX_LEGS];
  sakin_update(&bot, 1.0f / 60.0f, angles);
  return sakin_jump_active(&bot); // 0 when idle
}
