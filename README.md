# sakin

Robosarema kinematics library - STB-style single-header 3-DOF leg IK/FK +
NUKE-style gait engine (ripple / amble / tripod) for hexapods and other
legged robots. Dependency-free C99, C++-compatible, ESP32-safe (no heap,
no globals, float by default).

## Features

- **Mode 1 - core kinematics** (stateless): exact geometric leg IK/FK for any
  3-DOF leg, body roll/pitch/yaw compensation, reachability status flags
  (Craig Sec.4.2 workspace-existence check instead of silent clamping)
- **Mode 2 - gait engine** (stateful): NUKE state machine with tripod, amble,
  and ripple (left/right) gaits, turn-in-place, step-boundary interpolation,
  continuous gait switching, settle-to-stand, and an optional jump behaviour
- Runtime-configurable geometry - one header serves any robot; 
  defaults included

## Quick Start

In **exactly one** translation unit:

```c
#define SAKIN_IMPLEMENTATION
#include "sakin.h"
```

```c
sakin_t bot;                       // caller-allocated, no globals
sakin_init(&bot, NULL);            // NULL = ERIC hexapod defaults
sakin_gait_select(&bot, SAKIN_GAIT_TRIPOD);

// control loop:
sakin_command(&bot, 100.0f, 0.0f, 0.0f);      // vx, vy mm/s; omega rad/s
sakin_leg_angles_t angles[SAKIN_MAX_LEGS];
sakin_update(&bot, dt_seconds, angles);       // -> send to your servos
```

Core-only usage (any 3-DOF leg, e.g. a quadruped or arm segment):

```c
sakin_config_t cfg;
sakin_config_default(&cfg);        // or fill by hand for your robot
sakin_leg_angles_t a;
int status = sakin_leg_ik(&cfg, (sakin_vec3_t){ 115.0f, 0.0f, -85.0f }, &a);
if (status & SAKIN_IK_CLIPPED_WORKSPACE) { /* target out of reach */ }
```

## Configuration Macros

| Macro | Effect |
|---|---|
| `SAKIN_USE_DOUBLE` | Use `double` instead of `float` |
| `SAKIN_MAX_LEGS` | Context capacity (default 6; set 4 for a quadruped) |
| `SAKIN_NO_GAIT` | Compile out the gait engine (core IK/FK only) |
| `SAKIN_NO_JUMP` | Compile out the jump behaviour |
| `SAKIN_STATIC` | Internal linkage for all public functions |

All geometry, joint limits, stance parameters, gait tables, deadbands, and
jump tuning live in `sakin_config_t` - see the doc block at the top of
`sakin.h` for frames/conventions and the theory references (Craig ch. 4;
De Luca, *Robotics 1: Inverse kinematics*).

## Tests

```sh
cc -std=c99 -Wall -Wextra -Werror -pedantic tests/test_sakin.c -lm -o test_sakin
./test_sakin        # 25 checks, 0 failures
```

`tests/cpp_compile_check.cpp` verifies C++ builds (Arduino/ESP32);
`tests/config_matrix.c` verifies the `SAKIN_*` macro combinations.
