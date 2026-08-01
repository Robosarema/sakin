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

## Customizing for N Legs

The library defaults to the **ERIC hexapod** (`sakin_config_default()`),
which is the 6-legged robot this club actually built. Defaults: 6 legs,
coxa 35 mm, femur 82 mm, tibia 80 mm, NUKE gait tables (ripple / amble /
tripod), body-frame mounts at +/-55 mm.

The ERIC firmware constants map to `sakin_config_t` like this:

| ERIC constant (`nuke.h`) | sakin field | notes |
|---|---|---|
| `LEG_COUNT 6` | `cfg.leg_count` | active legs to solve each tick |
| `L_COXA 35` / `L_FEMUR 82` / `L_TIBIA 80` | `coxa_len` / `femur_len` / `tibia_len` | link lengths, mm |
| `X_COXA 55` / `Y_COXA 55` | `cfg.mounts[].x/y` | body-frame mount positions |
| leg order RF,RR,LF,LR,RM,LM | `SAKIN_LEG_*` (same order) | index into `mounts[]` |
| `gaitLegNo[]`, `pushSteps`, `stepsInCycle` | `cfg.gaits[i].leg_no[]` etc. | identical NUKE values |
| `STD_TRANSITION 1000` / TRIPOD 65 ms | `cfg.gaits[i].tran_ms` | NOTE: see below |

> NOTE: **Two things to verify against your real robot:**
> 1. `tran_ms` - ERIC sets TRIPOD to 65 ms (sakin matches) but RIPPLE/AMBLE
>    to the `STD_TRANSITION = 1000` placeholder. sakin ships 120/150 ms,
>    which is the *tuned* value; re-check what actually walks well.
> 2. `M_COXA 100` is **defined in `nuke.h` but never used** - `doIK()` passes
>    `Y_COXA` (55) for the middle legs too. If your physical chassis really
>    mounts the middle legs at +/-100 mm, the dead constant is one reason the
>    old gait looked wrong; set `cfg.mounts[4].y` / `mounts[5].y` to match
>    the actual chassis.

### 4 legs (quadruped)

Compile-time capacity must be set *before* the include and identically in
every translation unit (it changes `sakin_t`'s layout):

```c
#define SAKIN_MAX_LEGS 4
#define SAKIN_IMPLEMENTATION
#include "sakin.h"

static void my_config(sakin_config_t* c) {
  sakin_config_default(c);                 // fill shared geometry + limits
  c->leg_count = 4;
  // RF, RR, LF, LR -- yaw = direction of the leg's outward radial (body frame)
  c->mounts[0] = (sakin_mount_t){ 55, -55, -20, -0.785f }; // RF: yaw -45 deg
  c->mounts[1] = (sakin_mount_t){ -55, -55, -20, -2.356f }; // RR: yaw -135 deg
  c->mounts[2] = (sakin_mount_t){ 55, 55, -20, 0.785f };   // LF: yaw 45 deg
  c->mounts[3] = (sakin_mount_t){ -55, 55, -20, 2.356f };  // LR: yaw 135 deg

  // trot: diagonal pairs (RF+LR, then LF+RR) swing together, 4-step cycle.
  // leg_no[l] = step index at which leg l enters swing; 0..steps_in_cycle-1
  c->gaits[SAKIN_GAIT_TRIPOD].leg_no[0] = 0;  // RF
  c->gaits[SAKIN_GAIT_TRIPOD].leg_no[1] = 2;  // RR
  c->gaits[SAKIN_GAIT_TRIPOD].leg_no[2] = 2;  // LF
  c->gaits[SAKIN_GAIT_TRIPOD].leg_no[3] = 0;  // LR
  c->gaits[SAKIN_GAIT_TRIPOD].push_steps = 2;
  c->gaits[SAKIN_GAIT_TRIPOD].steps_in_cycle = 4;
  c->gaits[SAKIN_GAIT_TRIPOD].tran_ms = 100.0f;
}
```

> NOTE: The **default gait tables are hexapod-only** (ripple/amble assume 6
> legs). A 4-legged robot must rewrite at least one table - the trot above
> is a complete, walkable starting point. `SAKIN_GAIT_RIPPLE`/`AMBLE` slots
> are then just spare patterns you can fill in later.

### 8 legs

```c
#define SAKIN_MAX_LEGS 8
#define SAKIN_IMPLEMENTATION
#include "sakin.h"

static void my_config(sakin_config_t* c) {
  sakin_config_default(c);      // fills shared geometry + the first 6 mounts
  c->leg_count = 8;
  // legs 0..5 already come from the hexapod defaults; legs 6,7 are
  // ZERO-FILLED by sakin_config_default and MUST be supplied by you:
  c->mounts[6] = (sakin_mount_t){ 100, 0, -20, 1.5708f };  // extra right
  c->mounts[7] = (sakin_mount_t){ -100, 0, -20, -1.5708f }; // extra left

  // alternating wave: legs 0,2,4,6 swing together, then 1,3,5,7
  for (int i = 0; i < SAKIN_GAIT_COUNT; ++i)
    for (int l = 0; l < 8; ++l)
      c->gaits[i].leg_no[l] = (l % 2 == 0) ? 0 : 2;
}
```

> NOTE: `sakin_config_default()` only fills the **first 6** mounts and gait-table
> columns (`n = min(SAKIN_MAX_LEGS, 6)`); everything beyond leg 6 is
> zero-filled and must be set by you.

### Using a subset without recompiling

Leave `SAKIN_MAX_LEGS = 6` and set `cfg.leg_count = 4`: `sakin_update()` only
solves the first `leg_count` legs, so the extra capacity is simply ignored.
Use this when you don't want to touch the compile-time layout.

### Rule of thumb

| Want | Change | Recompile needed? |
|---|---|---|
| fewer active legs, same chassis | `cfg.leg_count` only | no |
| different link lengths / limits / mounts | `cfg` fields | no |
| different leg capacity | `SAKIN_MAX_LEGS` | **yes - all files** |
| new gait pattern | `cfg.gaits[]` | no |

`SAKIN_MAX_LEGS` must be `#define`d identically in every file that includes
`sakin.h` - mismatches silently corrupt the shared struct layout (ODR trap).

## Tests

```sh
cc -std=c99 -Wall -Wextra -Werror -pedantic tests/test_sakin.c -lm -o test_sakin
./test_sakin        # 25 checks, 0 failures
```

`tests/cpp_compile_check.cpp` verifies C++ builds (Arduino/ESP32);
`tests/config_matrix.c` verifies the `SAKIN_*` macro combinations.
