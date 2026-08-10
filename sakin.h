/* sakin - kinematics + gait engine for legged robots - Robosarema
 * no warranty implied; use at your own risk

   Do this:
      #define SAKIN_IMPLEMENTATION
   before you include this file in *one* C or C++ file to create the implementation.

   // i.e. it should look like this:
   #include ...
   #include ...
   #define SAKIN_IMPLEMENTATION
   #include "sakin.h"

   You can #define SAKIN_USE_DOUBLE before the #include to compute in double
   precision. The default is float, which is what you want on ESP32-class
   single-precision FPUs (doubles are software-emulated there).

   Other #defines you can set before the #include:

      SAKIN_MAX_LEGS ... context array capacity, default 6. Set to 4 for a
                         quadruped to shrink sakin_t, then fill the mounts
                         and gait tables yourself (the defaults are the
                         built-in 6-leg set). Must be defined identically in
                         every translation unit -- it changes struct layout.
                         For other leg counts (4/8/...) see "Customizing for
                         N Legs" in the README, including ready-made mounts
                         and gait-table examples.

      SAKIN_NO_GAIT .... leave out the gait engine, keeping only the core
                         leg IK/FK (mode 1 below).

      SAKIN_NO_JUMP .... leave out the jump behaviour.

      SAKIN_STATIC ..... give all public functions internal linkage, for
                         single-file builds.

   QUICK NOTES:
      Primarily aimed at hexapods (hexapod geometry built in) but the core
      leg solver works for any 3-DOF leg on any robot.

      Two layers, use either or both:

         mode 1, core kinematics (stateless):
            sakin_leg_ik / sakin_fk .... one 3-DOF leg, exact geometric solver
            sakin_body_rotate_inv ...... body roll/pitch/yaw compensation
            sakin_stance_foot .......... nominal standing foothold

         mode 2, gait engine (stateful, caller-owned context, no globals):
            sakin_init / sakin_reset_pose
            sakin_gait_select .......... tripod, amble, ripple/left/right
            sakin_command .............. velocity: vx, vy mm/s, omega rad/s
            sakin_body_pose / shift / height . body orientation / XY and Z
                                                translation (Z + = body drops)
            sakin_update ............... advance clock, solve all legs
            sakin_jump_start ........... optional jump behaviour

      Example, walking at 50 Hz:

         sakin_t bot;
         sakin_init(&bot, NULL);                    // NULL = built-in defaults
         sakin_gait_select(&bot, SAKIN_GAIT_TRIPOD);

         for (;;) {
            sakin_command(&bot, 100, 0, 0);         // vx, vy mm/s, omega rad/s
            sakin_leg_angles_t angles[SAKIN_MAX_LEGS];
            sakin_update(&bot, 0.02f, angles);      // -> angles to your servos
            ...
         }

      Example, one leg of anything:

         sakin_config_t cfg;
         sakin_config_default(&cfg);                // or fill it by hand
         sakin_leg_angles_t a;
         int status = sakin_leg_ik(&cfg, (sakin_vec3_t){ 115, 0, -85 }, &a);
         if (status & SAKIN_IK_CLIPPED_WORKSPACE) { // out of reach, saturated
            ...
         }

   DOCUMENTATION

   Conventions (one frame everywhere -- mixed frames are the #1 IK bug source):

      body frame ... X forward, Y left, Z up; millimetres, radians.
      coxa frame ... +X radially outward along the leg, +Z up. The mount yaw
                     encodes each leg's orientation, so one leg_ik serves all
                     legs with no per-leg sign hacks.
      joint values . DIRECT servo/URDF angles: coxa = gamma (yaw about +Z),
                     femur = -beta (the URDF +Y axis pitches the link down),
                     tibia = +kappa (knee fold from straight, always >= 0).

   One leg, side view in the (radial, Z) plane (coxa frame, +X radial out):

        +Z  (up)
        ^
        |        beta>0 points the femur up, femur = -beta
        |       _
        |      /| femur            (elbow_up: knee above the r-axis line,
        |     / |                    insect posture; elbow_down mirrors it)
        |    /__\
        |        kappa = fold of the tibia from straight, always >= 0
        |         |
        +-------->--------+X radial outward
       (coxa)
        gamma is the coxa yaw about +Z; a point p in the coxa frame is at
        radius r = |p| - coxa_len, so the reach annulus is measured from the
        coxa joint, not the body axis (De Luca WS1).

   Leg IK:

      Geometric method (Craig, "Introduction to Robotics", 3rd ed., ch. 4.4):
      the leg decomposes into a coxa yaw plus a planar 2-link arm solved with
      the law of cosines. Constant-time and exact -- no DH matrices, no
      iteration -- which is what you want on a 240 MHz MCU.

      sakin_leg_ik never fails and never returns NaN: unreachable targets are
      projected onto the workspace annulus

         |femur - tibia| <= d <= femur + tibia

      and then saturated at the joint limits (Craig 4.2 / De Luca Robotics 1:
      a goal outside the workspace has no solution; a servo should still get
      the closest safe one -- Craig 4.2 raises exactly this "nearest
      attainable goal frame" question for under-actuated chains, and this is
      its answer). The return value tells you when this happened:

         SAKIN_IK_OK ................ exact solution
         SAKIN_IK_CLIPPED_WORKSPACE . target was outside the annulus
         SAKIN_IK_CLIPPED_JOINTS .... a joint hit its configured limit
         SAKIN_IK_BAD_CONFIG ........ cfg has a zero/negative link length

      A positioning leg has two solutions (elbow up/down). The branch is
      fixed by the hardware, so it lives in cfg.elbow_up, not in the call.
      sakin_leg_fk is the exact inverse for whichever branch is configured
      (the knee fold direction mirrors with the branch).

   Gait engine:

      NUKE state machine (Vanadium Labs), with two fixes relative to
      nuke.ino, both validated in PyBullet:

         1. the step counter advances on an accumulated tranTime clock
            instead of once per frame;
         2. foot offsets are linearly interpolated between step boundaries,
            because the real servos interpolate too -- teleporting targets
            makes the feet skate.

      Turning rotates the stance-foot offsets about the body-centre Z axis
      exactly (replacing NUKE's small-angle bodyIK hack), and body
      roll/pitch/yaw compensation applies the exact inverse rotation R^T to
      the feet, with a fast path when the body is level.

      sakin_gait_select freezes the current offsets into the interpolation
      state, so switching gait mid-walk never teleports a foot. When the
      velocity command drops below the deadbands, the offsets decay to zero
      exponentially (time constant cfg.settle_time, hard-zeroed once each
      component is below 0.5 mm) and the robot stands at its nominal pose.

      The gait tables (leg phase order, push steps, cycle steps, step time)
      are the exact NUKE values proven on the real robot, exposed through
      cfg.gaits[] so you can tune or replace them.

   Jump:

      sakin_jump_start runs a squat-thrust-retract-land sequence that
      overrides the gait until touchdown. The timing matches the original
      behaviour exactly, including the foot-target discontinuity on the
      squat->thrust boundary tick.

   CREDITS

      Gait tables and hexapod geometry: NUKE / nuke.h (Vanadium Labs).
      Theory: Craig, "Introduction to Robotics", 3rd ed., chapter 4;
      De Luca, "Robotics 1: Inverse kinematics" (Sapienza).

RECENT REVISION HISTORY:

      2026-08-08  body-Z channel: sakin_body_height() + sakin_t.body_pos_z
                  (standing-height control; +z lowers the body, -z raises it)
      2026-07-31  initial release

   See end of file for full revision history.
*/

#ifndef SAKIN_H_INCLUDE
#define SAKIN_H_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

// declaration specifier: override SAKIN_DEF directly, or #define SAKIN_STATIC
#ifndef SAKIN_DEF
#ifdef SAKIN_STATIC
#define SAKIN_DEF static
#else
#define SAKIN_DEF extern
#endif
#endif

// scalar type: float by default (ESP32 FPU is single-precision)
#ifdef SAKIN_USE_DOUBLE
typedef double sakin_real_t;
#else
typedef float sakin_real_t;
#endif

#ifndef SAKIN_MAX_LEGS
#define SAKIN_MAX_LEGS 6
#endif

#define SAKIN_PI ((sakin_real_t)3.14159265358979323846)
#define SAKIN_RAD(deg) ((sakin_real_t)(deg) * (SAKIN_PI / (sakin_real_t)180))

// leg indices (built-in ordering)
#define SAKIN_LEG_RF 0
#define SAKIN_LEG_RR 1
#define SAKIN_LEG_LF 2
#define SAKIN_LEG_LR 3
#define SAKIN_LEG_RM 4
#define SAKIN_LEG_LM 5

// gait IDs (dense indices into cfg.gaits[]; NUKE used 0,1,2,5,6)
#define SAKIN_GAIT_RIPPLE 0
#define SAKIN_GAIT_RIPPLE_LEFT 1
#define SAKIN_GAIT_RIPPLE_RIGHT 2
#define SAKIN_GAIT_AMBLE 3
#define SAKIN_GAIT_TRIPOD 4
#define SAKIN_GAIT_COUNT 5

// IK status flags (bitwise OR-able)
#define SAKIN_IK_OK 0
#define SAKIN_IK_CLIPPED_WORKSPACE 1 // target out of reach, projected to boundary
#define SAKIN_IK_CLIPPED_JOINTS 2    // a joint hit its configured limit
#define SAKIN_IK_BAD_CONFIG 4        // degenerate cfg (e.g. zero link length)

// jump phases (sakin_jump_phase)
#define SAKIN_JUMP_NONE 0
#define SAKIN_JUMP_SQUAT 1
#define SAKIN_JUMP_THRUST 2
#define SAKIN_JUMP_RETRACT 3
#define SAKIN_JUMP_LAND 4

typedef struct {
  sakin_real_t x, y, z;
} sakin_vec3_t;

// direct joint values, radians -- see DOCUMENTATION above
typedef struct {
  sakin_real_t coxa, femur, tibia;
} sakin_leg_angles_t;

// leg mount in the body frame; yaw = direction of the leg's outward radial
typedef struct {
  sakin_real_t x, y, z, yaw;
} sakin_mount_t;

// gait table: leg_no[l] = step index at which leg l enters swing
typedef struct {
  int leg_no[SAKIN_MAX_LEGS];
  int push_steps;       // steps each leg spends in stance per cycle
  int steps_in_cycle;   // total steps per cycle
  sakin_real_t tran_ms; // milliseconds per step
} sakin_gait_params_t;

typedef struct {
  int leg_count; // active legs (1..SAKIN_MAX_LEGS)

  // link lengths (mm)
  sakin_real_t coxa_len, femur_len, tibia_len;

  // joint limits (rad): coxa/femur symmetric +-lim, tibia in [min, max]
  sakin_real_t coxa_lim, femur_lim, tibia_min, tibia_max;

  int elbow_up; // IK branch: 1 = insect (default), 0 = down

  // standing-pose geometry (mm)
  sakin_real_t body_half_h; // body half-thickness
  sakin_real_t coxa_height; // coxa joint height above ground at stance
  sakin_real_t foot_reach;  // horizontal femur-axis -> foot at stance
  sakin_real_t foot_radius; // foot-tip sphere radius (z compensation)

  // gait tuning (mm)
  sakin_real_t lift_height; // swing apex height
  sakin_real_t max_offset;  // safety clamp on horizontal gait offsets

  // velocity deadbands + settle behaviour
  sakin_real_t deadband_lin; // mm/s below which the robot counts as still
  sakin_real_t deadband_rot; // rad/s below which the robot counts as still
  sakin_real_t settle_time;  // s: EXPONENTIAL decay time constant (not a
                             // deadline) -- offsets shrink by e-fold every
                             // settle_time s and are hard-zeroed below 0.5 mm

  sakin_mount_t mounts[SAKIN_MAX_LEGS];
  sakin_gait_params_t gaits[SAKIN_GAIT_COUNT];

#ifndef SAKIN_NO_JUMP
  // jump tuning (mm, s)
  sakin_real_t jump_squat_depth, jump_thrust_lift, jump_retract_lift;
  sakin_real_t jump_squat_time, jump_thrust_time, jump_retract_time, jump_land_time;
#endif
} sakin_config_t;

// fill cfg with the built-in hexapod defaults (6 legs); with SAKIN_MAX_LEGS
// < 6 only the shared geometry is filled - provide your own mounts/tables
SAKIN_DEF void sakin_config_default(sakin_config_t* cfg);

// solve one leg for a foot target in the coxa frame (mm); always produces a
// valid NaN-free answer, the SAKIN_IK_* flags tell you when it clipped
SAKIN_DEF int sakin_leg_ik(const sakin_config_t* cfg, sakin_vec3_t target, sakin_leg_angles_t* out);

// foot position in the coxa frame (mm); exact inverse of sakin_leg_ik
SAKIN_DEF void
sakin_leg_fk(const sakin_config_t* cfg, sakin_leg_angles_t angles, sakin_vec3_t* out_foot);

// inverse body rotation applied to a body-frame foot position:
// foot' = R^T * foot with R = Rz(yaw)*Ry(pitch)*Rx(roll); identity fast path
SAKIN_DEF sakin_vec3_t sakin_body_rotate_inv(
  sakin_vec3_t foot_body,
  sakin_real_t roll,
  sakin_real_t pitch,
  sakin_real_t yaw
);

// standing foot position of one leg in the body frame (mm), sole on ground
SAKIN_DEF sakin_vec3_t sakin_stance_foot(const sakin_config_t* cfg, int leg);

#ifndef SAKIN_NO_GAIT

typedef struct {
  sakin_config_t cfg; // copied at init

  sakin_vec3_t endpoints[SAKIN_MAX_LEGS]; // body-frame nominal footholds
  sakin_vec3_t offsets[SAKIN_MAX_LEGS];   // current gait offsets (output)

  // commanded body pose / translation
  sakin_real_t body_rot_x, body_rot_y, body_rot_z;
  sakin_real_t body_pos_x, body_pos_y;
  sakin_real_t body_pos_z; // body translation in Z (mm): + = feet targets rise
                           // in the body frame -> legs fold -> BODY DROPS

  // commanded velocity: mm/s (X fwd, Y left), rad/s (CCW positive)
  sakin_real_t vx, vy, omega;

  // NUKE state machine
  int gait_id;
  int gait_leg_no[SAKIN_MAX_LEGS];
  int push_steps, steps_in_cycle;
  sakin_real_t tran_ms, cycle_time;
  int step;
  sakin_real_t t_in_step;
  int primed;
  sakin_vec3_t prev[SAKIN_MAX_LEGS], next[SAKIN_MAX_LEGS];

#ifndef SAKIN_NO_JUMP
  int jump_active, jump_phase;
  sakin_real_t jump_timer;
#endif
} sakin_t;

// initialise with cfg (copied) or built-in defaults when cfg is NULL;
// resets endpoints to the standing pose and selects TRIPOD (NUKE default)
SAKIN_DEF void sakin_init(sakin_t* s, const sakin_config_t* cfg);

// switch gait; freezes current offsets so the transition stays continuous
SAKIN_DEF void sakin_gait_select(sakin_t* s, int gait_id);

// velocity command: mm/s forward (X) and left (Y), rad/s yaw (CCW +)
SAKIN_DEF void sakin_command(sakin_t* s, sakin_real_t vx, sakin_real_t vy, sakin_real_t omega);

// body pose command (rad): roll, pitch, yaw about the body centre
SAKIN_DEF void sakin_body_pose(sakin_t* s, sakin_real_t roll, sakin_real_t pitch, sakin_real_t yaw);

// body translation command (mm) in the body XY plane
SAKIN_DEF void sakin_body_shift(sakin_t* s, sakin_real_t x, sakin_real_t y);

// body translation command (mm) in the body Z axis. +z raises the foot
// targets in the body frame -> legs fold -> BODY DROPS; -z extends them
// -> BODY RISES. Composes with the gait, jump and body pose channels.
SAKIN_DEF void sakin_body_height(sakin_t* s, sakin_real_t z);

// reset endpoints to the standing pose and restart the gait engine
// (back to TRIPOD, the power-on default)
SAKIN_DEF void sakin_reset_pose(sakin_t* s);

// advance the gait by dt seconds and solve every leg; angles_out must hold
// at least cfg.leg_count entries (SAKIN_MAX_LEGS to be safe); returns the
// bitwise OR of all per-leg IK status flags
SAKIN_DEF int sakin_update(sakin_t* s, sakin_real_t dt, sakin_leg_angles_t* angles_out);

#ifndef SAKIN_NO_JUMP
// trigger a jump from the current state (ignored while one is in flight)
SAKIN_DEF void sakin_jump_start(sakin_t* s);
SAKIN_DEF int sakin_jump_active(const sakin_t* s);
SAKIN_DEF int sakin_jump_phase(const sakin_t* s); // SAKIN_JUMP_*
#endif

#endif // SAKIN_NO_GAIT

#ifdef __cplusplus
}
#endif

////   end header file   /////////////////////////////////////////////////////
#endif // SAKIN_H_INCLUDE

#ifdef SAKIN_IMPLEMENTATION

#include <math.h>

// math layer: no tgmath (unreliable on newlib), one variant per precision
#ifdef SAKIN_USE_DOUBLE
#define SAKIN__SIN sin
#define SAKIN__COS cos
#define SAKIN__ATAN2 atan2
#define SAKIN__ACOS acos
#define SAKIN__HYPOT hypot
#define SAKIN__FABS fabs
#define SAKIN__FMIN fmin
#define SAKIN__FMAX fmax
#else
#define SAKIN__SIN sinf
#define SAKIN__COS cosf
#define SAKIN__ATAN2 atan2f
#define SAKIN__ACOS acosf
#define SAKIN__HYPOT hypotf
#define SAKIN__FABS fabsf
#define SAKIN__FMIN fminf
#define SAKIN__FMAX fmaxf
#endif

static sakin_real_t sakin__clamp(sakin_real_t v, sakin_real_t lo, sakin_real_t hi) {
  return SAKIN__FMAX(lo, SAKIN__FMIN(hi, v));
}

static sakin_vec3_t sakin__vec3(sakin_real_t x, sakin_real_t y, sakin_real_t z) {
  sakin_vec3_t v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

// built-in default mounts (6 legs), copied element-wise so SAKIN_MAX_LEGS
// < 6 compiles without excess-initializer warnings
static const sakin_mount_t sakin__default_mounts[6] = {
  { 55.0f, -55.0f, -20.0f, -0.78539816f },  // RF: yaw  -45 deg
  { -55.0f, -55.0f, -20.0f, -2.35619449f }, // RR: yaw -135 deg
  { 55.0f, 55.0f, -20.0f, 0.78539816f },    // LF: yaw   45 deg
  { -55.0f, 55.0f, -20.0f, 2.35619449f },   // LR: yaw  135 deg
  { 0.0f, -55.0f, -20.0f, -1.57079633f },   // RM: yaw  -90 deg
  { 0.0f, 55.0f, -20.0f, 1.57079633f }      // LM: yaw   90 deg
};

// NUKE leg_no sequences per gait, EXACT values proven on the real robot.
// leg_no[l] = step index (0..steps_in_cycle-1) at which leg l enters swing;
// entry 0 in each row is the first leg to step, matching the R/L order
static const int sakin__default_leg_no[SAKIN_GAIT_COUNT][6] = {
  { 0, 8, 6, 2, 10, 4 }, // RIPPLE
  { 4, 8, 6, 0, 10, 2 }, // RIPPLE_LEFT
  { 2, 0, 8, 6, 10, 4 }, // RIPPLE_RIGHT
  { 0, 2, 2, 0, 4, 4 },  // AMBLE
  { 0, 0, 2, 2, 2, 0 }   // TRIPOD
};

SAKIN_DEF void sakin_config_default(sakin_config_t* cfg) {
  static const int push[SAKIN_GAIT_COUNT] = { 10, 10, 10, 4, 2 };
  static const int steps[SAKIN_GAIT_COUNT] = { 12, 12, 12, 6, 4 };
  static const sakin_real_t tran[SAKIN_GAIT_COUNT] = { 120.0f, 120.0f, 120.0f, 150.0f, 65.0f };
  int i, l, n = SAKIN_MAX_LEGS < 6 ? SAKIN_MAX_LEGS : 6;

  cfg->leg_count = n;

  cfg->coxa_len = 35.0f;
  cfg->femur_len = 82.0f;
  cfg->tibia_len = 80.0f;

  cfg->coxa_lim = 1.5f; // must match the URDF joint limits
  cfg->femur_lim = 1.5f;
  cfg->tibia_min = 0.0f;     // one-way knee
  cfg->tibia_max = 2.96706f; // ~170 deg

  cfg->elbow_up = 1; // insect configuration

  cfg->body_half_h = 20.0f;
  cfg->coxa_height = 70.0f;
  cfg->foot_reach = 80.0f;
  cfg->foot_radius = 5.0f;

  cfg->lift_height = 40.0f;
  cfg->max_offset = 35.0f;

  cfg->deadband_lin = 5.0f;  // mm/s
  cfg->deadband_rot = 0.05f; // rad/s
  cfg->settle_time = 0.15f;  // s

  {
    sakin_mount_t zero_mount = { 0, 0, 0, 0 };
    for (l = 0; l < n; ++l)
      cfg->mounts[l] = sakin__default_mounts[l];
    for (l = n; l < SAKIN_MAX_LEGS; ++l)
      cfg->mounts[l] = zero_mount;
  }

  for (i = 0; i < SAKIN_GAIT_COUNT; ++i) {
    for (l = 0; l < SAKIN_MAX_LEGS; ++l)
      cfg->gaits[i].leg_no[l] = (l < n) ? sakin__default_leg_no[i][l] : 0;
    cfg->gaits[i].push_steps = push[i];
    cfg->gaits[i].steps_in_cycle = steps[i];
    cfg->gaits[i].tran_ms = tran[i];
  }

#ifndef SAKIN_NO_JUMP
  cfg->jump_squat_depth = 25.0f;
  cfg->jump_thrust_lift = 100.0f;
  cfg->jump_retract_lift = 50.0f;
  cfg->jump_squat_time = 0.15f;
  cfg->jump_thrust_time = 0.12f;
  cfg->jump_retract_time = 0.12f;
  cfg->jump_land_time = 0.10f;
#endif
}

// core leg IK -- Craig ch. 4.4 geometric method: coxa yaw + planar 2-link
// arm; the annulus clip is Craig 4.2's workspace-existence check made safe
// for hardware (better a saturated solution than NaN at a servo)
SAKIN_DEF int
sakin_leg_ik(const sakin_config_t* cfg, sakin_vec3_t target, sakin_leg_angles_t* out) {
  const sakin_real_t lf = cfg->femur_len, lt = cfg->tibia_len;
  const sakin_real_t eps = (sakin_real_t)1e-6;
  int status = SAKIN_IK_OK;
  sakin_real_t gamma, r, d, d_lo, d_hi, alpha, cos_b, beta, cos_p, kappa, femur;

  // degenerate configs would make the law-of-cosines denominators below
  // zero (0/0 -> NaN); report BAD_CONFIG and return a safe zero pose so the
  // "never returns NaN" contract holds for every input
  if (lf <= (sakin_real_t)0 || lt <= (sakin_real_t)0) {
    out->coxa = out->femur = out->tibia = 0;
    return SAKIN_IK_BAD_CONFIG;
  }

  gamma = SAKIN__ATAN2(target.y, target.x);             // coxa yaw
  r = SAKIN__HYPOT(target.x, target.y) - cfg->coxa_len; // radial reach
  d = SAKIN__HYPOT(r, target.z);                        // 3-D reach

  // workspace annulus |lf - lt| <= d <= lf + lt (Craig 4.2 / De Luca WS1)
  d_lo = SAKIN__FABS(lf - lt) + eps;
  d_hi = lf + lt - eps;
  if (d < d_lo || d > d_hi)
    status |= SAKIN_IK_CLIPPED_WORKSPACE;
  d = sakin__clamp(d, d_lo, d_hi);

  alpha = SAKIN__ATAN2(target.z, r); // foot-line elevation
  cos_b = (lf * lf + d * d - lt * lt) / (2 * lf * d);
  cos_b = sakin__clamp(cos_b, -1.0f, 1.0f); // float dust guard

  // branch is a config flag: the hardware fixes it, not the caller
  beta = cfg->elbow_up ? alpha + SAKIN__ACOS(cos_b) : alpha - SAKIN__ACOS(cos_b);

  cos_p = (lf * lf + lt * lt - d * d) / (2 * lf * lt);
  cos_p = sakin__clamp(cos_p, -1.0f, 1.0f);
  kappa = SAKIN_PI - SAKIN__ACOS(cos_p); // knee fold, >= 0

  femur = -beta; // URDF: +Y joint axis pitches the link down -> minus

  // joint limits shrink the workspace (Craig 4.2); report when they bite
  {
    sakin_real_t g2 = sakin__clamp(gamma, -cfg->coxa_lim, cfg->coxa_lim);
    sakin_real_t f2 = sakin__clamp(femur, -cfg->femur_lim, cfg->femur_lim);
    sakin_real_t k2 = sakin__clamp(kappa, cfg->tibia_min, cfg->tibia_max);
    if (g2 != gamma || f2 != femur || k2 != kappa)
      status |= SAKIN_IK_CLIPPED_JOINTS;
    out->coxa = g2;
    out->femur = f2;
    out->tibia = k2;
  }
  return status;
}

SAKIN_DEF void
sakin_leg_fk(const sakin_config_t* cfg, sakin_leg_angles_t angles, sakin_vec3_t* out_foot) {
  // exact inverse of sakin_leg_ik -- includes the tibia term in r; the knee
  // fold direction mirrors with the elbow branch
  sakin_real_t beta = -angles.femur, kappa = angles.tibia;
  sakin_real_t fold = cfg->elbow_up ? beta - kappa : beta + kappa;
  sakin_real_t r =
    cfg->coxa_len + cfg->femur_len * SAKIN__COS(beta) + cfg->tibia_len * SAKIN__COS(fold);
  sakin_real_t z = cfg->femur_len * SAKIN__SIN(beta) + cfg->tibia_len * SAKIN__SIN(fold);
  out_foot->x = r * SAKIN__COS(angles.coxa);
  out_foot->y = r * SAKIN__SIN(angles.coxa);
  out_foot->z = z;
}

SAKIN_DEF sakin_vec3_t
sakin_body_rotate_inv(sakin_vec3_t p, sakin_real_t roll, sakin_real_t pitch, sakin_real_t yaw) {
  sakin_real_t cx, sx, cy, sy, cz, sz;
  sakin_real_t r00, r01, r02, r10, r11, r12, r20, r21, r22;
  if (roll == 0 && pitch == 0 && yaw == 0)
    return p; // level fast path
  cx = SAKIN__COS(roll);
  sx = SAKIN__SIN(roll);
  cy = SAKIN__COS(pitch);
  sy = SAKIN__SIN(pitch);
  cz = SAKIN__COS(yaw);
  sz = SAKIN__SIN(yaw);
  // R = Rz*Ry*Rx; we apply R^T, so rows of R become columns here
  r00 = cz * cy;
  r01 = cz * sy * sx - sz * cx;
  r02 = cz * sy * cx + sz * sx;
  r10 = sz * cy;
  r11 = sz * sy * sx + cz * cx;
  r12 = sz * sy * cx - cz * sx;
  r20 = -sy;
  r21 = cy * sx;
  r22 = cy * cx;
  return sakin__vec3(
    r00 * p.x + r10 * p.y + r20 * p.z,
    r01 * p.x + r11 * p.y + r21 * p.z,
    r02 * p.x + r12 * p.y + r22 * p.z
  );
}

SAKIN_DEF sakin_vec3_t sakin_stance_foot(const sakin_config_t* cfg, int leg) {
  const sakin_mount_t* m = &cfg->mounts[leg];
  sakin_real_t reach = cfg->coxa_len + cfg->foot_reach;
  sakin_real_t z = -(cfg->coxa_height + cfg->body_half_h) + cfg->foot_radius;
  return sakin__vec3(m->x + SAKIN__COS(m->yaw) * reach, m->y + SAKIN__SIN(m->yaw) * reach, z);
}

#ifndef SAKIN_NO_GAIT

static void sakin__load_gait(sakin_t* s, int gait_id) {
  const sakin_gait_params_t* p = &s->cfg.gaits[gait_id];
  int l;
  s->gait_id = gait_id;
  for (l = 0; l < SAKIN_MAX_LEGS; ++l)
    s->gait_leg_no[l] = p->leg_no[l];
  s->push_steps = p->push_steps;
  s->steps_in_cycle = p->steps_in_cycle > 0 ? p->steps_in_cycle : 1;
  s->tran_ms = p->tran_ms > 0 ? p->tran_ms : 1.0f;
  s->cycle_time = (sakin_real_t)s->steps_in_cycle * s->tran_ms / 1000.0f;
  s->step = 0;
  s->t_in_step = 0;
  s->primed = 0;
  for (l = 0; l < SAKIN_MAX_LEGS; ++l) { // freeze for continuity
    s->prev[l] = s->offsets[l];
    s->next[l] = s->offsets[l];
  }
}

SAKIN_DEF void sakin_gait_select(sakin_t* s, int gait_id) {
  if (gait_id == s->gait_id)
    return;
  if (gait_id < 0 || gait_id >= SAKIN_GAIT_COUNT)
    return;
  sakin__load_gait(s, gait_id);
}

SAKIN_DEF void sakin_command(sakin_t* s, sakin_real_t vx, sakin_real_t vy, sakin_real_t omega) {
  s->vx = vx;
  s->vy = vy;
  s->omega = omega;
}

SAKIN_DEF void
sakin_body_pose(sakin_t* s, sakin_real_t roll, sakin_real_t pitch, sakin_real_t yaw) {
  s->body_rot_x = roll;
  s->body_rot_y = pitch;
  s->body_rot_z = yaw;
}

SAKIN_DEF void sakin_body_shift(sakin_t* s, sakin_real_t x, sakin_real_t y) {
  s->body_pos_x = x;
  s->body_pos_y = y;
}

SAKIN_DEF void sakin_body_height(sakin_t* s, sakin_real_t z) {
  s->body_pos_z = z;
}

// the NUKE rule, evaluated once per step boundary for one leg
static sakin_vec3_t sakin__boundary_target(const sakin_t* s, int leg) {
  const sakin_config_t* c = &s->cfg;
  sakin_vec3_t t;
  int swing_step = s->gait_leg_no[leg];

  if (s->step == swing_step) {
    // swing apex: foot lifted, centred over the nominal foothold
    t = sakin__vec3(0, 0, c->lift_height);
  } else if (s->step == (swing_step + 1) % s->steps_in_cycle && s->next[leg].z > 1.0f) {
    // touch down ahead: translation stride + exact rotation share.
    // next[leg].z > 1.0f means the leg was airborne (its last target was the
    // swing apex), so the step just completed was its swing step.
    sakin_real_t k = s->cycle_time * s->push_steps / (2.0f * s->steps_in_cycle);
    // k = HALF the stance-phase duration. The foot is placed v*k ahead at
    // touchdown so that, dragging back at -v through the whole stance, it
    // sweeps a symmetric +k..-k arc about the endpoint and returns to the
    // same world spot every cycle (push_steps = stance duration in steps).
    sakin_real_t ang = s->omega * k;
    sakin_real_t ca = SAKIN__COS(ang), sa = SAKIN__SIN(ang);
    sakin_vec3_t ep = s->endpoints[leg];
    t = sakin__vec3(
      (ca * ep.x - sa * ep.y) - ep.x + s->vx * k,
      (sa * ep.x + ca * ep.y) - ep.y + s->vy * k,
      0
    );
  } else {
    // stance: drag the foot back opposite the velocity and rotate it about
    // the body centre -- the foot is world-fixed, the body moves over it
    sakin_real_t k = s->cycle_time / s->steps_in_cycle;
    sakin_vec3_t nxt = s->next[leg];
    t = sakin__vec3(nxt.x - s->vx * k, nxt.y - s->vy * k, 0);
    if (SAKIN__FABS(s->omega) > (sakin_real_t)1e-9) {
      sakin_vec3_t ep = s->endpoints[leg];
      sakin_real_t fx = ep.x + t.x, fy = ep.y + t.y;
      sakin_real_t ang = -s->omega * k;
      sakin_real_t ca = SAKIN__COS(ang), sa = SAKIN__SIN(ang);
      t.x = (ca * fx - sa * fy) - ep.x;
      t.y = (sa * fx + ca * fy) - ep.y;
    }
  }

  // never command the foot outside a sane radius
  {
    sakin_real_t m = SAKIN__HYPOT(t.x, t.y);
    if (m > c->max_offset) {
      t.x *= c->max_offset / m;
      t.y *= c->max_offset / m;
    }
  }
  return t;
}

// decay offsets to zero when the robot is told to stand still
static void sakin__settle(sakin_t* s, sakin_real_t dt) {
  sakin_real_t decay = SAKIN__FMAX(0.0f, 1.0f - dt / s->cfg.settle_time);
  int settled = 1, l;
  s->primed = 0;
  for (l = 0; l < s->cfg.leg_count; ++l) {
    sakin_vec3_t* o = &s->offsets[l];
    o->x *= decay;
    o->y *= decay;
    o->z *= decay;
    if (SAKIN__FABS(o->x) > 0.5f || SAKIN__FABS(o->y) > 0.5f || SAKIN__FABS(o->z) > 0.5f)
      settled = 0;
  }
  if (settled) {
    s->step = 0;
    s->t_in_step = 0;
    for (l = 0; l < SAKIN_MAX_LEGS; ++l) {
      s->offsets[l] = sakin__vec3(0, 0, 0);
      s->prev[l] = sakin__vec3(0, 0, 0);
      s->next[l] = sakin__vec3(0, 0, 0);
    }
  } else {
    for (l = 0; l < SAKIN_MAX_LEGS; ++l) {
      s->prev[l] = s->offsets[l];
      s->next[l] = s->offsets[l];
    }
  }
}

// advance the clock and produce this tick's interpolated offsets
static void sakin__gait_update(sakin_t* s, sakin_real_t dt) {
  int moving = SAKIN__FABS(s->vx) > s->cfg.deadband_lin ||
               SAKIN__FABS(s->vy) > s->cfg.deadband_lin ||
               SAKIN__FABS(s->omega) > s->cfg.deadband_rot;
  int l;

  if (!moving) {
    sakin__settle(s, dt);
    return;
  }

  if (!s->primed) { // first moving tick: target the CURRENT step
    s->primed = 1;
    for (l = 0; l < SAKIN_MAX_LEGS; ++l) {
      s->prev[l] = s->offsets[l];
      s->next[l] = sakin__boundary_target(s, l);
    }
  }

  s->t_in_step += dt * 1000.0f;
  while (s->t_in_step >= s->tran_ms) {
    s->t_in_step -= s->tran_ms;
    s->step = (s->step + 1) % s->steps_in_cycle;
    for (l = 0; l < SAKIN_MAX_LEGS; ++l) {
      s->prev[l] = s->next[l];
      s->next[l] = sakin__boundary_target(s, l);
    }
  }

  {
    sakin_real_t tau = s->t_in_step / s->tran_ms;
    // smoothstep: ease foot velocity in/out across each step so the legs do
    // not snap direction at step boundaries (plan-gait-smoothing.md, Option A).
    // Monotonic [0,1] -> [0,1], zero velocity at both ends; bounds/clamps in
    // the tests are unaffected (it only reshapes the rate, not the extrema).
    tau = tau * tau * (3.0f - 2.0f * tau);
    for (l = 0; l < SAKIN_MAX_LEGS; ++l) {
      sakin_vec3_t p = s->prev[l], n = s->next[l], *o = &s->offsets[l];
      o->x = p.x + (n.x - p.x) * tau;
      o->y = p.y + (n.y - p.y) * tau;
      o->z = p.z + (n.z - p.z) * tau;
    }
  }
}

// shared tail of every IK path: body compensation -> coxa frame -> leg IK
static int
sakin__solve_leg(const sakin_t* s, int leg, sakin_vec3_t foot_body, sakin_leg_angles_t* out) {
  const sakin_mount_t* m = &s->cfg.mounts[leg];
  sakin_vec3_t p = sakin_body_rotate_inv(foot_body, s->body_rot_x, s->body_rot_y, s->body_rot_z);
  sakin_real_t dx = p.x - m->x, dy = p.y - m->y;
  sakin_real_t cy = SAKIN__COS(m->yaw), sy = SAKIN__SIN(m->yaw);
  sakin_vec3_t local = sakin__vec3(cy * dx + sy * dy, -sy * dx + cy * dy, p.z - m->z);
  return sakin_leg_ik(&s->cfg, local, out);
}

#ifndef SAKIN_NO_JUMP
// returns 1 when the jump consumed this tick (out filled), 0 to run gait
static int sakin__jump_update(sakin_t* s, sakin_real_t dt, sakin_leg_angles_t* out, int* flags) {
  const sakin_config_t* c = &s->cfg;
  sakin_real_t dz = 0, lift = 0, frac;
  int l;

  if (!s->jump_active)
    return 0;
  s->jump_timer += dt;

  switch (s->jump_phase) {
  case SAKIN_JUMP_SQUAT:
    frac = SAKIN__FMIN(1.0f, s->jump_timer / c->jump_squat_time);
    dz = c->jump_squat_depth * frac; // feet rise <=> body drops
    if (frac >= 1.0f) {
      s->jump_phase = SAKIN_JUMP_THRUST;
      s->jump_timer = 0;
    }
    break;
  case SAKIN_JUMP_THRUST:
    frac = SAKIN__FMIN(1.0f, s->jump_timer / c->jump_thrust_time);
    dz = -c->jump_squat_depth - (c->jump_squat_depth + c->jump_thrust_lift) * frac;
    if (frac >= 1.0f) {
      s->jump_phase = SAKIN_JUMP_RETRACT;
      s->jump_timer = 0;
    }
    break;
  case SAKIN_JUMP_RETRACT:
    frac = SAKIN__FMIN(1.0f, s->jump_timer / c->jump_retract_time);
    lift = c->jump_retract_lift * frac; // toes up so they don't catch
    if (frac >= 1.0f) {
      s->jump_phase = SAKIN_JUMP_LAND;
      s->jump_timer = 0;
    }
    goto solve;
  case SAKIN_JUMP_LAND:
    frac = SAKIN__FMIN(1.0f, s->jump_timer / c->jump_land_time);
    dz = c->jump_thrust_lift * (1.0f - frac); // absorb and settle
    if (frac >= 1.0f) {
      s->jump_active = 0;
      s->jump_phase = SAKIN_JUMP_NONE;
      s->jump_timer = 0;
      return 0; // resume normal gait this tick
    }
    goto solve;
  default:
    s->jump_active = 0;
    s->jump_phase = SAKIN_JUMP_NONE;
    return 0;
  }

  // squat/thrust solve here; on the thrust->retract boundary tick this
  // falls through to the gait (original-behaviour quirk, kept faithful)
  if (s->jump_phase != SAKIN_JUMP_SQUAT && s->jump_phase != SAKIN_JUMP_THRUST)
    return 0;

solve:
  for (l = 0; l < c->leg_count; ++l) {
    sakin_vec3_t ep = s->endpoints[l];
    sakin_vec3_t foot = sakin__vec3(ep.x + s->body_pos_x, ep.y + s->body_pos_y,
                                    ep.z + s->body_pos_z + dz + lift);
    *flags |= sakin__solve_leg(s, l, foot, &out[l]);
  }
  return 1;
}

SAKIN_DEF void sakin_jump_start(sakin_t* s) {
  if (s->jump_active)
    return;
  s->jump_active = 1;
  s->jump_phase = SAKIN_JUMP_SQUAT;
  s->jump_timer = 0;
}

SAKIN_DEF int sakin_jump_active(const sakin_t* s) { return s->jump_active; }
SAKIN_DEF int sakin_jump_phase(const sakin_t* s) { return s->jump_phase; }
#endif // SAKIN_NO_JUMP

SAKIN_DEF void sakin_reset_pose(sakin_t* s) {
  int l;
  for (l = 0; l < SAKIN_MAX_LEGS; ++l) {
    s->offsets[l] = sakin__vec3(0, 0, 0);
    s->prev[l] = sakin__vec3(0, 0, 0);
    s->next[l] = sakin__vec3(0, 0, 0);
  }
  for (l = 0; l < s->cfg.leg_count; ++l)
    s->endpoints[l] = sakin_stance_foot(&s->cfg, l);
  s->gait_id = -1;
  sakin__load_gait(s, SAKIN_GAIT_TRIPOD); // NUKE default
}

SAKIN_DEF void sakin_init(sakin_t* s, const sakin_config_t* cfg) {
  int l;
  if (cfg)
    s->cfg = *cfg;
  else
    sakin_config_default(&s->cfg);

  if (s->cfg.leg_count < 1)
    s->cfg.leg_count = 1;
  if (s->cfg.leg_count > SAKIN_MAX_LEGS)
    s->cfg.leg_count = SAKIN_MAX_LEGS;

  s->body_rot_x = s->body_rot_y = s->body_rot_z = 0;
  s->body_pos_x = s->body_pos_y = s->body_pos_z = 0;
  s->vx = s->vy = s->omega = 0;
#ifndef SAKIN_NO_JUMP
  s->jump_active = 0;
  s->jump_phase = SAKIN_JUMP_NONE;
  s->jump_timer = 0;
#endif
  for (l = 0; l < SAKIN_MAX_LEGS; ++l)
    s->endpoints[l] = sakin__vec3(0, 0, 0);
  sakin_reset_pose(s);
}

SAKIN_DEF int sakin_update(sakin_t* s, sakin_real_t dt, sakin_leg_angles_t* angles_out) {
  int flags = SAKIN_IK_OK, l;
  const sakin_config_t* c = &s->cfg;

#ifndef SAKIN_NO_JUMP
  if (sakin__jump_update(s, dt, angles_out, &flags))
    return flags;
#endif

  sakin__gait_update(s, dt);

  for (l = 0; l < c->leg_count; ++l) {
    sakin_vec3_t ep = s->endpoints[l], off = s->offsets[l];
    sakin_vec3_t foot =
      sakin__vec3(ep.x + off.x + s->body_pos_x, ep.y + off.y + s->body_pos_y,
                  ep.z + off.z + s->body_pos_z);
    flags |= sakin__solve_leg(s, l, foot, &angles_out[l]);
  }
  return flags;
}

#endif // SAKIN_NO_GAIT

#endif // SAKIN_IMPLEMENTATION

/*
   FULL REVISION HISTORY

      2026-08-08  body-Z channel: sakin_body_height() + sakin_t.body_pos_z;
                  foot targets in sakin_update() and the jump path gain the
                  Z translation (+z folds the legs -> body drops)
      2026-07-31  initial release: core leg IK/FK with reachability flags,
                  gait engine (ripple, ripple-left/right, amble, tripod),
                  exact body compensation, jump, C99 + C++ compatible
*/
