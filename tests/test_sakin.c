/* test_sakin.c — behaviour tests for sakin.h
 *
 * Covers the critical tier (100% per test-coverage standards):
 *   - leg IK/FK roundtrip over the workspace (both elbow branches)
 *   - reachability status (Craig §4.2 existence check)
 *   - inside-coxa-cylinder region (quadrant-safe atan2, Craig §4.4)
 *   - degenerate config safety (never-NaN contract)
 *   - body compensation (roll/pitch/yaw), standing pose, jump lifecycle
 *   - gait engine sanity + gait-transition continuity
 *
 * Build:  cc -std=c99 -Wall -Wextra -Werror -pedantic test_sakin.c -lm
 */
#include <stdio.h>

#define SAKIN_IMPLEMENTATION
#include "../sakin.h"

static int g_checks = 0, g_fails = 0;

#define CHECK(cond, msg)                                  \
  do {                                                    \
    ++g_checks;                                           \
    if (!(cond)) {                                        \
      ++g_fails;                                          \
      printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); \
    }                                                     \
  } while (0)

static sakin_real_t dist3(sakin_vec3_t a, sakin_vec3_t b) {
  sakin_real_t dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return (sakin_real_t)SAKIN__HYPOT(SAKIN__HYPOT(dx, dy), dz);
}

static int all_angles_finite_and_limited(const sakin_config_t *cfg,
                                         const sakin_leg_angles_t *a, int n) {
  int l;
  for (l = 0; l < n; ++l) {
    sakin_leg_angles_t q = a[l];
    if (q.coxa != q.coxa || q.femur != q.femur || q.tibia != q.tibia) return 0;
    if (q.coxa < -cfg->coxa_lim - 1e-4f || q.coxa > cfg->coxa_lim + 1e-4f) return 0;
    if (q.femur < -cfg->femur_lim - 1e-4f || q.femur > cfg->femur_lim + 1e-4f) return 0;
    if (q.tibia < cfg->tibia_min - 1e-4f || q.tibia > cfg->tibia_max + 1e-4f) return 0;
  }
  return 1;
}

/* --- IK/FK roundtrip: FK(IK(p)) == p everywhere inside the workspace ----- */
static void ik_fk_roundtrip_holds_for_both_elbow_branches(void) {
  /* Arrange */
  sakin_config_t cfg;
  int branch;
  sakin_config_default(&cfg);
  for (branch = 0; branch <= 1; ++branch) {
    sakin_real_t x, y, z, worst = 0;
    int tested = 0;
    cfg.elbow_up = branch ? 0 : 1;
    /* Act: sweep a grid spanning the leg workspace (coxa frame) */
    for (x = 50.0f; x <= 170.0f; x += 20.0f)
      for (y = -60.0f; y <= 60.0f; y += 30.0f)
        for (z = -130.0f; z <= -5.0f; z += 25.0f) {
          sakin_vec3_t p = { x, y, z }, back;
          sakin_leg_angles_t a;
          sakin_real_t r, d, err;
          r = (sakin_real_t)SAKIN__HYPOT(x, y) - cfg.coxa_len;
          d = (sakin_real_t)SAKIN__HYPOT(r, z);
          if (d < SAKIN__FABS(cfg.femur_len - cfg.tibia_len) + 2.0f ||
              d > cfg.femur_len + cfg.tibia_len - 2.0f)
            continue; /* outside workspace: covered by the clip test below */
          if (sakin_leg_ik(&cfg, p, &a) != SAKIN_IK_OK) continue;
          sakin_leg_fk(&cfg, a, &back);
          err = dist3(p, back);
          if (err > worst) worst = err;
          ++tested;
        }
    /* Assert */
    CHECK(tested > 100, "roundtrip grid should cover >100 reachable points");
    CHECK(worst < 0.5f,
          branch ? "elbow-down roundtrip error < 0.5 mm"
                 : "elbow-up roundtrip error < 0.5 mm");
  }
}

/* --- Known solution: IK must recover the angles that FK placed ----------- */
static void ik_recovers_precomputed_angles_for_both_branches(void) {
  /* Arrange */
  sakin_config_t cfg;
  int branch;
  sakin_config_default(&cfg);
  for (branch = 0; branch <= 1; ++branch) {
    /* gamma = 0, beta = 45 deg elevation, kappa = 90 deg fold */
    sakin_leg_angles_t want = { 0.0f, -SAKIN_PI / 4.0f, SAKIN_PI / 2.0f }, got;
    sakin_vec3_t p;
    int status;
    cfg.elbow_up = branch ? 0 : 1;
    /* Act: put a foot exactly at FK(want), then solve back */
    sakin_leg_fk(&cfg, want, &p);
    status = sakin_leg_ik(&cfg, p, &got);
    /* Assert */
    CHECK(status == SAKIN_IK_OK,
          branch ? "elbow-down: known point solves without clipping"
                 : "elbow-up: known point solves without clipping");
    CHECK(SAKIN__FABS(got.coxa - want.coxa) < 1e-3f &&
          SAKIN__FABS(got.femur - want.femur) < 1e-3f &&
          SAKIN__FABS(got.tibia - want.tibia) < 1e-3f,
          branch ? "elbow-down: IK recovers FK angles (45/90 deg)"
                 : "elbow-up: IK recovers FK angles (45/90 deg)");
  }
}

/* --- Reachability: far targets report CLIPPED and saturate --------------- */
static void ik_clips_unreachable_targets_and_reports_it(void) {
  /* Arrange */
  sakin_config_t cfg;
  sakin_leg_angles_t a;
  sakin_vec3_t far = { 500.0f, 0.0f, 0.0f }, foot;
  sakin_real_t r, d;
  int status;
  sakin_config_default(&cfg);
  /* Act */
  status = sakin_leg_ik(&cfg, far, &a);
  sakin_leg_fk(&cfg, a, &foot);
  r = (sakin_real_t)SAKIN__HYPOT(foot.x, foot.y) - cfg.coxa_len;
  d = (sakin_real_t)SAKIN__HYPOT(r, foot.z);
  /* Assert */
  CHECK(status & SAKIN_IK_CLIPPED_WORKSPACE,
        "far target sets the workspace-clip flag");
  CHECK(SAKIN__FABS(d - (cfg.femur_len + cfg.tibia_len)) < 1.0f,
        "clipped solution saturates at the workspace boundary");
}

/* --- Inside the coxa cylinder (|x,y| < coxa_len): still finite, exact when
     reachable. The quadrant-safe atan2 formulation (Craig 4.4) keeps this
     region solvable; only the joint/workspace-clip flags may differ. ------ */
static void ik_inside_coxa_cylinder_stays_exact_when_reachable(void) {
  /* Arrange */
  sakin_config_t cfg;
  int branch, ok = 1;
  sakin_real_t worst = 0;
  sakin_config_default(&cfg);
  for (branch = 0; branch <= 1; ++branch) {
    sakin_real_t x, y, z;
    cfg.elbow_up = branch ? 0 : 1;
    /* Act: grid strictly inside the coxa cylinder (coxa_len = 35) */
    for (x = 0.0f; x <= 30.0f; x += 10.0f)
      for (y = 0.0f; y <= 30.0f; y += 10.0f)
        for (z = -120.0f; z <= -10.0f; z += 30.0f) {
          sakin_vec3_t p = { x, y, z }, back;
          sakin_leg_angles_t a;
          sakin_real_t r0 = (sakin_real_t)SAKIN__HYPOT(x, y) - cfg.coxa_len;
          sakin_real_t d = (sakin_real_t)SAKIN__HYPOT(r0, z);
          int status;
          if (d < 2.0f || d > cfg.femur_len + cfg.tibia_len - 2.0f)
            continue; /* outside the annulus: covered by the clip test above */
          status = sakin_leg_ik(&cfg, p, &a);
          /* never NaN, always within joint limits */
          if (a.coxa != a.coxa || a.femur != a.femur || a.tibia != a.tibia) ok = 0;
          if (a.coxa < -cfg.coxa_lim - 1e-3f || a.coxa > cfg.coxa_lim + 1e-3f) ok = 0;
          if (a.femur < -cfg.femur_lim - 1e-3f || a.femur > cfg.femur_lim + 1e-3f) ok = 0;
          if (a.tibia < cfg.tibia_min - 1e-3f || a.tibia > cfg.tibia_max + 1e-3f) ok = 0;
          /* exactness is only promised when IK reports OK */
          if (status == SAKIN_IK_OK) {
            sakin_leg_fk(&cfg, a, &back);
            if (dist3(p, back) > worst) worst = dist3(p, back);
            if (dist3(p, back) > 1.0f) ok = 0;
          }
        }
  }
  /* Assert */
  CHECK(ok, "inside-coxa targets: finite, limited, exact-when-OK (both branches)");
  CHECK(worst < 1.0f, "inside-coxa OK roundtrips within 1 mm");
}

/* --- Degenerate config: zero link length must not NaN the solver ---------- */
static void ik_degenerate_config_reports_bad_config(void) {
  /* Arrange */
  sakin_config_t cfg;
  sakin_leg_angles_t a = { 9.0f, 9.0f, 9.0f };
  int status;
  sakin_config_default(&cfg);
  cfg.femur_len = 0.0f;
  /* Act */
  status = sakin_leg_ik(&cfg, (sakin_vec3_t){ 115.0f, 0.0f, -85.0f }, &a);
  /* Assert */
  CHECK(status == SAKIN_IK_BAD_CONFIG, "zero femur_len sets BAD_CONFIG");
  CHECK(a.coxa == 0.0f && a.femur == 0.0f && a.tibia == 0.0f,
        "BAD_CONFIG returns a safe zero pose (no NaN)");
}

/* --- Body compensation: level = identity, yaw-90 rotates exactly --------- */
static void body_rotate_inv_is_exact(void) {
  /* Arrange */
  sakin_vec3_t p = { 100.0f, 0.0f, -50.0f }, level, turned, rolled, pitched;
  /* Act */
  level = sakin_body_rotate_inv(p, 0.0f, 0.0f, 0.0f);
  turned = sakin_body_rotate_inv(p, 0.0f, 0.0f, SAKIN_PI / 2.0f);
  rolled = sakin_body_rotate_inv(p, SAKIN_PI / 2.0f, 0.0f, 0.0f);
  pitched = sakin_body_rotate_inv(p, 0.0f, SAKIN_PI / 2.0f, 0.0f);
  /* Assert */
  CHECK(dist3(p, level) < 1e-4f, "level body leaves feet untouched");
  CHECK(SAKIN__FABS(turned.x - 0.0f) < 1e-2f &&
        SAKIN__FABS(turned.y + 100.0f) < 1e-2f &&
        SAKIN__FABS(turned.z + 50.0f) < 1e-4f,
        "yaw +90 maps (x,y,z) -> (y,-x,z) exactly (R^T)");
  CHECK(SAKIN__FABS(rolled.x - 100.0f) < 1e-2f &&
        SAKIN__FABS(rolled.y + 50.0f) < 1e-2f &&
        SAKIN__FABS(rolled.z - 0.0f) < 1e-2f,
        "roll +90 maps (x,y,z) -> (x,z,-y) exactly (R^T)");
  CHECK(SAKIN__FABS(pitched.x - 50.0f) < 1e-2f &&
        SAKIN__FABS(pitched.y - 0.0f) < 1e-2f &&
        SAKIN__FABS(pitched.z - 100.0f) < 1e-2f,
        "pitch +90 maps (x,y,z) -> (z,y,x) exactly (R^T)");
}

/* --- Standing pose: feet on the ground, correct radial distance ---------- */
static void stance_feet_sit_on_ground_at_nominal_reach(void) {
  /* Arrange */
  sakin_config_t cfg;
  int l, ok = 1;
  sakin_config_default(&cfg);
  /* Act + Assert per leg */
  for (l = 0; l < cfg.leg_count; ++l) {
    sakin_vec3_t ep = sakin_stance_foot(&cfg, l);
    sakin_mount_t m = cfg.mounts[l];
    sakin_real_t dx = ep.x - m.x, dy = ep.y - m.y;
    sakin_real_t horiz = (sakin_real_t)SAKIN__HYPOT(dx, dy);
    if (SAKIN__FABS(ep.z - (-(cfg.coxa_height + cfg.body_half_h) + cfg.foot_radius)) > 1e-3f)
      ok = 0;
    if (SAKIN__FABS(horiz - (cfg.coxa_len + cfg.foot_reach)) > 1e-3f)
      ok = 0;
  }
  CHECK(ok, "all feet: ground height + radial coxa_len+foot_reach from mount");
}

/* --- Gait engine: bounded, finite, limit-respecting angles while walking - */
static void gait_engine_walk_is_finite_bounded_and_within_limits(void) {
  /* Arrange */
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  int i, ok = 1, gait;
  sakin_init(&bot, NULL);
  /* Act: walk forward 5 s at 60 Hz in three different gaits */
  for (gait = 0; gait < 3 && ok; ++gait) {
    static const int ids[3] = { SAKIN_GAIT_TRIPOD, SAKIN_GAIT_RIPPLE, SAKIN_GAIT_AMBLE };
    sakin_reset_pose(&bot);
    sakin_gait_select(&bot, ids[gait]);
    sakin_command(&bot, 100.0f, 0.0f, 0.5f);
    for (i = 0; i < 300; ++i) {
      int l;
      sakin_update(&bot, 1.0f / 60.0f, a);
      if (!all_angles_finite_and_limited(&bot.cfg, a, bot.cfg.leg_count)) ok = 0;
      for (l = 0; l < bot.cfg.leg_count; ++l) {
        sakin_real_t m = (sakin_real_t)SAKIN__HYPOT(bot.offsets[l].x, bot.offsets[l].y);
        if (m > bot.cfg.max_offset + 0.5f) ok = 0;
        if (bot.offsets[l].z < -0.5f || bot.offsets[l].z > bot.cfg.lift_height + 0.5f) ok = 0;
      }
    }
  }
  /* Assert */
  CHECK(ok, "tripod/ripple/amble: angles finite+limited, offsets clamped");
}

/* --- Gait switch mid-walk must not teleport the feet --------------------- */
static void gait_switch_mid_walk_stays_continuous(void) {
  /* Arrange */
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  sakin_vec3_t before[SAKIN_MAX_LEGS];
  int i, l, ok = 1;
  sakin_init(&bot, NULL);
  sakin_command(&bot, 80.0f, 0.0f, 0.0f);
  for (i = 0; i < 60; ++i) sakin_update(&bot, 1.0f / 60.0f, a);
  for (l = 0; l < SAKIN_MAX_LEGS; ++l) before[l] = bot.offsets[l];
  /* Act */
  sakin_gait_select(&bot, SAKIN_GAIT_RIPPLE);
  sakin_update(&bot, 1.0f / 60.0f, a);
  /* Assert: per-tick offset jump stays small (stride speed ~1.3 mm/tick) */
  for (l = 0; l < bot.cfg.leg_count; ++l)
    if (dist3(before[l], bot.offsets[l]) > 15.0f) ok = 0;
  CHECK(ok, "gait transition keeps foot offsets continuous (<15 mm/tick)");
}

/* --- Stopping settles the robot back to the nominal stance --------------- */
static void stopping_decays_offsets_to_zero(void) {
  /* Arrange */
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  int i, l;
  sakin_real_t worst = 0;
  sakin_init(&bot, NULL);
  sakin_command(&bot, 100.0f, 0.0f, 0.0f);
  for (i = 0; i < 60; ++i) sakin_update(&bot, 1.0f / 60.0f, a);
  /* Act: command stop, run 3 s (>> settle_time) */
  sakin_command(&bot, 0.0f, 0.0f, 0.0f);
  for (i = 0; i < 180; ++i) sakin_update(&bot, 1.0f / 60.0f, a);
  for (l = 0; l < bot.cfg.leg_count; ++l) {
    sakin_real_t m = dist3(bot.offsets[l], (sakin_vec3_t){ 0, 0, 0 });
    if (m > worst) worst = m;
  }
  /* Assert */
  CHECK(worst < 0.5f, "offsets decay to (near) zero when standing still");
}

#ifndef SAKIN_NO_JUMP
/* --- Jump: runs through all phases and hands control back to the gait ---- */
static void jump_completes_and_returns_to_gait(void) {
  /* Arrange */
  sakin_t bot;
  sakin_leg_angles_t a[SAKIN_MAX_LEGS];
  int i, done = 0, ok = 1, saw_retract = 0;
  sakin_init(&bot, NULL);
  /* Act */
  sakin_jump_start(&bot);
  for (i = 0; i < 600 && !done; ++i) {   /* 10 s at 60 Hz safety bound */
    sakin_update(&bot, 1.0f / 60.0f, a);
    if (sakin_jump_phase(&bot) == SAKIN_JUMP_RETRACT) saw_retract = 1;
    if (!sakin_jump_active(&bot)) done = 1;
    if (!all_angles_finite_and_limited(&bot.cfg, a, bot.cfg.leg_count)) ok = 0;
  }
  /* Assert */
  CHECK(done, "jump finishes within 10 simulated seconds");
  CHECK(saw_retract, "jump passes through the retract (toe-clearance) phase");
  CHECK(ok, "angles stay finite and within limits during the jump");
}
#endif

int main(void) {
  ik_fk_roundtrip_holds_for_both_elbow_branches();
  ik_recovers_precomputed_angles_for_both_branches();
  ik_clips_unreachable_targets_and_reports_it();
  ik_inside_coxa_cylinder_stays_exact_when_reachable();
  ik_degenerate_config_reports_bad_config();
  body_rotate_inv_is_exact();
  stance_feet_sit_on_ground_at_nominal_reach();
  gait_engine_walk_is_finite_bounded_and_within_limits();
  gait_switch_mid_walk_stays_continuous();
  stopping_decays_offsets_to_zero();
#ifndef SAKIN_NO_JUMP
  jump_completes_and_returns_to_gait();
#endif

  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
