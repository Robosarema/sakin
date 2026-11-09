/* SPDX-License-Identifier: 0BSD */
/* config_matrix.c - compile-only check for the SAKIN_* configuration matrix.
 * Uses core (Mode 1) API only, so it builds under every macro combination:
 *   -DSAKIN_NO_GAIT -DSAKIN_NO_JUMP -DSAKIN_USE_DOUBLE -DSAKIN_STATIC
 *   -DSAKIN_MAX_LEGS=4
 */
#define SAKIN_IMPLEMENTATION
#include "../sakin.h"

#ifndef SAKIN_NO_GAIT
/* reference the body-Z setter so SAKIN_STATIC builds don't trip
 * -Wunused-function (the setter is compiled out under SAKIN_NO_GAIT) */
static void reference_body_height(sakin_t* s) {
  sakin_body_height(s, 12.5f);
}
#endif

int main(void) {
  sakin_config_t cfg;
  sakin_leg_angles_t a;
  sakin_vec3_t p = { 115.0f, 0.0f, -85.0f }, back, rot, stance;
  sakin_config_default(&cfg);
  (void)sakin_leg_ik(&cfg, p, &a);
  sakin_leg_fk(&cfg, a, &back);
  /* reference every public core function so SAKIN_STATIC builds don't trip
   * -Wunused-function on the ones the gait engine would normally use */
  rot = sakin_body_rotate_inv(p, 0.1f, 0.0f, 0.0f);
  stance = sakin_stance_foot(&cfg, 0);
#ifndef SAKIN_NO_GAIT
  {
    sakin_t bot;
    sakin_init(&bot, &cfg);
    reference_body_height(&bot);
  }
#endif
  return (int)(back.x - p.x + rot.y * 0.0f + stance.z * 0.0f); /* ~0 */
}
