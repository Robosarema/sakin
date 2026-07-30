// sakin.h - STB-style single-header Library Inverse Kinematics & Gait Engine Hexapod based on
// nuke.h

#ifndef SAKIN_H

#define SAKIN_H

#include <math.h>

#define LEG_COUNT 6

// Leg index constant
#define RIGHT_FRONT 0
#define RIGHT_REAR 1
#define LEFT_FRONT 2
#define LEFT_REAR 3
#define RIGHT_MIDDLE 4
#define LEFT_MIDDLE 5

#define X_COXA 55.0  // mount offset forward of body center
#define Y_COXA 55.0  // mount offset sideways of body center
#define M_COXA 100.0 // unused right now

// Joint limits (radians) = must match urdf
#define COXA_LIM 1.5
#define FEMUR_LIM 1.5
#define TIBIA_MAX 2.96706

// link distance
#define L_COXA 35.0  // groin
#define L_FEMUR 82.0 // Femur length
#define L_TIBIA 80.0 // TIBIAN Length

#define RIPPLE 0 // Gaya jalan gelombang (Napak 5, Angkat 1 bergantian)
#define RIPPLE_LEFT 1
#define RIPPLE_RIGHT 2
#define AMBLE 5  // Gaya jalan lambat
#define TRIPOD 6 // Gaya jalan 3 kaki (Paling cepat dan stabil untuk Hexapod)

#define BODY_HALF_H 20.0       // body half-thickness
#define COXA_HEIGHT 70.0       // coxa joint height above ground when standing
#define FOOT_REACH 80.0        // horizontal femur-joint → foot distance at stance
#define FOOT_RADIUS 5.0        // foot-tip sphere radius in the URDF (compensated in z)
#define LIFT_HEIGHT 40.0       // swing apex height
#define MAX_OFFSET 35.0        // safety clamp on horizontal gait offsets
#define JUMP_SQUAT_DEPTH 25.0  // mm - how much body drops before
#define JUMP_THRUST_LIFT 100.0 // mm - how height body center rises during thrust
#define JUMP_RETRACT_LIFT 50.0 // mm - legs pull up during flight
#define JUMP_SQUAT_TIME 0.15   // seconds - squat duration
#define JUMP_THRUST_TIME 0.12  // seconds - thrust duration
#define JUMP_RETRACT_TIME 0.12 // seconds - retract legs for flight
#define JUMP_LAND_TIME 0.10    // seconds - extend legs to absorb the land impact

typedef struct {
  double x, y, z;
} sakin_vec3_t;

#endif // SAKIN_H

#ifdef SAKIN_IMPLEMENTATION

sakin_vec3_t leg_ik(sakin_vec3_t v) {
  // Solve the 3-DOF leg for the foot target (x, y, z) in the coxa frame
  // return (coxa, femur, tibia)
  double gamma = atan2(v.y, v.x);
  double r = hypot(v.x, v.y) - (double)L_COXA;
  double d = hypot(r, v.z);
  d = fmax(abs(L_FEMUR - L_TIBIA) + 1e-6, fmin(L_FEMUR + L_TIBIA - 1e-6, d));

  double alpha = atan2(v.z, r); // foot-line elevation
  double cos_b = (L_FEMUR * L_FEMUR + d * d - L_TIBIA * L_TIBIA) / (2.0 * L_FEMUR * d);

  double beta = alpha + acos(fmax(-1.0, fmin(1.0, cos_b))); // elbow-UP

  double cos_p = (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - d * d) / (2.0 * L_FEMUR * L_TIBIA);
  double kappa = M_PI - acos(fmax(-1.0, fmin(1.0, cos_p))); // kee fold

  gamma = fmax(-COXA_LIM, fmin(COXA_LIM, gamma));
  double femur = fmax(-FEMUR_LIM, fmin(FEMUR_LIM, -beta)); // URDF: +Y axis pitches down
  kappa = fmax(0.0, fmin(TIBIA_MAX, kappa));
  return (sakin_vec3_t){ .x = gamma, .y = femur, .z = kappa };
}

sakin_vec3_t leg_fk(double coxa, double femur, double tibia) {

  double beta = -femur;
  double kappa = tibia;

  double r = (L_COXA + L_FEMUR * cos(beta));
  double z = (L_FEMUR * sin(beta) + L_TIBIA * sin(beta - kappa));

  return (sakin_vec3_t){ .x = r * cos(coxa), .y = r * sin(coxa), .z = z };
}

#endif // SAKIN_IMPLEMENTATION
