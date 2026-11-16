/*
 * CK_NAV_CONTROL - altitude and position controllers
 *
 * Cascaded structure (ArduPilot AC_PosControl / INAV):
 *   position error -> velocity target -> acceleration target -> throttle (vertical) or tilt angle (horizontal)
 */
#ifndef CK_NAV_CONTROL_H_
#define CK_NAV_CONTROL_H_

#include "CK_DEFINITIONS.h"

typedef struct {
    float throttle;             /* 0..1 */
    float pitch_deg;            /* nose down + */
    float roll_deg;             /* right + */
} nav_output_t;

/* Velocity target from a position error: linear for small errors, acceleration limited square root for large ones */
float CK_NAV_CTRL_SqrtController(float error, float kp, float accel_limit);

void  CK_NAV_CTRL_ResetVertical(float initial_throttle);
float CK_NAV_CTRL_Vertical(float vel_target, bool learn_hover, float dt);   /* returns throttle 0..1 */
float CK_NAV_CTRL_HoverThrottle(void);

void  CK_NAV_CTRL_ResetHorizontal(void);
void  CK_NAV_CTRL_Horizontal(const float vel_target[2], float max_tilt_deg, float dt, nav_output_t *out);
void  CK_NAV_CTRL_TiltFromAccel(float accel_n, float accel_e, float max_tilt_deg, nav_output_t *out);
void  CK_NAV_CTRL_WindAccel(float accel_ne[2]);             /* acceleration holding against the wind (integral) */

#endif
