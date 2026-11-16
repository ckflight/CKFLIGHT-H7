#include "CK_DEFINITIONS.h"

#if NAVIGATION_

#include <COMMON/maths.h>
#include "COMMON/axis.h"
#include "COMMON/vector.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_IMU.h"
#include "FLIGHT/CK_NAV_ESTIMATOR.h"
#include "FLIGHT/CK_NAV_CONTROL.h"

#define GRAVITY             9.80665f
#define DEG2RAD             (M_PIf / 180.0f)

/* Vertical velocity -> acceleration */
#define VZ_KP               3.0f
#define VZ_KI               1.2f
#define VZ_ACCEL_LIMIT      6.0f
#define THROTTLE_I_LIMIT    0.30f
#define THROTTLE_MIN        0.04f
#define THROTTLE_MAX        0.90f
#define HOVER_LEARN_TAU     4.0f
#define HOVER_MIN           0.12f
#define HOVER_MAX           0.70f

/* Horizontal velocity -> acceleration */
#define VXY_KP              1.6f
#define VXY_KI              0.5f
#define VXY_I_LIMIT         5.5f            /* m/s2, for wind */
#define TILT_LPF_HZ         8.0f

static float hover_throttle = 0.40f;
static float vz_integral;

static float vxy_integral[2];
static float accel_filtered[2];

float CK_NAV_CTRL_SqrtController(float error, float kp, float accel_limit)
{
    const float linear_limit = accel_limit / (kp * kp);

    if (ABS(error) <= linear_limit) {
        return error * kp;
    }
    const float root = sqrtf(2.0f * accel_limit * (ABS(error) - 0.5f * linear_limit));
    return (error > 0.0f) ? root : -root;
}

/* ---------------------------------------------------------- vertical */

void CK_NAV_CTRL_ResetVertical(float initial_throttle)
{
    /* No throttle jump when switching modes: start the integral from the current throttle */
    vz_integral = constrainf(initial_throttle - hover_throttle, -THROTTLE_I_LIMIT, THROTTLE_I_LIMIT);
}

float CK_NAV_CTRL_HoverThrottle(void)
{
    return hover_throttle;
}

static float tilt_compensation(void)
{
    /* rMat[2][2]: cosine of the angle between body Z and earth Z */
    return 1.0f / constrainf(rMat.m[2][2], 0.5f, 1.0f);
}

float CK_NAV_CTRL_Vertical(float vel_target, bool learn_hover, float dt)
{
    const float error = vel_target - nav_est.vel[NAV_U];

    const float accel = constrainf(error * VZ_KP, -VZ_ACCEL_LIMIT, VZ_ACCEL_LIMIT);

    vz_integral += error * VZ_KI * dt * (hover_throttle / GRAVITY);

    const float raw = (hover_throttle + vz_integral + accel * hover_throttle / GRAVITY) * tilt_compensation();
    const float output = constrainf(raw, THROTTLE_MIN, THROTTLE_MAX);

    /* Do not grow the integral while saturated */
    if (raw != output) {
        vz_integral -= error * VZ_KI * dt * (hover_throttle / GRAVITY);
    }
    vz_integral = constrainf(vz_integral, -THROTTLE_I_LIMIT, THROTTLE_I_LIMIT);

    /* Learn the hover throttle in steady flight: the integral is slowly moved into the hover value */
    if (learn_hover && ABS(nav_est.vel[NAV_U]) < 0.5f && ABS(nav_est.acc[NAV_U]) < 1.0f) {
        const float rate = dt / HOVER_LEARN_TAU;
        const float move = vz_integral * rate;
        hover_throttle = constrainf(hover_throttle + move, HOVER_MIN, HOVER_MAX);
        vz_integral -= move;
    }

    return output;
}

/* ---------------------------------------------------------- horizontal */

void CK_NAV_CTRL_ResetHorizontal(void)
{
    vxy_integral[0] = vxy_integral[1] = 0.0f;
    accel_filtered[0] = accel_filtered[1] = 0.0f;
}

void CK_NAV_CTRL_WindAccel(float accel_ne[2])
{
    accel_ne[0] = vxy_integral[0];
    accel_ne[1] = vxy_integral[1];
}

void CK_NAV_CTRL_TiltFromAccel(float accel_n, float accel_e, float max_tilt_deg, nav_output_t *out)
{
    const float heading = nav_est.heading_deg * DEG2RAD;
    const float c = cosf(heading), s = sinf(heading);

    /* North-east acceleration to body forward-right */
    const float forward =  accel_n * c + accel_e * s;
    const float right   = -accel_n * s + accel_e * c;

    out->pitch_deg = atan2f(forward, GRAVITY) / DEG2RAD;
    out->roll_deg  = atan2f(right,   GRAVITY) / DEG2RAD;

    /* Total tilt limit: direction kept, magnitude limited */
    const float tilt = sqrtf(out->pitch_deg * out->pitch_deg + out->roll_deg * out->roll_deg);
    if (tilt > max_tilt_deg) {
        out->pitch_deg *= max_tilt_deg / tilt;
        out->roll_deg  *= max_tilt_deg / tilt;
    }
}

void CK_NAV_CTRL_Horizontal(const float vel_target[2], float max_tilt_deg, float dt, nav_output_t *out)
{
    const float accel_limit = GRAVITY * tanf(max_tilt_deg * DEG2RAD);
    const float lpf = dt / (dt + 1.0f / (2.0f * M_PIf * TILT_LPF_HZ));

    float accel[2];

    for (int i = 0; i < 2; i++) {
        const float error = vel_target[i] - nav_est.vel[i];
        accel[i] = error * VXY_KP + vxy_integral[i];
    }

    const bool saturated = sqrtf(accel[0] * accel[0] + accel[1] * accel[1]) > accel_limit;

    for (int i = 0; i < 2; i++) {
        const float error = vel_target[i] - nav_est.vel[i];
        const bool shrinking = (error * vxy_integral[i]) < 0.0f;
        /* While saturated the integral may only shrink, otherwise recovery locks up */
        if (!saturated || shrinking) {
            vxy_integral[i] = constrainf(vxy_integral[i] + error * VXY_KI * dt, -VXY_I_LIMIT, VXY_I_LIMIT);
        }
        accel_filtered[i] += (accel[i] - accel_filtered[i]) * lpf;
    }

    CK_NAV_CTRL_TiltFromAccel(accel_filtered[0], accel_filtered[1], max_tilt_deg, out);
}

#endif /* NAVIGATION_ */
