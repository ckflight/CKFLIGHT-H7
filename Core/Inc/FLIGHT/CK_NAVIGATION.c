#include "CK_DEFINITIONS.h"
#include "FLIGHT/CK_NAVIGATION.h"

#if NAVIGATION_

#include <COMMON/maths.h>
#include "COMMON/axis.h"
#include "COMMON/vector.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_IMU.h"

#include "COMMUNICATION/CK_PRINTER.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/CK_NAV_ESTIMATOR.h"
#include "FLIGHT/CK_NAV_CONTROL.h"

/*
 * Ported from the full CKFLIGHT navigation (state machine, trajectories, landing detection).
 * Missions and autonomous failsafe actions are not part of the light firmware.
 */

#define NAV_PERIOD_US       10000u          // 100 Hz
#define DEG2RAD             (M_PIf / 180.0f)

// Speed and altitude limits
#define CLIMB_RATE          3.0f            // m/s
#define DESCENT_RATE        1.5f
#define LAND_RATE_HIGH      1.5f
#define LAND_RATE_LOW       0.5f
#define LAND_SLOW_ALT       4.0f            // m: slow descent below this altitude
#define RTH_ALT             20.0f           // m
#define RTH_SPEED           8.0f            // m/s
#define MAX_TILT_DEG        30.0f
#define TILT_LOW_DEG        20.0f
#define TILT_LOW_ALT        1.0f

// Trajectory
#define POS_KP              1.0f
#define ALT_KP              1.0f
#define ACCEL_LIMIT         3.0f            // m/s2: velocity target slew
#define ALT_ACCEL_LIMIT     2.5f
#define WP_RADIUS           2.0f            // m
#define WP_ALT_TOLERANCE    1.5f
#define XTRACK_KP           0.8f
#define HOME_RADIUS         3.0f
#define YAW_FACE_DIST       4.0f            // m: do not turn the nose when closer than this
#define YAW_KP              2.0f
#define YAW_RATE_MAX        60.0f           // deg/s

// Pilot input
#define STICK_DEADBAND      50
#define STICK_TAKEOVER      150
#define STICK_SPEED         5.0f            // m/s
#define THR_DEADBAND        80

// GPS loss (dead reckoning)
#define GPS_LOST_MAX_S      25.0f
#define GPS_LOST_TILT_DEG   10.0f
#define GPS_LOST_MAX_TILT   22.0f
#define GPS_LOST_SPEED      6.0f            // approximate ground speed at this tilt, m/s
#define WIND_LEARN_MAX_SPEED 1.5f
#define WIND_LEARN_TAU      3.0f

// Landing detection
#define LANDED_THROTTLE     0.85f           // relative to the hover throttle
#define LANDED_VEL          0.30f           // m/s: slower than this while descent is commanded
#define LANDED_ACC          2.0f
#define LANDED_TIME         1.5f

#define AIRBORNE_ALT        0.5f
#define AIRBORNE_THROTTLE   0.20f
#define AIRBORNE_TIME       0.8f
#define BLIND_LAND_THROTTLE 0.95f           // relative to the pilot hover throttle
#define BLIND_LAND_TIME     20.0f           // s
#define ALIGN_TILT_DEG      12.0f
#define ALIGN_TIMEOUT       10.0f

typedef struct {
    nav_state_e state;
    nav_state_e prev_state;
    nav_state_e resume;         // state to continue when GPS comes back

    bool  was_armed;
    bool  airborne;
    bool  pilot_override;       // taken by the sticks; stays with the pilot until a switch changes
    int   last_request;

    float target[3];            // N, E, U
    float vel_cmd[2];           // acceleration limited horizontal velocity target
    float z_vel_cmd;
    float yaw_target;
    float thr_center;

    bool  home_set;
    float home[2];

    float wp_start[2];
    float dr_bearing;
    float dr_home_dist;
    float wind_accel[2];        // acceleration holding against the wind, learnt while hovering
    float state_time;
    float landed_timer;
    float throttle_up_time;
    float pilot_hover;

    nav_output_t out;
    float yaw_rate;
    bool  braking;              // stop first: target is the current position, velocity slews to 0
    float brake_time;
    bool  stick_xy_was_active;
    bool  control_xy;
    bool  control_z;
    bool  control_yaw;
    bool  output_fresh;
} nav_t;

static nav_t nav;

DEBUG_TIME_t navigation_debug;

/* ---------------------------------------------------------- helpers */

static float wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static float distance_to(float n, float e)
{
    const float dn = n - nav_est.pos[NAV_N];
    const float de = e - nav_est.pos[NAV_E];
    return sqrtf(dn * dn + de * de);
}

static float bearing_to(float n, float e)
{
    return atan2f(e - nav_est.pos[NAV_E], n - nav_est.pos[NAV_N]) / DEG2RAD;
}

static float slew(float current, float target, float max_step)
{
    return current + constrainf(target - current, -max_step, max_step);
}

static bool stick_active(int axis, int threshold)
{
    return ABS(getRCDataRaw(axis) - 1500) > threshold;
}

/* ---------------------------------------------------------- capability */

static nav_capability_e capability(void)
{
    const bool z = nav_est.z_valid;
    const bool xy = nav_est.xy_valid;

    if (xy && z && nav_est.has_baro && nav_est.has_mag) return NAV_CAP_FULL;
    if (xy && z)                                        return NAV_CAP_RTH_BASIC;
    if (z)                                              return NAV_CAP_ALT;
    return NAV_CAP_MANUAL;
}

nav_capability_e CK_NAVIGATION_GetCapability(void) { return capability(); }

/* ---------------------------------------------------------- state entry */

static void set_position_target_here(void)
{
    nav.target[NAV_N] = nav_est.pos[NAV_N];
    nav.target[NAV_E] = nav_est.pos[NAV_E];
    nav.vel_cmd[0] = nav_est.vel[NAV_N];
    nav.vel_cmd[1] = nav_est.vel[NAV_E];
}

static void enter(nav_state_e state)
{
    const bool was_idle = (nav.state == NAV_IDLE);

    nav.prev_state = nav.state;
    nav.state = state;
    nav.state_time = 0.0f;
    nav.landed_timer = 0.0f;
    nav.yaw_target = nav_est.heading_deg;
    {
        // Entering fast (> 2 m/s): stop first, then hold where it stopped; slow: hold the current position.
        // Only when coming from pilot flight. After RTH the landing keeps its target point.
        const float sp = sqrtf(nav_est.vel[NAV_N] * nav_est.vel[NAV_N] + nav_est.vel[NAV_E] * nav_est.vel[NAV_E]);
        const bool from_pilot = was_idle || nav.prev_state == NAV_ALT_HOLD || nav.prev_state == NAV_POS_HOLD;
        nav.braking = (state == NAV_LAND || state == NAV_POS_HOLD) && from_pilot && nav_est.xy_valid && sp > 2.0f;
        nav.brake_time = 0.0f;
    }
    nav.stick_xy_was_active = false;

    if (state == NAV_POS_HOLD && !was_idle) {
        // From altitude hold: the horizontal target becomes the current position
        CK_NAV_CTRL_ResetHorizontal();
        set_position_target_here();
    }

    if (was_idle && state != NAV_IDLE) {
        // Taking over from the pilot: no jump in throttle or tilt
        CK_NAV_CTRL_ResetVertical((getRCCommand(THROTTLE) - 1000.0f) * 0.001f);
        CK_NAV_CTRL_ResetHorizontal();
        nav.target[NAV_U] = nav_est.pos[NAV_U];
        nav.z_vel_cmd = nav_est.vel[NAV_U];
        set_position_target_here();
        const int thr = getRCDataRaw(THROTTLE);
        nav.thr_center = (thr > 1250 && thr < 1750) ? (float)thr : 1500.0f;
    }
}

static void start_rth(void)
{
    if (!nav.home_set || !nav_est.xy_valid) {
        enter(NAV_LAND);
        return;
    }
    if (!nav_est.heading_valid) {
        enter(NAV_RTH_ALIGN);
        return;
    }
    enter(NAV_RTH_CLIMB);
    nav.target[NAV_U] = MAX(nav_est.pos[NAV_U], RTH_ALT);
    if (distance_to(nav.home[0], nav.home[1]) < HOME_RADIUS) {
        nav.target[NAV_U] = nav_est.pos[NAV_U];
    }
}

/* ---------------------------------------------------------- pilot request (switches) */

enum { REQ_NONE, REQ_ALT, REQ_POS, REQ_RTH, REQ_LAND };

static int pilot_request(void)
{
    if (flags.LANDING)              return REQ_LAND;
    if (flags.GPS_RESCUE)           return REQ_RTH;
    if (flags.GPS_POS_HOLD)         return REQ_POS;
    if (flags.ALTITUDE_HOLD)        return REQ_ALT;
    return REQ_NONE;
}

static void apply_request(int request)
{
    switch (request) {
    case REQ_LAND:
        enter((nav.airborne && nav_est.z_valid) ? NAV_LAND : NAV_IDLE);
        break;
    case REQ_RTH:
        if (!nav.airborne)                          enter(NAV_IDLE);
        else if (capability() >= NAV_CAP_RTH_BASIC) start_rth();
        else if (nav_est.z_valid)                   enter(NAV_LAND);
        else                                        enter(NAV_IDLE);
        break;
    case REQ_POS:
        if (capability() == NAV_CAP_FULL)       enter(NAV_POS_HOLD);
        else if (nav_est.z_valid)               enter(NAV_ALT_HOLD);
        else                                    enter(NAV_IDLE);
        break;
    case REQ_ALT:
        enter(nav_est.z_valid ? NAV_ALT_HOLD : NAV_IDLE);
        break;
    default:
        enter(NAV_IDLE);
        break;
    }
}

static void update_mode_selection(void)
{
    const int request = pilot_request();

    if (request != nav.last_request) {
        nav.last_request = request;
        nav.pilot_override = false;
        apply_request(request);
        return;
    }

    if (nav.pilot_override) {
        return;     // the pilot took over: stays manual until a switch changes
    }

    // Moving roll or pitch during an automatic flight gives the control back to the pilot at once
    const bool auto_flight = nav.state >= NAV_RTH_ALIGN && nav.state <= NAV_GPS_LOST;
    if (auto_flight && (stick_active(ROLL, STICK_TAKEOVER) || stick_active(PITCH, STICK_TAKEOVER))) {
        nav.pilot_override = true;
        enter(NAV_IDLE);
    }
}

/* ---------------------------------------------------------- trajectory helpers */

static void hold_altitude_step(float dt)
{
    const float err = nav.target[NAV_U] - nav_est.pos[NAV_U];
    float vz = CK_NAV_CTRL_SqrtController(err, ALT_KP, ALT_ACCEL_LIMIT);
    vz = constrainf(vz, -DESCENT_RATE, CLIMB_RATE);
    nav.z_vel_cmd = slew(nav.z_vel_cmd, vz, ALT_ACCEL_LIMIT * dt);
}

static void hold_position_step(float dt)
{
    float vn = CK_NAV_CTRL_SqrtController(nav.target[NAV_N] - nav_est.pos[NAV_N], POS_KP, ACCEL_LIMIT);
    float ve = CK_NAV_CTRL_SqrtController(nav.target[NAV_E] - nav_est.pos[NAV_E], POS_KP, ACCEL_LIMIT);
    nav.vel_cmd[0] = slew(nav.vel_cmd[0], vn, ACCEL_LIMIT * dt);
    nav.vel_cmd[1] = slew(nav.vel_cmd[1], ve, ACCEL_LIMIT * dt);
}

// Entering a hold while moving fast must not fly back: brake first (target follows the aircraft,
// velocity command slews to 0), then hold where it stopped.
static void brake_or_hold_step(float dt)
{
    if (nav.braking) {
        nav.vel_cmd[0] = slew(nav.vel_cmd[0], 0.0f, ACCEL_LIMIT * dt);
        nav.vel_cmd[1] = slew(nav.vel_cmd[1], 0.0f, ACCEL_LIMIT * dt);
        nav.target[NAV_N] = nav_est.pos[NAV_N];
        nav.target[NAV_E] = nav_est.pos[NAV_E];
        const float sp = sqrtf(nav_est.vel[NAV_N] * nav_est.vel[NAV_N] + nav_est.vel[NAV_E] * nav_est.vel[NAV_E]);
        const float sc = sqrtf(nav.vel_cmd[0] * nav.vel_cmd[0] + nav.vel_cmd[1] * nav.vel_cmd[1]);
        nav.brake_time += dt;
        // stopped, or at most 4 s (wind / gusts): hold that point
        if ((sp < 0.7f && sc < 0.3f) || nav.brake_time > 4.0f) {
            nav.braking = false;
            nav.target[NAV_N] = nav_est.pos[NAV_N];
            nav.target[NAV_E] = nav_est.pos[NAV_E];
        }
        return;
    }
    hold_position_step(dt);
}

// Follow the line from wp_start to the target. Returns the remaining distance along the line.
static float track_line_step(float speed, float v_end, float dt)
{
    const float pn = nav_est.pos[NAV_N], pe = nav_est.pos[NAV_E];
    const float tn = nav.target[NAV_N], te = nav.target[NAV_E];
    float ln = tn - nav.wp_start[0], le = te - nav.wp_start[1];
    const float len = sqrtf(ln * ln + le * le);

    float along = distance_to(tn, te);
    float cross = 0.0f, un = 0.0f, ue = 0.0f;

    if (len > 0.5f) {
        un = ln / len; ue = le / len;
        along = (tn - pn) * un + (te - pe) * ue;
        cross = (pn - nav.wp_start[0]) * (-ue) + (pe - nav.wp_start[1]) * un;
    } else if (along > 0.1f) {
        un = (tn - pn) / along; ue = (te - pe) / along;
    }

    // Speed from the stopping distance; turns back if the target was passed
    float v = sqrtf(2.0f * ACCEL_LIMIT * 0.6f * ABS(along) + ((along > 0.0f) ? v_end * v_end : 0.0f));
    v = constrainf(v, 0.0f, speed);
    if (along < 0.0f) {
        v = -v;
    }

    const float v_cross = constrainf(-cross * XTRACK_KP, -speed, speed);

    const float vn = un * v + (-ue) * v_cross;
    const float ve = ue * v + un * v_cross;

    nav.vel_cmd[0] = slew(nav.vel_cmd[0], vn, ACCEL_LIMIT * dt);
    nav.vel_cmd[1] = slew(nav.vel_cmd[1], ve, ACCEL_LIMIT * dt);

    return along;
}

static void face_target(float n, float e)
{
    if (nav_est.heading_valid && distance_to(n, e) > YAW_FACE_DIST) {
        nav.yaw_target = bearing_to(n, e);
    }
}

static void yaw_step(void)
{
    const float err = wrap180(nav.yaw_target - nav_est.heading_deg);
    nav.yaw_rate = constrainf(err * YAW_KP, -YAW_RATE_MAX, YAW_RATE_MAX);
    nav.control_yaw = true;
}

/* ---------------------------------------------------------- states */

static void run_alt_hold(void)
{
    const float dev = (float)getRCDataRaw(THROTTLE) - nav.thr_center;
    nav.control_z = true;

    // Throttle stick away from its centre: climb / descend rate, the new altitude is held on release
    if (ABS(dev) > THR_DEADBAND) {
        const float span = 500.0f - THR_DEADBAND;
        const float cmd = (dev > 0 ? dev - THR_DEADBAND : dev + THR_DEADBAND) / span;
        nav.z_vel_cmd = constrainf(cmd, -1.0f, 1.0f) * (cmd > 0 ? CLIMB_RATE : DESCENT_RATE);
        nav.target[NAV_U] = nav_est.pos[NAV_U];
    }
}

static void run_pos_hold(float dt)
{
    run_alt_hold();
    nav.control_xy = true;

    const float rs = (float)(getRCDataRaw(ROLL) - 1500);
    const float ps = (float)(getRCDataRaw(PITCH) - 1500);

    if (ABS(rs) > STICK_DEADBAND || ABS(ps) > STICK_DEADBAND) {
        // Sticks move the hold point: velocity in the pilot's frame
        const float h = nav_est.heading_deg * DEG2RAD;
        const float fwd   = applyDeadband((int)ps, STICK_DEADBAND) / 450.0f * STICK_SPEED;
        const float right = applyDeadband((int)rs, STICK_DEADBAND) / 450.0f * STICK_SPEED;
        const float vn = fwd * cosf(h) - right * sinf(h);
        const float ve = fwd * sinf(h) + right * cosf(h);
        nav.vel_cmd[0] = slew(nav.vel_cmd[0], vn, ACCEL_LIMIT * dt);
        nav.vel_cmd[1] = slew(nav.vel_cmd[1], ve, ACCEL_LIMIT * dt);
        nav.target[NAV_N] = nav_est.pos[NAV_N];
        nav.target[NAV_E] = nav_est.pos[NAV_E];
        nav.stick_xy_was_active = true;
    } else {
        if (nav.stick_xy_was_active) {
            nav.stick_xy_was_active = false;
            nav.braking = true;                 // sticks released: stop and stay there (no fly back)
            nav.brake_time = 0.0f;
        }
        brake_or_hold_step(dt);
    }
}

// No magnetometer: fly forward for a while, the GPS course gives the nose direction
static void run_rth_align(float dt)
{
    nav.control_z = true;
    hold_altitude_step(dt);

    nav.out.pitch_deg = ALIGN_TILT_DEG;
    nav.out.roll_deg = 0.0f;
    nav.yaw_rate = 0.0f;
    nav.control_yaw = true;

    if (CK_NAV_ESTIMATOR_AlignHeading(dt)) {
        start_rth();
    } else if (nav.state_time > ALIGN_TIMEOUT) {
        enter(NAV_LAND);
    }
}

static void run_rth_climb(float dt)
{
    nav.control_z = true;
    nav.control_xy = true;

    hold_altitude_step(dt);
    hold_position_step(dt);
    yaw_step();

    if (nav_est.pos[NAV_U] > nav.target[NAV_U] - WP_ALT_TOLERANCE) {
        const float alt = nav.target[NAV_U];
        nav.wp_start[0] = nav_est.pos[NAV_N];
        nav.wp_start[1] = nav_est.pos[NAV_E];
        nav.target[NAV_N] = nav.home[0];
        nav.target[NAV_E] = nav.home[1];
        enter(NAV_RTH_GO);
        nav.target[NAV_U] = alt;
    }
}

static void run_rth_go(float dt)
{
    nav.control_z = true;
    nav.control_xy = true;

    hold_altitude_step(dt);
    track_line_step(RTH_SPEED, 0.0f, dt);
    face_target(nav.target[NAV_N], nav.target[NAV_E]);
    yaw_step();
    CK_NAV_ESTIMATOR_RefineHeading(dt);

    if (distance_to(nav.target[NAV_N], nav.target[NAV_E]) < WP_RADIUS) {
        enter(NAV_RTH_ARRIVE);
    }
}

static void run_rth_arrive(float dt)
{
    nav.control_z = true;
    nav.control_xy = true;

    hold_altitude_step(dt);
    hold_position_step(dt);
    yaw_step();

    if (nav.state_time > 1.5f) {
        enter(NAV_LAND);
    }
}

static void run_land(float dt)
{
    const float alt = nav_est.pos[NAV_U];
    const float rate = (alt > LAND_SLOW_ALT) ? LAND_RATE_HIGH : LAND_RATE_LOW;

    nav.control_z = true;
    nav.z_vel_cmd = slew(nav.z_vel_cmd, -rate, ALT_ACCEL_LIMIT * dt);

    if (nav_est.xy_valid) {
        nav.control_xy = true;
        brake_or_hold_step(dt);                 // land where it stopped
    } else {
        // No GPS: land level, only with the wind tilt learnt while GPS was available
        CK_NAV_CTRL_TiltFromAccel(nav.wind_accel[0], nav.wind_accel[1], TILT_LOW_DEG, &nav.out);
    }
    yaw_step();

    // Touchdown: descent is commanded but the aircraft does not descend and the controller pulled
    // the throttle below hover
    const bool stopped = alt < LAND_SLOW_ALT
                      && nav_est.vel[NAV_U] > -LANDED_VEL
                      && ABS(nav_est.acc[NAV_U]) < LANDED_ACC
                      && nav.out.throttle < CK_NAV_CTRL_HoverThrottle() * LANDED_THROTTLE;
    nav.landed_timer = stopped ? nav.landed_timer + dt : 0.0f;

    if (nav.landed_timer > LANDED_TIME || (!nav_est.z_valid && nav.state_time > BLIND_LAND_TIME)) {
        enter(NAV_DONE);
    }
}

/*
 * GPS lost during return to home: heading and altitude are kept, the aircraft flies towards home
 * (bearing at the moment of the loss) with a fixed tilt for a limited time, then it lands.
 */
static void run_gps_lost(float dt)
{
    nav.control_z = true;
    hold_altitude_step(dt);

    if (nav.state_time <= dt) {
        nav.dr_home_dist = nav.home_set ? distance_to(nav.home[0], nav.home[1]) : 0.0f;
        nav.dr_bearing   = nav.home_set ? bearing_to(nav.home[0], nav.home[1]) : nav_est.heading_deg;
    }

    nav.yaw_target = nav.dr_bearing;
    yaw_step();

    // Last learnt wind tilt + push towards home
    float an = nav.wind_accel[0], ae = nav.wind_accel[1];
    if (nav.dr_home_dist > HOME_RADIUS) {
        const float b = nav.dr_bearing * DEG2RAD;
        const float push = 9.80665f * tanf(GPS_LOST_TILT_DEG * DEG2RAD);
        an += cosf(b) * push;
        ae += sinf(b) * push;
    }
    CK_NAV_CTRL_TiltFromAccel(an, ae, GPS_LOST_MAX_TILT, &nav.out);

    if (nav.state_time > MIN(GPS_LOST_MAX_S, nav.dr_home_dist / GPS_LOST_SPEED)) {
        enter(NAV_LAND);
    }
}

static void update_gps_loss(void)
{
    const bool needs_xy = nav.state == NAV_RTH_CLIMB || nav.state == NAV_RTH_GO
                       || nav.state == NAV_RTH_ARRIVE || nav.state == NAV_RTH_ALIGN;

    if (needs_xy && !nav_est.xy_valid) {
        nav.resume = nav.state;
        enter(NAV_GPS_LOST);
    } else if (nav.state == NAV_POS_HOLD && !nav_est.xy_valid) {
        enter(NAV_ALT_HOLD);
    } else if (nav.state == NAV_GPS_LOST && nav_est.xy_valid) {
        // GPS is back: continue the return to home
        CK_NAV_CTRL_ResetHorizontal();
        set_position_target_here();
        start_rth();
    }
}

// While slow the integral only carries the wind (in cruise it also holds drag, not learnt)
static void learn_wind(float dt)
{
    const float speed = sqrtf(nav_est.vel[NAV_N] * nav_est.vel[NAV_N] + nav_est.vel[NAV_E] * nav_est.vel[NAV_E]);
    if (speed > WIND_LEARN_MAX_SPEED) {
        return;
    }
    float wind[2];
    CK_NAV_CTRL_WindAccel(wind);
    const float k = dt / WIND_LEARN_TAU;
    nav.wind_accel[0] += (wind[0] - nav.wind_accel[0]) * k;
    nav.wind_accel[1] += (wind[1] - nav.wind_accel[1]) * k;
}

static void run_state(float dt)
{
    nav.state_time += dt;
    nav.control_xy = nav.control_z = nav.control_yaw = false;

    switch (nav.state) {
    case NAV_ALT_HOLD:   run_alt_hold(); break;
    case NAV_POS_HOLD:   run_pos_hold(dt); break;
    case NAV_RTH_ALIGN:  run_rth_align(dt); break;
    case NAV_RTH_CLIMB:  run_rth_climb(dt); break;
    case NAV_RTH_GO:     run_rth_go(dt); break;
    case NAV_RTH_ARRIVE: run_rth_arrive(dt); break;
    case NAV_LAND:       run_land(dt); break;
    case NAV_GPS_LOST:   run_gps_lost(dt); break;
    default: break;
    }

    if (nav.control_z) {
        if (nav.state == NAV_ALT_HOLD || nav.state == NAV_POS_HOLD) {
            if (ABS(getRCDataRaw(THROTTLE) - nav.thr_center) <= THR_DEADBAND) {
                hold_altitude_step(dt);
            }
        }
        // No hover learning close to the ground or while landing: ground contact spoils it
        const bool learn = nav.state != NAV_LAND && nav_est.pos[NAV_U] > AIRBORNE_ALT * 2.0f;
        if (nav_est.z_valid) {
            nav.out.throttle = CK_NAV_CTRL_Vertical(nav.z_vel_cmd, learn, dt);
        } else {
            nav.out.throttle = nav.pilot_hover * BLIND_LAND_THROTTLE;   // blind landing
        }
    }
    if (nav.control_xy) {
        // Reduced tilt in the last metre: do not touch the ground sideways
        const bool near_ground = nav.state == NAV_LAND && nav_est.pos[NAV_U] < TILT_LOW_ALT;
        CK_NAV_CTRL_Horizontal(nav.vel_cmd, near_ground ? TILT_LOW_DEG : MAX_TILT_DEG, dt, &nav.out);
        learn_wind(dt);
    }
}

/* ---------------------------------------------------------- arm / disarm */

static void update_arming(void)
{
    if (flags.ARMED && !nav.was_armed) {
        CK_NAV_ESTIMATOR_ZeroAltitude();
        nav.airborne = false;
        nav.throttle_up_time = 0.0f;
        nav.pilot_hover = 0.0f;
        nav.home_set = nav_est.xy_valid;
        if (nav.home_set) {
            nav.home[0] = nav_est.pos[NAV_N];
            nav.home[1] = nav_est.pos[NAV_E];
        }
        CK_NAV_CTRL_ResetVertical(0.0f);
        // A switch already on at arming is not a request: it must be changed first
        nav.last_request = pilot_request();
        nav.pilot_override = false;
        nav.state = NAV_IDLE;
    }

    if (!flags.ARMED && nav.was_armed) {
        nav.state = NAV_IDLE;
        nav.airborne = false;
        nav.pilot_override = false;
    }

    nav.was_armed = flags.ARMED;

    // Airborne: throttle up for a while (also without an altitude estimate)
    if (flags.ARMED && !nav.airborne) {
        const float thr = (nav.state != NAV_IDLE) ? nav.out.throttle : (getRCCommand(THROTTLE) - 1000.0f) * 0.001f;
        nav.throttle_up_time = (thr > AIRBORNE_THROTTLE) ? nav.throttle_up_time + NAV_PERIOD_US * 1e-6f : 0.0f;
        if (nav.throttle_up_time > AIRBORNE_TIME || (thr > AIRBORNE_THROTTLE && nav_est.z_valid && nav_est.pos[NAV_U] > AIRBORNE_ALT)) {
            nav.airborne = true;
        }
    }

    // Pilot hover throttle: used for a blind landing without an altitude estimate
    if (flags.ARMED && nav.state == NAV_IDLE && nav.airborne) {
        const float thr = (getRCCommand(THROTTLE) - 1000.0f) * 0.001f;
        nav.pilot_hover += (thr - nav.pilot_hover) * 0.01f;
    }
}

/* ---------------------------------------------------------- interface */

void CK_NAVIGATION_Init(void)
{
    memset(&nav, 0, sizeof(nav));
    nav.state = NAV_IDLE;
    CK_NAV_ESTIMATOR_Init();
}

void CK_NAVIGATION_Update(void)
{
    static uint32_t last_us = 0;

    CK_NAV_ESTIMATOR_AccumulateAcc();

    const uint32_t now_us = CK_TIME_GetMicroSec();
    if ((now_us - last_us) < NAV_PERIOD_US) {
        return;
    }
    const float dt = constrainf((now_us - last_us) * 1e-6f, 0.001f, 0.05f);
    last_us = now_us;

    #if defined(DEBUG_TIMING)
    navigation_debug.start_time = now_us;
    #endif

    CK_NAV_ESTIMATOR_Update(dt, now_us / 1000u);

    update_arming();

    if (flags.ARMED) {
        update_mode_selection();
        update_gps_loss();
        run_state(dt);
    }

    if (nav.state == NAV_DONE) {
        // Landed by the switched landing / return to home: disarm, the arm switch must be cycled
        flags.ARMED = false;
        overwrite_flags.FORCE_TO_DISARM = true;
        nav.state = NAV_IDLE;
    }

    // Autonomous flight runs in angle mode. The receiver refreshes the flags with every frame,
    // so the request is kept in the overwrite flag as well.
    const bool autonomous = flags.ARMED && nav.state != NAV_IDLE;
    overwrite_flags.FORCE_TO_ANGLE_MODE = autonomous;
    if (autonomous) {
        flags.ANGLE_MODE = true;
        flags.ACRO_MODE = false;
        flags.HORIZON_MODE = false;
    }

    nav.output_fresh = true;

    #if defined(DEBUG_TIMING)
    navigation_debug.update_time = CK_TIME_GetMicroSec() - navigation_debug.start_time;
    #endif
}

bool CK_NAVIGATION_RcTick(bool rx_new)
{
    const bool fresh = nav.output_fresh && flags.ARMED && nav.state != NAV_IDLE;
    nav.output_fresh = false;
    return rx_new || fresh;
}

void CK_NAVIGATION_ApplyRcCommand(float *rc_command)
{
    if (!flags.ARMED || nav.state == NAV_IDLE || !nav.control_z) {
        return;
    }
    rc_command[THROTTLE] = 1000.0f + 1000.0f * nav.out.throttle;
}

bool CK_NAVIGATION_GetSetpoint(int axis, float *setpoint)
{
    if (!flags.ARMED || nav.state == NAV_IDLE || nav.state == NAV_ALT_HOLD) {
        return false;
    }

    if (axis == FD_YAW) {
        if (!nav.control_yaw) {
            return false;
        }
        // Positive yaw_rate = heading increases (nose right). Same sign rule as the yaw stick.
        *setpoint = nav.yaw_rate * (rc_config.yaw_control_reversed ? 1.0f : -1.0f);
        return true;
    }

    if (nav.state == NAV_POS_HOLD || nav.control_xy || nav.state == NAV_GPS_LOST || nav.state == NAV_RTH_ALIGN || nav.state == NAV_LAND) {
        const float angle = (axis == FD_ROLL) ? nav.out.roll_deg : nav.out.pitch_deg;
        *setpoint = angle / (float)pidProfile.angle_limit * getMaxRcRate(axis);
        return true;
    }
    return false;
}

nav_state_e CK_NAVIGATION_GetState(void) { return nav.state; }

void CK_NAVIGATION_GetTarget(float target[3])
{
    target[0] = nav.target[0]; target[1] = nav.target[1]; target[2] = nav.target[2];
}

void CK_NAVIGATION_GetHome(float home[2], bool *is_set)
{
    home[0] = nav.home[0]; home[1] = nav.home[1]; *is_set = nav.home_set;
}

const char *CK_NAVIGATION_StateName(nav_state_e state)
{
    static const char *const names[NAV_STATE_COUNT] = {
        "PILOT", "ALT HOLD", "POS HOLD", "RTH ALIGN", "RTH CLIMB", "RTH GO", "RTH ARRIVE",
        "LAND", "GPS LOST", "DONE"
    };
    return (state < NAV_STATE_COUNT) ? names[state] : "?";
}

#else

DEBUG_TIME_t navigation_debug;

#endif /* NAVIGATION_ */
