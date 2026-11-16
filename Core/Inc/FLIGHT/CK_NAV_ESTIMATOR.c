#include "CK_DEFINITIONS.h"
#include "FLIGHT/CK_NAV_ESTIMATOR.h"

/* Read by the printer / OSD also when navigation is off (stays zero) */
nav_estimate_t nav_est;

#if NAVIGATION_

#include <COMMON/maths.h>
#include "COMMON/axis.h"
#include "COMMON/vector.h"
#include "DRIVERS/CK_TIME_HAL.h"

#include "MOTION/CK_ACC.h"
#include "MOTION/CK_IMU.h"
#include "MOTION/CK_BAROMETER.h"
#include "MOTION/CK_MAGNETO.h"

#include "FLIGHT/CK_GPS.h"
#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_NAV_ESTIMATOR.h"

#define GRAVITY             9.80665f
#define DEG2RAD             (M_PIf / 180.0f)
#define M_PER_DEG_E7        0.011131949f    /* 1e-7 degree -> metre */

#define HISTORY_LEN         64              /* 100 Hz: 0.64 s */
#define GPS_DELAY_STEPS     12              /* NAV-PVT delay ~120 ms */
#define BARO_DELAY_STEPS    22              /* CK_BAROMETER averaging ~220 ms */
#define BARO_PERIOD_S       0.02f

/* Correction gains (1/s) */
#define GPS_POS_TO_POS      0.8f
#define GPS_POS_TO_VEL      0.25f
#define GPS_VEL_TO_VEL      1.6f
#define GPS_VEL_TO_BIAS     0.08f
#define BARO_POS_TO_POS     1.6f
#define BARO_POS_TO_VEL     1.0f
#define BARO_POS_TO_BIAS    0.12f
#define GPS_ALT_TO_POS      0.5f
#define GPS_ALT_TO_VEL      0.3f

#define BIAS_LIMIT          1.5f            /* m/s2 */
#define GPS_MIN_SATS        6
#define GPS_MAX_HACC_MM     5000u
#define GPS_TIMEOUT_MS      600u
#define GPS_JUMP_GATE_M     30.0f           /* a single-sample jump larger than this re-locks the filter */

#define GPS_ACCEL_LPF_S     0.25f
#define GPS_ACCEL_LIMIT     12.0f

#define COG_MIN_SPEED       2.0f            /* m/s: GPS course is meaningful above this speed */
#define COG_ALIGN_TIME      1.5f            /* s */
#define COG_REFINE_GAIN     0.3f            /* 1/s */
#define COG_REFINE_MAX_ERR  45.0f


extern UBX_NAV_PVT_s pvt;

static float acc_sum[3];
static int   acc_count;
static float bias_body[3];

static float pos_hist[HISTORY_LEN][3];
static float vel_hist[HISTORY_LEN][3];
static int   hist_head;

static bool     origin_set;
static int32_t  origin_lat, origin_lon;
static float    lon_scale;
static float    gps_alt_zero;

static float    alt_zero;           /* baro altitude at ARM */
static uint32_t last_itow;
static uint32_t last_gps_ms;
static float    baro_timer;
static float    cog_timer;
static float    heading_offset;     /* IMU heading -> true heading, degrees (no magnetometer) */

static float    gps_accel[3];       /* earth acceleration derived from GPS (U unused) */
static float    gps_vel_prev[2];

/* ---------------------------------------------------------- helpers */

/* Without a magnetometer the IMU heading is arbitrary: horizontal axes are rotated by the angle found from GPS */
static void rotate_heading(float *north, float *east, float angle_deg)
{
    const float a = angle_deg * DEG2RAD, c = cosf(a), s = sinf(a);
    const float n = *north, e = *east;
    *north = n * c - e * s;
    *east  = n * s + e * c;
}

static void body_to_earth(const float b[3], float e[3])
{
    /* rMat turns body axes into north-west-up */
    float north = rMat.m[0][0] * b[0] + rMat.m[0][1] * b[1] + rMat.m[0][2] * b[2];
    float west  = rMat.m[1][0] * b[0] + rMat.m[1][1] * b[1] + rMat.m[1][2] * b[2];
    float up    = rMat.m[2][0] * b[0] + rMat.m[2][1] * b[1] + rMat.m[2][2] * b[2];
    e[NAV_N] = north;
    e[NAV_E] = -west;
    e[NAV_U] = up;
    rotate_heading(&e[NAV_N], &e[NAV_E], heading_offset);
}

static void earth_to_body(const float e[3], float b[3])
{
    float rn = e[NAV_N], re = e[NAV_E];
    rotate_heading(&rn, &re, -heading_offset);
    float n = rn, w = -re, u = e[NAV_U];
    b[0] = rMat.m[0][0] * n + rMat.m[1][0] * w + rMat.m[2][0] * u;
    b[1] = rMat.m[0][1] * n + rMat.m[1][1] * w + rMat.m[2][1] * u;
    b[2] = rMat.m[0][2] * n + rMat.m[1][2] * w + rMat.m[2][2] * u;
}

/* Update the body axis accelerometer bias from an earth frame velocity error */
static void correct_bias(const float err_earth[3], float dt)
{
    float d[3];
    earth_to_body(err_earth, d);
    for (int i = 0; i < 3; i++) {
        bias_body[i] = constrainf(bias_body[i] - d[i] * dt, -BIAS_LIMIT, BIAS_LIMIT);
    }
}

static const float *history(float table[][3], int steps_back)
{
    return table[(hist_head - 1 - steps_back + HISTORY_LEN * 2) % HISTORY_LEN];
}

static float wrap360(float a)
{
    while (a < 0.0f)    a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

static float wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* ---------------------------------------------------------- prediction */

static void predict(float dt)
{
    if (acc_count == 0 || acc.acc1G == 0) {
        return;
    }

    float a_body[3];
    for (int i = 0; i < 3; i++) {
        a_body[i] = acc_sum[i] / acc_count * GRAVITY / acc.acc1G - bias_body[i];
        acc_sum[i] = 0.0f;
    }
    acc_count = 0;

    body_to_earth(a_body, nav_est.acc);
    nav_est.acc[NAV_U] -= GRAVITY;

    for (int i = 0; i < 3; i++) {
        nav_est.pos[i] += nav_est.vel[i] * dt + 0.5f * nav_est.acc[i] * dt * dt;
        nav_est.vel[i] += nav_est.acc[i] * dt;
        pos_hist[hist_head][i] = nav_est.pos[i];
        vel_hist[hist_head][i] = nav_est.vel[i];
    }
    hist_head = (hist_head + 1) % HISTORY_LEN;
}

/* ---------------------------------------------------------- barometer */

static void correct_baro(float dt)
{
#if USE_BARO_
    if (!barometer.isBarometerReady || barometer.pressure < 30000 || barometer.pressure > 120000) {   /* not connected: do not use */
        return;
    }
    baro_timer += dt;
    if (baro_timer < BARO_PERIOD_S) {
        return;
    }
    const float period = baro_timer;
    baro_timer = 0.0f;

    const float baro_m = (float)CK_BAROMETER_CalculateAltitude() * 0.01f - alt_zero;
    const float err = baro_m - history(pos_hist, BARO_DELAY_STEPS)[NAV_U];

    nav_est.pos[NAV_U] += err * BARO_POS_TO_POS * period;
    nav_est.vel[NAV_U] += err * BARO_POS_TO_VEL * period;

    const float e[3] = {0.0f, 0.0f, err * BARO_POS_TO_BIAS};
    correct_bias(e, period);

    nav_est.z_valid = true;
    nav_est.has_baro = true;
#else
    UNUSED(dt);
#endif
}

/* ---------------------------------------------------------- GPS */

#if GPS_
static bool gps_sample_ok(void)
{
    return pvt.nav_pvt_fixType >= 3
        && pvt.nav_pvt_numSv >= GPS_MIN_SATS
        && pvt.nav_pvt_hAcc <= GPS_MAX_HACC_MM;
}

static void reset_xy_to_gps(float n, float e, float vn, float ve)
{
    nav_est.pos[NAV_N] = n;
    nav_est.pos[NAV_E] = e;
    nav_est.vel[NAV_N] = vn;
    nav_est.vel[NAV_E] = ve;
    for (int i = 0; i < HISTORY_LEN; i++) {
        pos_hist[i][NAV_N] = n;
        pos_hist[i][NAV_E] = e;
        vel_hist[i][NAV_N] = vn;
        vel_hist[i][NAV_E] = ve;
    }
}

static void correct_gps(uint32_t now_ms)
{
    if (pvt.nav_pvt_iTOW == last_itow) {
        return;
    }
    const float dt = (last_gps_ms != 0) ? constrainf((now_ms - last_gps_ms) * 0.001f, 0.02f, 0.5f) : 0.1f;
    last_itow = pvt.nav_pvt_iTOW;

    if (!gps_sample_ok()) {
        return;
    }

    if (!origin_set) {
        origin_lat = pvt.nav_pvt_lat;
        origin_lon = pvt.nav_pvt_lon;
        lon_scale  = cosf((float)origin_lat * 1e-7f * DEG2RAD);
        origin_set = true;
    }

    const float n  = (float)(pvt.nav_pvt_lat - origin_lat) * M_PER_DEG_E7;
    const float e  = (float)(pvt.nav_pvt_lon - origin_lon) * M_PER_DEG_E7 * lon_scale;
    const float vn = (float)pvt.nav_pvt_ve1N * 0.001f;
    const float ve = (float)pvt.nav_pvt_ve1E * 0.001f;
    const float vu = -(float)pvt.nav_pvt_ve1D * 0.001f;

    if (!nav_est.xy_valid) {
        reset_xy_to_gps(n, e, vn, ve);
    } else {
        /* Real horizontal acceleration from the GPS velocity difference: corrects the IMU assumption "acceleration = gravity" */
        const float k = dt / (dt + GPS_ACCEL_LPF_S);
        gps_accel[NAV_N] += (constrainf((vn - gps_vel_prev[0]) / dt, -GPS_ACCEL_LIMIT, GPS_ACCEL_LIMIT) - gps_accel[NAV_N]) * k;
        gps_accel[NAV_E] += (constrainf((ve - gps_vel_prev[1]) / dt, -GPS_ACCEL_LIMIT, GPS_ACCEL_LIMIT) - gps_accel[NAV_E]) * k;
    }
    gps_vel_prev[0] = vn;
    gps_vel_prev[1] = ve;

    const float *p = history(pos_hist, GPS_DELAY_STEPS);
    const float *v = history(vel_hist, GPS_DELAY_STEPS);

    float ep[2] = { n - p[NAV_N], e - p[NAV_E] };
    float ev[2] = { vn - v[NAV_N], ve - v[NAV_E] };

    if (nav_est.xy_valid && (ABS(ep[0]) > GPS_JUMP_GATE_M || ABS(ep[1]) > GPS_JUMP_GATE_M)) {
        reset_xy_to_gps(n, e, vn, ve);       /* after a long gap: lock the filter to GPS */
        ep[0] = ep[1] = ev[0] = ev[1] = 0.0f;
    }

    for (int i = 0; i < 2; i++) {
        nav_est.pos[i] += ep[i] * GPS_POS_TO_POS * dt;
        nav_est.vel[i] += (ep[i] * GPS_POS_TO_VEL + ev[i] * GPS_VEL_TO_VEL) * dt;
    }

    const float bias_err[3] = { ev[0] * GPS_VEL_TO_BIAS, ev[1] * GPS_VEL_TO_BIAS, 0.0f };
    correct_bias(bias_err, dt);

    if (!nav_est.has_baro) {
        /* No barometer: altitude from GPS, noisier but good enough */
        const float alt = (float)pvt.nav_pvt_hMSL * 0.001f - gps_alt_zero;
        if (gps_alt_zero == 0.0f) {
            gps_alt_zero = (float)pvt.nav_pvt_hMSL * 0.001f;
        } else {
            const float ez = alt - history(pos_hist, GPS_DELAY_STEPS)[NAV_U];
            const float evz = vu - history(vel_hist, GPS_DELAY_STEPS)[NAV_U];
            nav_est.pos[NAV_U] += ez * GPS_ALT_TO_POS * dt;
            nav_est.vel[NAV_U] += (ez * GPS_ALT_TO_VEL + evz * GPS_VEL_TO_VEL) * dt;
            nav_est.z_valid = true;
        }
    }

    last_gps_ms = now_ms;
    nav_est.xy_valid = true;

}
#endif

/* ---------------------------------------------------------- interface */

void CK_NAV_ESTIMATOR_Init(void)
{
    memset(&nav_est, 0, sizeof(nav_est));
    memset(bias_body, 0, sizeof(bias_body));
    acc_count = 0;
    hist_head = 0;
    origin_set = false;

#if USE_MAG_
    nav_est.has_mag = (mag.is_mag_spi_init || mag.is_mag_i2c_init);
    nav_est.heading_valid = nav_est.has_mag;
#endif
}

void CK_NAV_ESTIMATOR_AccumulateAcc(void)
{
    for (int i = 0; i < 3; i++) {
        acc_sum[i] += acc.accADCf[i];
    }
    acc_count++;
}

void CK_NAV_ESTIMATOR_Update(float dt, uint32_t now_ms)
{
    if (!acc.is_acc_init) {   /* no accelerometer: no autonomy */
        return;
    }
    predict(dt);

    nav_est.heading_deg = wrap360((float)attitude.values.yaw * 0.1f + heading_offset);

    correct_baro(dt);

#if GPS_
    correct_gps(now_ms);
#if USE_MAG_
    /* Magnetometer enabled but not connected: take the heading from the GPS course */
    if (nav_est.has_mag && now_ms > 5000U && mag.magADCRaw[X] == 0 && mag.magADCRaw[Y] == 0 && mag.magADCRaw[Z] == 0) {
        nav_est.has_mag = false;
        nav_est.heading_valid = false;
    }
#endif

    nav_est.gps_age_ms = (last_gps_ms != 0) ? (now_ms - last_gps_ms) : 0xFFFFFFFFu;
    if (nav_est.gps_age_ms > GPS_TIMEOUT_MS) {
        nav_est.xy_valid = false;
    }
#else
    UNUSED(now_ms);
#endif
}

void CK_NAV_ESTIMATOR_GetAccelCompensation(float out[3])
{
    if (!nav_est.xy_valid) {
        out[0] = out[1] = out[2] = 0.0f;
        return;
    }
    earth_to_body(gps_accel, out);
    const float scale = (float)acc.acc1G / GRAVITY;
    out[0] *= scale;
    out[1] *= scale;
    out[2] *= scale;
}

static bool course_valid(float *course)
{
#if GPS_
    const float vn = (float)pvt.nav_pvt_ve1N * 0.001f;
    const float ve = (float)pvt.nav_pvt_ve1E * 0.001f;
    if (!nav_est.xy_valid || (vn * vn + ve * ve) < (COG_MIN_SPEED * COG_MIN_SPEED)) {
        return false;
    }
    *course = wrap360(atan2f(ve, vn) / DEG2RAD);
    return true;
#else
    UNUSED(course);
    return false;
#endif
}

bool CK_NAV_ESTIMATOR_AlignHeading(float dt)
{
    float course;
    cog_timer = course_valid(&course) ? cog_timer + dt : 0.0f;

    if (cog_timer > COG_ALIGN_TIME) {
        heading_offset = wrap180(heading_offset + wrap180(course - nav_est.heading_deg));
        nav_est.heading_valid = true;
    }
    return nav_est.heading_valid;
}

void CK_NAV_ESTIMATOR_RefineHeading(float dt)
{
    float course;
    if (!nav_est.has_mag && nav_est.heading_valid && course_valid(&course)) {
        const float err = wrap180(course - nav_est.heading_deg);
        if (ABS(err) < COG_REFINE_MAX_ERR) {
            heading_offset = wrap180(heading_offset + err * COG_REFINE_GAIN * dt);
        }
    }
}

void CK_NAV_ESTIMATOR_ZeroAltitude(void)
{
    alt_zero += nav_est.pos[NAV_U];
    gps_alt_zero += nav_est.pos[NAV_U];
    nav_est.pos[NAV_U] = 0.0f;
    nav_est.vel[NAV_U] = 0.0f;
    for (int i = 0; i < HISTORY_LEN; i++) {
        pos_hist[i][NAV_U] = 0.0f;
        vel_hist[i][NAV_U] = 0.0f;
    }
}

void CK_NAV_ESTIMATOR_LatLonToLocal(int32_t lat, int32_t lon, float *north, float *east)
{
    *north = (float)(lat - origin_lat) * M_PER_DEG_E7;
    *east  = (float)(lon - origin_lon) * M_PER_DEG_E7 * lon_scale;
}

#endif /* NAVIGATION_ */
