/*
 * CK_NAV_ESTIMATOR - position / velocity / altitude estimate
 *
 * The accelerometer prediction is corrected by GPS (position + velocity) and the barometer:
 * a third order complementary filter with delay compensation.
 * Method: ArduPilot AP_InertialNav and INAV navigation_pos_estimator.
 *
 * Axes: N north, E east, U up (m, m/s). Position is relative to the first valid GPS fix;
 * the U axis is zeroed when the aircraft is armed.
 */
#ifndef CK_NAV_ESTIMATOR_H_
#define CK_NAV_ESTIMATOR_H_

#include "CK_DEFINITIONS.h"

#define NAV_N 0
#define NAV_E 1
#define NAV_U 2

typedef struct {
    float pos[3];
    float vel[3];
    float acc[3];           // earth frame acceleration, m/s2, bias removed, without gravity
    float heading_deg;      // 0..360, clockwise positive
    bool  xy_valid;         // GPS healthy, horizontal estimate can be trusted
    bool  z_valid;          // vertical estimate can be trusted
    bool  heading_valid;    // absolute heading is known (magnetometer or GPS course)
    bool  has_mag;
    bool  has_baro;
    uint32_t gps_age_ms;    // time since the last valid GPS sample
} nav_estimate_t;

extern nav_estimate_t nav_est;

#if NAVIGATION_

void CK_NAV_ESTIMATOR_Init(void);
void CK_NAV_ESTIMATOR_AccumulateAcc(void);                  // every flight loop
void CK_NAV_ESTIMATOR_Update(float dt, uint32_t now_ms);    // 100 Hz

// No magnetometer: find the heading from the GPS course while flying nose forward. True when aligned.
bool CK_NAV_ESTIMATOR_AlignHeading(float dt);
void CK_NAV_ESTIMATOR_RefineHeading(float dt);              // slow correction in straight flight

// For the IMU tilt correction: GPS measured horizontal acceleration in body axes, accelerometer units
void CK_NAV_ESTIMATOR_GetAccelCompensation(float out[3]);
void CK_NAV_ESTIMATOR_ZeroAltitude(void);                   // at ARM
void CK_NAV_ESTIMATOR_LatLonToLocal(int32_t lat, int32_t lon, float *north, float *east);

#else

// Navigation off: no IMU correction
static inline void CK_NAV_ESTIMATOR_GetAccelCompensation(float out[3])
{
    out[0] = out[1] = out[2] = 0.0f;
}

#endif /* NAVIGATION_ */

#endif /* CK_NAV_ESTIMATOR_H_ */
