/*
 * CK_NAVIGATION - navigation state machine (light firmware)
 *
 * Capability follows the enabled sensors:
 *   gyro + acc                  : Acro / Angle only
 *   + barometer (or GPS height) : Altitude hold
 *   + GPS, no magnetometer      : Basic return to home (heading from the GPS course, aligned by flying forward)
 *   GPS + barometer + mag       : Position hold as well
 *
 * Switches (CK_RECEIVER):
 *   AUX7  middle: ALT HOLD    high: LAND
 *   AUX8  middle: POS HOLD    high: GPS RESCUE (return to home, then land)
 *
 * Rules:
 *   - Autonomy only on the pilot's switch. No mode is ever entered by the firmware on its own.
 *   - Link loss stays DROP (CK_RECEIVER): the motors stop, the aircraft is not flown home.
 *   - Moving roll or pitch during return to home or landing gives the control back to the pilot
 *     at once; it stays with the pilot until the switch is changed.
 *   - A switched landing or return to home disarms after touchdown (arm switch must be cycled).
 */
#ifndef CK_NAVIGATION_H_
#define CK_NAVIGATION_H_

#include "CK_DEFINITIONS.h"

typedef enum {
    NAV_IDLE = 0,       // the pilot flies
    NAV_ALT_HOLD,
    NAV_POS_HOLD,
    NAV_RTH_ALIGN,      // no magnetometer: align the heading by flying forward
    NAV_RTH_CLIMB,
    NAV_RTH_GO,
    NAV_RTH_ARRIVE,
    NAV_LAND,
    NAV_GPS_LOST,       // dead reckoning towards home
    NAV_DONE,           // landed, disarm
    NAV_STATE_COUNT
} nav_state_e;

typedef enum {
    NAV_CAP_MANUAL = 0,
    NAV_CAP_ALT,
    NAV_CAP_RTH_BASIC,
    NAV_CAP_FULL
} nav_capability_e;

#if NAVIGATION_

void CK_NAVIGATION_Init(void);
void CK_NAVIGATION_Update(void);                    // every flight loop, after the IMU

// CK_RC hooks
bool CK_NAVIGATION_RcTick(bool rx_new);
void CK_NAVIGATION_ApplyRcCommand(float *rc_command);
bool CK_NAVIGATION_GetSetpoint(int axis, float *setpoint);

nav_state_e      CK_NAVIGATION_GetState(void);
nav_capability_e CK_NAVIGATION_GetCapability(void);
const char      *CK_NAVIGATION_StateName(nav_state_e state);
void             CK_NAVIGATION_GetTarget(float target[3]);
void             CK_NAVIGATION_GetHome(float home[2], bool *is_set);

#else

// Navigation compiled out: the calls do nothing, the FC flies as if there was no navigation
static inline void CK_NAVIGATION_Init(void) {}
static inline void CK_NAVIGATION_Update(void) {}
static inline bool CK_NAVIGATION_RcTick(bool rx_new) { return rx_new; }
static inline void CK_NAVIGATION_ApplyRcCommand(float *rc_command) { (void)rc_command; }
static inline bool CK_NAVIGATION_GetSetpoint(int axis, float *setpoint) { (void)axis; (void)setpoint; return false; }
static inline nav_state_e CK_NAVIGATION_GetState(void) { return NAV_IDLE; }
static inline nav_capability_e CK_NAVIGATION_GetCapability(void) { return NAV_CAP_MANUAL; }
static inline const char *CK_NAVIGATION_StateName(nav_state_e state) { (void)state; return "OFF"; }

#endif /* NAVIGATION_ */

#endif /* CK_NAVIGATION_H_ */
