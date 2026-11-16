
#ifndef CK_ESC_H_
#define CK_ESC_H_

#include "CK_DEFINITIONS.h"
#include "CK_PWM.h"
#include "CK_DSHOT.h"

typedef enum
{
	PWM_MODE,
	DSHOT_MODE

}CK_ESC_Mode;

extern CK_ESC_Mode esc_mode;

void CK_ESC_Init(CK_ESC_Mode md, targetFreq_e main_time);

void CK_ESC_SetMotor(int num1, int num2, int num3, int num4);

void CK_ESC_StopMotors(void);

// Fault handlers: sends STOP to the motors even when no interrupt can run
void CK_ESC_EmergencyStop(void);

// Sends STOP to the motors, then resets the MCU (every NVIC_SystemReset call goes through this)
void CK_ESC_SystemReset(void);

// DShot output guard tripped (main loop stalled, motors at STOP): the mixer forces DISARM
bool CK_ESC_OutputGuardTripped(void);

// Diagnostics (printer 'x'): guard trips and skipped frames since power-up
void CK_ESC_GetOutputStats(uint32_t *guard_trips, uint32_t *busy_skips);

void CK_ESC_MOTOR_TEST_MODE_Enable(int num);

void CK_ESC_MOTOR_TEST_MODE_Disable(void);

#endif /* CK_ESC_H_ */
