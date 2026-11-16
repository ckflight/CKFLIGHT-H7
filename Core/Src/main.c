/*
 * CKFLIGHT H7 Light - main
 *
 * Bare-metal flight loop for STM32H7 (and STM32F4 targets of the same code base).
 *
 *   main()            : caches, system clock, HAL, then CKFLIGHT_Init() and CKFLIGHT_Loop() forever
 *   CKFLIGHT_Init()   : storage, peripherals, ESC, receiver, sensors, PID and IMU, in dependency order
 *   CKFLIGHT_Loop()   : one control period (8 kHz on H7):
 *                       USB -> RX -> adjustments -> gyro -> acc -> (mag, baro, GPS) -> IMU -> navigation
 *                       -> setpoint -> PID -> mixer + DShot -> OSD -> blackbox -> monitor -> printer
 *
 * Navigation (altitude hold, position hold, GPS rescue, landing) runs only on the pilot's switch.
 * Link loss is DROP: the motors stop, the firmware never starts an autonomous flight by itself.
 *
 * Motor safety does not depend on this loop being healthy:
 *   - DShot frames are sent only by the mixer of this loop (CK_ESC_SetMotor -> CK_DSHOT_Update).
 *   - If the loop stalls for 250 ms the DShot output guard (TIM7) sends STOP and disarms.
 *   - Fault handlers send STOP frames without needing interrupts (CK_ESC_EmergencyStop).
 *
 * Loop diagnostics are measured here and printed with the terminal command 'x'. They only measure,
 * the flight path is unchanged.
 */

#include "CK_DEFINITIONS.h"

#include "COMMUNICATION/CK_MSP.h"
#include "COMMUNICATION/CK_PRINTER.h"
#include "COMMUNICATION/CK_INPUTSTREAM.h"
#include "COMMUNICATION/CK_CONFIGURATION.h"
#include "COMMUNICATION/USBD_CDC/CK_USBD_INTERFACE.h"

#include "DRIVERS/CK_SYSTEM.h"
#include "DRIVERS/CK_TIME_HAL.h"
#include "DRIVERS/CK_LED.h"
#include "DRIVERS/CK_BUZZER.h"
#include "DRIVERS/CK_MICROCARD.h"
#include "DRIVERS/CK_GPIO.h"
#include "DRIVERS/CK_RGB.h"
#include "DRIVERS/CK_SOFTSERIAL.h"

#include "MOTION/CK_GYRO.h"
#include "MOTION/CK_ACC.h"
#include "MOTION/CK_MAGNETO.h"
#include "MOTION/CK_BAROMETER.h"
#include "MOTION/CK_IMU.h"
#include "MOTION/CK_BNO055.h"

#include "FLIGHT/CK_RECEIVER.h"
#include "FLIGHT/CK_RC.h"
#include "FLIGHT/CK_SBUS.h"
#include "FLIGHT/CK_ESC.h"
#include "FLIGHT/CK_PID.h"
#include "FLIGHT/pid_init.h"
#include "FLIGHT/CK_MIXER.h"
#include "FLIGHT/CK_NAVIGATION.h"
#include "FLIGHT/CK_GPS.h"
#include "FLIGHT/CK_LOG.h"
#include "FLIGHT/CK_ADJUSTMENT.h"
#include "FLIGHT/CK_DSHOT.h"
#include "FLIGHT/CK_PERIPHERAL.h"
#include "FLIGHT/CK_SMARTAUDIO.h"
#include "FLIGHT/flight_monitor.h"

#include "COMMON/CK_FILTERS.h"

#include "FLASH/CK_FLASH.h"

#include "OSD/CK_OSD.h"
#include "OSD/CK_MAX7456.h"

/*
 * Build step that writes git_commit_hash.h (STM32CubeIDE):
 * Properties -> C/C++ Build -> Settings -> Build Steps -> Pre-build command:
 *   python3 "${ProjDirPath}/Core/Inc/git_hash.py"
 */

void CKFLIGHT_Init(void);
void CKFLIGHT_Loop(void);


/* ============================================================
 * LOOP TIMING AND DIAGNOSTICS
 * ============================================================ */

uint32_t loopTimer = 0;

uint32_t computeStartTime = 0;
uint32_t computeEndTime = 0;     // work time of the previous loop (OSD, blackbox and printer use it)

/*
 * Busy time of one loop = from the start of the loop until it starts waiting for the next period
 * (interrupts in between included). Read and cleared by the printer ('x').
 */
volatile uint32_t ck_loop_busy_max_us = 0;     // longest loop since the last read
volatile uint32_t ck_loop_overruns = 0;        // loops longer than the target period (since power-up)
volatile uint32_t ck_loop_clock_glitches = 0;  // the microsecond clock went backwards, not counted

/*
 * Section times of the current loop. For a loop that overruns, the slowest section is kept
 * ("slow: <section> <us>" on the 'x' line). About 17 clock reads per loop, nothing else.
 */
enum{
	CK_SEC_INPUT = 0, CK_SEC_RX, CK_SEC_ADJ, CK_SEC_GYRO, CK_SEC_ACC, CK_SEC_MAG, CK_SEC_BARO, CK_SEC_GPS,
	CK_SEC_IMU, CK_SEC_NAV, CK_SEC_PID, CK_SEC_MIX, CK_SEC_OSD, CK_SEC_LOG, CK_SEC_MON, CK_SEC_PRINT,
	CK_SEC_OTHER, CK_SEC_COUNT
};

const uint8_t ck_loop_sec_count = CK_SEC_COUNT;

const char *const ck_loop_sec_names[CK_SEC_COUNT] = {
	"USB", "RX", "ADJ", "GYRO", "ACC", "MAG", "BARO", "GPS",
	"IMU", "NAV", "PID", "MIX", "OSD", "LOG", "MON", "PRINT", "OTHER"
};

volatile uint8_t  ck_loop_slow_sec = 0xFF;     // section of the worst overrun since the last read
volatile uint32_t ck_loop_slow_us  = 0;

static uint32_t ck_sec_us[CK_SEC_COUNT];
static uint32_t ck_sec_t;

static inline void CK_SEC_START(void){

	ck_sec_t = CK_TIME_GetMicroSec();
}

static inline void CK_SEC_MARK(uint8_t id){

	const uint32_t now = CK_TIME_GetMicroSec();
	ck_sec_us[id] = now - ck_sec_t;
	ck_sec_t = now;
}

static inline void CK_SEC_CLEAR(void){

	for(uint8_t i = 0; i < CK_SEC_COUNT; i++){
		ck_sec_us[i] = 0;
	}
}

static inline void CK_LOOP_RecordBusyTime(uint32_t loop_busy_us){

	// A "negative" time (above 2^31 as uint32) is physically impossible: the microsecond clock was read
	// ~1 ms off for a moment. Do not let it spoil max / overrun, count it separately.
	if(loop_busy_us > 0x7FFFFFFFu){
		ck_loop_clock_glitches++;
		return;
	}

	if(loop_busy_us > ck_loop_busy_max_us){
		ck_loop_busy_max_us = loop_busy_us;
	}

	if(loop_busy_us > TARGET_MAIN_TIME_US){

		ck_loop_overruns++;

		uint8_t worst = 0;
		for(uint8_t i = 1; i < CK_SEC_COUNT; i++){
			if(ck_sec_us[i] > ck_sec_us[worst]){
				worst = i;
			}
		}

		if(ck_sec_us[worst] < 0x7FFFFFFFu && ck_sec_us[worst] > ck_loop_slow_us){
			ck_loop_slow_us  = ck_sec_us[worst];
			ck_loop_slow_sec = worst;
		}
	}
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void){

#if USE_H7 == 1

	SCB_EnableICache();
	SCB_EnableDCache();

	CK_SYSTEM_SetSystemClock(SYSTEM_CLK_480MHz);  // always first

#elif USE_F4 == 1

	CK_SYSTEM_SetSystemClock(SYSTEM_CLK_168MHz);  // always first

#endif

	HAL_Init();

	CKFLIGHT_Init();

	while(1){
		CKFLIGHT_Loop();
	}
}


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

void CKFLIGHT_Init(void){

	// Parameters live in internal flash, even when an external flash is available
	CK_FLASH_Init_Internal(TARGET_FLASH);

	CK_PERIPHERAL_Init(TARGET_PERIPHERAL_TIME_US);

	CK_CONFIGURATION_Init(); // mass erase the flash to load new default parameters

	CK_USBD_Init();

#if BUZZER_PWM
	CK_BUZZER_Init(BUZZER_GPIO, BUZZER_GPIO_PIN, BUZZER_MODE_PWM);
#endif

#if BUZZER_DC
	CK_BUZZER_Init(BUZZER_GPIO, BUZZER_GPIO_PIN, BUZZER_MODE_DC);
#endif

	// ESCs get STOP frames from here on (start-up frames, then the output guard keep-alive)
	CK_ESC_Init(DSHOT_MODE, TARGET_DSHOT_TIME_US);

#if SMART_AUDIO_
	CK_SMARTAUDIO_Init(RACEBAND, CH1, mW_1000, TBS_UNIFY_PRO32);
#endif

#if LED1_
	CK_LED_Init1(LED1_GPIO, LED1_GPIO_PIN);
#endif

#if LED2_
	CK_LED_Init2(LED2_GPIO, LED2_GPIO_PIN);
#endif

#if LOG_SPI_ || LOG_SDIO_ || LOG_FLASH_
	CK_LOG_Init(TARGET_MAIN_TIME_US, TARGET_LOG_TIME_US);

	#if LOG_SDIO_ == 1
	CK_MICROCARD_Init(SDIO_DMA_INTERRUPT_MULTIBLOCK);
	#endif

	#if LOG_SPI_ == 1
		#if LOG_MULTIWRITE_ == 1
	CK_MICROCARD_Init(SPI_DMA_INTERRUPT_MULTIBLOCK);
		#else
	CK_MICROCARD_Init(SPI_DMA_INTERRUPT_SINGLEBLOCK);
		#endif
	#endif
#endif

	CK_RC_Init(TARGET_MAIN_TIME_US);

#if RGB_
	CK_RGB_Init(RGB_TIM, RGB_DMA, RGB_DMA_Stream, COLOR_BLUE2);
#endif

	CK_RECEIVER_Init(RX_CRSF);

	CK_MIXER_Init();

	CK_ADJUSTMENT_Init(TARGET_2HZ_US, TARGET_MAIN_TIME_US);

	// Start-up arming sequence (arm, disarm, arm with throttle low). Buzzer tone 1.
	CK_RECEIVER_WaitARM();

#if GPS_
	CK_GPS_Init(GPS_UART, GPS_MODULE);
#endif

#if GYRO1_SPI_
	CK_GYRO_Init(GYRO1_SPI, GYRO1_CS_PORT, GYRO1_CS_PIN, TARGET_GYRO1, DPS2000, TARGET_GYRO_TIME_US, TARGET_MAIN_TIME_US);

	while(!gyro.is_gyro_init){
		CK_LED_ToggleLedForMs(1, 10, 100);
		CK_LED_ToggleLedForMs(2, 10, 100);
	}
#endif

#if ACC1_SPI_
	CK_ACC_Init(ACC1_SPI, ACC1_CS_PORT, ACC1_CS_PIN, TARGET_ACC1, G16, TARGET_ACC_TIME_US, TARGET_MAIN_TIME_US);
#endif

#if GYRO2_SPI_
	CK_GYRO_Init(GYRO2_SPI, GYRO2_CS_PORT, GYRO2_CS_PIN, TARGET_GYRO2, DPS2000, TARGET_GYRO_TIME_US, TARGET_MAIN_TIME_US);
#endif

#if ACC2_SPI_
	CK_ACC_Init(ACC2_SPI, ACC2_CS_PORT, ACC2_CS_PIN, TARGET_ACC2, G16, TARGET_ACC_TIME_US, TARGET_MAIN_TIME_US);
#endif

#if EXT_SPI_
	#if EXT_CS1_
	CK_GYRO_Init(EXT_SPI, EXT_CS1_PORT, EXT_CS1_PIN, ICM20602_GYRO, DPS2000, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
	CK_ACC_Init(EXT_SPI, EXT_CS1_PORT, EXT_CS1_PIN, ICM20602_ACC, G16, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
	#endif

	#if EXT_CS2_
	CK_GYRO_Init(EXT_SPI, EXT_CS2_PORT, EXT_CS2_PIN, ICM20602_GYRO, DPS2000, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
	CK_ACC_Init(EXT_SPI, EXT_CS2_PORT, EXT_CS2_PIN, ICM20602_ACC, G16, TARGET_MAIN_TIME_US, TARGET_MAIN_TIME_US);
	#endif

	while(!gyro.is_gyro_init){
		CK_LED_ToggleLedForMs(1, 10, 100);
		CK_LED_ToggleLedForMs(2, 10, 100);
	}
#endif

#if ACC_I2C_
	CK_ACC_Init2(ACC_I2C, ICM20602_ACC, G16, TARGET_1KHZ_US, TARGET_MAIN_TIME_US);
#endif

#if MAG_SPI_
	CK_MAGNETO_Init(MAG_SPI, MAG_CS_PORT, MAG_CS_PIN, TARGET_MAG, TARGET_250HZ_US, TARGET_MAIN_TIME_US);
#endif

#if MAG_I2C_
	CK_MAGNETO_Init2(MAG_I2C, TARGET_MAG, TARGET_10HZ_US, TARGET_MAIN_TIME_US);
#endif

#if BARO_SPI_
	CK_BAROMETER_Init(BARO_SPI, BARO_CS_PORT, BARO_CS_PIN, TARGET_BARO, TARGET_100HZ_US, TARGET_MAIN_TIME_US);
#endif

#if BARO_I2C_
	CK_BAROMETER_Init2(BARO_I2C, TARGET_BARO, TARGET_25HZ_US, TARGET_MAIN_TIME_US);
#endif

#if BNO055_
	CK_BNO055_Init(BNO055_I2C, TARGET_100HZ_US, TARGET_MAIN_TIME_US);
#endif

#if LOG_FLASH_
	CK_FLASH_Init_External(FLASH_SPI, FLASH_CS_PORT, FLASH_CS_PIN, FLASH_W25Q128FV_);
#endif

	pidInit(TARGET_MAIN_TIME_US);

	/*
	 * IMU starts after the sensors. 500 Hz in every mode (Betaflight ANGLE/HORIZON acc_rate /
	 * imu_process_denom = 500 Hz). At 100 Hz a 660 deg/s flip is a 6.6 degree step and the first order
	 * quaternion step left 0.4-0.7 degree behind on every flip (angle mode drift after flips).
	 */
	imuInit(TARGET_500HZ_US, TARGET_MAIN_TIME_US);

#if NAVIGATION_
	CK_NAVIGATION_Init();
#endif

#if OSD_ONBOARD_ || OSD_PDB_ || OSD_DJI_
	CK_OSD_Init(TARGET_200HZ_US, TARGET_MAIN_TIME_US);
#endif

	flight_monitor_init(TARGET_250HZ_US, TARGET_MAIN_TIME_US);

	CK_PRINTER_Init(TARGET_MAIN_TIME_US);

	CK_BUZZER_Tone2(); // flight loop starting

#if MAIN_INTERRUPT_
	CK_PERIPHERAL_StartInterrupt();
#endif

	loopTimer = CK_TIME_GetMicroSec();
}


/* ============================================================
 * FLIGHT LOOP (one control period)
 * ============================================================ */

void CKFLIGHT_Loop(void){

#if SCOPE_CHECK_MAIN == 1
	CK_GPIO_SetPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
#endif

	computeStartTime = CK_TIME_GetMicroSec();

	CK_SEC_CLEAR();
	CK_SEC_START();

	// ---- Inputs
	CK_INPUTSTREAM_Update();
	CK_SEC_MARK(CK_SEC_INPUT);

#if !MAIN_INTERRUPT_
	CK_RECEIVER_Update(CK_TIME_GetMicroSec());
	CK_RC_Update();
#endif
	CK_SEC_MARK(CK_SEC_RX);

	CK_ADJUSTMENT_Update();
	CK_SEC_MARK(CK_SEC_ADJ);

	// ---- Sensors
	CK_GYRO_Update(CK_TIME_GetMicroSec());
	CK_SEC_MARK(CK_SEC_GYRO);

	CK_ACC_Update();
	CK_SEC_MARK(CK_SEC_ACC);

#if USE_MAG_
	CK_MAGNETO_Update();
#endif
	CK_SEC_MARK(CK_SEC_MAG);

#if USE_BARO_
	CK_BAROMETER_Update();
#endif
	CK_SEC_MARK(CK_SEC_BARO);

#if GPS_
	CK_GPS_Update();
#endif
	CK_SEC_MARK(CK_SEC_GPS);

#if BNO055_
	CK_BNO055_Update();
#endif

	// ---- Attitude and navigation
	imuUpdateAttitude(CK_TIME_GetMicroSec());
	CK_SEC_MARK(CK_SEC_IMU);

#if NAVIGATION_
	// Estimator every loop (acc accumulation), state machine at 100 Hz. Switch requests only.
	CK_NAVIGATION_Update();
#endif
	CK_SEC_MARK(CK_SEC_NAV);

	// ---- Control: setpoint -> PID -> mixer -> DShot (frames leave right after the mixer)
	calculateRCSetpoint();

	pidController(CK_TIME_GetMicroSec());
	CK_SEC_MARK(CK_SEC_PID);

	CK_MIXER_Update(CK_TIME_GetMicroSec());
	CK_SEC_MARK(CK_SEC_MIX);

	// ---- Outputs that are not time critical
#if OSD_ONBOARD_ || OSD_PDB_ || OSD_DJI_
	CK_OSD_Update(CK_TIME_GetMicroSec(), computeEndTime);
#endif
	CK_SEC_MARK(CK_SEC_OSD);

#if LOG_SPI_ || LOG_SDIO_ || LOG_FLASH_
	CK_LOG_Update(computeEndTime);
#endif
	CK_SEC_MARK(CK_SEC_LOG);

	flight_monitor_update();
	CK_SEC_MARK(CK_SEC_MON);

	computeEndTime = CK_TIME_GetMicroSec() - computeStartTime;

	CK_PRINTER_Update(EVERY_25MS, computeEndTime);
	CK_SEC_MARK(CK_SEC_PRINT);

#if SMART_AUDIO_
	CK_SMARTAUDIO_Update();
#endif

	CK_LED_ToggleLed(1);    // loop heartbeat

#if SCOPE_CHECK_MAIN == 1
	CK_GPIO_ClearPin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
#endif

	CK_SEC_MARK(CK_SEC_OTHER);
	CK_LOOP_RecordBusyTime(CK_TIME_GetMicroSec() - loopTimer);

	// ---- Wait for the next period
	while((CK_TIME_GetMicroSec() - loopTimer) < TARGET_MAIN_TIME_US);
	loopTimer = CK_TIME_GetMicroSec();
}


/* ============================================================
 * OPTIONAL RECEIVER INTERRUPT (MAIN_INTERRUPT_ = 1)
 * ============================================================ */

#if MAIN_INTERRUPT_

void MAIN_INTERRUPT_Handler(void){

	HAL_TIM_IRQHandler(&htim_main_interrupt);

	#if SCOPE_CHECK_MAIN_INTERRUPT == 1
	CK_GPIO_TogglePin(SCOPE_CHECK_GPIO, SCOPE_CHECK_GPIO_PIN);
	#endif

	CK_RECEIVER_Update(CK_TIME_GetMicroSec());

	CK_RC_Update();
}

#endif
