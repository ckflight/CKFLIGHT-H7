#ifndef CK_DEFINITIONS_H_
#define CK_DEFINITIONS_H_

#include "stdbool.h"
#include "string.h"
#include "git_commit_hash.h"

#define 		CKFLIGHT_F4			false
#define 		CKFLIGHT_H7_V1		false
#define 		CKFLIGHT_H7_V2		true
#define 		KAKUTE_H7_1v3		false
#define 		MATEKH743_SLIMV3	false
#define 		RF_REVOLT			false

#define COMMIT_HASH					CURRENT_COMMIT_HASH
#define CURRENT_VERSION_MAJOR		VERSION_MAJOR
#define CURRENT_VERSION_MINOR		VERSION_MINOR

/*
 * Interrupt priorities (lower number = higher priority)
 *
 * HAL_Init() selects NVIC_PRIORITYGROUP_4 on F4 and H7: 16 preemption levels and NO sub-priority.
 * The sub-priorities of the previous table were therefore never applied: USB, gyro DMA, CRSF UART and
 * the DShot timer (whose priority was never set) were all at level 0. Pending interrupts of the same
 * level are served in IRQ number order, so the CRSF UART was always served last. At 420k baud one byte
 * lasts 23.8 us; a ~40 us USB interrupt (printer on) let the UART overrun and a CRSF byte was lost.
 *
 * Rule: short and time-critical on top, long and wait-tolerant at the bottom.
 *   0  CRSF / SBUS UART           : ~0.4 us per byte, must be read within 23.8 us
 *   0  DShot timer (output guard) : a counter increment while the loop is healthy
 *   1  Gyro SPI DMA               : short, sets the loop timing
 *   2  DShot DMA complete
 *   3  SD card
 *   4  GPS UART (115200 baud: 87 us per byte)
 *   5  OSD UART / DMA, MAX7456
 *   6  ADC, RGB LED
 *   7  Peripheral timer
 *   8  USB OTG FS                 : the longest interrupt (packet copy), wait tolerant
 *  15  SysTick (TICK_INT_PRIORITY, unchanged)
 * The relative order of the previous table is kept; only CRSF moved to the top and USB to the bottom.
 * Sub-priorities have no effect with group 4, so they are 0.
 */
#define RECEIVER_PreemptPriority	0
#define RECEIVER_SubPriority		0

#define DSHOT_TIMER_PreemptPriority	0

#define GYRO_TX_PreemptPriority		1
#define GYRO_TX_SubPriority			0

#define GYRO_RX_PreemptPriority		1
#define GYRO_RX_SubPriority			0

#define DSHOT_M1_PreemptPriority	2
#define DSHOT_M1_SubPriority		0

#define DSHOT_M2_PreemptPriority	2
#define DSHOT_M2_SubPriority		0

#define DSHOT_M3_PreemptPriority	2
#define DSHOT_M3_SubPriority		0

#define DSHOT_M4_PreemptPriority	2
#define DSHOT_M4_SubPriority		0

#define SDCARD_PreemptPriority		3
#define SDCARD_SubPriority			0

#define GPS_PreemptPriority			4
#define GPS_SubPriority				0

#define OSD_UART_PreemptPriority	5
#define OSD_UART_SubPriority		0

#define OSD_DMA_PreemptPriority		5
#define OSD_DMA_SubPriority			0

#define MAX7456_PreemptPriority		5
#define MAX7456_SubPriority			0

#define ADC_PreemptPriority			6
#define ADC_SubPriority				0

#define RGB_PreemptPriority			6
#define RGB_SubPriority				0

#define PERIPHERAL_PreemptPriority	7
#define PERIPHERAL_SubPriority		0

#define USB_PreemptPriority			8
#define USB_SubPriority				0

#if MATEKH743_SLIMV3 == true
#define TARGET_BOARD	"MATEKH743_SLIMV3"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if CKFLIGHT_H7_V1 == true
#define TARGET_BOARD	"CKFLIGHT_H7_V1"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if CKFLIGHT_H7_V2 == true
#define TARGET_BOARD	"CKFLIGHT_H7_V2"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if KAKUTE_H7_1v3 == true
#define TARGET_BOARD	"KAKUTE_H7_v1.3"
#define TARGET_MCU 		"H7"

// For dma cache etc so copy paste wont create problem
#define USE_H7	1
#define USE_F4	0
#endif

#if CKFLIGHT_F4 == true
#define TARGET_BOARD			"CKFLIGHT_F4"
#define TARGET_MCU 				"F4"

// For dma cache etc so copy paste wont create problem
#define USE_H7	0
#define USE_F4	1
#endif

#if RF_REVOLT == true
#define TARGET_BOARD			"RF_REVOLT"
#define TARGET_MCU 				"F4"

// For dma cache etc so copy paste wont create problem
#define USE_H7	0
#define USE_F4	1
#endif

#if USE_H7 == 1
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#endif

#if USE_F4 == 1
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#endif

#if CKFLIGHT_F4 == true
#include "config/ckflightf4_v4.h"
#endif

#if CKFLIGHT_H7_V1 == true
#include "config/ckflighth7_v1.h"
#endif

#if CKFLIGHT_H7_V2 == true
#include "config/ckflighth7_v2.h"
#endif

#if KAKUTE_H7_1v3 == true
#include "config/kakuteh7_1v3.h"
#endif

#if MATEKH743_SLIMV3 == true
#include "config/matekh743slim_v3.h"
#endif

#if RF_REVOLT == true
#include "config/revolt.h"
#endif


typedef struct{
    uint16_t syncRate;

    uint16_t syncCounter;

    uint32_t targetLoopTime; // in microseconds 1k is 1000 microsecond 2k is 500 8k is 125 etc.

    uint32_t targetFrequency;

}syncTimer_t;


// Target times in microseconds 8k is 125 microsecond etc.
typedef enum
{
    TARGET_1HZ_US                              	= 1000000/1,
    TARGET_2HZ_US                              	= 1000000/2,
	TARGET_4HZ_US                              	= 1000000/4,
    TARGET_5HZ_US                              	= 1000000/5,
	TARGET_6HZ_US                              	= 1000000/6,
	TARGET_8HZ_US								= 1000000/8,
	TARGET_10HZ_US 								= 1000000/10,
	TARGET_15HZ_US                             	= 1000000/15,
	TARGET_20HZ_US 								= 1000000/20,
	TARGET_25HZ_US 								= 1000000/25,
	TARGET_50HZ_US 								= 1000000/50,
	TARGET_75HZ_US                             	= 1000000/75,
	TARGET_80HZ_US 								= 1000000/80,
	TARGET_100HZ_US 							= 1000000/100,
	TARGET_105HZ_US                            	= 1000000/105,
	TARGET_110HZ_US                            	= 1000000/110,
	TARGET_125HZ_US                            	= 1000000/125,
	TARGET_200HZ_US 							= 1000000/200,
	TARGET_220HZ_US                            	= 1000000/220,
	TARGET_250HZ_US                            	= 1000000/250,
	TARGET_300HZ_US 							= 1000000/300,
	TARGET_400HZ_US 							= 1000000/400,
	TARGET_500HZ_US 							= 1000000/500,
	TARGET_600HZ_US 							= 1000000/600,
	TARGET_800HZ_US 							= 1000000/800,
	TARGET_1KHZ_US 								= 1000000/1000,
	TARGET_2KHZ_US 								= 1000000/2000,
	TARGET_4KHZ_US 								= 1000000/4000,
	TARGET_8KHZ_US 								= 1000000/8000,
	TARGET_10KHZ_US 							= 1000000/10000,
	TARGET_12KHZ_US 							= 1000000/12000,
	TARGET_16KHZ_US                           	= 1000000/16000,
	TARGET_32KHZ_US                           	= 1000000/32000

}targetFreq_e;


typedef enum
{
	ICM20602_GYRO,
	ICM45686_GYRO,
	ICM42688P_GYRO,
	L3GD20H_GYRO,
	ICM42605_GYRO,
	IIM42652_GYRO,
	MPU6000_GYRO,

	ICM20602_ACC,
	ICM45686_ACC,
	ICM42688P_ACC,
	IIM42652_ACC,
	LSM303D_ACC,
	FXOS8700CQ_ACC,
	ICM42605_ACC,
	MPU6000_ACC,

	MAG3110_MAGNETO,
	LSM303D_MAGNETO,
	FXOS8700CQ_MAGNETO,
	HMC5983_MAGNETO,
	QMC5883L_MAGNETO,
	MLX90393_MAGNETO,

	MS5607_BAROMETER,
	BMP280_BAROMETER,
	MS5611_BAROMETER,
	DPS310_BAROMETER,

	BNO055_IMU,

	GPS_UBLOX7,
	GPS_UBLOX8,

	SENSOR_NONE

}sensorModel_e;



#endif /* CK_DEFINITIONS_H_ */






