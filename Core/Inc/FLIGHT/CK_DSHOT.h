
#ifndef INC_CK_DSHOT_H_
#define INC_CK_DSHOT_H_

#include "CK_DEFINITIONS.h"

#define DSHOT_COMMAND_MOTOR_STOP		0 // motor stops

#define DSHOT_MIN_THROTTLE				48 // motor spins at lowest idle speed
#define DSHOT_MAX_THROTTLE				2047

#define DSHOT_RANGE (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)


/*
 * Motor output model (same path as Betaflight motorWriteAll() + updateComplete()):
 *
 *   - CK_DSHOT_SetMotor1..4 only prepare the packets.
 *   - CK_DSHOT_Update() starts all four frames together, right after the mixer, in the same loop.
 *     Mixer -> DShot delay is fixed, there is no phase drift and a DMA buffer is never rewritten
 *     while its frame is on the wire.
 *   - The DShot timer (TIM7) no longer sends motor values. It is only an output guard:
 *       * before the flight loop starts (init, WaitARM): 1 kHz STOP keep-alive for the ESCs,
 *       * if the loop sends no frame for DSHOT_GUARD_TIMEOUT_US (250 ms): STOP and latch.
 *         The latch is released only when the mixer outputs DISARM (all four motors STOP).
 */
typedef enum{

	SYNC_MODE

}DSHOT_Mode_t;

void CK_DSHOT_Init(DSHOT_Mode_t mode, targetFreq_e target_period);

// Main loop: call once after SetMotor1..4, sends the four frames together
void CK_DSHOT_Update(void);

// Fault handlers: no interrupts needed. Sends 10 STOP frames (~10 ms) and holds the lines low.
void CK_DSHOT_EmergencyStop(void);

// True when the output guard has stopped the motors (main loop stalled, waiting for DISARM)
bool CK_DSHOT_GuardTripped(void);

// Diagnostics since power-up: guard trips and frames skipped because the DMA was still busy
extern volatile uint32_t ck_dshot_guard_trips;
extern volatile uint32_t ck_dshot_busy_skips;

void CK_DSHOT_InitEndPoints(float outputLimit, float* outputLow, float* outputHigh, float* disarmMotorOutput);

void CK_DSHOT_SetMotor1(int num);

void CK_DSHOT_SetMotor2(int num);

void CK_DSHOT_SetMotor3(int num);

void CK_DSHOT_SetMotor4(int num);

uint16_t CK_DSHOT_PrepareDshotPacket(uint16_t value, bool requestTelemetry);

void CK_DSHOT_LoadDmaBufferDshot(uint32_t *dmaBuffer, int stride, uint16_t packet);

uint32_t CK_DSHOT_GetChannel(uint32_t c);

void CK_DSHOT_Enable(void);

void CK_DSHOT_SetTimerRequest(uint32_t c, int motor_num);

uint32_t CK_DSHOT_GetTimerRequest(uint32_t c);

void CK_DSHOT_Start1(int num);

void CK_DSHOT_Start2(int num);

void CK_DSHOT_Start3(int num);

void CK_DSHOT_Start4(int num);

void CK_DSHOT_Init1(void);

void CK_DSHOT_Init2(void);

void CK_DSHOT_Init3(void);

void CK_DSHOT_Init4(void);

void CK_DSHOT_InitInterrupt(targetFreq_e target_period);

void CK_DSHOT_DMA_SetInterruptFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream, int motor_num);

void CK_DSHOT_DMA_ClearInterruptFlags(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT1(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT2(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT3(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_DSHOT_DMA_ClearInterruptFlagsDSHOT4(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

uint32_t CK_DSHOT_DMA_GetInterruptFlags(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream);

void CK_DSHOT_DMA_SetTCFlag(DMA_TypeDef* dma, DMA_Stream_TypeDef* dma_stream, int motor_num);

uint8_t CK_DSHOT_DMA_CheckTransferComplete(DMA_Stream_TypeDef* dma_stream, uint32_t status);

#endif /* INC_CK_DSHOT_H_ */
