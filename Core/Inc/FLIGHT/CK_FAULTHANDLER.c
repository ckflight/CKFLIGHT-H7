
#include "FLIGHT/CK_FAULTHANDLER.h"
#include "FLIGHT/CK_ESC.h"

#include "DRIVERS/CK_LED.h"

/*
 * Every dead-end handler stops the motors first. CK_ESC_EmergencyStop() polls the hardware directly,
 * so it works even though no interrupt (DShot guard, DMA complete, SysTick) can run inside a fault.
 * BusFault, UsageFault and NMI are in stm32h7xx_it.c and do the same.
 */

void HardFault_Handler(void){

	CK_ESC_EmergencyStop();

	while(1){

		CK_LED_ToggleLedForMs(1, 1, 500);
		CK_LED_ToggleLedForMs(1, 2, 500);

	}

}

void MemManage_Handler(void){

	CK_ESC_EmergencyStop();

	while(1){

		CK_LED_ToggleLedForMs(1, 1, 500);
		CK_LED_ToggleLedForMs(1, 2, 500);

	}

}
