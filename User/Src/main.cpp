#include <stdio.h>
#include "cmsis_os2.h"
#include "system_BlueNRG_LP.h"
#include "rf_driver_ll_system.h"
#include "rf_driver_hal_gpio.h"


int main (void) {
	
	SystemInit(SYSCLK_64M, BLE_SYSCLK_32M);
	
	while (1) {
		
		osDelay(1000);
	}
	
	return 0;
}