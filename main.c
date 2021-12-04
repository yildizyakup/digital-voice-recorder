/*
 * YAKUP YILDIZ 171024006 - PROJECT-3
 */

#include "bsp.h"

int main (void){

	BSP_ssd_init();
	BSP_I2C_Init();
	BSP_ADC_init();
	timer2();
	BSP_pwm_init();

	timer3();
	BSP_keypad_init();
	timer1();
	BSP_start_conversion();

while(1){

			/*It's a Trap! */

		}

return 0;
}
