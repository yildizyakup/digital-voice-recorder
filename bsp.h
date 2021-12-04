/*
 * YAKUP YILDIZ 171024006 - PROJECT-3
 */

#ifndef BSP_H_
#define BSP_H_

#include "stm32g0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*Common API function for Nucleo STM32G031 board*/
void BSP_system_init(void);
void BSP_pwm_init(void);
void BSP_start_conversion(void);

//ADC related functions
void BSP_ADC_init(void);
uint16_t BSP_ADC_start(void);

//I2C related functions
void BSP_I2C_Init(void);
void BSP_random_read_I2C(uint8_t , uint16_t , uint8_t* , int);
void BSP_write_I2C(uint8_t ,uint16_t , uint8_t* , int);

//SSD related functions
void BSP_change_state(void);
void BSP_ssd_init(void);
void BSP_ssd_set(int);
void BSP_ssd_clear(void);
void BSP_ssd_display(void);
void BSP_ssd_ID_display(void);
void BSP_ssd_Digit_n_ON(int);
void BSP_ssd_Digit_n_OFF(int);
void BSP_ssd_Digit_all_OFF(void);
void BSP_ssd_states(int,int,int,int);

//KEYPAD related functions
void BSP_keypad_init(void);
void BSP_keypad_set(void);
void BSP_keypad_clear(void);
void BSP_keypad_press(int, float);

void delay(uint32_t);
void timer1(void);
void timer2(void);
void timer3(void);

#endif
