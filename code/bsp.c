/*
 * YAKUP YILDIZ 171024006 - PROJECT-3
 */

#include "bsp.h"
#include "stm32g0xx.h"

uint8_t  EEPROM1_ADDRESS = 0X50;

int counter;
int countDown; /*DOES NOT WORK*/
int i,j;

int displayNumber_d1;
int displayNumber_d2;
int displayNumber_d3;
int displayNumber_d4;

uint8_t voltageValue;
uint8_t voltageBuffer[32];
uint8_t voiceRecorded[32];

uint16_t writtenAddress = 0x50;
uint32_t freq;

/*TO STATES*/
int r = 21;
int c = 22;
int d = 17;
int p = 18;
int l = 14;
int b = 20;
int e = 19;


_Bool recordFlag;
_Bool readFlag;
_Bool readFlag;
_Bool clearFlag;
_Bool startFlag = 1;
_Bool preventBounce;

void delay(uint32_t s){

    for(;s>0;s--);

    }

void timer1(void) {

	/* Setup TIM1 */

	  RCC->APBENR2 |= (1U << 11);
	  TIM1->CR1 = 0;
	  TIM1->CR1 |= (1 << 7);
	  TIM1->CNT = 0;
	  TIM1->PSC = 9;
	  TIM1->ARR = 1600;
	  TIM1->DIER |= (1 << 0);
	  TIM1->CR1 |= (1 << 0);

	  NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 3);
	  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

}

/****************************** SSD DISPLAY ******************************/
void TIM1_BRK_UP_TRG_COM_IRQHandler(void){

	BSP_ssd_display();
	BSP_keypad_set();
	TIM1->SR &= ~(1U << 0);

}


void timer2(void) {

	RCC->APBENR1 |= (1U << 0);
	TIM2->DIER |= (1 << 0);

	/* Setup CH2 */
	TIM2->CCMR1 &= ~(7U << 12);
	TIM2->CCMR1 &= ~(1U << 24);
	TIM2->CCMR1 |=  (6U << 12);  		/* OC2M: -0110- PWM mode 1 */
	TIM2->CCMR1 |=  (1U << 11); 		/* OC2FE: Enable output compare 2 */

	TIM2->CCER |= (1U << 4);	/*CH2 Enable Capture*/
	TIM2-> CR1 |= (1U << 7);

	TIM2->ARR = 101;
	TIM2->EGR |= (1U << 0);		/*Update Generation*/

	TIM2-> CR1 |= (1U << 0);	/*Counter Enable*/
}



void timer3(void)
{
  /* Setup TIM3 */
  RCC->APBENR1 |= (1U << 1);

  TIM3->CR1 = 0;
  TIM3->CR1 |= (1U << 7);
  TIM3->CNT = 0;
  TIM3->PSC = 0;
  TIM3->ARR = 2000;
  TIM3->CR2 |= (2U << 4);    //MASTER MODE


  TIM3->CR1 |= (1U << 0);
}

void BSP_start_conversion(void){
	ADC1->CR |= (1U << 2);
	while (!(ADC1->ISR & (1U << 2)));
}

void BSP_pwm_init(void){
	RCC->IOPENR |= (1U << 1);			/* 	B BLOCK RCC OPEN */
	GPIOB->MODER &= ~(3U << 2 *3);		/* 	PB3 ALTERNA MODE CLEAR */
	GPIOB->MODER |= (2U << 2 *3);	    /* 	PB3 ALTERNA MODE */
	GPIOB->OSPEEDR |= (3U << 2 *3);	    /* 	HIGH PRECISION ON CHANGE */
	GPIOB->AFR[0] &= ~(0xFU << 4 *3);   /* 	AFSEL3 = RESET and PB3->AF1 = TIM1_CH2 */
	GPIOB->AFR[0] |= (2U << 4 *3);	    /*	AFSEL3 -> AF1 [0010] */
}

void BSP_ADC_init(void) {

	RCC->IOPENR |= (1U << 0);			/* A clock activated */
	GPIOA->MODER &= ~(3U << 2 * 0);		/* Clear PA0 */
	GPIOA->MODER |= (3U << 2 * 0);		/* Setting PA0 as analog */

	RCC->APBENR2 |= (1U << 20);			/* Enable ADC clock */
	ADC1->CR |= (1U << 28);	 			/* Enable ADC voltage regulator */
	delay(100);						    /* Wait for ADC enable */

	ADC1->CR |= (1U << 31);				/* Enable ADC calibration  */
	while((ADC1->ISR & (1 << 11)));    	/* Wait until EOCAL = 1*/

	// enable end of calibration or sequence interrupts

	ADC1->IER |= (1U <<2); 				/* End of conversion interrupt*/

	// resolution
	ADC1->CFGR1 |= (2U << 3); 			/*8-bit*/

	//configuring single/discontinuous
	ADC1->CFGR1 &= ~(1U << 13);
	ADC1->CFGR1 |=  (1U << 16);

	//sampling time from SMPR
	ADC1->SMPR |= (1U << 8);  //SMP2
	ADC1->SMPR |= (0x4 << 4); /*7.5 clock cycle*/

	//TIM3 TRGO definition
	ADC1->CFGR1 |= (3U << 6); 	//TRGO  0xB011=3U FOR TIM3_TRGO
	ADC1->CFGR1 |= (1U << 10);	//RISING EDGE

	//enable channels
	ADC1->CFGR1 |= (1U << 23); 	//ANALOG INPUT
	ADC1->CFGR1 &= ~(1U << 26); //ANALOG INPUT for channel 0
	ADC1->CHSELR |= (1U << 0); 	//ANALOG INPUT for channel 0

	//CLEAR ADRDY bit ADC_ISR
	ADC1->ISR |= (1U << 0);

	//ENABLE ADC
	ADC1->CR |= (1U << 0);				/* ENABLE ADC */
	while (!(ADC1->ISR & (1U << 0)));	/* Is ADC ready? */

	NVIC_SetPriority(ADC1_IRQn,2);
	NVIC_EnableIRQ(ADC1_IRQn);
}

uint16_t BSP_ADC_start(void){
	//ADC1->CR |= (1U << 2);			/* Start ADC */
	while(!(ADC1->ISR & (1U << 2)));	/* Is there any data? */
	return ADC1->DR;					/* Data from pin */
}

/******************************ADC HANDLER******************************/
void ADC_COMP_IRQHandler(void){

	preventBounce = 0;

 if (recordFlag == 1){
		voltageValue = (uint8_t) BSP_ADC_start();
		voltageBuffer[j] = voltageValue % 256;
	j++;

	if(j == 32){

		BSP_write_I2C(EEPROM1_ADDRESS, writtenAddress, &voltageBuffer, 32);
		delay(100);
		writtenAddress += 32;
		j = 0;
		++i;
	}

	 if(i == 1000){


		recordFlag = 0;
		i = 0;
		writtenAddress = 0x32;

	}

	}

 else if(readFlag == 1){

	freq = ((uint32_t)voiceRecorded[j]);
	TIM2->CCR2 = freq;
	j++;

	if(j == 32){

		BSP_random_read_I2C(EEPROM1_ADDRESS, writtenAddress, &voiceRecorded, 32);
		delay(100);
		writtenAddress += 32;
		j = 0;
		++i;
	}

	else if(i == 1000){
		readFlag = 0;
		writtenAddress = 0x32;
		i=0;

	}
}

 else if (clearFlag == 1){

		voltageBuffer[j] = 0;
		voiceRecorded[j] = 0;
	j++;

	if(j == 32){

		BSP_write_I2C(EEPROM1_ADDRESS, writtenAddress, &voltageBuffer, 32);
		delay(100);
		writtenAddress += 32;
		j = 0;
		++i;
	}

	 if(i == 1000){

		clearFlag = 0;
		i = 0;
		writtenAddress = 0x32;
	}

	}

	ADC1->ISR &= (1U << 2);
}

void BSP_I2C_Init(void){

		/*PB6-PB7 ALTERNA MOD*/

	    RCC->IOPENR |= (1U << 1);

	    GPIOB->MODER &= ~(3U << 2*6);
	    GPIOB->MODER |= (2U << 2*6);
	    GPIOB->PUPDR |= (1U << 2*6);

	    GPIOB->MODER &= ~(3U << 2*7);
	    GPIOB->MODER |= (2U << 2*7);
	    GPIOB->PUPDR |= (1U << 2*7  );

	    GPIOB->OTYPER &= ~(1U << 7);
	    GPIOB->OTYPER &= ~(1U << 6);

	    GPIOB->OTYPER |= (1U << 6);
	    GPIOB->OTYPER |= (1U << 7);

	    GPIOB->AFR[0] &= ~(0xFU << 4*6);
	    GPIOB->AFR[0] |= (6U << 4*6);

	    GPIOB->AFR[0] &= ~(0xFU << 4*7);
	    GPIOB->AFR[0] |= (6U << 4*7);

	    /*I2C ENABLE*/

	    RCC->APBENR1 |= (1U << 21);
	    I2C1->CR1 = 0;
	    I2C1->CR1 |= (1U << 7); // ERRI

	    I2C1->TIMINGR |= (3 << 2*8); 								// PRESC
	    I2C1->TIMINGR |= (0x13 << 0); 								// SCLL
	    I2C1->TIMINGR |= (0xF << 8);								// SCLH
	    I2C1->TIMINGR |= (0x2 << 1*6); 								// SDADEL
	    I2C1->TIMINGR |= (0x4 << 2*0); 								// SCLDEL
	    I2C1->CR1 |= (1U << 0); 									//PE

	    NVIC_SetPriority(I2C1_IRQn,1);
	    NVIC_EnableIRQ(I2C1_IRQn);
}

void BSP_random_read_I2C(uint8_t devAddr,uint16_t memAddr, uint8_t *data, int size){

	I2C1->CR2 = 0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (2U << 16);	//Number of bytes
	I2C1->CR2 |= (1U << 13);	//Generate Start

	while(!(I2C1->ISR & (1 << 1)));
	I2C1->TXDR = (uint32_t)(memAddr >> 8);

	while(!(I2C1->ISR & (1 << 1)));
	I2C1->TXDR = (uint32_t)(memAddr & 0xFF);

	while(!(I2C1->ISR & (1 << 6)));	//is transmission complete

	//read data
	I2C1->CR2 = 0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (1U << 10);	//Read mode
	I2C1->CR2 |= (uint32_t)(size << 16);	//Number of bytes
	I2C1->CR2 |= (1U << 25);	//AUTOEND
	I2C1->CR2 |= (1U << 13);	//Generate start

	while(size){
		while(!(I2C1->ISR & (1 << 2)));
		(*data++) = (uint8_t)I2C1->RXDR;
		size--;
	}

}

void BSP_write_I2C(uint8_t devAddr,uint16_t memAddr, uint8_t *data, int size){

	I2C1->CR2 = 0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (uint32_t)((size + 2)<< 16);
	I2C1->CR2 |= (1U << 25);	/*Auto-end*/
	I2C1->CR2 |= (1U << 13);	/*Generate start*/

	while(!(I2C1->ISR & (1 << 1)));	/*high address*/
	I2C1->TXDR = (uint32_t)(memAddr >> 8);

	while(!(I2C1->ISR & (1 << 1)));	/*low address*/
	I2C1->TXDR = (uint32_t)(memAddr & 0xFF);

	while(size){
		while(!(I2C1->ISR & (1 << 1)));
		I2C1->TXDR = (*data++);	/*DATA SEND*/
		size--;
	}
}


void BSP_system_init(void){

	BSP_keypad_init();
	BSP_ssd_init();

}

void BSP_keypad_init(void){

	/* Enable GPIOA and GPIOB clock */
	RCC->IOPENR |=(3U<<0);

	/* Setup PA15, PB1, PA10 as output (rows) */

	GPIOA->MODER &= ~(3U << 2*15); /* PA15 is output (31-30) */
	GPIOA->MODER |=  (1U << 2*15);

	GPIOB->MODER &= ~(3U << 2*1);  /* PB1 is output (3-2)  */
	GPIOB->MODER |=  (1U << 2*1);

	GPIOA->MODER &= ~(3U << 2*10); /* PA10 is output (21-20) */
	GPIOA->MODER |=  (1U << 2*10);


	/* Setup PA10, PA9, PB0, PB2 as input (columns) */

	GPIOA->MODER &= ~(3U << 2*9);  /* PA9 is input (21-20) */
	GPIOA->PUPDR |=  (2U << 2*9);

	GPIOB->MODER &= ~(3U << 2*0);  /* PB0 is input (01-00) */
	GPIOB->PUPDR |=  (2U << 2*0);

	GPIOB->MODER &= ~(3U << 2*2);  /* PB2 is input (03-02) */
	GPIOB->PUPDR |=  (2U << 2*2);

	/* Setup interrupts for inputs */

	EXTI->EXTICR[2] |= (0U << 8*1);  /*  PA9  */
	EXTI->EXTICR[0] |= (1U << 8*0);  /*  PB0  */
	EXTI->EXTICR[0] |= (1U << 8*2);  /*  PB2  */

	/* Rising Edge */

	EXTI->RTSR1 |= (1U << 9);   /*  PA9  */
	EXTI->RTSR1 |= (1U << 0);   /*  PB0  */
	EXTI->RTSR1 |= (1U << 2);   /*  PB2  */

	/* Mask */

	EXTI->IMR1 |= (1U << 9);    /*  PA9  */
	EXTI->IMR1 |= (1U << 0);    /*  PB0  */
	EXTI->IMR1 |= (1U << 2);    /*  PB2  */


	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	NVIC_SetPriority(EXTI2_3_IRQn, 0);
	NVIC_EnableIRQ(EXTI2_3_IRQn);

	NVIC_SetPriority(EXTI4_15_IRQn, 0);
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	BSP_keypad_set();
	BSP_ssd_clear();
}

void BSP_keypad_set(void){

	GPIOA->ODR |= (1U << 15);
	GPIOB->ODR |= (1U << 1);
	GPIOA->ODR |= (1U << 10);
}

void BSP_keypad_clear(void){

	GPIOA->ODR &= ~(1U << 15);
	GPIOB->ODR &= ~(1U << 1);
	GPIOA->ODR |= (1U << 10);
}

void BSP_ssd_states(int state1, int state2, int state3, int state4){

/*States: START(state1), IDLE(state2), RECORD(state3),PLAYBACK(dn4)*/

		displayNumber_d1 = state1;
		displayNumber_d2 = state2;
		displayNumber_d3 = state3;
		displayNumber_d4 = state4;

}

void BSP_change_state(void){

	if (startFlag == 1){
		BSP_ssd_states(1,7,0,6);
	}
	else if (recordFlag == 1){
		BSP_ssd_states(r,c,d,countDown);
	}
	else if (readFlag == 1){
		BSP_ssd_states(p,l,b,countDown);
	}
	else if (clearFlag == 1){
		BSP_ssd_states(c,l,r,countDown);
		}
	else
		BSP_ssd_states(1,d,l,e);

}

void BSP_ssd_init(void){

	/* Enable GPIOA and GPIOB clock */
	RCC->IOPENR |=(3U<<0);

    /*Setup PA0-1-4-5-6-7-11-12 for Digits 0011_1100_0000_0011_1111_0000_1100*/
    GPIOA -> MODER &= ~(15U <<2*0);  /* Necessary bits cleared 1111     */
    GPIOA -> MODER &= ~(255U <<2*4); /* Necessary bits cleared 11111111 */
    GPIOA -> MODER &= ~(15U <<2*11); /* Necessary bits cleared 1111     */

    GPIOA -> MODER |= (5U << 2*0);  /* Required pins is activated output" 0101     */
    GPIOA -> MODER |= (85U << 2*4); /* Required pins is activated output" 01010101 */
    GPIOA -> MODER |= (5U << 2*11); /* Required pins is activated output" 0101     */

	/* Setup PB4 as output D1 */
	GPIOB->MODER &= ~(3U << 2 * 4);
	GPIOB->MODER |= (1U << 2 * 4);

	/* Setup PB5 as output D2 */
	GPIOB->MODER &= ~(3U << 2 * 5);
	GPIOB->MODER |= (1U << 2 * 5);

	/* Setup PB8 as output D3 */
	GPIOB->MODER &= ~(3U << 2 * 8);
	GPIOB->MODER |= (1U << 2 * 8);

	/* Setup PB9 as output D4 */
	GPIOB->MODER &= ~(3U << 2 * 9);
	GPIOB->MODER |= (1U << 2 * 9);

	BSP_ssd_clear();
}


void BSP_ssd_set(int number){

	BSP_keypad_clear();
	BSP_ssd_clear();

		if (number == 0) {

			GPIOA->ODR &= ~(0x1872U);
		}

		else if (number == 1) {

			GPIOA->ODR &= ~(0x30U);
		}

		else if (number == 2) {

			GPIOA->ODR &= ~(0x1892U);
		}

		else if (number == 3) {

			GPIOA->ODR &= ~(0x10B2U);
		}

		else if (number == 4) {

			GPIOA->ODR &= ~(0xF0U);
		}

		else if (number == 5) {

			GPIOA->ODR &= ~(0x10E2U);
		}

		else if (number == 6) {

			GPIOA->ODR &= ~(0x18E2U);
		}

		else if (number == 7) {

			GPIOA->ODR &= ~(0x32U);
		}

		else if (number == 8) {

			GPIOA->ODR &= ~(0x18F2U);
		}

		else if (number == 9) {

			GPIOA->ODR &= ~(0x10F2U);
		}
		//- Negative Symbol
		else if (number == 10) {

			GPIOA->ODR &= ~(0x80U);
		}

		//Decimal
		else if (number == 11) {

			GPIOA->ODR &= ~(0x1U);
		}
		//U
		else if (number == 12) {

			GPIOA->ODR &= ~(0X1870U);
		}
		//0000_1000_1100_0010 = 0x1C4U //F
		else if (number == 13) {

			GPIOA->ODR &= ~(0x8C2U);

		}
		//0001_1000_0100_0000 = 0x1840U //L
		else if (number == 14) {

			GPIOA->ODR &= ~(0x1840U);

		}
		//n 0000_1000_1010_0000 = 0x08A0
		else if (number == 15) {

			GPIOA->ODR &= ~(0x08A0U);

		}
		//u 0001_1000_0010_0000 = 0x1820
		else if (number == 16) {

			GPIOA->ODR &= ~(0x1820U);

		}
		//d 0001_1000_1011_0000 = 0x18B0
		else if (number == 17) {

			GPIOA -> ODR &= ~(0x18B0U);

		}

		//p
		else if (number == 18) {

			GPIOA -> ODR &= ~(0x8D2U);
		}

		//e
		else if (number == 19) {

			GPIOA -> ODR &= ~(0x18C2U);

		}

		//b
		else if (number == 20) {

			  GPIOA -> ODR &= ~(0x18E0U);

		}

		//r
		else if (number == 21) {

			GPIOA -> ODR &= ~(0x880U);

		}

		//c
		else if (number == 22) {

			GPIOA -> ODR &= ~(0x1880U);

		}
}

void BSP_ssd_clear(void){
	GPIOA->ODR |= (0x18F3U);
}


void BSP_ssd_display(void){

		preventBounce = 0;
		BSP_ssd_clear();
		BSP_change_state();


	if (counter == 0){
		BSP_ssd_Digit_all_OFF();
		BSP_ssd_set(displayNumber_d1);
		BSP_ssd_Digit_n_ON(1);
		counter+=1;
	}

	else if(counter == 1){
		BSP_ssd_Digit_all_OFF();
		BSP_ssd_set(displayNumber_d2);
		BSP_ssd_Digit_n_ON(2);
		counter+=1;
	}

	else if(counter == 2){
		BSP_ssd_Digit_all_OFF();
		BSP_ssd_set(displayNumber_d3);
		BSP_ssd_Digit_n_ON(3);
		counter+=1;
	}
	else if (counter == 3){
		BSP_ssd_Digit_all_OFF();
		BSP_ssd_set(displayNumber_d4);
		BSP_ssd_Digit_n_ON(4);
		counter = 0;
	}
}

void BSP_ssd_Digit_n_ON(int n){

	if(n==1)
	GPIOB->ODR |= (1 << 4);

	else if (n==2)
	GPIOB->ODR |= (1 << 5);

	else if (n==3)
	GPIOB->ODR |= (1 << 9);

	else if (n==4)
	GPIOB->ODR |= (1 << 8);
}


void BSP_ssd_Digit_n_OFF(int n){

	if(n==1)
	GPIOB->BRR |= (1 << 4);

	else if (n==2)
	GPIOB->BRR |= (1 << 5);

	else if (n==3)
	GPIOB->BRR |= (1 << 9);

	else if (n==4)
	GPIOB->BRR |= (1 << 8);

}

void BSP_ssd_Digit_all_OFF(void){

	GPIOB->BRR |= (1 << 4);
	GPIOB->BRR |= (1 << 5);
	GPIOB->BRR |= (1 << 9);
	GPIOB->BRR |= (1 << 8);

}

void EXTI0_1_IRQHandler(void) /* PB0 */ {

	BSP_keypad_clear();

	/*** PB1 - 5 ***/
	GPIOB->ODR ^= (1U<<1);

	if(((GPIOB->IDR >> 0) & 1)  && (preventBounce == 0) ){

		preventBounce = 1;

		delay(400000);
	}

	GPIOB->ODR ^= (1U<<1);

	/*** PA15 - 2 ***/
	GPIOA->ODR ^= (1U<<15);

	if(((GPIOB->IDR >> 0) & 1) && (preventBounce == 0) ){

		preventBounce = 1;
		recordFlag = 0;
		startFlag = 0;
		readFlag = 1;

		TIM3->DIER |= (1U << 0);
		ADC1->CR |= (1U << 0);

		delay(16000);
	}

	GPIOA->ODR ^= (1U<<15);

	/*** PA10 - 8 ***/
	GPIOA->ODR ^= (1U << 10);

	if(((GPIOB->IDR >> 0) & 1)  && (preventBounce == 0) ){


		preventBounce = 1;

		delay(80000);
	}

	GPIOB->ODR ^= (1U<<10);

	EXTI->RPR1 |= (1U <<0); /* Clear interrupt flag */

	BSP_keypad_set();
}

void EXTI2_3_IRQHandler(void) /* PB2 */ {

	BSP_keypad_clear();

	/*** PB1 - 6 ***/
	GPIOB->ODR ^= (1U<<1);

	if(((GPIOB->IDR >> 2) & 1)  && (preventBounce == 0)){

		preventBounce = 1;

		delay(16000);
	}

	GPIOB->ODR ^= (1U<<1);

	/*** PA15 - 3 ***/
	GPIOA->ODR ^= (1U<<15);

	if(((GPIOB->IDR >> 2) & 1) && (preventBounce == 0)){

		preventBounce = 1;

		startFlag = 0;
		recordFlag = 0;
		readFlag = 0;
		clearFlag = 1;

		TIM3->DIER |= (1U << 0);
		ADC1->CR |= (1U << 0);

		delay(16000);
	}

	GPIOA->ODR ^= (1U<<15);

	/*** PA10 - 9 ***/
	GPIOA->ODR ^= (1U<<10);

	if(((GPIOB->IDR >> 2) & 1)  && (preventBounce == 0) ){

		preventBounce = 1;

		delay(16000);
	}

	GPIOB->ODR ^= (1U<<10);

	EXTI->RPR1 |= (1U <<2); /* Clear interrupt flag */
	BSP_keypad_set();
}

void EXTI4_15_IRQHandler(void) /* PA9  */ {

	BSP_keypad_clear();

	/*** PB1 - 4 ***/
	GPIOB->ODR ^= (1U<<1);

	if(((GPIOA->IDR >> 9) & 1) && (preventBounce == 0) ){


		preventBounce = 1;

		delay(16000);
	}

	GPIOB->ODR ^= (1U<<1);

	/*** PA15 - 1 ***/
	GPIOA->ODR ^= (1U<<15);

	if(((GPIOA->IDR >> 9) & 1) && (preventBounce == 0) ){


		preventBounce = 1;
		recordFlag = 1;
		startFlag = 0;
		readFlag = 0;
		delay(8000);

	}

	GPIOA->ODR ^= (1U<<15);

	/*** PA10 - 7 ***/
	GPIOA->ODR ^= (1U<<10);

	if(((GPIOA->IDR >> 9) & 1)  && (preventBounce == 0) ){


		preventBounce = 1;

		delay(16000);
	}

	GPIOB->ODR ^= (1U<<10);


	EXTI->RPR1 |= (1U <<9); /* Clear interrupt flag */
	BSP_keypad_set();

}
