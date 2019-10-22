// final project***********************************************
// 
// author: Alexander McGinnis, Parshva Vora
//
//description: This uses the stm32 nucleo f401re MCU to output a tone on a passive buzzer depending on the user's hand 
//distance from a ultrasonic sensor. It will output the note being played to a serial monitor
//
//PA9 goes to RX on HC-06 module
//PA10 goes to TX on HC-06 module
//PC2 goes to trig distance sensor
//PC3 goes to echo distance sensor
//PA7 goes to signal of buzzer
// ********************************************



#include "stdint.h"
#include "stm32f4xx.h"
#include "math.h"

#define OUTPIN 7 //PA7 is PWM output pin (corresponding to TIM1_CH1N on datasheet page 47
#define FMAX 2400 //PA7 is PWM output pin (corresponding to TIM1_CH1N on datasheet page 47
#define FMIN 100 //PA7 is PWM output pin (corresponding to TIM1_CH1N on datasheet page 47
#define FSTEP 5 //PA7 is PWM output pin (corresponding to TIM1_CH1N on datasheet page 47

#define TRIGPIN 2
#define ECHOPIN 3



int selection = 0;

void HWdelay_us(uint16_t us) { //hardware delay using timer 5 in microseconds
	
	
	TIM5->CR1 &= ~TIM_CR1_CEN;
	TIM5->SR = 0;
	TIM5->CNT = 0;
	TIM5->PSC = 15;
	TIM5->ARR = us - 1;
	TIM5->CR1 |= TIM_CR1_CEN;
	
	while ((TIM5->SR & TIM_SR_UIF) == 0);{} //waste time until update interrupt flag bit is set 
}

void HWdelay_ms(uint16_t ms) { //hardware delay using timer 5 in milliseconds
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //clock TIM5
	
	TIM5->CR1 &= ~TIM_CR1_CEN;
	TIM5->SR = 0;
	TIM5->CNT = 0;
	TIM5->PSC = 15999;
	TIM5->ARR = ms - 1;
	TIM5->CR1 |= TIM_CR1_CEN;
	
	while ((TIM5->SR & TIM_SR_UIF) == 0);{} //waste time until update interrupt flag bit is set 
}


uint32_t CaptureDistance() {
	
	uint32_t counter = 0;
	GPIOC->ODR |= (1 << TRIGPIN); //turn on trig pin for 10 us
	HWdelay_us(8);
	GPIOC->ODR &= ~(1 << TRIGPIN); 
	while (1) {
		if ( (GPIOC->IDR & (1 << ECHOPIN)) ) {	//wait for echo pin to go high
			while (1) {							//count until echo pin goes high
				counter++;
				// HWdelay_us(4);
				if ( !(GPIOC->IDR & (1 << ECHOPIN)) ) {	//wait for echo pin to go low
					return counter;
				}
			}
		}
	}
	return 0;
	
	
}


void InitalizePWMPA7(double freq) {
	//takes freq in Hz, and duty like 50% = 50, 75% = 75, etc
	if (!freq) {
	TIM1->CR1 &= ~TIM_CR1_CEN; //disable timer 1
	}
	else {
		double period = 1000000/freq;
		uint16_t arr_in = (period/2)-1;
		
		//set up GPIOA
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		GPIOA->MODER |= (2 << 2*7);// set outpin as alternative function
		GPIOA->AFR[0] |= (1 << 4*7);//select alternative function 1, for PA7 in alternative function register low AFR[0]

		
		// SETUP timer 1
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //clock timer 1
		// TIM1->CR1 &= ~TIM_CR1_DIR; //up counting
		TIM1->PSC = 15; //setting prescaler, 16MHz/3000 - 1 =~ 5332
		TIM1->ARR = arr_in; //reset value
		TIM1->CCR1 = 1;	//compare value //50% duty cycle
		TIM1->BDTR |= TIM_BDTR_MOE; //main output enableMOE 0 is disable, 1 is enable. TIM break and dead-time register
		// TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
		
		TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; //select toggle mode
		// TIM1->CCMR1 &= ~TIM_CCER_CC1NP; //select output polarity to active high
		TIM1->CCER |= TIM_CCER_CC1NE; //enable output for channel 1 complementary output
		TIM1->CR1 = TIM_CR1_CEN; //enable timer 1
	}
}


void InitialUSART() {		// initialize usart on pa 9, pa10

		RCC->AHB1ENR |= (1 << 0); // clock port GPIOA
		
		GPIOA->MODER |= (2 << (2*9)); //set pin pa9 as alternative function mode (0b10)
		GPIOA->MODER |= (2 << (2*10)); //set pin pa10 as alternative function mode (0b10)
			
		GPIOA->AFR[1] |= (0x7 << 4*1);		//set GPIOA alternative function register. pin pa9 is set to 0b0111 AF7
		GPIOA->AFR[1] |= (0x7 << 4*2);		//set GPIOA alternative function register. pin pa10 is set to 0b0111 AF7
	
	
		// ////////////////======================================initialize USART1
		
		RCC->APB2ENR |= (1 << 4); // clock USART1
		
		USART1->BRR = 0x683;
		
		USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE); // enable transmit and receive 
			
		USART1->CR1 |= USART_CR1_UE; //enable USART
		
		
		// check USART1->DR and wait 40 ms
		//	eg USART1->DR = 'A';
}


void EXTI15_10_IRQHandler(void){
	if((EXTI->PR & 0x2000) != 0) { 
		GPIOA->ODR |= (1 << 5);
		selection++;
		if (selection > 2) {
			selection = 0;
		}
		HWdelay_ms(500);
		GPIOA->ODR &= ~(1 << 5);
		EXTI->PR |= 0x2000;		
	}
}
	
	
void main() {
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //clock TIM5

	RCC->AHB1ENR |= (1 << 0); //clock gpioa
	RCC->AHB1ENR |= (1 << 2); //clock gpioc

	GPIOA->MODER |= (1 << 2*5); //pa5 output mode
	
	GPIOC->MODER |= (1 << 2*TRIGPIN); //pb8 output mode
	GPIOC->MODER |= (0 << 2*ECHOPIN); //pb9 input mode
	GPIOC->PUPDR |= (2 << 2*ECHOPIN); //pb9 input mode 0b01 is pull up, 0b10 is pull down
	
	
	///////////////////////interrupt configuration
	RCC->APB2ENR |= 0x4000; // clock SYSCFG used to set PORTC to line 13
	
	GPIOC->MODER &= ~(0 << 2*13); //blue button input mode

	///////////////////configure interrupts
	SYSCFG->EXTICR[3] |= 0x20; 
	EXTI->FTSR |= 0x2000; 
	EXTI->IMR |= 0x2000; 
	volatile uint32_t *nvic_enable = (volatile uint32_t *)0xE000E104; 
	*nvic_enable |= 0x100; 
	
	
	
	InitalizePWMPA7(146.83); //initialize square wave sound output for buzzer signal pin
	InitialUSART();	//initialize USART for bluetooth monitor of notes being played
	
	uint32_t distance = 0;
	
	double notes[5] = {110, 123.47, 130.81, 146.83, 164.81};	//A2,B2,C3,D3,E3
	double notessym[5] = {'A', 'B', 'C', 'E', 'D'};	
	
	char letternow = 0;	
	char lastletter = 0;

	double frequ = 0;

	while (1) {
		//distance ranges from 290 to 2400
		distance = CaptureDistance(); // capture distance from ultrasonic distance sensor
		
		if (selection == 1) {//mute
			TIM1->CR1 &= ~TIM_CR1_CEN; //disable timer 1
		}
			else {
				TIM1->CR1 = TIM_CR1_CEN; //enable timer 1
			}
		
		if (!selection) {// divide into 5 frequencies and update bluetooth USART monitor
			if (distance > 1*(FMAX-FMIN)/FSTEP && distance < 2*(FMAX-FMIN)/FSTEP) {
				frequ = notes[0];
				letternow = notessym[0];
			}
			if (distance > 2*(FMAX-FMIN)/FSTEP && distance < 3*(FMAX-FMIN)/FSTEP) {
				frequ = notes[1];
				letternow = notessym[1];
			}
			if (distance > 3*(FMAX-FMIN)/FSTEP && distance < 4*(FMAX-FMIN)/FSTEP) {
				frequ = notes[2];
				letternow = notessym[2];
			}
			if (distance > 4*(FMAX-FMIN)/FSTEP && distance < 6*(FMAX-FMIN)/FSTEP) {
				frequ = notes[3];
				letternow = notessym[3];
			}
			if (distance > 6*(FMAX-FMIN)/FSTEP && distance < 7*(FMAX-FMIN)/FSTEP) {
				frequ = notes[4];
				letternow = notessym[4];
			}
			uint16_t arr = (1000000/(frequ*2))-1;	//calculate ARR register
			TIM1->ARR = arr;	
			
			if (lastletter != letternow) {		//send note letter to serial display, but only do it when the note changes
				USART1->DR = letternow;
				HWdelay_ms(125);
				USART1->DR = '\n';
				lastletter = letternow;
				HWdelay_ms(125);
			}
		}
		

			
		if (selection == 2) {//continuously variable frequency
			uint16_t frequ = (distance);
			uint16_t arr = (1000000/(frequ*2))-1;	
			TIM1->ARR = arr;
		}
		HWdelay_ms(5);
			
	}

}
