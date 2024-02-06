/*
	KMUTT EDR EPSILON FIRMWARE VERSION 3
	DESIGNED FOR STM32F103 BASE BLDC MOTOR DRIVER BOARD 
*/
#include "stm32f10x.h"

//#define DEBUG

#ifdef DEBUG
	#include "stm32f1uart.h"
#endif

// 9 bit PWM generator module value
// Driver board will send the signal to drive the motor when the pwmIn > PWM_THRESHOLD
#define PWM_LIMIT 408
#define PWM_THRESHOLD 5

//The safety function is designed to prevent the driver from accidentally accelerate the car while starting the car
#define SAFETY_THRESHOLD 30

//Pedal Constant : 12 bit ADC value
#define PEDAL_IDLE	1250
#define PEDAL_MAX		3363

void clockInit (void);
void TIM1_CENTERALIGNED_INIT (unsigned short _ARR, unsigned short _PRE);
void phaseAOut (unsigned short _PWM);
void phaseBOut (unsigned short _PWM);
void phaseCOut (unsigned short _PWM);
void ALow (void);
void BLow (void);
void CLow (void);

int readHallEffectSensor (void);
int map(float inVal, float inMin, float inMax, float outMin, float outMax);
	
void runSequence (int stp, unsigned short pwm_val);
void ADC1_2_IRQHandler(void);
void safety(int threshold);

unsigned long val = 0;

int main (void){
	int step = 0, previousStep = 0, stepOut = 0;
	
	while(!((RCC->CR & RCC_CR_HSIRDY) >> 1));
	RCC->CR |= RCC_CR_HSEON;
	while(!((RCC->CR & RCC_CR_HSERDY) >> 1));
	RCC->CR |= RCC_CR_PLLON;
	while(!((RCC->CR & RCC_CR_PLLRDY) >> 1));
	RCC->CR |= RCC_CR_CSSON;
	
	RCC->CFGR |= RCC_CFGR_PLLSRC;
	RCC->CFGR |= (RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2);
	RCC->CFGR |= RCC_CFGR_PPRE1_2	;
	RCC->CFGR |= RCC_CFGR_SW_1;
	
	RCC->CFGR &= (~(RCC_CFGR_PLLMULL_3 | RCC_CFGR_PLLXTPRE | RCC_CFGR_SW_0 | RCC_CFGR_HPRE | RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE2));
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
	
	//Enable clock for various module
	
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	//TIM1_CENTERALIGNED_INIT(511,2);
	
	// Alternative Function pin remapping 
	// and do partial pins remapping fot TIM1 module 
	AFIO->MAPR |= (AFIO_MAPR_TIM1_REMAP_0);
	AFIO->MAPR &= (~(AFIO_MAPR_TIM1_REMAP_1));
	
	
/*
	Configure GPIOA8 GPIOA9 GPIOA10 as the high-side-output motor driving signals by using 
	GPIOA8  >> as phase A high output
	GPIOA9  >> as phase B high output
	GPIOA10 >> as phase C high output
	*/
	GPIOA->CRH |= (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1);
	GPIOA->CRH &= ~(GPIO_CRH_CNF8_0);
	GPIOA->CRH |= (GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1);
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0);
	GPIOA->CRH |= (GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1);
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);
	
	
	// Configure GPIOB12 GPIOB13 GPIOB14 as input for reading hall-efffect sensor signals.
	GPIOB->CRH |= GPIO_CRH_CNF12_0;
	GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12_1);
	GPIOB->CRH |= GPIO_CRH_CNF13_0;
	GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13_1);
	GPIOB->CRH |= GPIO_CRH_CNF14_0;
	GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14_1);	
	
	
/*
	Configure GPIOA7 GPIOB0 GPIOB1 as the low-side-output motor driving signals by using 
	GPIOA7  >> as phase A low output
	GPIOB0  >> as phase B low output
	GPIOB1  >> as phase C low output
	*/	
	GPIOA->CRL |= (GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF7_0);
	GPIOB->CRL |= (GPIO_CRL_MODE0 | GPIO_CRL_CNF0_1);
	GPIOB->CRL &= ~GPIO_CRL_CNF0_0;
	GPIOB->CRL |= (GPIO_CRL_MODE1 | GPIO_CRL_CNF1_1);
	GPIOB->CRL &= ~GPIO_CRL_CNF1_0;
	
	
// Configure GPIOC13 as running indicator (GPIOC13 is connected to built in led on STM32 bluepill)
	GPIOC->CRH &= ~GPIO_CRH_CNF13;
	GPIOC->CRH |= GPIO_CRH_MODE13;
	GPIOC->BSRR = GPIO_BSRR_BS13;

	/*GPIOB->CRH &= ~GPIO_CRH_CNF8;
	GPIOB->CRH |= GPIO_CRH_MODE8;
	GPIOB->BSRR = GPIO_BSRR_BR8;*/
	
	
	
	
	// Prepare timer1 module for pwm mode
	TIM1->PSC = 2;
	TIM1->ARR = 511;
	TIM1->CR1 |= (TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CEN);
	TIM1->CR1 &= (uint32_t)(~(TIM_CR1_CKD | TIM_CR1_CMS_1 | TIM_CR1_DIR));
	
	TIM1->EGR |= (TIM_EGR_UG);
	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
	TIM1->CCMR1 &= (~(TIM_CCMR1_OC1M_0 | TIM_CCMR1_CC1S));
	TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE);
	TIM1->CCMR1 &= (~(TIM_CCMR1_OC2M_0 | TIM_CCMR1_CC2S));
	TIM1->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE);
	TIM1->CCMR2 &= (~(TIM_CCMR2_OC3M_0 | TIM_CCMR2_CC3S));

	TIM1->BDTR |= (TIM_BDTR_MOE | TIM_BDTR_AOE );
	TIM1->BDTR &= (~(TIM_BDTR_OSSR));
	
	TIM1->BDTR &= (~(TIM_BDTR_DTG_7));
	TIM1->BDTR |= (TIM_BDTR_DTG_0 | TIM_BDTR_DTG_2 | TIM_BDTR_DTG_3 |TIM_BDTR_DTG_4 | TIM_BDTR_DTG_6 );
	
	TIM1->CCER &= (~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC3P | TIM_CCER_CC3NP));

	TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
	TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
	
	
	//Init ADC
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0);
	NVIC_EnableIRQ(ADC1_2_IRQn);
	ADC1->CR1 |= (ADC_CR1_EOCIE);
	ADC1->SMPR2 |= (ADC_SMPR2_SMP0);
	ADC1->SQR1 &= ~(ADC_SQR1_L);
	ADC1->SQR3 &= ~(ADC_SQR3_SQ1);
	ADC1->CR2 &= ~(ADC_CR2_ALIGN);
	ADC1->CR2 |= (ADC_CR2_ADON | ADC_CR2_CONT);
	
	for(int i = 0; i < 150000; i++);
	ADC1->CR2 |= (ADC_CR2_ADON);  
	ADC1->CR2 |= (ADC_CR2_CAL);
	while(ADC1->CR2 & ADC_CR2_CAL);
	GPIOC->BSRR = (GPIO_BSRR_BR13);
	for(int i = 0; i < 150000; i++);
	
	#ifdef DEBUG
		seriailInit(72000000UL,9600UL);
	#endif

	int mappedPWM = 0;
	
	safety(SAFETY_THRESHOLD);
	GPIOC->BSRR = (GPIO_BSRR_BR13);
	while(1){
		#ifdef DEBUG
			serialprintf("%d step > %d\n",mappedPWM,stepOut);
		#endif
		step = readHallEffectSensor();
		/*if(step != previousStep){
			stepOut = step;
			previousStep = step;
		}*/
			mappedPWM = map(val,PEDAL_IDLE,PEDAL_MAX,0,PWM_LIMIT);
			runSequence(step,mappedPWM);	
	}
}

void clockInit (void){
	RCC->CR |= (RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);
	RCC->CFGR |= (RCC_CFGR_PLLMULL_0 | RCC_CFGR_PLLMULL_1 | RCC_CFGR_PLLMULL_2 | RCC_CFGR_PLLSRC | RCC_CFGR_SW_1 | RCC_CFGR_PPRE1_2);
	RCC->CFGR &= (~(RCC_CFGR_PLLMULL_3 | RCC_CFGR_PLLXTPRE | RCC_CFGR_SW_0 | RCC_CFGR_HPRE | RCC_CFGR_PPRE1_0 | RCC_CFGR_PPRE1_1 | RCC_CFGR_PPRE2));
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

}

void TIM1_CENTERALIGNED_INIT (unsigned short _ARR, unsigned short _PRE){
	TIM1->PSC = _PRE;
	TIM1->ARR = _ARR;
	TIM1->CR1 |= (TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CEN);
	TIM1->CR1 &= (uint32_t)(~(TIM_CR1_CKD | TIM_CR1_CMS_1 | TIM_CR1_DIR));

}

void phaseAOut(unsigned short _PWM){
	if(_PWM >= PWM_THRESHOLD && _PWM <= PWM_LIMIT){
			TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
			TIM1->CCR1 = _PWM;
	}
	else
		TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
	
}

void phaseBOut(unsigned short _PWM){
	if(_PWM >= PWM_THRESHOLD && _PWM <= PWM_LIMIT){
			TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
			TIM1->CCR2 = _PWM;
	}
	else
		TIM1->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2NE);
	
}
void phaseCOut(unsigned short _PWM){
	if(_PWM >= PWM_THRESHOLD && _PWM <= PWM_LIMIT){
			TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
			TIM1->CCR3 = _PWM;
	}
	else
		TIM1->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3NE);
	
}

void ALow (void){
			TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
			TIM1->CCR1 = 0;
}

void BLow (void){
			TIM1->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
			TIM1->CCR2 = 0;
}

void CLow (void){
			TIM1->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
			TIM1->CCR3 = 0;
}

int readHallEffectSensor (void){
	unsigned short step = 0;
	unsigned short readVal = ((GPIOB->IDR) >> 12) & 7;
		switch (readVal)
	{
		case 5:
		step = 1;
		break;
		case 1:
		step = 2;
		break;
		case 3:
		step = 3;
		break;
		case 2:
		step = 4;
		break;
		case 6:
		step = 5;
		break;
		case 4:
		step = 6;
		break;
		default:
		step = 0;
	}
	return step;
}

void runSequence (int stp, unsigned short pwm_val){
	if (pwm_val >= PWM_THRESHOLD)
	{
		switch (stp)
		{
			case 1:
				ALow();
				phaseBOut(pwm_val);
				phaseCOut(0);
			break;
			case 2:
				ALow();
				phaseBOut(0);
				phaseCOut(pwm_val);
			break;
			case 3:
				phaseAOut(0);
				BLow();
				phaseCOut(pwm_val);
			break;
			case 4:
				phaseAOut(pwm_val);
				BLow();
				phaseCOut(0);
			break;
			case 5:
				phaseAOut(pwm_val);
				phaseBOut(0);
				CLow();
			break;
			case 6:
				phaseAOut(0);
				phaseBOut(pwm_val);
				CLow();
			break;
			default:
				phaseAOut(0);
				phaseBOut(0);
				phaseCOut(0);
			
		}
	}
	else{
		phaseAOut(0);
		phaseBOut(0);
		phaseCOut(0);		
	}
}

void ADC1_2_IRQHandler(void){
	if(ADC1->SR & ADC_SR_EOC){
		val = ADC1->DR;
	}
}

int map(float inVal, float inMin, float inMax, float outMin, float outMax){
	int retval = 0;
	retval = (int)((((inVal-inMin)*(outMax-outMin))/(inMax-inMin))+outMin);
	if(retval < outMin)
		retval = outMin;
	else if (retval > outMax)
		retval = outMax;
	return retval;
}

void safety(int threshold){
	int pwm = map(val,PEDAL_IDLE,PEDAL_MAX,0,PWM_LIMIT);
	GPIOC->BSRR = (GPIO_BSRR_BS13);
		phaseAOut(0);
		phaseBOut(0);
		phaseCOut(0);
	while(pwm > threshold){
		GPIOC->BSRR = (GPIO_BSRR_BS13);
		pwm = map(val,PEDAL_IDLE,PEDAL_MAX,0,PWM_LIMIT);
		#ifdef DEBUG
			serialprintf("Safety\n");
		#endif
	}
	GPIOC->BSRR = (GPIO_BSRR_BR13);
}

