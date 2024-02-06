#include "stm32f1uart.h"

void seriailInit(unsigned long F_CPU,unsigned long baud){
	unsigned long brrVal = 0;
	unsigned long temp = 0;
	double fractional_part = 0;
	
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1;
	GPIOB->CRH |= GPIO_CRH_CNF10_1;
	GPIOB->CRH &= ~GPIO_CRH_CNF10_0;
	
	USART3->CR1 |= USART_CR1_UE;
	USART3->CR1 &= ~USART_CR1_M;
	USART3->CR2 &= ~USART_CR2_STOP;
	USART3->CR1 |= USART_CR1_TE;
	
	brrVal = ((F_CPU/(32*baud)) << 4);
	temp = F_CPU%(32*baud);
	fractional_part = temp/baud;
	brrVal = brrVal + fractional_part;
	USART3->BRR = brrVal; 
}

void serialprintf(const char *stream ,...){
	char buffer[255];
	va_list args;
	va_start(args,stream);
	vsnprintf(buffer,sizeof(buffer),stream,args);
	va_end(args);
	for(unsigned int i = 0; i < strlen(buffer);i++){
		USART3->DR = buffer[i];
		while(!((USART3->SR & USART_SR_TC) >> 6));
	}
}

