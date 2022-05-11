#include "stm32f0xx.h"

#define SIZE_ADC 1024
uint16_t adc_data[SIZE_ADC];


float aver_left, aver_right, aver_all;

void DMA1_Channel1_IRQHandler() {
	if ((DMA1->ISR & DMA_ISR_HTIF1) == DMA_ISR_HTIF1) {
		GPIOC->BSRR = GPIO_BSRR_BS_8;
		DMA1->IFCR |= DMA_IFCR_CHTIF1;
		///@todo

		//here result of aver_left
	}

	if ((DMA1->ISR & DMA_ISR_TCIF1) == DMA_ISR_TCIF1) {
//		GPIOC->BSRR = GPIO_BSRR_BS_8;
		DMA1->IFCR |= DMA_IFCR_CTCIF1;
		///@todo
		//here result of aver_right

//		aver_all = (aver_left + aver_right)/ 2;
		GPIOC->BSRR = GPIO_BSRR_BR_8;
	}


}

uint8_t byteRx;

void bufferPutToEnd() {

}

void USART1_IRQHandler() {
	if ((USART1->ISR & USART_ISR_TXE) == USART_ISR_TXE) {
		static uint8_t byte = 0;
		USART1->TDR = byte++;
	}

	if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		byteRx = USART1->RDR;
		bufferPutToEnd();
	}
}

void init_tim3 (){
	//RCC->APB1ENR
}

void adc_init_cont() {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->CR2 |= RCC_CR2_HSI14ON;

	ADC1->CHSELR = ADC_CHSELR_CHSEL0;// | ADC_CHSELR_CHSEL1;	//adc_in0 - pa0, adc_in1 - pa1
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER0 /*GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1*/ |
					GPIO_MODER_MODER1;	//analog function for pa0, pa1

	ADC1->IER |= ADC_IER_EOCIE;
	ADC1->IER |= ADC_IER_EOSEQIE;
	NVIC_SetPriority(ADC1_COMP_IRQn, 5);
	NVIC_EnableIRQ(ADC1_COMP_IRQn);

	ADC1->CFGR1 |= ADC_CFGR1_CONT;

	ADC1->CR |= ADC_CR_ADEN;

	// DMA settings

	ADC1->CFGR1 |= ADC_CFGR1_DMACFG;	//circular mode
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN;		//enable dma...

	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;	//disable overrun

	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	DMA1_Channel1->CCR |= DMA_CCR_HTIE | DMA_CCR_TCIE;
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;	//16 bit
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
	DMA1_Channel1->CCR |= DMA_CCR_MINC;
	DMA1_Channel1->CCR &= ~DMA_CCR_DIR;	//per -> mem
	DMA1_Channel1->CCR |= DMA_CCR_CIRC;

	DMA1_Channel1->CMAR = (uint32_t)(&adc_data[0]);
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	DMA1_Channel1->CNDTR = SIZE_ADC;

	NVIC_SetPriority(DMA1_Ch1_IRQn, 10);
	NVIC_EnableIRQ(DMA1_Ch1_IRQn);

	DMA1_Channel1->CCR |= DMA_CCR_EN;
}


void init_gpio_as_AF_for_usart() {
	//1
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	//2
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->AFR[1] |=  (1 << 4)|(1 << 8);	//AF1

	//3 - turn on peripheral...
}


void init_usart1() {


	init_gpio_as_AF_for_usart();

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->CR1 |= USART_CR1_RE; //enable receive
	USART1->CR1 |= USART_CR1_TE; //transmit enable
	USART1->CR2 &= ~USART_CR2_STOP; //1 stop bit

	//init interruption
	USART1->CR1 |= USART_CR1_TXEIE;
	USART1->BRR = SystemCoreClock / 8000;
	USART1->CR1 |= USART_CR1_UE;

	NVIC_SetPriority(USART1_IRQn, 2);

	NVIC_EnableIRQ(USART1_IRQn);


}

uint32_t data[256];
void init_usart_dma() {

	for (int i = 0; i < 256; i++) {
		data[i] = i;
	}

//0 (3)
	init_gpio_as_AF_for_usart();
	//1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	//2

	USART1->CR1 |= USART_CR1_RE;	//enable receive
	USART1->CR1 |= USART_CR1_TE;	//transmit enable
	USART1->CR2 &= ~USART_CR2_STOP;	//1 stop bit

	USART1->BRR = SystemCoreClock / 115200;
	USART1->CR3 |= USART_CR3_DMAT;	//allow usart to work via DMA


	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	DMA1_Channel2->CPAR = (uint32_t)(&(USART1->TDR));
	DMA1_Channel2->CMAR = (uint32_t)(&data[0]);
	DMA1_Channel2->CNDTR = 256;
	DMA1_Channel2->CCR |= DMA_CCR_TCIE;	//interruption
	DMA1_Channel2->CCR |= DMA_CCR_DIR;	//read from memory
	DMA1_Channel2->CCR |= DMA_CCR_MINC;
//	DMA1_Channel2->CCR |= DMA_CCR_PSIZE | DMA_CCR_MSIZE;	size of data

	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 9);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);


	DMA1_Channel2->CCR |= DMA_CCR_EN;
	//4
	USART1->CR1 |= USART_CR1_UE;
}


void adc_start() {
	ADC1->CR |= ADC_CR_ADSTART;
}

void adc_stop() {
	ADC1->CR |= ADC_CR_ADSTP;
}


int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	init_usart1();
	init_usart_dma();
	adc_init_cont();
	adc_start();



  while (1)
  {

  }
}
