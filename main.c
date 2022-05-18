#include "stm32f0xx.h"

#include <stdint.h>
#include <string.h>

#define TEMP_CMD_SEND_DATA 0x7E
#define TEMP_SAMPLES_COUNT 128


#define TEMP_TS_CAL_1_ADDR (uint8_t*)0x1FFFF7B8
#define TEMP_TS_CAL_2_ADDR (uint8_t*)0x1FFFF7C2

#define TEMP_FIXED_POINT_FACTOR 100

uint8_t adc_data[TEMP_SAMPLES_COUNT];
int16_t temperatures[TEMP_SAMPLES_COUNT];
int16_t calibration_factor;
const unsigned char error = 0xFF;

void usart_transmit(const unsigned char* buffer, uint16_t bytes_count)
{
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CMAR = (uint32_t)buffer;
	DMA1_Channel2->CNDTR = bytes_count;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
}

void USART1_IRQHandler(void) {
	uint16_t value;
	if (USART1->ISR & USART_ISR_RXNE) {
		value = USART1->RDR;
		if (value != TEMP_CMD_SEND_DATA)
		{
			usart_transmit(&error, 1);
		}
		else
		{
			usart_transmit((const unsigned char*)temperatures, sizeof temperatures);
		}
	}
	else
	{
		// Error
		value = USART1->RDR;
	}
}

void usart_init(void)
{
	// Настройка GPIO
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;
	GPIOA->AFR[1] |= 0x00000110;

	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR |= 69;  // 115200
	USART1->CR3 |= USART_CR3_DMAT | USART_CR3_EIE;
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn, 3);
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->CR1 |= USART_CR1_UE;
}

void temp_init(void)
{
	// Настройка DMA (8-bit MSIZE/PSIZE, из периферии в память)
	RCC->AHBENR |= RCC_AHBENR_DMAEN;
	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE;
	DMA1_Channel1->CMAR = (uint32_t)adc_data;
	DMA1_Channel1->CNDTR = TEMP_SAMPLES_COUNT;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CCR |= DMA_CCR_EN;

	// Настройка АЦП
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	ADC1->SMPR |= ADC_SMPR1_SMPR;
	ADC->CCR |= ADC_CCR_TSEN;
	ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0 | ADC_CFGR1_EXTSEL_1
		| ADC_CFGR1_RES_0
		| ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL16;
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}
	ADC1->CR |= ADC_CR_ADSTART;

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 3999;
	TIM2->ARR = 1;
	TIM2->CR2 |= TIM_CR2_MMS_1;
	TIM2->CR1 |= TIM_CR1_CEN;
}

int16_t temp_calibrate(void)
{
	const int16_t ts_cal_delta = *TEMP_TS_CAL_2_ADDR - *TEMP_TS_CAL_1_ADDR;
	return (110 - 30) * TEMP_FIXED_POINT_FACTOR / ts_cal_delta;
}

int16_t temp_convert(uint8_t raw)
{
	const uint8_t ts_cal_1 = *TEMP_TS_CAL_1_ADDR;
	return calibration_factor * ((int16_t)raw - ts_cal_1) + 30 * TEMP_FIXED_POINT_FACTOR;
}

void DMA1_Channel1_IRQHandler(void)
{
	uint16_t idx = 0;
	while(idx < TEMP_SAMPLES_COUNT)
	{
		temperatures[idx] = temp_convert(adc_data[idx]);
		++idx;
	}
}

int main(void)
{
	calibration_factor = temp_calibrate();
	temp_init();
	usart_init();
	return 0;
}
