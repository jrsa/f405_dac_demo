#include "stm32f4xx.h"
#include "stm32f4xx_dac.h"

#define LED_PORT GPIOA
#define LED_PIN 5

#define LED_TOGGLE GPIOA->ODR ^= (1 << LED_PIN)

void led_setup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    LED_PORT->MODER |= (1 << (LED_PIN << 1));
    LED_PORT->OSPEEDR |= (3 << (LED_PIN << 1));
}

void tim2_setup(void)
{
    // enable GPIOA and TIM2 in the RCC system
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOA->MODER |= 0x2; // PA0 in alternate function mode

    // clear pin 0 (first bit) alternate function and initialize to AF01 (TIM2 CH1)
    GPIOA->AFR[0] |= (uint8_t)1;

    // configure timer
    TIM2->PSC = 1; // clear prescaler
    TIM2->ARR = 4000; // set period (Auto Reload Register)

    TIM2->CR2 &= (uint16_t)~TIM_CR2_MMS; // 
    TIM2->CR2 |= 0x20; 

    // configure capture/compare 1
    TIM2->CCMR1 |= 0x60; // PWM1 mode
    
    // pulse width in cycles
    TIM2->CCR1 = 2000;

    // enable cc1
    TIM2->CCER |= 1;

    // enable TIM1
    TIM2->CR1 |= TIM_CR1_CEN;
}

void sp_dac_setup(void)
{
    DAC_InitTypeDef dac_init;
    GPIO_InitTypeDef gpio_init;

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    gpio_init.GPIO_Pin = GPIO_Pin_4;
    gpio_init.GPIO_Mode = GPIO_Mode_AN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio_init);

    DAC_StructInit(&dac_init);

    dac_init.DAC_Trigger = DAC_Trigger_T2_TRGO;
    dac_init.DAC_WaveGeneration = DAC_WaveGeneration_Noise;
    dac_init.DAC_LFSRUnmask_TriangleAmplitude = 0xb00;
    dac_init.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &dac_init);

    DAC_Cmd(DAC_Channel_1, ENABLE);
}

void main(void)
{
    led_setup();
    tim2_setup();

    sp_dac_setup();

    while (1) {
    }
}
