#include "main.h"

extern volatile uint32_t tick_ms;

void delay_ms(uint32_t ms);

int main(void)
{  

  SysTick_Config(16000000 / 1000);   // 16 MHz HSI / 1000 = interrupt every 1 ms

  //Enable IO port clock for GPIO ports A, B, D
  RCC->IOPENR = 0b1011;
  (void)RCC->IOPENR;

  //Set PA0-PA7, PB0-PB7 GPIO to output 
  GPIOA->MODER = (GPIOA->MODER & ~0xFFFFu) | 0x5555;
  GPIOB->MODER = (GPIOB->MODER & ~0xFFFFu) | 0x5555;

  //Set PD0, PD1, PA15 to input
  GPIOD->MODER = GPIOD->MODER & ~0xFu;
  GPIOA->MODER = GPIOA->MODER & ~0xC0000000u;

  



  while (1)
  {
    //LED ON = PA0 high PB0 low
    GPIOA->BSRR = 1u<<0;
    GPIOB->BSRR = 0b11111110 | 1u<<16;
    delay_ms(1000);
    //LED OFF = PA0 low PB0 low
    GPIOA->BSRR = 1u<<16;
    delay_ms(1000);
  }
}





void delay_ms(uint32_t ms)
{
  uint32_t start = tick_ms;
  while ((tick_ms - start) < ms) { }
}
