#include "main.h"
#include "stm32g030xx.h"

#define DMAMUX_REQ_TIM3_UP  37u

extern volatile uint32_t tick_ms;
static volatile uint16_t row_frame[64] = {0};
static volatile uint16_t col_frame[64] = {0};
static const uint8_t test_image1[8] = {
  0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
};

void delay_ms(uint32_t ms);
static void generate_ODR_arrays(const uint8_t pixel_values[8]);

int main(void)
{  

  SysTick_Config(16000000 / 1000);   // 16 MHz HSI / 1000 = interrupt every 1 ms

  //Enable IO port clock for GPIO ports A, B, D
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIODEN;
  (void)RCC->IOPENR;
  //Set PA0-PA7, PB0-PB7 GPIO to output 
  GPIOA->MODER = (GPIOA->MODER & ~0xFFFFu) | 0x5555;
  GPIOB->MODER = (GPIOB->MODER & ~0xFFFFu) | 0x5555;

  //Set PD0, PD1, PA15 to input
  GPIOD->MODER = GPIOD->MODER & ~0xFu;
  GPIOA->MODER = GPIOA->MODER & ~0xC0000000u;

  //Timer 3: 10 us tick, 80 us period
  RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
  (void)RCC->APBENR1;
  TIM3->PSC = 160 - 1;            // 16 MHz / (159 + 1) = 100 kHz -> 10 us per count
  TIM3->ARR = 8 - 1;             // 
  TIM3->CR1 = TIM_CR1_URS;    // only overflow generates update events
  TIM3->EGR = TIM_EGR_UG;     // load PSC/ARR into the shadow registers now
  TIM3->SR = ~TIM_SR_UIF;     // clear the flag UG just set
  TIM3->DIER = TIM_DIER_UDE;  // update DMA request (bit 8)
  
  //DMA setup
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  (void)RCC->AHBENR;

  //DMA1 Channel 1: memory -> GPIOB->ODR
  DMA1_Channel1->CCR = 0;                       // EN must be 0 to configure
  DMA1_Channel1->CPAR  = (uint32_t)&GPIOB->ODR;
  DMA1_Channel1->CMAR  = (uint32_t)row_frame;
  DMA1_Channel1->CNDTR = 64; //64 transfers for a full frame
  DMA1_Channel1->CCR =
        DMA_CCR_DIR                             // read from memory -> peripheral
      | DMA_CCR_MINC                            // step through the array
      | DMA_CCR_CIRC                            // auto-reload, repeats forever
      | DMA_CCR_PSIZE_0                         // 16-bit peripheral (ODR)
      | DMA_CCR_MSIZE_0;                        // 16-bit memory

  //DMAMUX: feed channel 1 from TIM3 update
  DMAMUX1_Channel0->CCR = DMAMUX_REQ_TIM3_UP;   // Channel0 == DMA1_Channel1

  //DMA1 Channel 2: memory -> GPIOA->ODR
  DMA1_Channel2->CCR = 0;
  DMA1_Channel2->CPAR  = (uint32_t)&GPIOA->ODR;
  DMA1_Channel2->CMAR  = (uint32_t)col_frame;
  DMA1_Channel2->CNDTR = 64;
  DMA1_Channel2->CCR =
        DMA_CCR_DIR
      | DMA_CCR_MINC
      | DMA_CCR_CIRC
      | DMA_CCR_PSIZE_0
      | DMA_CCR_MSIZE_0;

  //DMAMUX: feed channel 2 from the same TIM3 update
  DMAMUX1_Channel1->CCR = DMAMUX_REQ_TIM3_UP;


  generate_ODR_arrays(test_image1);


  //Arm both, then start the timer so they stay in step
  DMA1_Channel1->CCR |= DMA_CCR_EN;
  DMA1_Channel2->CCR |= DMA_CCR_EN;
  TIM3->CR1 |= TIM_CR1_CEN;



  while (1)
  {
    delay_ms(1);
    //__NOP();
    // //LED ON = PA0 high PB0 low
    // GPIOA->BSRR = 1u<<0;
    // GPIOB->BSRR = 0b11111110 | 1u<<16;
    // delay_ms(1000);
    // //LED OFF = PA0 low PB0 low
    // GPIOA->BSRR = 1u<<16;
    // delay_ms(1000);
  }
}





void delay_ms(uint32_t ms)
{
  uint32_t start = tick_ms;
  while ((tick_ms - start) < ms) { }
}

// Rows are the LED cathodes on PB0-PB7, active low.
// Columns are the LED anodes on PA0-PA7, active high.
// MSB of each pixel_values byte is column 0.
static void generate_ODR_arrays(const uint8_t pixel_values[8])
{
  for (uint8_t row = 0; row < 8; row++)
  {
    for (uint8_t col = 0; col < 8; col++)
    {
      uint8_t i = (row * 8) + col;

      if ((pixel_values[row] >> (7 - col)) & 1)
      {
        row_frame[i] = 0xFFu & ~(1u << row);   // pull only this cathode low
        col_frame[i] = 1u << col;              // drive only this anode high
      }
      else
      {
        row_frame[i] = 0xFFu;                  // all cathodes high
        col_frame[i] = 0x00u;                  // all anodes low
      }
    }
  }
}
