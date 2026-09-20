#include "main.h"
#include "stm32g030xx.h"

extern volatile uint32_t tick_ms;
static volatile uint16_t row_frame[64] = {0};
static volatile uint16_t col_frame[64] = {0};
static volatile uint8_t pixel_index = 0;


static const uint8_t test_image1[8] = {
  0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
};

static const uint8_t animation[24][8] = {
  { 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  0: (x=0, y=0)
  { 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  1: (x=1, y=0)
  { 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  2: (x=2, y=0)
  { 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  3: (x=3, y=0)
  { 0x0C, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  4: (x=4, y=0)
  { 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  5: (x=5, y=0)
  { 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  6: (x=6, y=0)
  { 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 },  //  7: (x=6, y=1)
  { 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00 },  //  8: (x=6, y=2)
  { 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00 },  //  9: (x=6, y=3)
  { 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00 },  // 10: (x=6, y=4)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00 },  // 11: (x=6, y=5)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03 },  // 12: (x=6, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06 },  // 13: (x=5, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C },  // 14: (x=4, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18 },  // 15: (x=3, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30 },  // 16: (x=2, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60 },  // 17: (x=1, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0 },  // 18: (x=0, y=6)
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00 },  // 19: (x=0, y=5)
  { 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00 },  // 20: (x=0, y=4)
  { 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00 },  // 21: (x=0, y=3)
  { 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00 },  // 22: (x=0, y=2)
  { 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00 },  // 23: (x=0, y=1)
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
  TIM3->PSC = 160 - 1;        // 16 MHz / 160 = 100 kHz -> 10 us per count
  TIM3->ARR = 8 - 1;          // 8 counts -> 80 us per LED, 64 LEDs -> 5.12 ms/frame
  TIM3->CR1 = TIM_CR1_URS;    // only overflow generates update events
  TIM3->EGR = TIM_EGR_UG;     // load PSC/ARR into the shadow registers now
  TIM3->SR = ~TIM_SR_UIF;     // clear the flag UG just set
  TIM3->DIER = TIM_DIER_UIE;  // update interrupt: the CPU drives the pins
  NVIC_EnableIRQ(TIM3_IRQn);

  // The DMA cannot be used here: on the STM32G0x0 the GPIO ports sit on the
  // Cortex-M0+ IOPORT bus, which is reachable by the core only. The bus matrix
  // slaves are SRAM, flash and the AHB-to-APB bridge (RM0454 section 2.1), so a
  // DMA write to GPIOx->ODR faults instead of driving the pins.

  generate_ODR_arrays(test_image1);

  TIM3->CR1 |= TIM_CR1_CEN;   // start the scan



  while (1)
  {
    static uint8_t frame_index = 0;
    generate_ODR_arrays(animation[frame_index]);
    delay_ms(40);
    frame_index++;
    if (frame_index == 24){
      frame_index = 0;
    }

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

// Lights one LED per update event, stepping through the frame buffers.
void TIM3_IRQHandler(void)
{
  TIM3->SR = ~TIM_SR_UIF;

  uint8_t i = pixel_index;

  GPIOA->ODR = 0;                 // blank the anodes before switching rows
  GPIOB->ODR = row_frame[i];
  GPIOA->ODR = col_frame[i];

  pixel_index = (i + 1) & 63u;
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
