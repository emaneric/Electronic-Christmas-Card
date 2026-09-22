#include "main.h"
#include "stm32g030xx.h"

#define PIXEL_TIMER_PERIOD_US 80
#define REFRESHES_PER_FRAME 16   // 16 x 5.12 ms refresh -> ~12 FPS

extern volatile uint32_t tick_ms;

// Row and column masks live in one struct so a single pointer publishes both.
typedef struct {
  uint16_t row[64];
  uint16_t col[64];
} frame_t;

static frame_t frames[2];
static frame_t * volatile active_frame = &frames[0];  // the ISR scans this one

static volatile uint8_t new_frame_flag = 0;

static const uint8_t test_image1[8] = {
  0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00,
};

static const uint8_t dot_frames[][8] = {
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

// A short original silhouette loop: a figure dancing in place.
// The six poses it cycles through (MSB of each byte = column 0):
//
//   arms_down  arms_out  arms_up   hop       lean_left  lean_right
//   ...##...  ...##...  .#....#.  ........  ..##....  ....##..
//   ...##...  ...##...  .#.##.#.  ...##...  ..##....  ....##..
//   ..####..  .######.  ..####..  ...##...  ####....  ....####
//   .#.##.#.  ...##...  ...##...  .######.  ..##....  ....##..
//   .#.##.#.  ...##...  ...##...  ...##...  ..###...  ...###..
//   ...##...  ...##...  ...##...  ...##...  ...##...  ...##...
//   ..#..#..  ..#..#..  ..#..#..  ...##...  ..#..#..  ..#..#..
//   .#....#.  .#....#.  .#....#.  ..#..#..  .#....#.  .#....#.
static const uint8_t dancer_frames[][8] = {
  { 0x18, 0x18, 0x3C, 0x5A, 0x5A, 0x18, 0x24, 0x42 },  //  0 arms_down
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  1 arms_out
  { 0x42, 0x5A, 0x3C, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  2 arms_up
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  3 arms_out
  { 0x18, 0x18, 0x3C, 0x5A, 0x5A, 0x18, 0x24, 0x42 },  //  4 arms_down
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  5 arms_out
  { 0x42, 0x5A, 0x3C, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  6 arms_up
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  7 arms_out
  { 0x30, 0x30, 0xF0, 0x30, 0x38, 0x18, 0x24, 0x42 },  //  8 lean_left
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  //  9 arms_out
  { 0x0C, 0x0C, 0x0F, 0x0C, 0x1C, 0x18, 0x24, 0x42 },  // 10 lean_right
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  // 11 arms_out
  { 0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24 },  // 12 hop
  { 0x42, 0x5A, 0x3C, 0x18, 0x18, 0x18, 0x24, 0x42 },  // 13 arms_up
  { 0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24 },  // 14 hop
  { 0x18, 0x18, 0x7E, 0x18, 0x18, 0x18, 0x24, 0x42 },  // 15 arms_out
};

// An animation is a frame array plus its length, so clips of any size can be
// swapped in without touching the playback loop.
typedef struct {
  const uint8_t (*frames)[8];
  uint16_t count;
} animation_t;

#define ANIMATION(a) { (a), (uint16_t)(sizeof(a) / sizeof((a)[0])) }

enum { CLIP_DANCER, CLIP_BOUNCING_DOT, CLIP_COUNT };

static const animation_t clips[CLIP_COUNT] = {
  [CLIP_DANCER]       = ANIMATION(dancer_frames),
  [CLIP_BOUNCING_DOT] = ANIMATION(dot_frames),
};

static const animation_t *current_anim = &clips[CLIP_BOUNCING_DOT];






void delay_ms(uint32_t ms);
static void generate_ODR_arrays(frame_t *dest, const uint8_t pixel_values[8]);






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
  TIM3->ARR = (PIXEL_TIMER_PERIOD_US / 10) - 1; // 8 counts -> 80 us per LED
  TIM3->CR1 = TIM_CR1_URS;    // only overflow generates update events
  TIM3->EGR = TIM_EGR_UG;     // load PSC/ARR into the shadow registers now
  TIM3->SR = ~TIM_SR_UIF;     // clear the flag UG just set
  TIM3->DIER = TIM_DIER_UIE;  // update interrupt: the CPU drives the pins
  NVIC_EnableIRQ(TIM3_IRQn);

  generate_ODR_arrays(&frames[0], test_image1);

  TIM3->CR1 |= TIM_CR1_CEN;   // start the scan



  frame_t *back_frame = &frames[1];

  while (1)
  {
    static uint16_t frame_index = 0;

    if (new_frame_flag){
      new_frame_flag = 0;

      generate_ODR_arrays(back_frame, current_anim->frames[frame_index]);

      // Publish the finished buffer. The pointer store is a single aligned word,
      // so the ISR sees either the whole old frame or the whole new one.
      __DMB();
      frame_t *retired = active_frame;
      active_frame = back_frame;
      back_frame = retired;

      frame_index++;
      if (frame_index >= current_anim->count){
        frame_index = 0;
      }
    }
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

  static uint8_t pixel_index = 0;
  uint8_t i = pixel_index;
  frame_t *f = active_frame;      // read once: both masks come from one frame

  GPIOA->ODR = 0;                 // blank the anodes before switching rows
  GPIOB->ODR = f->row[i];
  GPIOA->ODR = f->col[i];

  pixel_index = (i + 1) & 63u;

  if (pixel_index == 0){            // a full 64-LED refresh just finished
    static uint8_t refresh_count = 0;
    refresh_count++;
    if (refresh_count == REFRESHES_PER_FRAME){
      refresh_count = 0;
      new_frame_flag = 1;
    }
  }
}

// Rows are the LED cathodes on PB0-PB7, active low.
// Columns are the LED anodes on PA0-PA7, active high.
// MSB of each pixel_values byte is column 0.
static void generate_ODR_arrays(frame_t *dest, const uint8_t pixel_values[8])
{
  for (uint8_t row = 0; row < 8; row++)
  {
    for (uint8_t col = 0; col < 8; col++)
    {
      uint8_t i = (row * 8) + col;

      if ((pixel_values[row] >> (7 - col)) & 1)
      {
        dest->row[i] = 0xFFu & ~(1u << row);   // pull only this cathode low
        dest->col[i] = 1u << col;              // drive only this anode high
      }
      else
      {
        dest->row[i] = 0xFFu;                  // all cathodes high
        dest->col[i] = 0x00u;                  // all anodes low
      }
    }
  }
}
