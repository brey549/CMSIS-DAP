
#ifdef ENABLE_POWER_PROFILING

#include "LPC11Uxx.h"
#include <stdio.h>

#define ADC_DONE              (0x80000000)
#define ADC_CHANNELS          (8)
#define ADC_CLK               (4000000) /* 4MHz */

#define CHANNEL_PIN0          13
#define CHANNEL_PIN1          14

volatile uint8_t power_profiling_current_available = 0;
volatile uint32_t power_profiling_current = 0;

volatile uint8_t power_profiling_channel = 2;      // current measurement channel
volatile uint32_t power_profiling_period = 100000; // us

static void timer_init(void);
static void timer_fini(void);
static void timer_set_period(uint32_t us);
static void adc_init(void);
static void adc_fini(void);
static void channel_init(void);
static void channel_select(uint8_t channel);


void power_profiling_start()
{
    channel_init();
    channel_select(2);
    
    adc_init();
    timer_init();
    timer_set_period(power_profiling_period);
}

void power_profiling_stop()
{
    timer_fini();
    adc_fini();
}

void power_profiling_set_period(uint32_t us)
{
  timer_set_period(us);
    
  power_profiling_period = us;
}

int power_profiling_read(uint8_t *data, uint16_t size) {
    int len = 0;
    if (size < 11) {
        return 0;
    }
    
    if (power_profiling_current_available) {
        uint32_t value = power_profiling_current;
        char buf[10];
        int i;
        
        power_profiling_current_available = 0;
        do {
            buf[len++] = '0' + (value % 10);
            value = value / 10;
        } while (value > 0);
        
        for (i = 0; i < len; i++) {
            data[i] = buf[len - i - 1];
        }
        
        data[len++] = '\n';
    }
    
    return len;
}

void ADC_IRQHandler (void) 
{
    uint16_t value  = (LPC_ADC->DR[1] >> 6) & 0x3FF;
    uint8_t channel = power_profiling_channel;
    uint32_t current;
    
    if (0 == channel) {
        current =  ((uint64_t)(value * 3.3 * 1000 / 2.5) >> 10);
        if (current > 1000) {   // > 1mA
            channel_select(1);
        }
    } else if (1 == channel) {
        current = ((uint64_t)(value * 3.3 * 10000 / 3.0) >> 10);
        if (current > 10000) {  // > 10mA
            channel_select(2);
        } else if (current < 900) {
            channel_select(0);
        }
    } else if (2 == channel) {
        current = ((uint64_t)(value * 3.3 * 100000 / 3.0) >> 10);
        if (current < 9000) {
            channel_select(1);
        }
    }
    
    power_profiling_current = current;
    power_profiling_current_available = 1;
}


static void adc_init(void)
{
  /* Disable Power down bit to the ADC block. */
  LPC_SYSCON->PDRUNCFG &= ~(0x1<<4);

  /* Enable AHB clock to the ADC. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1<<13);

  /* P0.11 = ADC0 */
  // LPC_IOCON->TDI_PIO0_11   &= ~0x9F;
  // LPC_IOCON->TDI_PIO0_11   |= 0x02;

  /* P0.12 = ADC1 */
  LPC_IOCON->TMS_PIO0_12   &= ~0x9F;
  LPC_IOCON->TMS_PIO0_12   |= 0x02;

  /* P0.13 = ADC2 */
  // LPC_IOCON->TDO_PIO0_13   &= ~0x9F;
  // LPC_IOCON->TDO_PIO0_13   |= 0x02;

  /* P0.14 = ADC3 */
  // LPC_IOCON->TRST_PIO0_14  &= ~0x9F;
  // LPC_IOCON->TRST_PIO0_14  |= 0x02;

  /* P0.15 = ADC4 ... this is also SWDIO so be careful with this pin! */
  // LPC_IOCON->SWDIO_PIO0_15 &= ~0x9F;
  // LPC_IOCON->SWDIO_PIO0_15 |= 0x02;

  /* P0.16 = ADC5 */
  // LPC_IOCON->PIO0_16       &= ~0x9F;
  // LPC_IOCON->PIO0_16       |= 0x01;

  /* P0.22 = ADC6 */
  // LPC_IOCON->PIO0_22       &= ~0x9F;
  // LPC_IOCON->PIO0_22       |= 0x01;

  /* P0.23 = ADC7 */
  // LPC_IOCON->PIO0_23       &= ~0x9F;
  // LPC_IOCON->PIO0_23       |= 0x01;


    /* Setup the ADC clock, conversion mode, etc. */
    LPC_ADC->CR = (0x01 << 1)                            |  // select ADC1
                  ((SystemCoreClock / ADC_CLK - 1) << 8) |  // CLKDIV = Fpclk / Fadc - 1
                  (0 << 16)                              |  // BURST = 0, no BURST, software controlled
                  (0 << 19)                              |  // 10 bits (11 clocks)
                  (4 << 24)                              |  // ADC convertion started by CT32B0
                  (0 << 27);                                // EDGE = 0 (CAP/MAT rising edge, trigger A/D conversion)

    NVIC_EnableIRQ(ADC_IRQn);
    LPC_ADC->INTEN = 0x01 << 1;		/* Enable ADC1 interrupt */
}

static void adc_fini()
{
    
    LPC_ADC->CR &= ~(0x7 << 24);	/* stop ADC */ 
    
    /* Enable Power down bit to the ADC block. */
    LPC_SYSCON->PDRUNCFG |= (0x1<<4);

    /* Disable AHB clock to the ADC. */
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 13);
    
    NVIC_DisableIRQ(ADC_IRQn);
}

// set CT32B0 MAT0 to trigger ADC convertion
static void timer_init()
{
    uint32_t ticks;
    
    /* Make sure 32-bit timer 0 is enabled */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);

//   /* Setup the external match register (clear on match) */
//   LPC_CT32B0->EMR =  (1<<10) | (1<<8) | (1<<6) | (1<<4) |
//                      (1<<0)  | (1<<1) | (1<<2) | (1<<3);

  /* Enable PWM function */
  LPC_CT32B0->PWMC = (1 << 3)|(1 << 2)|(1 << 1)|(1 << 0);
    
  /* Reset Functionality on MR3 controlling the PWM period */
  LPC_CT32B0->MCR = 1 << 10;
  
  SystemCoreClockUpdate();
  ticks = (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)power_profiling_period) / (uint64_t)1000000);
  LPC_CT32B0->MR3 = ticks;
    
  LPC_CT32B0->MR0  = LPC_CT32B0->MR3 >> 1;
    
  LPC_CT32B0->TCR = 1;
}

static void timer_set_period(uint32_t us)
{
    uint32_t ticks = (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)us) / (uint64_t)1000000);
    LPC_CT32B0->MR3 = ticks;
    LPC_CT32B0->TCR = (0x3 << 1);  // reset
}

static void timer_fini()
{
    LPC_CT32B0->TCR = 0;
    
    /* Disable AHB clock to the CT32B0 */
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 9);
}

static void channel_init()
{
    // enable clock for GPIO port 0
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6);
    
    LPC_IOCON->TDO_PIO0_13  &= ~0x9F;
    LPC_IOCON->TDO_PIO0_13  |= 0x1;
    
    LPC_IOCON->TRST_PIO0_14 &= ~0x9F;
    LPC_IOCON->TRST_PIO0_14 |= 0x1;

    LPC_GPIO->DIR[0]  |= (1 << CHANNEL_PIN0) | (1 << CHANNEL_PIN1);
}

static void channel_select(uint8_t channel)
{
    if (2 < channel) {
        return;
    }
    
    power_profiling_channel = channel;
    if (0 == channel) {
        LPC_GPIO->CLR[0] |= (1 << CHANNEL_PIN0) | (1 << CHANNEL_PIN1);
    } else if (1 == channel) {
        LPC_GPIO->CLR[0] |= (1 << CHANNEL_PIN0);
        LPC_GPIO->SET[0] |= (1 << CHANNEL_PIN1);
    } else if (2 == channel) {
        LPC_GPIO->SET[0] |= (1 << CHANNEL_PIN0) | (1 << CHANNEL_PIN1);
    }
}

#endif // ENABLE_POWER_PROFILING
