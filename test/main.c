#include <stdint.h>

#define RCC_BASE        0x40021000UL
#define RCC_AHB2ENR     (*(volatile uint32_t *)(RCC_BASE + 0x4C))

#define GPIOA_BASE      0x48000000UL
#define GPIOA_MODER     (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR       (*(volatile uint32_t *)(GPIOA_BASE + 0x14))

static void delay(volatile uint32_t t){ while(t--) __asm__ volatile("nop"); }

int main(void) {
    /* enable GPIOA clock */
    RCC_AHB2ENR |= (1u << 0);

    /* PA5 (LD2 LED) as general purpose output */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (5*2))) | (1u << (5*2));

    for(;;){
        GPIOA_ODR ^= (1u << 5);  // toggle LED
        delay(200000);
    }
}

