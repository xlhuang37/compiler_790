#include <stdint.h>

extern unsigned long _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void Reset_Handler(void);
void Default_Handler(void){ for(;;){} }

/* vector table */
__attribute__((section(".isr_vector")))
void (* const vector_table[])(void) = {
    (void (*)(void))(&_estack), /* initial SP */
    Reset_Handler,              /* Reset */
    Default_Handler,            /* NMI */
    Default_Handler,            /* HardFault */
    /* keep the rest as Default_Handler for a minimal demo */
};

void Reset_Handler(void){
    /* copy .data from FLASH to RAM */
    unsigned long *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    /* zero .bss */
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    main();
    for(;;){}
}

