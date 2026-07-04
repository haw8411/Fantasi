/* Fantasi / Chameleon Ultra (nRF52840) startup
 *
 * See linker.ld for why we link at 0x27000 and leave the SoftDevice
 * alone. The first thing Reset_Handler does is retarget VTOR to our
 * table; the SoftDevice's forwarding table is therefore bypassed and
 * SVC/PendSV/SysTick go directly to FreeRTOS handlers. */

#include <stdint.h>
#include "nrf.h"

extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss,  _ebss;
extern uint32_t _stack_end;

extern int  main(void);
extern void SystemInit(void);

void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK __attribute__((weak, alias("Default_Handler")))
WEAK void NMI_Handler(void);
void HardFault_Handler(void);
void MemoryManagement_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
WEAK void DebugMon_Handler(void);

/* nRF52840 peripheral IRQs in nrf52840.h order. */
WEAK void POWER_CLOCK_IRQHandler(void);
WEAK void RADIO_IRQHandler(void);
WEAK void UARTE0_UART0_IRQHandler(void);
WEAK void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void);
WEAK void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void);
WEAK void NFCT_IRQHandler(void);
WEAK void GPIOTE_IRQHandler(void);
WEAK void SAADC_IRQHandler(void);
WEAK void TIMER0_IRQHandler(void);
WEAK void TIMER1_IRQHandler(void);
WEAK void TIMER2_IRQHandler(void);
WEAK void RTC0_IRQHandler(void);
WEAK void TEMP_IRQHandler(void);
WEAK void RNG_IRQHandler(void);
WEAK void ECB_IRQHandler(void);
WEAK void CCM_AAR_IRQHandler(void);
WEAK void WDT_IRQHandler(void);
WEAK void RTC1_IRQHandler(void);
WEAK void QDEC_IRQHandler(void);
WEAK void COMP_LPCOMP_IRQHandler(void);
WEAK void SWI0_EGU0_IRQHandler(void);
WEAK void SWI1_EGU1_IRQHandler(void);
WEAK void SWI2_EGU2_IRQHandler(void);
WEAK void SWI3_EGU3_IRQHandler(void);
WEAK void SWI4_EGU4_IRQHandler(void);
WEAK void SWI5_EGU5_IRQHandler(void);
WEAK void TIMER3_IRQHandler(void);
WEAK void TIMER4_IRQHandler(void);
WEAK void PWM0_IRQHandler(void);
WEAK void PDM_IRQHandler(void);
WEAK void MWU_IRQHandler(void);
WEAK void PWM1_IRQHandler(void);
WEAK void PWM2_IRQHandler(void);
WEAK void SPIM2_SPIS2_SPI2_IRQHandler(void);
WEAK void RTC2_IRQHandler(void);
WEAK void I2S_IRQHandler(void);
WEAK void FPU_IRQHandler(void);
WEAK void USBD_IRQHandler(void);
WEAK void UARTE1_IRQHandler(void);
WEAK void QSPI_IRQHandler(void);
WEAK void CRYPTOCELL_IRQHandler(void);
WEAK void PWM3_IRQHandler(void);
WEAK void SPIM3_IRQHandler(void);

__attribute__((section(".isr_vector"), used))
void (* const g_vector_table[])(void) = {
    (void (*)(void))&_stack_end,
    Reset_Handler,
    NMI_Handler, HardFault_Handler, MemoryManagement_Handler,
    BusFault_Handler, UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler, DebugMon_Handler, 0,
    PendSV_Handler, SysTick_Handler,

    /* IRQ 0..29 */
    POWER_CLOCK_IRQHandler, RADIO_IRQHandler, UARTE0_UART0_IRQHandler,
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler,
    SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler,
    NFCT_IRQHandler, GPIOTE_IRQHandler, SAADC_IRQHandler,
    TIMER0_IRQHandler, TIMER1_IRQHandler, TIMER2_IRQHandler,
    RTC0_IRQHandler, TEMP_IRQHandler, RNG_IRQHandler,
    ECB_IRQHandler, CCM_AAR_IRQHandler, WDT_IRQHandler,
    RTC1_IRQHandler, QDEC_IRQHandler, COMP_LPCOMP_IRQHandler,
    SWI0_EGU0_IRQHandler, SWI1_EGU1_IRQHandler, SWI2_EGU2_IRQHandler,
    SWI3_EGU3_IRQHandler, SWI4_EGU4_IRQHandler, SWI5_EGU5_IRQHandler,
    TIMER3_IRQHandler, TIMER4_IRQHandler,
    PWM0_IRQHandler, PDM_IRQHandler,
    /* IRQ 30, 31: reserved - must be zero-padded or USBD (IRQ 39)
     * vectors to the wrong slot. Without the gap, NVIC_EnableIRQ(USBD)
     * turns on IRQ 39 at the NVIC level but the CPU jumps to our
     * QSPI handler, which is weak-aliased to Default_Handler's spin. */
    0, 0,
    /* IRQ 32..42 */
    MWU_IRQHandler,
    PWM1_IRQHandler, PWM2_IRQHandler, SPIM2_SPIS2_SPI2_IRQHandler,
    RTC2_IRQHandler, I2S_IRQHandler, FPU_IRQHandler,
    USBD_IRQHandler, UARTE1_IRQHandler, QSPI_IRQHandler,
    CRYPTOCELL_IRQHandler,
    /* IRQ 43, 44: reserved */
    0, 0,
    /* IRQ 45 */
    PWM3_IRQHandler,
    /* IRQ 46: reserved */
    0,
    /* IRQ 47 */
    SPIM3_IRQHandler,
};

void Default_Handler(void) { for (;;); }

/* Reset on fault so the device self-recovers instead of stalling: a bare spin
 * here would keep USB enumerated by the ISR while every task is dead, needing a
 * physical power-cycle. A fault then shows up as an unexpectedly low uptime
 * after the operation. */
void HardFault_Handler(void)        { NVIC_SystemReset(); }
void MemoryManagement_Handler(void) { NVIC_SystemReset(); }
void BusFault_Handler(void)         { NVIC_SystemReset(); }
void UsageFault_Handler(void)       { NVIC_SystemReset(); }

/* Called by the FreeRTOS stack-overflow / malloc-failed hooks (core code,
 * no CMSIS) so they can reset rather than spin. */
void fantasi_reset(void) { NVIC_SystemReset(); }

__attribute__((noreturn))
void Reset_Handler(void)
{
    /* Take interrupts away from the SoftDevice forwarding table by
     * pointing the core at our vector table. Must run before any
     * interrupt can fire - __disable_irq is implicit from reset. */
    SCB->VTOR = (uint32_t)&g_vector_table;

    /* Mask + clear-pending all peripheral IRQs so no leftover SD state
     * fires into our still-uninitialised handlers. 8 registers × 32 =
     * 256 IRQs, far more than nRF52840 actually has, but safe to blast. */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }

    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    SystemInit();
    main();
    for (;;);
}
