/* Fantasi / Flipper Zero (STM32WB55) startup
 *
 * Cortex-M4 vector table + C-startup. Clears .bss, copies .data, calls
 * SystemInit() for clock bring-up, then main(). All peripheral IRQs
 * fall through to a spin-forever default so a stray interrupt is
 * visible under a debugger rather than silently misbehaving.
 */

#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss,  _ebss;
extern uint32_t _stack_end;

extern int  main(void);
extern void SystemInit(void);

/* FreeRTOS provides these; declared here so they can sit in the vector
 * table without pulling the whole kernel header into asm-free startup. */
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK __attribute__((weak, alias("Default_Handler")))
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void DebugMon_Handler(void);

/* Peripheral IRQs, in IRQn order 0..62. Each is weak-aliased to the
 * default handler and can be overridden by strong definition elsewhere
 * (e.g. USB_LP_IRQHandler is overridden in hal.c to drive TinyUSB). */
WEAK void WWDG_IRQHandler(void);
WEAK void PVD_PVM_IRQHandler(void);
WEAK void TAMP_STAMP_LSECSS_IRQHandler(void);
WEAK void RTC_WKUP_IRQHandler(void);
WEAK void FLASH_IRQHandler(void);
WEAK void RCC_IRQHandler(void);
WEAK void EXTI0_IRQHandler(void);
WEAK void EXTI1_IRQHandler(void);
WEAK void EXTI2_IRQHandler(void);
WEAK void EXTI3_IRQHandler(void);
WEAK void EXTI4_IRQHandler(void);
WEAK void DMA1_Channel1_IRQHandler(void);
WEAK void DMA1_Channel2_IRQHandler(void);
WEAK void DMA1_Channel3_IRQHandler(void);
WEAK void DMA1_Channel4_IRQHandler(void);
WEAK void DMA1_Channel5_IRQHandler(void);
WEAK void DMA1_Channel6_IRQHandler(void);
WEAK void DMA1_Channel7_IRQHandler(void);
WEAK void ADC1_IRQHandler(void);
WEAK void USB_HP_IRQHandler(void);
WEAK void USB_LP_IRQHandler(void);
WEAK void C2SEV_PWR_C2H_IRQHandler(void);
WEAK void COMP_IRQHandler(void);
WEAK void EXTI9_5_IRQHandler(void);
WEAK void TIM1_BRK_IRQHandler(void);
WEAK void TIM1_UP_TIM16_IRQHandler(void);
WEAK void TIM1_TRG_COM_TIM17_IRQHandler(void);
WEAK void TIM1_CC_IRQHandler(void);
WEAK void TIM2_IRQHandler(void);
WEAK void PKA_IRQHandler(void);
WEAK void I2C1_EV_IRQHandler(void);
WEAK void I2C1_ER_IRQHandler(void);
WEAK void I2C3_EV_IRQHandler(void);
WEAK void I2C3_ER_IRQHandler(void);
WEAK void SPI1_IRQHandler(void);
WEAK void SPI2_IRQHandler(void);
WEAK void USART1_IRQHandler(void);
WEAK void LPUART1_IRQHandler(void);
WEAK void SAI1_IRQHandler(void);
WEAK void TSC_IRQHandler(void);
WEAK void EXTI15_10_IRQHandler(void);
WEAK void RTC_Alarm_IRQHandler(void);
WEAK void CRS_IRQHandler(void);
WEAK void PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler(void);
WEAK void IPCC_C1_RX_IRQHandler(void);
WEAK void IPCC_C1_TX_IRQHandler(void);
WEAK void HSEM_IRQHandler(void);
WEAK void LPTIM1_IRQHandler(void);
WEAK void LPTIM2_IRQHandler(void);
WEAK void LCD_IRQHandler(void);
WEAK void QUADSPI_IRQHandler(void);
WEAK void AES1_IRQHandler(void);
WEAK void AES2_IRQHandler(void);
WEAK void RNG_IRQHandler(void);
WEAK void FPU_IRQHandler(void);
WEAK void DMA2_Channel1_IRQHandler(void);
WEAK void DMA2_Channel2_IRQHandler(void);
WEAK void DMA2_Channel3_IRQHandler(void);
WEAK void DMA2_Channel4_IRQHandler(void);
WEAK void DMA2_Channel5_IRQHandler(void);
WEAK void DMA2_Channel6_IRQHandler(void);
WEAK void DMA2_Channel7_IRQHandler(void);
WEAK void DMAMUX1_OVR_IRQHandler(void);

__attribute__((section(".isr_vector"), used))
void (* const g_vector_table[])(void) = {
    (void (*)(void))&_stack_end,   /* initial SP */
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    WWDG_IRQHandler, PVD_PVM_IRQHandler, TAMP_STAMP_LSECSS_IRQHandler,
    RTC_WKUP_IRQHandler, FLASH_IRQHandler, RCC_IRQHandler,
    EXTI0_IRQHandler, EXTI1_IRQHandler, EXTI2_IRQHandler,
    EXTI3_IRQHandler, EXTI4_IRQHandler,
    DMA1_Channel1_IRQHandler, DMA1_Channel2_IRQHandler, DMA1_Channel3_IRQHandler,
    DMA1_Channel4_IRQHandler, DMA1_Channel5_IRQHandler, DMA1_Channel6_IRQHandler,
    DMA1_Channel7_IRQHandler, ADC1_IRQHandler,
    USB_HP_IRQHandler, USB_LP_IRQHandler,
    C2SEV_PWR_C2H_IRQHandler, COMP_IRQHandler, EXTI9_5_IRQHandler,
    TIM1_BRK_IRQHandler, TIM1_UP_TIM16_IRQHandler,
    TIM1_TRG_COM_TIM17_IRQHandler, TIM1_CC_IRQHandler, TIM2_IRQHandler,
    PKA_IRQHandler, I2C1_EV_IRQHandler, I2C1_ER_IRQHandler,
    I2C3_EV_IRQHandler, I2C3_ER_IRQHandler, SPI1_IRQHandler, SPI2_IRQHandler,
    USART1_IRQHandler, LPUART1_IRQHandler, SAI1_IRQHandler, TSC_IRQHandler,
    EXTI15_10_IRQHandler, RTC_Alarm_IRQHandler, CRS_IRQHandler,
    PWR_SOTF_BLEACT_802ACT_RFPHASE_IRQHandler,
    IPCC_C1_RX_IRQHandler, IPCC_C1_TX_IRQHandler, HSEM_IRQHandler,
    LPTIM1_IRQHandler, LPTIM2_IRQHandler, LCD_IRQHandler, QUADSPI_IRQHandler,
    AES1_IRQHandler, AES2_IRQHandler, RNG_IRQHandler, FPU_IRQHandler,
    DMA2_Channel1_IRQHandler, DMA2_Channel2_IRQHandler, DMA2_Channel3_IRQHandler,
    DMA2_Channel4_IRQHandler, DMA2_Channel5_IRQHandler, DMA2_Channel6_IRQHandler,
    DMA2_Channel7_IRQHandler, DMAMUX1_OVR_IRQHandler,
};

void Default_Handler(void)
{
    /* Loop forever - leaves the state inspectable on the debugger. */
    for (;;);
}

void fantasi_reset(void)
{
    __asm volatile("dsb");
    *(volatile uint32_t *)0xE000ED0CUL = (0x5FAUL << 16) | (1UL << 2);
    for (;;);
}

#define FANTASI_DFU_MAGIC 0xD0F0FADAUL
#define FANTASI_COLDBOOT_MAGIC 0xC01DB007UL
__attribute__((section(".noinit"))) volatile uint32_t g_fantasi_dfu_magic;
__attribute__((section(".noinit"))) volatile uint32_t g_fantasi_coldboot;


__attribute__((noreturn))
void Reset_Handler(void)
{
    if (g_fantasi_dfu_magic == FANTASI_DFU_MAGIC) {
        /* The ROM system bootloader is unreliable while CPU2 (the wireless
         * stack) is still running - after a warm reset C2BOOT in PWR_CR4 stays
         * set and the jump to 0x1FFF0000 intermittently fails to enumerate DFU.
         * A `dfu` request issued after BLE activity may therefore never enter
         * the bootloader. Do ONE coldboot reset first (SYSRESETREQ clears
         * C2BOOT), KEEPING the DFU magic, so the next boot jumps with CPU2
         * stopped. The coldboot guard prevents a loop. */
        volatile uint32_t *pwr_cr4 = (volatile uint32_t *)0x5800040CUL;
        if ((*pwr_cr4 & (1UL << 15)) &&
            g_fantasi_coldboot != FANTASI_COLDBOOT_MAGIC) {
            g_fantasi_coldboot = FANTASI_COLDBOOT_MAGIC;   /* keep dfu magic set */
            *(volatile uint32_t *)0xE000ED0CUL = (0x5FAUL << 16) | (1UL << 2);
            for (;;);
        }
        g_fantasi_coldboot = 0;
        g_fantasi_dfu_magic = 0;
        uint32_t sp = *(volatile uint32_t *)0x1FFF0000UL;
        uint32_t pc = *(volatile uint32_t *)0x1FFF0004UL;
        __asm volatile(
            "msr msp, %0\n"
            "bx  %1\n"
            :: "r"(sp), "r"(pc));
        for (;;);
    }

    /* If C2BOOT is set (stale from a DFU session), CPU2 is running
     * the FUS and ignores new mailbox tables.  SYSRESETREQ clears
     * PWR_CR4 including C2BOOT, giving ble_init() a fresh start.
     * The .noinit guard prevents an infinite reset loop. */
    if (g_fantasi_coldboot != FANTASI_COLDBOOT_MAGIC) {
        volatile uint32_t *pwr_cr4 = (volatile uint32_t *)0x5800040CUL;
        if (*pwr_cr4 & (1UL << 15)) {
            g_fantasi_coldboot = FANTASI_COLDBOOT_MAGIC;
            *(volatile uint32_t *)0xE000ED0CUL =
                (0x5FAUL << 16) | (1UL << 2);
            for (;;);
        }
    }
    g_fantasi_coldboot = 0;

    /* Point VTOR at our table. After a DFU `:leave` the CPU is already
     * executing with VTOR=0x1FFF0000 (ROM system memory), and the ROM
     * doesn't reset VTOR before jumping to user flash. Without this
     * write, every interrupt (SVC, PendSV, SysTick, any peripheral IRQ)
     * would vector into the ROM bootloader's handlers.
     * Cortex-M4 VTOR lives at 0xE000ED08; bits 31:7 are the table base,
     * so we require the table to be aligned on a 128-byte boundary
     * (the linker's .isr_vector ALIGN(4) already over-satisfies this). */
    volatile uint32_t *vtor = (volatile uint32_t *)0xE000ED08UL;
    *vtor = (uint32_t)&g_vector_table;

    /* Copy .data from LMA in FLASH to VMA in RAM1. */
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;

    /* Zero .bss. */
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    SystemInit();
    main();
    for (;;);
}
