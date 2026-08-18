/* Fantasi / Proxmark5 (AT32F435, Cortex-M4) startup
 *
 * Cortex-M4 vector table + C-startup. Clears .bss, copies .data, calls
 * SystemInit() for clock bring-up, then main(). Peripheral IRQs fall
 * through to a spin-forever default so a stray interrupt is visible under
 * a debugger. OTGFS2_IRQHandler (IRQ 77) is overridden in hal.c to drive
 * TinyUSB's DWC2 device driver.
 *
 * The AT32F435 IRQ order is from RM_AT32F435_437 Table 1-3 (vector table).
 * Reserved positions are held as 0 so every populated slot keeps its IRQ
 * number - the NVIC indexes this array by position.
 */

#include <stdint.h>
#include "at32f435.h"

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
WEAK void DebugMon_Handler(void);
/* The four fault handlers below are defined strongly (reset-on-fault). */
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);

/* AT32F435/437 peripheral IRQs, in IRQn order 0..114 (RM Table 1-3). */
WEAK void WWDT_IRQHandler(void);
WEAK void PVM_IRQHandler(void);
WEAK void TAMP_IRQHandler(void);
WEAK void ERTC_WKUP_IRQHandler(void);
WEAK void FLASH_IRQHandler(void);
WEAK void CRM_IRQHandler(void);
WEAK void EXINT0_IRQHandler(void);
WEAK void EXINT1_IRQHandler(void);
WEAK void EXINT2_IRQHandler(void);
WEAK void EXINT3_IRQHandler(void);
WEAK void EXINT4_IRQHandler(void);
WEAK void EDMA_Stream1_IRQHandler(void);
WEAK void EDMA_Stream2_IRQHandler(void);
WEAK void EDMA_Stream3_IRQHandler(void);
WEAK void EDMA_Stream4_IRQHandler(void);
WEAK void EDMA_Stream5_IRQHandler(void);
WEAK void EDMA_Stream6_IRQHandler(void);
WEAK void EDMA_Stream7_IRQHandler(void);
WEAK void ADC1_2_3_IRQHandler(void);
WEAK void CAN1_TX_IRQHandler(void);
WEAK void CAN1_RX0_IRQHandler(void);
WEAK void CAN1_RX1_IRQHandler(void);
WEAK void CAN1_SE_IRQHandler(void);
WEAK void EXINT9_5_IRQHandler(void);
WEAK void TMR1_BRK_TMR9_IRQHandler(void);
WEAK void TMR1_OVF_TMR10_IRQHandler(void);
WEAK void TMR1_TRG_HALL_TMR11_IRQHandler(void);
WEAK void TMR1_CH_IRQHandler(void);
WEAK void TMR2_IRQHandler(void);
WEAK void TMR3_IRQHandler(void);
WEAK void TMR4_IRQHandler(void);
WEAK void I2C1_EVT_IRQHandler(void);
WEAK void I2C1_ERR_IRQHandler(void);
WEAK void I2C2_EVT_IRQHandler(void);
WEAK void I2C2_ERR_IRQHandler(void);
WEAK void SPI1_IRQHandler(void);
WEAK void SPI2_I2S2EXT_IRQHandler(void);
WEAK void USART1_IRQHandler(void);
WEAK void USART2_IRQHandler(void);
WEAK void USART3_IRQHandler(void);
WEAK void EXINT15_10_IRQHandler(void);
WEAK void ERTCAlarm_IRQHandler(void);
WEAK void OTGFS1_WKUP_IRQHandler(void);
WEAK void TMR8_BRK_TMR12_IRQHandler(void);
WEAK void TMR8_OVF_TMR13_IRQHandler(void);
WEAK void TMR8_TRG_HALL_TMR14_IRQHandler(void);
WEAK void TMR8_CH_IRQHandler(void);
WEAK void EDMA_Stream8_IRQHandler(void);
WEAK void XMC_IRQHandler(void);
WEAK void SDIO1_IRQHandler(void);
WEAK void TMR5_IRQHandler(void);
WEAK void SPI3_I2S3EXT_IRQHandler(void);
WEAK void UART4_IRQHandler(void);
WEAK void UART5_IRQHandler(void);
WEAK void TMR6_DAC_IRQHandler(void);
WEAK void TMR7_IRQHandler(void);
WEAK void DMA1_Channel1_IRQHandler(void);
WEAK void DMA1_Channel2_IRQHandler(void);
WEAK void DMA1_Channel3_IRQHandler(void);
WEAK void DMA1_Channel4_IRQHandler(void);
WEAK void DMA1_Channel5_IRQHandler(void);
WEAK void EMAC2_IRQHandler(void);          /* AT32F437 only */
WEAK void EMAC_WKUP2_IRQHandler(void);     /* AT32F437 only */
WEAK void CAN2_TX_IRQHandler(void);
WEAK void CAN2_RX0_IRQHandler(void);
WEAK void CAN2_RX1_IRQHandler(void);
WEAK void CAN2_SE_IRQHandler(void);
WEAK void OTGFS1_IRQHandler(void);
WEAK void DMA1_Channel6_IRQHandler(void);
WEAK void DMA1_Channel7_IRQHandler(void);
WEAK void USART6_IRQHandler(void);
WEAK void I2C3_EVT_IRQHandler(void);
WEAK void I2C3_ERR_IRQHandler(void);
WEAK void OTGFS2_WKUP_IRQHandler(void);
WEAK void OTGFS2_IRQHandler(void);         /* overridden in hal.c */
WEAK void DVP_IRQHandler(void);
WEAK void FPU_IRQHandler(void);
WEAK void UART7_IRQHandler(void);
WEAK void UART8_IRQHandler(void);
WEAK void SPI4_IRQHandler(void);
WEAK void QSPI2_IRQHandler(void);
WEAK void QSPI1_IRQHandler(void);
WEAK void DMAMUX_IRQHandler(void);
WEAK void SDIO2_IRQHandler(void);
WEAK void ACC_IRQHandler(void);
WEAK void TMR20_BRK_IRQHandler(void);
WEAK void TMR20_OVF_IRQHandler(void);
WEAK void TMR20_TRG_HALL_IRQHandler(void);
WEAK void TMR20_CH_IRQHandler(void);
WEAK void DMA2_Channel1_IRQHandler(void);
WEAK void DMA2_Channel2_IRQHandler(void);
WEAK void DMA2_Channel3_IRQHandler(void);
WEAK void DMA2_Channel4_IRQHandler(void);
WEAK void DMA2_Channel5_IRQHandler(void);
WEAK void DMA2_Channel6_IRQHandler(void);
WEAK void DMA2_Channel7_IRQHandler(void);

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

    /* IRQ 0.. (RM Table 1-3) */
    WWDT_IRQHandler,               /* 0  */
    PVM_IRQHandler,                /* 1  */
    TAMP_IRQHandler,               /* 2  */
    ERTC_WKUP_IRQHandler,          /* 3  */
    FLASH_IRQHandler,              /* 4  */
    CRM_IRQHandler,                /* 5  */
    EXINT0_IRQHandler,             /* 6  */
    EXINT1_IRQHandler,             /* 7  */
    EXINT2_IRQHandler,             /* 8  */
    EXINT3_IRQHandler,             /* 9  */
    EXINT4_IRQHandler,             /* 10 */
    EDMA_Stream1_IRQHandler,       /* 11 */
    EDMA_Stream2_IRQHandler,       /* 12 */
    EDMA_Stream3_IRQHandler,       /* 13 */
    EDMA_Stream4_IRQHandler,       /* 14 */
    EDMA_Stream5_IRQHandler,       /* 15 */
    EDMA_Stream6_IRQHandler,       /* 16 */
    EDMA_Stream7_IRQHandler,       /* 17 */
    ADC1_2_3_IRQHandler,           /* 18 */
    CAN1_TX_IRQHandler,            /* 19 */
    CAN1_RX0_IRQHandler,           /* 20 */
    CAN1_RX1_IRQHandler,           /* 21 */
    CAN1_SE_IRQHandler,            /* 22 */
    EXINT9_5_IRQHandler,           /* 23 */
    TMR1_BRK_TMR9_IRQHandler,      /* 24 */
    TMR1_OVF_TMR10_IRQHandler,     /* 25 */
    TMR1_TRG_HALL_TMR11_IRQHandler,/* 26 */
    TMR1_CH_IRQHandler,            /* 27 */
    TMR2_IRQHandler,               /* 28 */
    TMR3_IRQHandler,               /* 29 */
    TMR4_IRQHandler,               /* 30 */
    I2C1_EVT_IRQHandler,           /* 31 */
    I2C1_ERR_IRQHandler,           /* 32 */
    I2C2_EVT_IRQHandler,           /* 33 */
    I2C2_ERR_IRQHandler,           /* 34 */
    SPI1_IRQHandler,               /* 35 */
    SPI2_I2S2EXT_IRQHandler,       /* 36 */
    USART1_IRQHandler,             /* 37 */
    USART2_IRQHandler,             /* 38 */
    USART3_IRQHandler,             /* 39 */
    EXINT15_10_IRQHandler,         /* 40 */
    ERTCAlarm_IRQHandler,          /* 41 */
    OTGFS1_WKUP_IRQHandler,        /* 42 */
    TMR8_BRK_TMR12_IRQHandler,     /* 43 */
    TMR8_OVF_TMR13_IRQHandler,     /* 44 */
    TMR8_TRG_HALL_TMR14_IRQHandler,/* 45 */
    TMR8_CH_IRQHandler,            /* 46 */
    EDMA_Stream8_IRQHandler,       /* 47 */
    XMC_IRQHandler,                /* 48 */
    SDIO1_IRQHandler,              /* 49 */
    TMR5_IRQHandler,               /* 50 */
    SPI3_I2S3EXT_IRQHandler,       /* 51 */
    UART4_IRQHandler,              /* 52 */
    UART5_IRQHandler,              /* 53 */
    TMR6_DAC_IRQHandler,           /* 54 */
    TMR7_IRQHandler,               /* 55 */
    DMA1_Channel1_IRQHandler,      /* 56 */
    DMA1_Channel2_IRQHandler,      /* 57 */
    DMA1_Channel3_IRQHandler,      /* 58 */
    DMA1_Channel4_IRQHandler,      /* 59 */
    DMA1_Channel5_IRQHandler,      /* 60 */
    EMAC2_IRQHandler,              /* 61 (437 only) */
    EMAC_WKUP2_IRQHandler,         /* 62 (437 only) */
    CAN2_TX_IRQHandler,            /* 63 */
    CAN2_RX0_IRQHandler,           /* 64 */
    CAN2_RX1_IRQHandler,           /* 65 */
    CAN2_SE_IRQHandler,            /* 66 */
    OTGFS1_IRQHandler,             /* 67 */
    DMA1_Channel6_IRQHandler,      /* 68 */
    DMA1_Channel7_IRQHandler,      /* 69 */
    0,                             /* 70 reserved */
    USART6_IRQHandler,             /* 71 */
    I2C3_EVT_IRQHandler,           /* 72 */
    I2C3_ERR_IRQHandler,           /* 73 */
    0,                             /* 74 reserved */
    0,                             /* 75 reserved */
    OTGFS2_WKUP_IRQHandler,        /* 76 */
    OTGFS2_IRQHandler,             /* 77 <- USB */
    DVP_IRQHandler,                /* 78 */
    0,                             /* 79 reserved */
    0,                             /* 80 reserved */
    FPU_IRQHandler,                /* 81 */
    UART7_IRQHandler,              /* 82 */
    UART8_IRQHandler,              /* 83 */
    SPI4_IRQHandler,               /* 84 */
    0, 0, 0, 0, 0, 0,              /* 85..90 reserved */
    QSPI2_IRQHandler,              /* 91 */
    QSPI1_IRQHandler,              /* 92 */
    0,                             /* 93 reserved */
    DMAMUX_IRQHandler,             /* 94 */
    0, 0, 0, 0, 0, 0, 0,           /* 95..101 reserved */
    SDIO2_IRQHandler,              /* 102 */
    ACC_IRQHandler,                /* 103 */
    TMR20_BRK_IRQHandler,          /* 104 */
    TMR20_OVF_IRQHandler,          /* 105 */
    TMR20_TRG_HALL_IRQHandler,     /* 106 */
    TMR20_CH_IRQHandler,           /* 107 */
    DMA2_Channel1_IRQHandler,      /* 108 */
    DMA2_Channel2_IRQHandler,      /* 109 */
    DMA2_Channel3_IRQHandler,      /* 110 */
    DMA2_Channel4_IRQHandler,      /* 111 */
    DMA2_Channel5_IRQHandler,      /* 112 */
    DMA2_Channel6_IRQHandler,      /* 113 */
    DMA2_Channel7_IRQHandler,      /* 114 */
};

void Default_Handler(void)
{
    /* Loop forever - leaves the state inspectable on the debugger. */
    for (;;);
}

/* Reboot in place, keeping the PB0 power-lock held. A hardware reset (SYSRESETREQ)
 * releases GPIOB/PB0 and the self-latching supply cuts power, so the board does not
 * survive it even on USB. Instead: full CRM deinit (clocks to HICK, every peripheral
 * reset except GPIOB, only GPIOB left clocked), then jump to the target's reset
 * vector with PB0 still driven high. image_base = PM5_APP_BASE restarts our firmware;
 * the ROM base enters the Artery USB DFU. (AN0008 Method 1, plus the PSP->MSP switch
 * its example omits.) */
__attribute__((noreturn))
void pm5_deinit_and_jump(uint32_t image_base)
{
    __disable_irq();

    /* Clocks back to HICK. */
    CRM->CTRL |= CRM_CTRL_HICKEN;
    while (!(CRM->CTRL & CRM_CTRL_HICKSTBL)) { }
    CRM->CFG = (CRM->CFG & ~CRM_CFG_SCLKSEL_MSK) | (CRM_SCLK_HICK << CRM_CFG_SCLKSEL_POS);
    while (((CRM->CFG & CRM_CFG_SCLKSTS_MSK) >> CRM_CFG_SCLKSTS_POS) != CRM_SCLK_HICK) { }
    CRM->CTRL  &= ~0x010D0000u;      /* disable PLL / HEXT / HEXTBYP / CFD */
    CRM->CFG    = 0;
    CRM->PLLCFG = 0x00033002u;       /* reset value */
    CRM->MISC1  = 0;
    CRM->MISC2  = 0;

    /* Reset every peripheral we may have touched, excluding GPIOB (AHBRST1 bit 1)
     * so PB0 keeps driving high. Resetting OTGFS2 (bit 29) drops the USB pull-up.
     * AHBRST2/3 cover the peripherals in those banks; APB1/2 covers the rest. */
    uint32_t ahb1 = (1u<<0)|(1u<<2)|(1u<<3)|(1u<<4)|(1u<<5)|(1u<<6)|(1u<<7)  /* GPIOA,C,D,E,F,G,H */
                  | (1u<<12)|(1u<<22)|(1u<<24)|CRM_AHBEN1_OTGFS2EN;          /* CRC,DMA1,DMA2,OTGFS2 */
    CRM->AHBRST1 = ahb1;   CRM->AHBRST1 = 0;
    CRM->AHBRST2 = 0xFFFFFFFFu; CRM->AHBRST2 = 0;
    CRM->AHBRST3 = 0xFFFFFFFFu; CRM->AHBRST3 = 0;
    CRM->APB1RST = 0xFFFFFFFFu; CRM->APB1RST = 0;
    CRM->APB2RST = 0xFFFFFFFFu; CRM->APB2RST = 0;

    /* Keep only GPIOB clocked (PB0); gate everything else. */
    CRM->AHBEN1 = CRM_AHBEN1_GPIOBEN;
    CRM->AHBEN2 = 0; CRM->AHBEN3 = 0;
    CRM->APB1EN = 0; CRM->APB2EN = 0;
    CRM->CLKINT = 0x009F0000u;                  /* clear clock-stable interrupt flags */

    /* Settle ~30 ms at HICK so the host registers the USB disconnect (pull-up
     * dropped above) before the target re-enumerates. */
    for (volatile uint32_t i = 0; i < 300000u; i++) { __asm volatile("nop"); }

    /* Hand off. Mask+clear IRQs, stop SysTick, switch to the privileged main stack
     * (we run from a FreeRTOS task on PSP; __set_MSP alone does not switch stacks),
     * load MSP, unmask, branch. AN0008 Method 1 does not set VTOR - the target
     * establishes its own. */
    for (int i = 0; i < 8; i++) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
    SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;

    uint32_t sp = *(volatile uint32_t *)(image_base + 0);
    uint32_t pc = *(volatile uint32_t *)(image_base + 4);
    __set_CONTROL(0x00);   /* SPSEL=0 (MSP), nPRIV=0 (privileged) */
    __ISB();
    __set_MSP(sp);
    __DSB();
    __ISB();
    __enable_irq();
    ((void (*)(void))(pc | 1u))();
    for (;;);
}

/* Restart on fault, preserving PB0 (a hardware reset would power the board off). */
void HardFault_Handler(void)  { pm5_deinit_and_jump(PM5_APP_BASE); }
void MemManage_Handler(void)  { pm5_deinit_and_jump(PM5_APP_BASE); }
void BusFault_Handler(void)   { pm5_deinit_and_jump(PM5_APP_BASE); }
void UsageFault_Handler(void) { pm5_deinit_and_jump(PM5_APP_BASE); }

/* Called by the FreeRTOS stack-overflow / malloc-failed hooks (core code, no CMSIS). */
void fantasi_reset(void)
{
    pm5_deinit_and_jump(PM5_APP_BASE);
}

/* Hold the Proxmark5 power-supply self-lock (PB0). This must be the first thing
 * the reset handler does - before the .bss clear - because at the 8 MHz reset
 * clock, zeroing 16 KB of .bss first takes long enough to exceed the latch's
 * power-off timeout. Linking at 0x08000000 replaced the factory bootrom, which is
 * what normally holds this pin, so our image must do it itself. Matches the
 * bootrom's sequence exactly: enable GPIOB clock, drive the output-data bit HIGH
 * via SCR while still an input, then switch to push-pull output - so PB0 never
 * glitches LOW (which releases the latch). Touches only registers (no globals),
 * so it is safe pre-.data/.bss. Also lights LED_A (PC3, open-drain, active-low)
 * as a "reset reached" indicator. I am still deciding if we keep the LED_A
 * indicator long-term. - noproto */
static void pm5_board_power_latch(void)
{
    CRM->AHBEN1 |= CRM_AHBEN1_GPIOBEN;
    (void)CRM->AHBEN1;
    GPIOB->SCR = (1u << PM5_PWR_LOCK_PIN);          /* stage ODT = 1 (HIGH) */
    gpio_set_otype(GPIOB, PM5_PWR_LOCK_PIN, GPIO_OTYPE_PP);
    gpio_set_pull (GPIOB, PM5_PWR_LOCK_PIN, GPIO_PULL_NONE);
    gpio_set_mode (GPIOB, PM5_PWR_LOCK_PIN, GPIO_MODE_OUTPUT);

    CRM->AHBEN1 |= CRM_AHBEN1_GPIOCEN;
    (void)CRM->AHBEN1;
    GPIOC->CLR = (1u << PM5_LED_A_PIN);              /* ODT = 0: LOW = on (open-drain) */
    gpio_set_otype(GPIOC, PM5_LED_A_PIN, GPIO_OTYPE_OD);
    gpio_set_mode (GPIOC, PM5_LED_A_PIN, GPIO_MODE_OUTPUT);
}

static void pm5_stop_dfu_latch_refresh(void)
{
    if (CRM->APB1EN & CRM_APB1EN_TMR7EN) {
        TMR7->CTRL1 &= ~TMR_CTRL1_TMREN;
        TMR7->IDEN &= ~TMR_IDEN_OVFDEN;
        CRM->APB1EN &= ~CRM_APB1EN_TMR7EN;
    }
    if (CRM->AHBEN1 & CRM_AHBEN1_DMA1EN) {
        DMA1->CH[0].CTRL &= ~DMA_CTRL_CHEN;
        DMA1->CLR = DMA_CLR_CH1;
        CRM->AHBEN1 &= ~CRM_AHBEN1_DMA1EN;
    }
}

__attribute__((noreturn))
void Reset_Handler(void)
{
    pm5_board_power_latch();   /* hold PB0 before anything else, or the board powers off */
    pm5_stop_dfu_latch_refresh();

    /* Point VTOR at our table. After a ROM DFU jump the CPU may still have
     * VTOR at the ROM bootloader base; without this write every exception
     * (SVC, PendSV, SysTick, any IRQ) would vector into the ROM's handlers.
     * VTOR is at 0xE000ED08; bits 31:7 are the table base, so the table must
     * be 128-byte aligned (the linker's .isr_vector ALIGN(4) plus the table
     * living at flash base over-satisfies this). */
    *(volatile uint32_t *)0xE000ED08UL = (uint32_t)&g_vector_table;

    /* Copy .data from LMA in FLASH to VMA in RAM. */
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;

    /* Zero .bss. */
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    SystemInit();
    main();
    for (;;);
}
