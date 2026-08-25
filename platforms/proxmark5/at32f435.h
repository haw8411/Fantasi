/* Minimal AT32F435 device header for Fantasi / Proxmark5.
 *
 * Not the full Artery BSP - only the peripherals the firmware touches
 * (CRM clocks/reset, FLASH controller, GPIO, PWC LDO), plus the Cortex-M4
 * core configuration and the interrupt vector enum that core_cm4.h and the
 * NVIC need. Register layouts and bit definitions are from
 * RM_AT32F435_437_V2.08 / DS_AT32F435_437_V2.30 (see datasheets/); page
 * citations are on each block. Widen this as more peripherals are integrated
 * (the RFID SPI/timers/DMA in a later phase).
 */
#ifndef FANTASI_AT32F435_H
#define FANTASI_AT32F435_H

#include <stdint.h>
#include <stdbool.h>

/* ---- Cortex-M4 core configuration (consumed by core_cm4.h) ---- */
#define __CM4_REV              0x0001U
#define __MPU_PRESENT         1U
#define __NVIC_PRIO_BITS      4U       /* AT32F435 NVIC: 16 priority levels (DS p20) */
#define __Vendor_SysTickConfig 0U
#define __FPU_PRESENT         1U       /* Cortex-M4F */

/* ---- Interrupt vector numbers (RM Table 1-3) ----
 * Names follow the Artery/CMSIS convention; the numeric positions are the
 * load-bearing part and match the startup.c vector table exactly. */
typedef enum {
    NonMaskableInt_IRQn      = -14,
    HardFault_IRQn           = -13,
    MemoryManagement_IRQn    = -12,
    BusFault_IRQn            = -11,
    UsageFault_IRQn          = -10,
    SVCall_IRQn              =  -5,
    DebugMonitor_IRQn        =  -4,
    PendSV_IRQn              =  -2,
    SysTick_IRQn             =  -1,

    WWDT_IRQn                =   0,
    PVM_IRQn                 =   1,
    TAMP_IRQn                =   2,
    ERTC_WKUP_IRQn           =   3,
    FLASH_IRQn               =   4,
    CRM_IRQn                 =   5,
    EXINT0_IRQn              =   6,
    EXINT1_IRQn              =   7,
    EXINT2_IRQn              =   8,
    EXINT3_IRQn              =   9,
    EXINT4_IRQn              =  10,
    EDMA_Stream1_IRQn        =  11,
    EDMA_Stream2_IRQn        =  12,
    EDMA_Stream3_IRQn        =  13,
    EDMA_Stream4_IRQn        =  14,
    EDMA_Stream5_IRQn        =  15,
    EDMA_Stream6_IRQn        =  16,
    EDMA_Stream7_IRQn        =  17,
    ADC1_2_3_IRQn            =  18,
    CAN1_TX_IRQn             =  19,
    CAN1_RX0_IRQn            =  20,
    CAN1_RX1_IRQn            =  21,
    CAN1_SE_IRQn             =  22,
    EXINT9_5_IRQn            =  23,
    TMR1_BRK_TMR9_IRQn       =  24,
    TMR1_OVF_TMR10_IRQn      =  25,
    TMR1_TRG_HALL_TMR11_IRQn =  26,
    TMR1_CH_IRQn             =  27,
    TMR2_GLOBAL_IRQn         =  28,
    TMR3_GLOBAL_IRQn         =  29,
    TMR4_GLOBAL_IRQn         =  30,
    I2C1_EVT_IRQn            =  31,
    I2C1_ERR_IRQn            =  32,
    I2C2_EVT_IRQn            =  33,
    I2C2_ERR_IRQn            =  34,
    SPI1_IRQn                =  35,
    SPI2_I2S2EXT_IRQn        =  36,
    USART1_IRQn              =  37,
    USART2_IRQn              =  38,
    USART3_IRQn              =  39,
    EXINT15_10_IRQn          =  40,
    ERTCAlarm_IRQn           =  41,
    OTGFS1_WKUP_IRQn         =  42,
    TMR8_BRK_TMR12_IRQn      =  43,
    TMR8_OVF_TMR13_IRQn      =  44,
    TMR8_TRG_HALL_TMR14_IRQn =  45,
    TMR8_CH_IRQn             =  46,
    EDMA_Stream8_IRQn        =  47,
    XMC_IRQn                 =  48,
    SDIO1_IRQn               =  49,
    TMR5_GLOBAL_IRQn         =  50,
    SPI3_I2S3EXT_IRQn        =  51,
    UART4_IRQn               =  52,
    UART5_IRQn               =  53,
    TMR6_DAC_GLOBAL_IRQn     =  54,
    TMR7_GLOBAL_IRQn         =  55,
    DMA1_Channel1_IRQn       =  56,
    DMA1_Channel2_IRQn       =  57,
    DMA1_Channel3_IRQn       =  58,
    DMA1_Channel4_IRQn       =  59,
    DMA1_Channel5_IRQn       =  60,
    EMAC2_IRQn               =  61,
    EMAC_WKUP2_IRQn          =  62,
    CAN2_TX_IRQn             =  63,
    CAN2_RX0_IRQn            =  64,
    CAN2_RX1_IRQn            =  65,
    CAN2_SE_IRQn             =  66,
    OTGFS1_IRQn              =  67,
    DMA1_Channel6_IRQn       =  68,
    DMA1_Channel7_IRQn       =  69,
    USART6_IRQn              =  71,
    I2C3_EVT_IRQn            =  72,
    I2C3_ERR_IRQn            =  73,
    OTGFS2_WKUP_IRQn         =  76,
    OTGFS2_IRQn              =  77,     /* USB (OTG_FS2) global interrupt */
    DVP_IRQn                 =  78,
    FPU_IRQn                 =  81,
    UART7_IRQn               =  82,
    UART8_IRQn               =  83,
    SPI4_IRQn                =  84,
    QSPI2_IRQn               =  91,
    QSPI1_IRQn               =  92,
    DMAMUX_IRQn              =  94,
    SDIO2_IRQn               = 102,
    ACC_IRQn                 = 103,
    TMR20_BRK_IRQn           = 104,
    TMR20_OVF_IRQn           = 105,
    TMR20_TRG_HALL_IRQn      = 106,
    TMR20_CH_IRQn            = 107,
    DMA2_Channel1_IRQn       = 108,
    DMA2_Channel2_IRQn       = 109,
    DMA2_Channel3_IRQn       = 110,
    DMA2_Channel4_IRQn       = 111,
    DMA2_Channel5_IRQn       = 112,
    DMA2_Channel6_IRQn       = 113,
    DMA2_Channel7_IRQn       = 114,
} IRQn_Type;

#include "core_cm4.h"

/* ======================================================================
 * CRM - Clock & Reset Management   base 0x40023800  (RM Table 4-1, p73)
 * ==================================================================== */
typedef struct {
    __IO uint32_t CTRL;        /* 0x00 */
    __IO uint32_t PLLCFG;      /* 0x04 */
    __IO uint32_t CFG;         /* 0x08 */
    __IO uint32_t CLKINT;      /* 0x0C */
    __IO uint32_t AHBRST1;     /* 0x10 */
    __IO uint32_t AHBRST2;     /* 0x14 */
    __IO uint32_t AHBRST3;     /* 0x18 */
    __IO uint32_t RESERVED0;   /* 0x1C */
    __IO uint32_t APB1RST;     /* 0x20 */
    __IO uint32_t APB2RST;     /* 0x24 */
    __IO uint32_t RESERVED1[2];/* 0x28-0x2C */
    __IO uint32_t AHBEN1;      /* 0x30 */
    __IO uint32_t AHBEN2;      /* 0x34 */
    __IO uint32_t AHBEN3;      /* 0x38 */
    __IO uint32_t RESERVED2;   /* 0x3C */
    __IO uint32_t APB1EN;      /* 0x40 */
    __IO uint32_t APB2EN;      /* 0x44 */
    __IO uint32_t RESERVED3[2];/* 0x48-0x4C */
    __IO uint32_t AHBLPEN1;    /* 0x50 */
    __IO uint32_t AHBLPEN2;    /* 0x54 */
    __IO uint32_t AHBLPEN3;    /* 0x58 */
    __IO uint32_t RESERVED4;   /* 0x5C */
    __IO uint32_t APB1LPEN;    /* 0x60 */
    __IO uint32_t APB2LPEN;    /* 0x64 */
    __IO uint32_t RESERVED5[2];/* 0x68-0x6C */
    __IO uint32_t BPDC;        /* 0x70 */
    __IO uint32_t CTRLSTS;     /* 0x74 */
    __IO uint32_t RESERVED6[10];/* 0x78-0x9C */
    __IO uint32_t MISC1;       /* 0xA0 */
    __IO uint32_t MISC2;       /* 0xA4 */
} crm_type;
#define CRM ((crm_type *)0x40023800UL)

#define CRM_CTRL_HICKEN     (1u << 0)
#define CRM_CTRL_HICKSTBL   (1u << 1)
#define CRM_CTRL_HEXTEN     (1u << 16)
#define CRM_CTRL_HEXTSTBL   (1u << 17)
#define CRM_CTRL_PLLEN      (1u << 24)
#define CRM_CTRL_PLLSTBL    (1u << 25)

#define CRM_PLLCFG_PLL_MS_POS   0        /* [3:0]  */
#define CRM_PLLCFG_PLL_NS_POS   6        /* [14:6] */
#define CRM_PLLCFG_PLL_FR_POS   16       /* [18:16] */
#define CRM_PLLCFG_PLLRCS       (1u << 22)   /* 0=HICK ref, 1=HEXT ref */
#define CRM_PLL_FR_4   0x2u      /* PLL_FR field value for the /4 post-divider */
/* 288 MHz SCLK from an 8 MHz HEXT: MS=1, NS=144, FR=/4, ref=HEXT. */
#define CRM_PLLCFG_288M_HEXT8 \
    ((1u   << CRM_PLLCFG_PLL_MS_POS) | \
     (144u << CRM_PLLCFG_PLL_NS_POS) | \
     (CRM_PLL_FR_4 << CRM_PLLCFG_PLL_FR_POS) | \
     CRM_PLLCFG_PLLRCS)

#define CRM_CFG_SCLKSEL_POS  0
#define CRM_CFG_SCLKSEL_MSK  (0x3u << 0)
#define CRM_CFG_SCLKSTS_POS  2
#define CRM_CFG_SCLKSTS_MSK  (0x3u << 2)
#define CRM_SCLK_HICK  0x0u
#define CRM_SCLK_HEXT  0x1u
#define CRM_SCLK_PLL   0x2u
#define CRM_CFG_AHBDIV_POS   4
#define CRM_CFG_APB1DIV_POS  10
#define CRM_CFG_APB2DIV_POS  13
#define CRM_AHB_DIV1   0x0u
#define CRM_AHB_DIV2   0x8u      /* 0b1000 = /2  (idle HICK-48 -> HCLK 24 MHz) */
#define CRM_AHB_DIV4   0x9u      /* 0b1001 = /4 */
#define CRM_AHB_DIV8   0xAu      /* 0b1010 = /8 */
#define CRM_APB_DIV2   0x4u      /* 0b100 */

#define CRM_AHBEN1_GPIOAEN   (1u << 0)
#define CRM_AHBEN1_GPIOBEN   (1u << 1)
#define CRM_AHBEN1_GPIOCEN   (1u << 2)
#define CRM_AHBEN1_GPIODEN   (1u << 3)
#define CRM_AHBEN1_GPIOHEN   (1u << 7)
#define CRM_AHBEN1_OTGFS2EN  (1u << 29)

#define CRM_APB1EN_PWCEN     (1u << 28)

#define CRM_MISC1_HICKDIV       (1u << 12)   /* 0=HICK/6 (8 MHz); 1=HICK (48 MHz) */
#define CRM_MISC1_HICK_TO_USB   (1u << 13)   /* 0=PLL(div) sources USB, 1=HICK */
#define CRM_MISC1_HICK_TO_SCLK  (1u << 14)   /* 1=SCLK-from-HICK is 48 MHz (needs HICKDIV=1) */
#define CRM_APB2EN_ACCEN        (1u << 29)   /* auto clock calibration clock enable */
#define CRM_MISC2_USBDIV_POS    12           /* [15:12] PLL->USB divider */
#define CRM_MISC2_USBDIV_MSK    (0xFu << 12)
#define CRM_USBDIV_6            0xBu          /* 0b1011 = /6 -> 288/6 = 48 MHz */
#define CRM_MISC2_AUTO_STEP_EN  (0x3u << 4)  /* enable step clock switch (>108 MHz) */

/* ======================================================================
 * FLASH controller  base 0x40023C00  (RM Table 5-8, p112)
 * ==================================================================== */
typedef struct {
    __IO uint32_t PSR;         /* 0x00 */
    __IO uint32_t UNLOCK;      /* 0x04 */
    __IO uint32_t USD_UNLOCK;  /* 0x08 */
    __IO uint32_t STS;         /* 0x0C */
    __IO uint32_t CTRL;        /* 0x10 */
    __IO uint32_t ADDR;        /* 0x14 */
    __IO uint32_t RESERVED0;   /* 0x18 */
    __IO uint32_t USD;         /* 0x1C */
    __IO uint32_t EPPS0;       /* 0x20 */
    __IO uint32_t RESERVED1[2];/* 0x24-0x28 */
    __IO uint32_t EPPS1;       /* 0x2C */
    __IO uint32_t RESERVED2[5];/* 0x30-0x40 */
    __IO uint32_t UNLOCK2;     /* 0x44 */
    __IO uint32_t RESERVED3;   /* 0x48 */
    __IO uint32_t STS2;        /* 0x4C */
    __IO uint32_t CTRL2;       /* 0x50 */
    __IO uint32_t ADDR2;       /* 0x54 */
    __IO uint32_t CONTR;       /* 0x58 */
    __IO uint32_t RESERVED4;   /* 0x5C */
    __IO uint32_t DIVR;        /* 0x60 */
} flash_type;
#define FLASH ((flash_type *)0x40023C00UL)

#define FLASH_UNLOCK_KEY1  0x45670123u
#define FLASH_UNLOCK_KEY2  0xCDEF89ABu

#define FLASH_STS_OBF      (1u << 0)   /* operation busy (ro) */
#define FLASH_STS_PRGMERR  (1u << 2)   /* program error (w1c) */
#define FLASH_STS_EPPERR   (1u << 4)   /* erase/prog protect err (w1c) */
#define FLASH_STS_ODF      (1u << 5)   /* operation done (w1c) */

#define FLASH_CTRL_FPRGM   (1u << 0)   /* program enable */
#define FLASH_CTRL_SECERS  (1u << 1)   /* sector erase (4 KB) */
#define FLASH_CTRL_ERSTR   (1u << 6)   /* erase start */
#define FLASH_CTRL_OPLK    (1u << 7)   /* operation lock (1=locked) */

#define FLASH_SECTOR_SIZE  0x0800u     /* 2 KB on the 1024 KB part (RM Table 5-2) */
#define FLASH_BANK1_BASE   0x08000000u
#define FLASH_BANK2_BASE   0x08080000u /* 1024 KB part: bank1 = 512 KB */

/* Device electronic signature (RM section 1.3, p56) */
#define UID_BASE     0x1FFFF7E8u       /* 96-bit unique ID (3 words) */
#define FSIZE_BASE   0x1FFFF7E0u       /* uint16_t flash size in KB */

/* ======================================================================
 * PWC - Power control   base 0x40007000  (RM Table 3-1, p67)
 * ==================================================================== */
typedef struct {
    __IO uint32_t CTRL;        /* 0x00 */
    __IO uint32_t CTRLSTS;     /* 0x04 */
    __IO uint32_t RESERVED0[2];/* 0x08-0x0C */
    __IO uint32_t LDOOV;       /* 0x10 */
} pwc_type;
#define PWC ((pwc_type *)0x40007000UL)

#define PWC_LDOOV_1P2V   0x0u   /* default; caps SCLK at 240 MHz */
#define PWC_LDOOV_1P3V   0x1u   /* required for 288 MHz */
#define PWC_LDOOV_1P1V   0x4u   /* LDOOVSEL=100; low-power idle (SCLK <= 108 MHz) */

/* ACC (auto clock calibration): trims HICK to 48 MHz +/-0.25% against the USB
 * SOF so USB runs crystal-less on HICK while the PLL is powered down at idle.
 * Reset compare-values (C1/C2/C3) are already set for the 48 MHz USB path, so
 * bring-up is just: clock-enable + set ENTRIM|CALON. */
#define ACC_CTRL1        (*(volatile uint32_t *)(0x40017400UL + 0x04u))
#define ACC_CTRL1_SOF_OTG2 (1u << 2) /* use OTGFS2 SOF as calibration reference */
#define ACC_CTRL1_ENTRIM (1u << 1)   /* calibrate HICKTRIM (finer, 20 kHz/step) */
#define ACC_CTRL1_CALON  (1u << 0)   /* start calibrating off USB_SOF */

/* ======================================================================
 * GPIO  (RM Table 6-10, p139)
 * ==================================================================== */
typedef struct {
    __IO uint32_t CFGR;   /* 0x00 mode: 2 bits/pin */
    __IO uint32_t OMODE;  /* 0x04 output type: 1 bit/pin */
    __IO uint32_t ODRVR;  /* 0x08 drive strength: 2 bits/pin */
    __IO uint32_t PULL;   /* 0x0C pull: 2 bits/pin */
    __IO uint32_t IDT;    /* 0x10 input data (ro) */
    __IO uint32_t ODT;    /* 0x14 output data */
    __IO uint32_t SCR;    /* 0x18 set[15:0]/clear[31:16] (wo) */
    __IO uint32_t WPR;    /* 0x1C write protect */
    __IO uint32_t MUXL;   /* 0x20 alt-func pins 0-7 (4 bits/pin) */
    __IO uint32_t MUXH;   /* 0x24 alt-func pins 8-15 (4 bits/pin) */
    __IO uint32_t CLR;    /* 0x28 clear (write 1 clears ODT bit) */
} gpio_type;
#define GPIOA ((gpio_type *)0x40020000UL)
#define GPIOB ((gpio_type *)0x40020400UL)
#define GPIOC ((gpio_type *)0x40020800UL)
#define GPIOD ((gpio_type *)0x40020C00UL)
#define GPIOH ((gpio_type *)0x40021C00UL)

#define GPIO_MODE_INPUT   0x0u
#define GPIO_MODE_OUTPUT  0x1u
#define GPIO_MODE_MUX     0x2u   /* alternate function */
#define GPIO_MODE_ANALOG  0x3u

#define GPIO_PULL_NONE    0x0u
#define GPIO_PULL_UP      0x1u
#define GPIO_PULL_DOWN    0x2u

#define GPIO_OTYPE_PP     0x0u   /* push-pull */
#define GPIO_OTYPE_OD     0x1u   /* open-drain */

#define MUX_OTGFS2   0xCu    /* OTG2_D-/D+ on PB14/PB15 (RM Table 6-2) */

/* Image bases for the PB0-preserving reboot (see pm5_deinit_and_jump). */
#define PM5_APP_BASE             0x08000000u   /* our firmware (bank1) */
#define PM5_ROM_BOOTLOADER_BASE  0x1FFF0000u   /* Artery ROM USB DFU (2e3c:df11) */

/* ======================================================================
 * DMA1/DMA2 + DMAMUX (RM ch.9) and basic timers TMR6/TMR7 (RM ch.14) -
 * used by the PB0-refresh DMA (pm5_start_gpiob_refresh_dma in hal.c) that
 * holds PB0 across the ROM's GPIOB reset during DFU. DMA1 base 0x40026400
 * (RM p60); the channel + DMAMUX layout is RM Table 9-5.
 * ==================================================================== */
typedef struct {
    __IO uint32_t CTRL;      /* +0x00 DMA_CxCTRL  */
    __IO uint32_t DTCNT;     /* +0x04 DMA_CxDTCNT */
    __IO uint32_t PADDR;     /* +0x08 DMA_CxPADDR */
    __IO uint32_t MADDR;     /* +0x0C DMA_CxMADDR */
    __IO uint32_t RESERVED;  /* +0x10 (0x14 stride) */
} dma_channel_type;

typedef struct {
    __IO uint32_t STS;            /* 0x00 */
    __IO uint32_t CLR;            /* 0x04 */
    dma_channel_type CH[7];       /* 0x08..0x94 channels 1..7 */
    __IO uint32_t RESERVED[27];   /* 0x94..0x100 */
    __IO uint32_t MUXSEL;         /* 0x100 */
    __IO uint32_t MUXCCTRL[7];    /* 0x104 MUXC1..7CTRL */
    __IO uint32_t MUXGCTRL[4];    /* 0x120 */
} dma_type;
#define DMA1 ((dma_type *)0x40026400UL)

#define DMA_CTRL_CHEN      (1u << 0)
#define DMA_CTRL_DTD_M2P   (1u << 4)   /* 1 = read from memory (mem->periph) */
#define DMA_CTRL_LM        (1u << 5)   /* circular */
#define DMA_CTRL_PINCM     (1u << 6)
#define DMA_CTRL_MINCM     (1u << 7)
#define DMA_CTRL_PWIDTH_32 (2u << 8)
#define DMA_CTRL_MWIDTH_32 (2u << 10)
#define DMA_CTRL_CHPL_VHI  (3u << 12)

#define DMA_MUXSEL_TBL_SEL (1u << 0)
#define DMAREQ_TMR7_OVERFLOW  9u
#define DMAMUX_REQ_SPI4_RX  106u       /* RM Table 9-3 flexible request id */
/* DMA STS/CLR: 4 flags per channel (GF, FDT, HDT, DTERR). Channel 1 = bits 0-3. */
#define DMA_STS_FDT1  (1u << 1)        /* channel-1 full-data-transfer done */
#define DMA_CLR_CH1   (0xFu << 0)      /* clear all channel-1 flags */

#define CRM_AHBEN1_DMA1EN  (1u << 22)
#define CRM_APB1EN_TMR7EN  (1u << 5)

typedef struct {
    __IO uint32_t CTRL1;        /* 0x00 */
    __IO uint32_t CTRL2;        /* 0x04 */
    __IO uint32_t RESERVED0;    /* 0x08 */
    __IO uint32_t IDEN;         /* 0x0C */
    __IO uint32_t ISTS;         /* 0x10 */
    __IO uint32_t SWEVT;        /* 0x14 */
    __IO uint32_t RESERVED1[3]; /* 0x18-0x20 */
    __IO uint32_t CVAL;         /* 0x24 */
    __IO uint32_t DIV;          /* 0x28 */
    __IO uint32_t PR;           /* 0x2C */
} tmr_basic_type;
#define TMR7 ((tmr_basic_type *)0x40001400UL)
#define TMR_CTRL1_TMREN   (1u << 0)
#define TMR_IDEN_OVFDEN   (1u << 8)   /* overflow -> DMA request */

/* Full-feature timer (TMR2): adds sub-mode/trigger (STCTRL) + channel input
 * (CM1/CCTRL) - used for the HF-emu ssp_clk external-clock counter on PB3. */
typedef struct {
    __IO uint32_t CTRL1;   /* 0x00 */
    __IO uint32_t CTRL2;   /* 0x04 */
    __IO uint32_t STCTRL;  /* 0x08: smsel[2:0], stis[6:4] */
    __IO uint32_t IDEN;    /* 0x0C */
    __IO uint32_t ISTS;    /* 0x10 */
    __IO uint32_t SWEVT;   /* 0x14 */
    __IO uint32_t CM1;     /* 0x18: c2c[9:8], c2df[15:12] */
    __IO uint32_t CM2;     /* 0x1C */
    __IO uint32_t CCTRL;   /* 0x20: c2en[4], c2p[5] */
    __IO uint32_t CVAL;    /* 0x24 */
    __IO uint32_t DIV;     /* 0x28 */
    __IO uint32_t PR;      /* 0x2C */
} tmr_full_type;
#define TMR2 ((tmr_full_type *)0x40000000UL)
#define CRM_APB1EN_TMR2EN  (1u << 0)
#define TMR_CTRL1_PMEN     (1u << 10)   /* 32-bit "plus mode" (TMR2/TMR5 only) */

/* Reboot in place, keeping the PB0 power-lock held (a hardware reset powers the
 * board off - it does not survive even on USB): deinit clocks to HICK, reset every
 * peripheral except GPIOB, then jump to `image_base`'s reset vector. Defined in
 * startup.c. image_base = PM5_APP_BASE to restart, PM5_ROM_BOOTLOADER_BASE to enter
 * the Artery ROM USB DFU. */
void pm5_deinit_and_jump(uint32_t image_base) __attribute__((noreturn));

/* ---- Proxmark5 board pins ---- */
#define PM5_PWR_LOCK_PIN   0    /* PB0: power-supply self-lock; push-pull, HIGH = keep alive */
/* The four A-D LEDs are single-colour red/orange, open-drain, active-low.
 * Physical positions on this board: A=PC13, B=PC14, C=PC15, D=PC3. The launcher
 * shows the slot in binary on A/B/C (bit0/1/2); D is unused. The blue "Fantasi
 * on" indicator is a separate RGB LED behind the I2C controller at 0x48 - see
 * pm5_rgb_set(). */
#define PM5_LED_A_PIN     13    /* PC13 (physical A, slot bit0) */
#define PM5_LED_B_PIN     14    /* PC14 (physical B, slot bit1) */
#define PM5_LED_C_PIN     15    /* PC15 (physical C, slot bit2) */
#define PM5_LED_D_PIN      3    /* PC3  (physical D, unused)    */
#define PM5_BUTTON_PIN    12    /* PB12, input pull-down, active-high */

/* Set the board's RGB LED (I2C 0x48) to an RGB888 colour. Defined in rfid.c
 * (reuses the software-I2C bit-bang). Blue = pm5_rgb_set(0, 0, 200). Returns
 * true if the RGB controller ACKed (it is ready a short time after reset, so a
 * caller run early retries until this is true). */
bool pm5_rgb_set(uint8_t r, uint8_t g, uint8_t b);

/* Light the two antenna-board LEDs (HF, LF) via the software-I2C antenna controller
 * (0x51, map register bit2=HFLED / bit1=LFLED, active-high). Defined in rfid.c.
 * Returns the I2C ack. */
bool pm5_ant_led(bool hf, bool lf);

/* Switch the software-I2C bit-bang between spec (~100 kHz, false) and fast
 * (tens of microseconds/toggle, true) so the idle LED fade can PWM the on/off
 * LEDs smoothly. Safe: scl() honours any clock-stretch the slave asserts.
 * Defined in rfid.c. */
void pm5_i2c_set_fast(bool fast);

/* Dynamic core-clock scaling (system.c). The core idles at 48 MHz to cut heat
 * and boosts to 288 MHz only for timing-critical work; boost/unboost is
 * refcounted, so wrap any span that needs full speed (RFID, busy-wait PWM) in a
 * boost/unboost pair. pm5_clk_settle() is the idle-hook's one-shot boot drop. */
void pm5_clk_boost(void);
void pm5_clk_unboost(void);
void pm5_clk_settle(void);

/* Set CFGR 2-bit mode field for pin `p` to `m`. */
static inline void gpio_set_mode(gpio_type *g, int p, uint32_t m)
{
    g->CFGR = (g->CFGR & ~(0x3u << (2 * p))) | (m << (2 * p));
}
/* Set the 2-bit pull field for pin `p`. */
static inline void gpio_set_pull(gpio_type *g, int p, uint32_t v)
{
    g->PULL = (g->PULL & ~(0x3u << (2 * p))) | (v << (2 * p));
}
/* Set the 4-bit alternate-function (MUX) selector for pin `p`. */
static inline void gpio_set_mux(gpio_type *g, int p, uint32_t af)
{
    if (p < 8) g->MUXL = (g->MUXL & ~(0xFu << (4 * p)))       | (af << (4 * p));
    else       g->MUXH = (g->MUXH & ~(0xFu << (4 * (p - 8)))) | (af << (4 * (p - 8)));
}
/* Set the output type (push-pull / open-drain) for pin `p`. */
static inline void gpio_set_otype(gpio_type *g, int p, uint32_t od)
{
    if (od) g->OMODE |=  (1u << p);
    else    g->OMODE &= ~(1u << p);
}

/* Output drive strength (ODRVR, 2 bits/pin): 0 = weak, 1 = stronger, 2 = moderate.
 * A fast master (e.g. the 24 MHz QSPI) needs stronger drive than the weak reset
 * default to switch the lines cleanly. */
#define GPIO_DRIVE_STRONGER 0x1u
static inline void gpio_set_drive(gpio_type *g, int p, uint32_t drv)
{
    g->ODRVR = (g->ODRVR & ~(0x3u << (2 * p))) | ((drv & 0x3u) << (2 * p));
}

/* Alternate-function (MUX) numbers used by the RFID frontend (DS pin tables). */
#define MUX_CLKOUT1  0x0u    /* PA8 = CLKOUT1/MCO1 */
#define MUX_SPI34    0x6u    /* SPI3 (PC10-12/PA15) and SPI4 (PB6-9) */

/* ======================================================================
 * SPI / I2S  (RM Table 13-2, p248). Used for the FPGA sample bus (SPI4 in
 * TI/"SSP" slave mode). SPI3 base kept for reference (the FPGA command bus,
 * which this port bit-bangs instead).
 * ==================================================================== */
typedef struct {
    __IO uint32_t CTRL1;   /* 0x00 */
    __IO uint32_t CTRL2;   /* 0x04 */
    __IO uint32_t STS;     /* 0x08 */
    __IO uint32_t DT;      /* 0x0C data */
    __IO uint32_t CPOLY;   /* 0x10 */
    __IO uint32_t RCRC;    /* 0x14 */
    __IO uint32_t TCRC;    /* 0x18 */
    __IO uint32_t I2SCTRL; /* 0x1C */
    __IO uint32_t I2SCLKP; /* 0x20 */
} spi_type;
#define SPI3 ((spi_type *)0x40003C00UL)   /* APB1 */
#define SPI4 ((spi_type *)0x40013400UL)   /* APB2 */

#define SPI_CTRL1_CLKPHA   (1u << 0)
#define SPI_CTRL1_CLKPOL   (1u << 1)
#define SPI_CTRL1_MSTEN    (1u << 2)   /* 1=master, 0=slave */
#define SPI_CTRL1_SPIEN    (1u << 6)
#define SPI_CTRL1_LTF      (1u << 7)   /* 0=MSB first, 1=LSB first */
#define SPI_CTRL1_SWCSIL   (1u << 8)
#define SPI_CTRL1_SWCSEN   (1u << 9)
#define SPI_CTRL1_FBN      (1u << 11)  /* 0=8-bit, 1=16-bit */

#define SPI_CTRL2_DMAREN   (1u << 0)
#define SPI_CTRL2_TIEN     (1u << 4)   /* TI ("SSP") frame mode */

#define SPI_STS_RDBF       (1u << 0)   /* receive buffer full */
#define SPI_STS_TDBE       (1u << 1)
#define SPI_STS_ROERR      (1u << 6)   /* receiver overflow */
#define SPI_STS_BF         (1u << 7)   /* busy */

#define CRM_APB2EN_SPI4EN  (1u << 13)

/* CRM CLKOUT1 (24 MHz on PA8 for the FPGA): source in CRM_CFG[22:21], DIV1 in
 * CRM_CFG[26:24], DIV2 in CRM_MISC1[27:24]. 288 MHz PLL /3 /4 = 24 MHz. */
#define CRM_CFG_CLKOUT1SEL_MSK   (0x3u << 21)
#define CRM_CFG_CLKOUT1SEL_PLL   (0x3u << 21)
#define CRM_CFG_CLKOUT1DIV1_MSK  (0x7u << 24)
#define CRM_CFG_CLKOUT1DIV1_3    (0x5u << 24)
#define CRM_MISC1_CLKOUT1DIV2_MSK (0xFu << 24)
#define CRM_MISC1_CLKOUT1DIV2_4   (0x9u << 24)

#endif /* FANTASI_AT32F435_H */
