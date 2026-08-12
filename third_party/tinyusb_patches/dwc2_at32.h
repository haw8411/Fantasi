/*
 * Artery AT32F435 DWC2 port glue for TinyUSB.
 *
 * The AT32F435 OTGFS core is a Synopsys DWC2 full-speed device controller,
 * register-compatible with the STM32 OTG_FS (same GCCFG.PWRDWN PHY enable and
 * GUSBCFG turnaround). Fantasi drives OTGFS2 (base 0x40040000, IRQ 77) on
 * pins PB14/PB15. Part of the Fantasi/Proxmark5 port - see
 * third_party/tinyusb_patches/README.md.
 */
#ifndef DWC2_AT32_H_
#define DWC2_AT32_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f435.h"   /* NVIC helpers + OTGFS2_IRQn (Fantasi minimal device header) */

#define DWC2_EP_MAX        8
#define DWC2_EP_FIFO_SIZE  1280   /* OTGFS dedicated FIFO RAM (bytes) */

// Fantasi uses OTGFS2 (PB14/PB15); base 0x40040000, global IRQ 77.
static const dwc2_controller_t _dwc2_controller[] = {
  { .reg_base = 0x40040000UL, .irqnum = (uint32_t) OTGFS2_IRQn,
    .ep_count = DWC2_EP_MAX, .ep_fifo_size = DWC2_EP_FIFO_SIZE }
};

extern uint32_t SystemCoreClock;

TU_ATTR_ALWAYS_INLINE static inline void dwc2_dcd_int_enable(uint8_t rhport) {
  NVIC_EnableIRQ((IRQn_Type) _dwc2_controller[rhport].irqnum);
}

TU_ATTR_ALWAYS_INLINE static inline void dwc2_dcd_int_disable(uint8_t rhport) {
  NVIC_DisableIRQ((IRQn_Type) _dwc2_controller[rhport].irqnum);
}

TU_ATTR_ALWAYS_INLINE static inline void dwc2_remote_wakeup_delay(void) {
  // try to delay for 1 ms
  uint32_t count = SystemCoreClock / 1000;
  while (count--) __NOP();
}

// AT32 OTGFS_GCCFG (RM 21.6.3.12): bit16 PWRDOWN (1 = transceiver active),
// bit21 VBUSIG (ignore VBUS sensing).
#define AT32_GCCFG_VBUSIG   (1u << 21)

// MCU specific PHY init, called before core reset: power up the on-chip FS
// transceiver and ignore VBUS sensing. The Proxmark5 routes no usable VBUS to the
// OTG core, so with VBUS sensing active the core's session never becomes valid and
// the DP pull-up is never asserted (no connect). VBUSIG forces it to connect.
// TinyUSB's stock DWC2 phy_init sets only PWRDWN, hence this override.
static inline void dwc2_phy_init(dwc2_regs_t* dwc2, uint8_t hs_phy_type) {
  (void) hs_phy_type;
  dwc2->stm32_gccfg |= STM32_GCCFG_PWRDWN | AT32_GCCFG_VBUSIG;
}

// MCU specific PHY update, called AFTER core reset: set the FS turnaround time.
static inline void dwc2_phy_update(dwc2_regs_t* dwc2, uint8_t hs_phy_type) {
  (void) hs_phy_type;
  // AHB runs at 288 MHz (>= 32 MHz) -> turnaround 0x6 (per DWC2/STM32 FS table).
  dwc2->gusbcfg = (dwc2->gusbcfg & ~GUSBCFG_TRDT_Msk) | (0x6u << GUSBCFG_TRDT_Pos);
}

#ifdef __cplusplus
}
#endif

#endif /* DWC2_AT32_H_ */
