# TinyUSB modifications (Fantasi)

`third_party/tinyusb/` is a **fetched dependency** - it's `.gitignore`d and cloned
on demand (pinned to `TUSB_TAG`, currently **0.17.0**) by the top-level
`check-tinyusb` make target. Edits made directly in that tree are therefore
lost on a re-clone. This directory is the tracked source of truth for changes
we make to upstream TinyUSB.

## Contents

| File | Applies to | What it is |
|---|---|---|
| `dcd_at91sam7s.c` | **new file** → `src/portable/microchip/at91sam7s/` | The AT91SAM7S (ARM7TDMI) UDP device controller. Upstream has no SAM7S DCD. Adapted from `dcd_samg.c`; includes the DPRAM byte-leakage guard (gate UDP IRQ around FDR access; separate OUT/IN passes), the short-packet "stray char" fix (advance by actual `RXBYTECNT`, not epsize), and EP0 early-status-OUT handling. |
| `register-at91sam7s-mcu.patch` | `src/tusb_option.h`, `src/common/tusb_mcu.h` | Registers `OPT_MCU_AT91SAM7S` and its endpoint config so the MCU is a known target. |
| `vendor_device-ep_addr-reset.patch` | `src/class/vendor/vendor_device.c` | Clears `rx/tx.stream.ep_addr` in `vendord_reset`. `tud_vendor_n_mounted()` keys off `ep_addr`, but upstream never clears it on a bus reset, so the 2nd enumeration of the vendor config saw the interface still "mounted", `vendord_open` found no free interface, and `SET_CONFIGURATION` stalled - the Proxmark3 CDC↔WebUSB switch stall. Cross-platform (every target compiles `vendor_device.c`); upstream-worthy - re-check on the next TinyUSB bump. |
| `dwc2_at32.h` | **new file** → `src/portable/synopsys/dwc2/` | Artery AT32F435 (Proxmark5) DWC2 port glue: OTGFS2 base `0x40040000`, IRQ 77, on-chip FS PHY enable. Upstream 0.17.0 has no AT32 port. Includes `at32f435.h` (the platform's minimal device header) for NVIC + `OTGFS2_IRQn`. |
| `register-at32f435-mcu.patch` | `src/tusb_option.h`, `src/common/tusb_mcu.h` | Registers `OPT_MCU_AT32F435` (2500) and its `TUP_USBIP_DWC2` + `TUP_DCD_ENDPOINT_MAX 8` so the MCU is a known DWC2 target. |
| `dwc2-at32-dcd-include.patch` | `src/portable/synopsys/dwc2/dcd_dwc2.c` | Adds the `OPT_MCU_AT32F435 -> dwc2_at32.h` include branch to the DCD's per-chip header dispatch. |

These files are the **complete** modification set vs pristine 0.17.0. To verify:
`git -C third_party/tinyusb diff` shows the edits to existing files. There are
also two new additions - the SAM7S `at91sam7s/` DCD and the AT32 `dwc2_at32.h`.

## Applying to a fresh clone (automated)

The top-level `check-tinyusb` make target applies all of this automatically after
the clone - every `make PLATFORM=<x>` runs it. It is idempotent and
**non-destructive**:

- **Patches** are applied with a guard: `git apply --reverse --check` (already
  applied → skip), else `git apply --check` (applies cleanly → apply), else a
  warning and skip. It never clobbers a hand-edited clone.
- **`dcd_at91sam7s.c`** (a new file) is copied in **only when absent**, so a clone
  driver you're actively editing is never overwritten.

To do it by hand instead:

```sh
cd third_party/tinyusb
git apply ../tinyusb_patches/register-at91sam7s-mcu.patch
git apply ../tinyusb_patches/vendor_device-ep_addr-reset.patch
mkdir -p src/portable/microchip/at91sam7s
cp ../tinyusb_patches/dcd_at91sam7s.c src/portable/microchip/at91sam7s/
```

Without these, a fresh `make PLATFORM=proxmark3` fails to compile (missing DCD +
`OPT_MCU_AT91SAM7S`), and the CDC↔WebUSB switch regresses on all targets.

## Re-capturing after editing the clone

If you fix something *in the clone*, immediately refresh this stash so it never
goes stale again:

```sh
cd third_party/tinyusb
git diff src/class/vendor/vendor_device.c   > ../tinyusb_patches/vendor_device-ep_addr-reset.patch
git diff src/tusb_option.h src/common/tusb_mcu.h > ../tinyusb_patches/register-at91sam7s-mcu.patch
cp src/portable/microchip/at91sam7s/dcd_at91sam7s.c ../tinyusb_patches/dcd_at91sam7s.c
```

To verify the stash reconstructs the clone exactly: extract the pristine files
with `git show 0.17.0:<path>`, `git apply` the patches + drop in the DCD, and
`cmp` against the clone.
