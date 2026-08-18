/* USB bus-state -> sleep-governance votes (shared TinyUSB platforms).
 *
 * Policy (stock Flipper's, via the vote API): USB configured and not
 * bus-suspended holds PWR_CLIENT_USB_ACTIVE (light sleep only); a suspended
 * bus (host asleep / charger) or an unplugged cable releases it so the
 * platform may sleep deeply - resume/attach wakes via the USB IRQ.
 *
 * All callbacks run in the USB task (tud_task context), never an ISR.
 * tud_umount_cb is owned by hal/storage/msc_device.c (FAT cache release);
 * it forwards here via usb_power_umount(). */

#include "../../hal/hal_power.h"
#include "tusb.h"

void usb_power_umount(void);

static bool s_mounted, s_suspended, s_voted;

static void usb_power_update(void)
{
    bool active = s_mounted && !s_suspended;
    if (active && !s_voted) {
        pwr_inhibit_enter(PWR_CLIENT_USB_ACTIVE);
        s_voted = true;
    } else if (!active && s_voted) {
        pwr_inhibit_exit(PWR_CLIENT_USB_ACTIVE);
        s_voted = false;
    }
}

void tud_mount_cb(void)
{
    s_mounted = true;
    s_suspended = false;
    usb_power_update();
}

/* Fallback for a build without hal/storage/msc_device.c (which owns the real
 * tud_umount_cb and forwards here). */
__attribute__((weak)) void tud_umount_cb(void)
{
    usb_power_umount();
}

void usb_power_umount(void)
{
    s_mounted = false;
    usb_power_update();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    s_suspended = true;
    usb_power_update();
}

void tud_resume_cb(void)
{
    s_suspended = false;
    usb_power_update();
}
