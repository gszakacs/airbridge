#include "status_led.h"
#include "board.h"
#include "wifi_setup.h"
#include "uart_arbiter.h"
#include <Arduino.h>

#ifndef AB_LED_ACTIVE_LOW
#define AB_LED_ACTIVE_LOW 0
#endif

static inline void led_write(bool on) {
#if AB_LED_ACTIVE_LOW
    digitalWrite(PIN_LED, on ? LOW : HIGH);
#else
    digitalWrite(PIN_LED, on ? HIGH : LOW);
#endif
}

void StatusLed_init() {
    pinMode(PIN_LED, OUTPUT);
    // Briefly light the LED during startup so the hardware path is obvious.
    led_write(true);
}

void StatusLed_tick(bool airsense_present) {
    const uint32_t now = millis();
    const system_state_t st = Arbiter::get_state();

    // Highest priority: firmware transfer / bootloader activity.
    if (st == SYS_OTA_AIRSENSE || st == SYS_OTA_ESP || st == SYS_BOOTLOADER) {
        led_write(((now / 100U) & 1U) == 0U);          // rapid 5 Hz blink
        return;
    }

    // AirSense/UART health fault.
    if (st == SYS_ERROR) {
        led_write(((now / 150U) & 1U) == 0U);          // fast ~3.3 Hz blink
        return;
    }

    // Wi-Fi unavailable: obvious slow blink.
    if (!WiFiSetup::is_connected()) {
        led_write(((now / 500U) & 1U) == 0U);          // 1 Hz blink
        return;
    }

    // Wi-Fi is good but the AirSense has not answered the most recent health
    // poll. Two short flashes every 2 seconds distinguish this from Wi-Fi loss.
    if (!airsense_present) {
        const uint32_t phase = now % 2000U;
        const bool on = (phase < 120U) || (phase >= 260U && phase < 380U);
        led_write(on);
        return;
    }

    // Normal operation (idle or therapy): steady ON.
    led_write(true);
}
