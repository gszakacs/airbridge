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
    // Light immediately at boot so the GPIO/polarity can be verified.
    led_write(true);
}

void StatusLed_tick() {
    const uint32_t now = millis();
    const system_state_t st = Arbiter::get_state();

    // Firmware transfer / bootloader activity: very fast blink.
    if (st == SYS_OTA_AIRSENSE || st == SYS_OTA_ESP || st == SYS_BOOTLOADER) {
        led_write(((now / 100U) & 1U) == 0U);
        return;
    }

    // AirSense/UART health fault: fast blink.
    if (st == SYS_ERROR) {
        led_write(((now / 150U) & 1U) == 0U);
        return;
    }

    // Wi-Fi unavailable: slow 1 Hz blink.
    if (!WiFiSetup::is_connected()) {
        led_write(((now / 500U) & 1U) == 0U);
        return;
    }

    // Normal operation, including therapy: steady ON.
    led_write(true);
}
