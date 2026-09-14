#pragma once

// Initialize the board user/status LED.
void StatusLed_init();

// Non-blocking status indication. Call from loop().
void StatusLed_tick();
