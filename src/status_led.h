#pragma once

// Initialize the board user/status LED.
void StatusLed_init();

// Non-blocking status indication. Call from loop().
// airsense_present is the most recent health-poll result.
void StatusLed_tick(bool airsense_present);
