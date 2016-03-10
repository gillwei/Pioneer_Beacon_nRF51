
#ifndef BUTTON_LED_H
#define BUTTON_LED_H

#include "boards.h"

void button_led_init(void);

void large_accident_led(void);
void middle_accident_led(void);
void small_accident_led(void);
void confirmation_led(void);

void UTC_set(uint32_t currentUTC);
uint32_t UTC_get(void);

#endif /* BUTTON_LED_H */
