
#ifndef EVENT_DETECTION_H
#define EVENT_DETECTION_H

#include "boards.h"
	
void uart_init(void);
void event_sampling_interval_set(uint8_t internal_ms);
void event_detection_routine(void);

#endif /* EVENT_DETECTION_H */
