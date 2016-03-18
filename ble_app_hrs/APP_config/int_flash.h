
#ifndef INT_FLASH_H
#define INT_FLASH_H

#include "bsp.h"

#define DEFAULT_HZ		50

void flash_page_erase(uint32_t * page_address);
void flash_word_write(uint32_t * address, uint32_t value);
uint32_t flash_word_read(uint32_t * address);

void flash_data_set_init();
uint8_t current_event_ID_get();
uint8_t flash_data_set_write(float acc_g_xyz[3], float cal_acc_g_xyz[3], uint8_t event_type);
int flash_data_set_read(uint32_t *acc_g_xy, uint32_t *acc_g_z, uint32_t *cal_acc_g_xy, uint32_t *cal_acc_g_z, uint8_t *event_ID, uint32_t *UTC, uint8_t *sensor_interval);

#endif /* INT_MASTER_H */
