#include <int_flash.h>
#include "button_led.h"
#include <string.h>
#include "spi_flash_MXIC.h"

#include "bsp.h"

#include <stdio.h>

#define STORAGE_EXT_FLASH

void flash_page_erase(uint32_t * page_address)
{
#ifdef STORAGE_EXT_FLASH	
		spi_flash_eraseCmd(CMD_ERASE_4K, (uint32_t)page_address);
#else
    // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;
		
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
#endif
}


/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
void flash_word_write(uint32_t * address, uint32_t value)
{
#ifdef STORAGE_EXT_FLASH		
		uint8_t tx_data[4];
		tx_data[3] = (uint8_t) (value >> 24);
		tx_data[2] = (uint8_t) (value >> 16);
		tx_data[1] = (uint8_t) (value >> 8);
		tx_data[0] = (uint8_t) value;
	
		spi_flash_writepage((uint32_t) address, tx_data, 4);
#else		
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
#endif
}

uint32_t flash_word_read(uint32_t * address)
{
#ifdef STORAGE_EXT_FLASH		
		uint8_t rx_data[4];
		spi_flash_readpage((uint32_t) address, rx_data, 4);
		uint32_t ret = rx_data[0] | (rx_data[1] << 8) | (rx_data[2] << 16) | (rx_data[3] << 24);
		return ret;
#else	
    return *address;
#endif	
}

/************ Following is used for Pioner Beacon Service ***********/
uint32_t start_addr = 0x60000;
uint32_t w_addr_ptr = 0x60000;
uint32_t r_addr_ptr = 0x60000;
uint32_t page_size = 0x1000;
uint32_t four_page_size = 4*0x1000;

extern uint8_t sensor_internal_ms;

#define BUFFER_EVENT_SIZE 		8
uint32_t w_r_addr[BUFFER_EVENT_SIZE];
uint32_t event_addr[BUFFER_EVENT_SIZE];
uint8_t event_type[BUFFER_EVENT_SIZE];
uint32_t event_utc[BUFFER_EVENT_SIZE];
uint8_t event_sensor_internal[BUFFER_EVENT_SIZE];
uint8_t buffer_event_count = 0;
uint8_t w_buffer_event_ptr = 0;
uint8_t r_buffer_event_ptr = 0;
uint16_t count_down_3s_xHz = 3*DEFAULT_HZ;
uint16_t count_down_10s_xHz = 10*DEFAULT_HZ;
uint16_t to_addr_7s_xHz = 7*DEFAULT_HZ*16;
bool is_event_detected = false;

		uint32_t t_test = 0x100123;
		uint32_t r_test = 0;
		
void flash_data_set_init()
{		
		for (int i=0; i<BUFFER_EVENT_SIZE; i++)
			w_r_addr[i] = start_addr + i*four_page_size; 
	
		spi_flash_init();

//		uint8_t tx_test[3] = {0x1B, 0x1C, 0x1D}; 
//		uint8_t rx_test[3] = {0}; 

//		spi_flash_eraseCmd(CMD_ERASE_4K, 0);
//		spi_flash_writepage(0x00, tx_test, 3);
//		spi_flash_readpage(0x00, rx_test, 3);	
		

		
		flash_page_erase((uint32_t*)0x60000);
		flash_word_write((uint32_t*)0x60000, t_test);
		r_test = flash_word_read((uint32_t*)0x60000);
}

uint8_t current_event_ID_get() 
{
		if (is_event_detected) 
			return event_type[w_buffer_event_ptr];
		else 
			return 0;
}

uint8_t flash_data_set_write(float acc_g_xyz[3], float cal_acc_g_xyz[3], uint8_t event_ID)
{
		uint32_t temp;
		if (buffer_event_count==BUFFER_EVENT_SIZE) return buffer_event_count;// buffer full return immediately  
			
		if (event_ID) {
			is_event_detected = true;
			count_down_3s_xHz = 3*(1000/sensor_internal_ms);
			event_addr[w_buffer_event_ptr] = w_addr_ptr;
			event_type[w_buffer_event_ptr] = event_ID;
			event_utc[w_buffer_event_ptr] = UTC_get();
			event_sensor_internal[w_buffer_event_ptr] = sensor_internal_ms;
		}
		
		if (w_addr_ptr%page_size == 0) flash_page_erase((uint32_t*)w_addr_ptr);// new page, erase before writing
	
		if (acc_g_xyz != NULL) {
			temp = (uint32_t) ((float)(acc_g_xyz[0]+16)*100)<<16 | (uint32_t) ((float)(acc_g_xyz[1]+16)*100);
			flash_word_write((uint32_t*)w_addr_ptr, temp);
		}
		w_addr_ptr+=4;

		if (acc_g_xyz != NULL) {	
			temp = (uint32_t) ((float)(acc_g_xyz[2]+16)*100); 
			flash_word_write((uint32_t*)w_addr_ptr, temp);
		}
		w_addr_ptr+=4;

		if (cal_acc_g_xyz != NULL) {
			temp = (uint32_t) ((float)(cal_acc_g_xyz[0]+16)*100)<<16 | (uint32_t) ((float)(cal_acc_g_xyz[1]+16)*100); 
			flash_word_write((uint32_t*)w_addr_ptr, temp);
		}	
		w_addr_ptr+=4;

		if (cal_acc_g_xyz != NULL) {		
			temp = (uint32_t) ((float)(cal_acc_g_xyz[2]+16)*100); 
			flash_word_write((uint32_t*)w_addr_ptr, temp);
		}
		w_addr_ptr+=4;
	
		if (w_addr_ptr == w_r_addr[w_buffer_event_ptr]+four_page_size) // ring buffering management, round in 4 pase size
			w_addr_ptr = w_r_addr[w_buffer_event_ptr];
		
		if (is_event_detected) 
			count_down_3s_xHz--;
		if (count_down_3s_xHz==0) {
			is_event_detected = false;
			count_down_3s_xHz = 3*(1000/sensor_internal_ms);
			uint32_t last_w_r_addr = w_r_addr[w_buffer_event_ptr];
			uint32_t last_ptr = w_addr_ptr - w_r_addr[w_buffer_event_ptr];
			if(++w_buffer_event_ptr==BUFFER_EVENT_SIZE) w_buffer_event_ptr = 0;
			
			// start from the same ptr in next buffer space
			w_addr_ptr = w_r_addr[w_buffer_event_ptr] + last_ptr;

			// erase before copy the whole 4 page data to next buffer space
			for(uint32_t i=0; i<four_page_size; i+=0x1000)
				flash_page_erase((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i));
			for(uint32_t i=0; i<four_page_size; i+=4)
				flash_word_write((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i), flash_word_read((uint32_t *)(last_w_r_addr+i)));
			
			buffer_event_count++;
		}
		return buffer_event_count;
}	

int flag = 1;

int flash_data_set_read(uint32_t *acc_g_xy, uint32_t *acc_g_z, uint32_t *cal_acc_g_xy, uint32_t *cal_acc_g_z, uint8_t *event_ID, uint32_t *UTC, uint8_t *sensor_interval)
{
		if (buffer_event_count == 0) return -1;// buffer empty return immediately  
		
		if (flag) {
			count_down_10s_xHz=10*(1000/event_sensor_internal[r_buffer_event_ptr]);
			to_addr_7s_xHz = 7*(1000/event_sensor_internal[r_buffer_event_ptr])*16;
			r_addr_ptr = event_addr[r_buffer_event_ptr]-to_addr_7s_xHz;
			if (r_addr_ptr < w_r_addr[r_buffer_event_ptr]) r_addr_ptr+= four_page_size;
		}
		flag = 0;

		*event_ID = event_type[r_buffer_event_ptr];
		*UTC = event_utc[r_buffer_event_ptr];
		*sensor_interval = event_sensor_internal[r_buffer_event_ptr];
		
		if (acc_g_xy != NULL) {
			*acc_g_xy = flash_word_read((uint32_t *)r_addr_ptr);
		}
		r_addr_ptr+=4;
		
		if (acc_g_z != NULL) {
			*acc_g_z = flash_word_read((uint32_t *)r_addr_ptr);
		}
		r_addr_ptr+=4;
		
		if (cal_acc_g_xy != NULL) {
			*cal_acc_g_xy = flash_word_read((uint32_t *)r_addr_ptr);
		}
		r_addr_ptr+=4;
		
		if (cal_acc_g_z != NULL) {
			*cal_acc_g_z = flash_word_read((uint32_t *)r_addr_ptr);	
		}
		r_addr_ptr+=4;

		if (r_addr_ptr == w_r_addr[r_buffer_event_ptr]+four_page_size) // ring buffering management, round in 4 pase size
			w_addr_ptr = w_r_addr[r_buffer_event_ptr];
		
		count_down_10s_xHz--;
		if (count_down_10s_xHz==0) {
			r_buffer_event_ptr++;
			if(r_buffer_event_ptr==BUFFER_EVENT_SIZE) r_buffer_event_ptr = 0;
			buffer_event_count--;
			flag = 1;
		}
		return count_down_10s_xHz;
}	