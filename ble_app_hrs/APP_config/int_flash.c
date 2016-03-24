#include <int_flash.h>
#include "button_led.h"
#include <string.h>
#include "spi_flash_MXIC.h"

#include "bsp.h"

#include "softdevice_handler.h"

#include <stdio.h>

//#define SD_ACTIVE

#define STORAGE_SRAM

#define STORAGE_EXT_FLASH

bool is_ROM_busy = true;

static void Flash_Interrupt_Handler(uint32_t sys_evt)
{
		if (sys_evt == NRF_EVT_FLASH_OPERATION_SUCCESS) {
			is_ROM_busy = false;
		}
}
		
void flash_page_erase(uint32_t * page_address)
{
#ifdef STORAGE_EXT_FLASH	
		spi_flash_eraseCmd(CMD_ERASE_4K, (uint32_t)page_address);
#else
	#ifdef SD_ACTIVE
		sd_flash_page_erase((uint32_t)page_address);
		is_ROM_busy = true;
		while (is_ROM_busy) {}	
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
	#ifdef SD_ACTIVE
		sd_flash_write(address, (uint32_t *)value, 1);
		is_ROM_busy = true;
		while (is_ROM_busy) {}	
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

uint16_t SRAM_size_10s_xHz;

void flash_data_set_init()
{		
		for (int i=0; i<BUFFER_EVENT_SIZE; i++)
			w_r_addr[i] = start_addr + i*four_page_size; 

#ifdef STORAGE_EXT_FLASH	
		spi_flash_init();
#else	
	#ifdef SD_ACTIVE
		softdevice_sys_evt_handler_set(Flash_Interrupt_Handler);
	#endif
#endif	
	
//		NRF_NVMC->ICACHECNF = 1;
}

uint8_t current_event_ID_get() 
{
		if (is_event_detected) 
			return event_type[w_buffer_event_ptr];
		else 
			return 0;
}

#ifdef STORAGE_SRAM
#define SRAM_SIZE 2000
uint32_t SRAM[SRAM_SIZE];
uint32_t ordering_SRAM[SRAM_SIZE];
uint16_t SRAM_ptr = 0;
bool SRAM_full = false;

uint8_t flash_data_set_write(float acc_g_xyz[3], float cal_acc_g_xyz[3], uint8_t event_ID)
{
		uint32_t temp;
		////if (SRAM_full==true) return buffer_event_count;// buffer full return immediately  

		if (event_ID) {
			is_event_detected = true;
			count_down_3s_xHz = 3*(1000/sensor_internal_ms);
			SRAM_size_10s_xHz = 10*4*(1000/sensor_internal_ms);
			event_addr[w_buffer_event_ptr] = w_r_addr[w_buffer_event_ptr];
			event_type[w_buffer_event_ptr] = event_ID;
			event_utc[w_buffer_event_ptr] = UTC_get();
			event_sensor_internal[w_buffer_event_ptr] = sensor_internal_ms;
		}
		
		temp = (uint32_t) ((float)(acc_g_xyz[0]+16)*100)<<16 | (uint32_t) ((float)(acc_g_xyz[1]+16)*100);
		SRAM[SRAM_ptr++] = temp;
		temp = (uint32_t) ((float)(acc_g_xyz[2]+16)*100); 
		SRAM[SRAM_ptr++] = temp;
		temp = (uint32_t) ((float)(cal_acc_g_xyz[0]+16)*100)<<16 | (uint32_t) ((float)(cal_acc_g_xyz[1]+16)*100); 
		SRAM[SRAM_ptr++] = temp;
		temp = (uint32_t) ((float)(cal_acc_g_xyz[2]+16)*100); 		
		SRAM[SRAM_ptr++] = temp;
	
		if (SRAM_ptr==SRAM_SIZE) SRAM_ptr = 0;

		if (is_event_detected) 
			count_down_3s_xHz--;
		if (count_down_3s_xHz==0) {
			////SRAM_full = true;
			is_event_detected = false;
			count_down_3s_xHz = 3*(1000/sensor_internal_ms);
			
			if (buffer_event_count<BUFFER_EVENT_SIZE){
				buffer_event_count++;
				//erase then write into flash
				for(uint32_t i=0; i<four_page_size/2; i+=0x1000)
					flash_page_erase((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i));
				uint32_t event_start_addr;
//				if (SRAM_ptr+1>=SRAM_size_10s_xHz) {
//					event_start_addr = SRAM_ptr+1-SRAM_size_10s_xHz;
//					for (uint16_t i=0; i<SRAM_size_10s_xHz; i++)
//						ordering_SRAM[i] = SRAM[event_start_addr+i];
//				} else {
//					event_start_addr = SRAM_ptr+1+SRAM_SIZE-SRAM_size_10s_xHz;
//					for (uint16_t i=0; i<SRAM_size_10s_xHz; i++) {
//						uint16_t ptr;
//						ptr = event_start_addr+i;
//						if (ptr >= SRAM_SIZE) ptr-=SRAM_SIZE;
//						ordering_SRAM[i] = SRAM[ptr];
//					}						
//				}
					for (uint16_t i=0; i<SRAM_size_10s_xHz; i++) {
						uint16_t ptr;
						ptr = SRAM_ptr + i;
						if (ptr >= SRAM_SIZE) ptr-=SRAM_SIZE;
						ordering_SRAM[i] = SRAM[ptr];
					}
				uint32_t w_ptr = w_r_addr[w_buffer_event_ptr];
				for (uint16_t i=0; i<SRAM_size_10s_xHz; i++) {
					flash_word_write((uint32_t*)w_ptr, ordering_SRAM[i]);
					w_ptr+=4;
				}
				if(++w_buffer_event_ptr==BUFFER_EVENT_SIZE) w_buffer_event_ptr = 0;
			} else {
			}
				
				
			
			
//			if (w_buffer_event_ptr == r_buffer_event_ptr) {
//				if (w_buffer_event_ptr == 0) w_buffer_event_ptr = BUFFER_EVENT_SIZE-1;
//				else w_buffer_event_ptr--;
//			}
//			
//			buffer_event_count++;
		}
//		printf("w_buffer_event_ptr (r): %x (%x)\n\r", w_buffer_event_ptr,r_buffer_event_ptr);
		return buffer_event_count;
}	
#else
uint8_t flash_data_set_write(float acc_g_xyz[3], float cal_acc_g_xyz[3], uint8_t event_ID)
{
		uint32_t temp;

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

		//if (buffer_event_count==BUFFER_EVENT_SIZE-1) return buffer_event_count;// buffer full return immediately  
		
		if (is_event_detected) 
			count_down_3s_xHz--;
		if (count_down_3s_xHz==0) {
			is_event_detected = false;
			count_down_3s_xHz = 3*(1000/sensor_internal_ms);
			uint32_t last_w_r_addr = w_r_addr[w_buffer_event_ptr];
			uint32_t last_ptr = w_addr_ptr%four_page_size;// - w_r_addr[w_buffer_event_ptr];
			uint32_t last_w_buffer_event_ptr = w_buffer_event_ptr;

			if (++buffer_event_count>=BUFFER_EVENT_SIZE-1)
				buffer_event_count=BUFFER_EVENT_SIZE-1;

			if(++w_buffer_event_ptr==BUFFER_EVENT_SIZE) w_buffer_event_ptr = 0;
			
			if (w_buffer_event_ptr == r_buffer_event_ptr) {
				w_buffer_event_ptr = last_w_buffer_event_ptr;
				w_addr_ptr = last_w_r_addr;
				return buffer_event_count;
			}
			
			// start from the same ptr in next buffer space
			w_addr_ptr = w_r_addr[w_buffer_event_ptr] + last_ptr;

			// erase before copy the whole 4 page data to next buffer space
			for(uint32_t i=0; i<four_page_size; i+=0x1000)
				flash_page_erase((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i));
//#ifdef STORAGE_EXT_FLASH// NOT Stable
//			uint8_t wr_page_data[0x1000];
//			for(uint32_t i=0; i<four_page_size; i+=0x1000) {
//				spi_flash_readpage((uint32_t) last_w_r_addr+i, wr_page_data, 0x1000);
//				spi_flash_writepage((uint32_t) w_r_addr[w_buffer_event_ptr]+i, wr_page_data, 0x1000);
//			}
//#else			
			/*Bug, the space after last_ptr is be written*/ 
			for(uint32_t i=0; i<four_page_size; i+=4)
				flash_word_write((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i), flash_word_read((uint32_t *)(last_w_r_addr+i)));
			flash_page_erase((uint32_t*)(w_r_addr[w_buffer_event_ptr]+last_ptr/page_size));
			for(uint32_t i=last_ptr-last_ptr%page_size; i<last_ptr; i+=4)
				flash_word_write((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i), flash_word_read((uint32_t *)(last_w_r_addr+i)));
				
//			if (last_ptr >= to_addr_7s_xHz) {
//					for(uint32_t i=last_ptr-to_addr_7s_xHz; i<last_ptr; i+=4)
//						flash_word_write((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i), flash_word_read((uint32_t *)(last_w_r_addr+i)));
//			} else {
//					for(uint32_t i=four_page_size+last_ptr-to_addr_7s_xHz; i<four_page_size; i+=4)
//						flash_word_write((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i), flash_word_read((uint32_t *)(last_w_r_addr+i)));
//					for(uint32_t i=0; i<last_ptr; i+=4)
//						flash_word_write((uint32_t*)(w_r_addr[w_buffer_event_ptr]+i), flash_word_read((uint32_t *)(last_w_r_addr+i)));
//			}
//#endif				
		}
//		printf("w_buffer_event_ptr (r): %x (%x)\n\r", w_buffer_event_ptr,r_buffer_event_ptr);
		return buffer_event_count;
}	
#endif
int flag = 1;

#ifdef STORAGE_SRAM
int flash_data_set_read(uint32_t *acc_g_xy, uint32_t *acc_g_z, uint32_t *cal_acc_g_xy, uint32_t *cal_acc_g_z, uint8_t *event_ID, uint32_t *UTC, uint8_t *sensor_interval)
{
		if (buffer_event_count == 0) return -1;// buffer empty return immediately  
	
		if (flag) {
			count_down_10s_xHz=10*(1000/event_sensor_internal[r_buffer_event_ptr]);
			r_addr_ptr = event_addr[r_buffer_event_ptr];
			if (r_addr_ptr < w_r_addr[r_buffer_event_ptr]) r_addr_ptr+= four_page_size;
		}
		flag = 0;

		*event_ID = event_type[r_buffer_event_ptr];
		*UTC = event_utc[r_buffer_event_ptr];
		*sensor_interval = event_sensor_internal[r_buffer_event_ptr];
//		printf("r_addr_ptr: %x\n\r", r_addr_ptr);
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

		count_down_10s_xHz--;
		if (count_down_10s_xHz==0) {
			r_buffer_event_ptr++;
			if(r_buffer_event_ptr==BUFFER_EVENT_SIZE) r_buffer_event_ptr = 0;
			buffer_event_count--;
			flag = 1;
		}
		return count_down_10s_xHz;		
}
#else
int flash_data_set_read(uint32_t *acc_g_xy, uint32_t *acc_g_z, uint32_t *cal_acc_g_xy, uint32_t *cal_acc_g_z, uint8_t *event_ID, uint32_t *UTC, uint8_t *sensor_interval)
{
		if (buffer_event_count == 0) return -1;// buffer empty return immediately  
		//bug?	if (w_buffer_event_ptr == r_buffer_event_ptr) return -1;
	
		if (flag) {
			count_down_10s_xHz=10*(1000/event_sensor_internal[r_buffer_event_ptr]);
			to_addr_7s_xHz = 7*(1000/event_sensor_internal[r_buffer_event_ptr])*16;
			r_addr_ptr = event_addr[r_buffer_event_ptr]-to_addr_7s_xHz;
			if (r_addr_ptr < w_r_addr[r_buffer_event_ptr]) r_addr_ptr+= four_page_size;
			if (r_addr_ptr < 0x60000) {
				r_addr_ptr = r_addr_ptr;
//				printf("err r_addr_ptr: %x\n\r", r_addr_ptr);
			}
		}
		flag = 0;

		*event_ID = event_type[r_buffer_event_ptr];
		*UTC = event_utc[r_buffer_event_ptr];
		*sensor_interval = event_sensor_internal[r_buffer_event_ptr];
		//printf("r_addr_ptr: %x\n\r", r_addr_ptr);
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
#endif