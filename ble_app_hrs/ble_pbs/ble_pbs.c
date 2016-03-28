/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

/**
 * File: ble_pbs.c
 * Description: BLE Pioneer Beacon Service 
 *
 * Copyright 2016 by CYNTEC Corporation.  All rights reserved.
 * First Author: Gill Wei
 */


#include "ble_pbs.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "button_led.h"
#include "nrf_delay.h"
#include "event_detection.h"

//15422fe0-bce5-11e5-a837-0800200c9a66
#define PBS_UUID_BASE 										 {0x66,0x9A,0x0C,0x20,0x00,0x08,0x37,0xA8,0xE5,0x11,0xE5,0xBC,0xE0,0x2F,0x42,0x15}
#define PBS_SERVICE_SHORT_UUID 										0x2FE0
#define PBS_BEACON_SETTING_CHAR_SHORT_UUID 				0x2FE1
#define PBS_EVENT_STORAGE_CHAR_SHORT_UUID 				0x2FE2
#define PBS_DATA_REPORT_HEADER_CHAR_SHORT_UUID 		0x2FE3
#define PBS_CAL_DATA_REPORT_CHAR_SHORT_UUID 			0x2FE4
#define PBS_RAW_DATA_REPORT_CHAR_SHORT_UUID 			0x2FE5
#define PBS_THRESHOLD_SETTING_CHAR_SHORT_UUID			0X2FE6

#define PBS_BOARDING_DETECTION_REPORT_UUID {0xC8,0xAC,0x3A,0xC8,0xB4,0x89,0x84,0x83,0xD3,0x4A,0x87,0xE1,0x55,0x9F,0x81,0x3C}
#define PBS_BUTTON_DATA_REPORT_UUID 			 {0xBA,0x72,0x1C,0x1E,0x98,0x8B,0x19,0xAC,0xC7,0x45,0x4A,0x7F,0x09,0xE4,0xF1,0x15}
#define TSC_ALLDATA_LENGTH														17
#define BSC_ALLDATA_LENGTH														13
#define ESC_ALLDATA_LENGTH														 3
#define DRHC_ALLDATA_LENGTH														14
#define CDRC_ALLDATA_MIN_LENGTH												 3 // NOT USED
#define CDRC_ALLDATA_MAX_LENGTH												20
#define RDRC_ALLDATA_MIN_LENGTH												 3 // NOT USED
#define RDRC_ALLDATA_MAX_LENGTH												20
#define BDC_ALLDATA_LENGTH                             4
#define BDRC_ALLDATA_LENGTH                            4

#define PBS_DEBUG																			 1

static uint16_t                 service_handle;
//static ble_gatts_char_handles_t beacon_setting_handles;
//static ble_gatts_char_handles_t event_storage_handles;
//static ble_gatts_char_handles_t data_report_header_handles;
//static ble_gatts_char_handles_t cal_data_report_handles;
//static ble_gatts_char_handles_t raw_data_report_handles;

bool esc_notify_flag = false;
bool esc_write_flag = false;
bool start_dw_flag = false;
bool drhc_notify_flag = false;
bool cdrc_notify_flag = false;
bool rdrc_notify_flag = false;
bool bdrc_notify_flag = false;
//extern ble_pbs_t m_pbs;
	

// Encode All Characteristics' Data
static uint8_t uint16_big_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[1] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[0] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}

static void tsc_data_encode(uint8_t * p_encoded_buffer, const tsc_t * p_tsc)
{
		uint8_t len = 0;
	p_encoded_buffer[len++] = p_tsc->flag;
		len += uint16_big_encode(p_tsc->small_accident_level_x, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->small_accident_level_y, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->medium_accident_level, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->high_accident_level, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->hard_accelaration_level, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->hard_braking_level, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->hard_steering_level_left, &p_encoded_buffer[len]);
		len += uint16_big_encode(p_tsc->hard_steering_level_right, &p_encoded_buffer[len]);
}

static void bsc_data_encode(uint8_t * p_encoded_buffer, const bsc_t * p_bsc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_bsc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_bsc->flag;
	len += uint16_big_encode(p_bsc->sampling_frequency, &p_encoded_buffer[len]);
	len += uint16_big_encode(p_bsc->ambient_sensor_value, &p_encoded_buffer[len]);
	len += uint16_big_encode(p_bsc->acc_voltage, &p_encoded_buffer[len]);
	len += uint16_big_encode(p_bsc->ble_output_power, &p_encoded_buffer[len]);
		len += uint32_big_encode(p_bsc->current_utc, &p_encoded_buffer[len]);
}

static void esc_data_encode(uint8_t * p_encoded_buffer, const esc_t * p_esc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_esc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_esc->download_control_point;
		len += uint16_big_encode(p_esc->number_of_event, &p_encoded_buffer[len]);
}
static void drhc_data_encode(uint8_t * p_encoded_buffer, const drhc_t * p_drhc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_drhc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_drhc->recorded_data_interval;
		p_encoded_buffer[len++] = p_drhc->recorded_data_resolution;
		p_encoded_buffer[len++] = p_drhc->recorded_number_of_axis;
		p_encoded_buffer[len++] = p_drhc->recorded_event_id;
		p_encoded_buffer[len++] = p_drhc->recorded_event_duration;
		len += uint32_big_encode(p_drhc->recorded_utc_of_event_start, &p_encoded_buffer[len]);
		for (int i=0;i<5;i++)
			p_encoded_buffer[len++] = p_drhc->sensor_data_offset[i];
}
static void cdrc_data_encode(uint8_t * p_encoded_buffer, const cdrc_t * p_cdrc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_cdrc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
			p_encoded_buffer[len++] = p_cdrc->data_packet_id;
		p_encoded_buffer[len++] = p_cdrc->data_packet_length;
	for (int i=0;i<CDRC_ALLDATA_MAX_LENGTH-2;i++)
		p_encoded_buffer[len++] = p_cdrc->data_payload[i];
}
static void rdrc_data_encode(uint8_t * p_encoded_buffer, const rdrc_t * p_rdrc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_rdrc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_rdrc->data_packet_id;
		p_encoded_buffer[len++] = p_rdrc->data_packet_length;
	for (int i=0;i<p_rdrc->data_packet_length;i++)
		p_encoded_buffer[len++] = p_rdrc->data_payload[i];
}
static void bdc_data_encode(uint8_t * p_encoded_buffer, const bdc_t * p_bdc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_bdc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_bdc->advertising_period;
		p_encoded_buffer[len++] = p_bdc->advertising_duration;
		len += uint16_big_encode(p_bdc->bdc_advertising_interval, &p_encoded_buffer[len]);
}
void bdrc_data_encode(uint8_t * p_encoded_buffer, const bdrc_t * p_bdrc)
{
		uint8_t len = 0;
		APP_ERROR_CHECK_BOOL(p_bdrc != NULL);
		APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);
		p_encoded_buffer[len++] = p_bdrc->advertising_time;
		len += uint16_big_encode(p_bdrc->bdrc_advertising_interval, &p_encoded_buffer[len]);
		p_encoded_buffer[len++] = p_bdrc->button_status;
		
}

static void tsc_data_process(const uint8_t * p_encoded_buffer, tsc_t * p_tsc)
{
	uint8_t accident_level_update = (p_encoded_buffer[0] & 0x01);
	uint8_t dangerous_level_update = (p_encoded_buffer[0] & 0x02);
	if (accident_level_update == 0x01)
	{
		p_tsc->small_accident_level_x = uint16_big_decode(p_encoded_buffer+1);
		p_tsc->small_accident_level_y = uint16_big_decode(p_encoded_buffer+3);
		p_tsc->medium_accident_level = uint16_big_decode(p_encoded_buffer+5);
		p_tsc->high_accident_level = uint16_big_decode(p_encoded_buffer+7);
		printf("accident:%04X,%04X,%04X,%04X\r\n",p_tsc->small_accident_level_x,p_tsc->small_accident_level_y,p_tsc->medium_accident_level,p_tsc->high_accident_level );
	}
	if (dangerous_level_update == 0x02)
	{
		p_tsc->hard_accelaration_level = uint16_big_decode(p_encoded_buffer+9);
		p_tsc->hard_braking_level = uint16_big_decode(p_encoded_buffer+11);
		p_tsc->hard_steering_level_left = uint16_big_decode(p_encoded_buffer+13);
		p_tsc->hard_steering_level_right = uint16_big_decode(p_encoded_buffer+15);
		printf("dangerous:%04X,%04X,%04X,%04X\r\n",p_tsc->hard_accelaration_level,p_tsc->hard_braking_level,p_tsc->hard_steering_level_left,p_tsc->hard_steering_level_right );
	}
}
static void bsc_data_process(const uint8_t * p_encoded_buffer, bsc_t * p_bsc)
{
	uint8_t led_confirmation = (p_encoded_buffer[0] & 0x01);
	uint8_t update_sampling_freqency = (p_encoded_buffer[0] & 0x02);
	uint16_t sampling_interval;
	uint8_t update_ble_power = (p_encoded_buffer[0] & 0x04);
	int16_t ble_power_int16;
	int8_t ble_power_int8;
	uint8_t update_current_utc = (p_encoded_buffer[0] & 0x08);
	
	printf("bsc_data:");
	for (int i=0;i< 13;i++)
		printf("%02X ",p_encoded_buffer[i]);
	printf("\r\n");
	
	if (led_confirmation == 0x01)
	{
		confirmation_led();
	}
	if (update_sampling_freqency == 0x02)
	{
		p_bsc->sampling_frequency = uint16_big_decode(p_encoded_buffer+1);
		sampling_interval = 1000/p_bsc->sampling_frequency ; // 100,50,33,25,10
		printf("sampling_interval:%i\r\n",sampling_interval);
		event_sampling_interval_set(sampling_interval);
	}
	if (update_ble_power == 0x04)
	{
		ble_power_int16 = uint16_big_decode(p_encoded_buffer+7);
		ble_power_int8 = (int8_t)ble_power_int16;
		uint32_t err_code = sd_ble_gap_tx_power_set(ble_power_int8);
		printf("error:%04X,tx_power_set:%i\r\n",err_code,ble_power_int8);
	}
	if (update_current_utc == 0x08)
	{
		uint32_t tempUTC = uint32_big_decode(p_encoded_buffer+9);
		UTC_set(tempUTC);
		p_bsc->current_utc = tempUTC;
		printf("UTC:%08X\r\n",tempUTC);
	}
}
static void bdc_data_decode(const uint8_t * p_encoded_buffer, bdc_t * p_bdc)
{
	p_bdc->advertising_period= p_encoded_buffer[0];
	p_bdc->advertising_duration = p_encoded_buffer[1];
	p_bdc->bdc_advertising_interval = uint16_big_decode(p_encoded_buffer+2);
	printf("bdc:%02X,%02X,%04X\r\n",p_bdc->advertising_period,p_bdc->advertising_duration,p_bdc->bdc_advertising_interval);
}

static void bdrc_data_decode(const uint8_t * p_encoded_buffer, bdrc_t * p_bdrc)
{
	p_bdrc->advertising_time = p_encoded_buffer[0];
	p_bdrc->bdrc_advertising_interval = uint16_big_decode(p_encoded_buffer+1);
	printf("bdrc:%02X,%04X\r\n",p_bdrc->advertising_time,p_bdrc->bdrc_advertising_interval);
}


/**@brief Function for adding the Characteristic.
 *
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[in]   dis_attr_md    Security settings of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t pbs_char_add(const ble_gatts_char_md_t  char_md,
													ble_uuid_t                  char_uuid,
                         uint8_t                       * p_char_value,
                         uint16_t                        char_len,
                         const ble_srv_security_mode_t * pbs_attr_md,
                         ble_gatts_char_handles_t      * p_handles)
{
		char_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
	
    
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
		
	
    APP_ERROR_CHECK_BOOL(p_char_value != NULL);
    APP_ERROR_CHECK_BOOL(char_len > 0);

    

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = pbs_attr_md->read_perm;
    attr_md.write_perm = pbs_attr_md->write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = char_len;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = char_len;
    attr_char_value.p_value   = p_char_value;

    return sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value, p_handles);
}

// Event Handle 
static void on_connect(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
    p_pbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_pbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

uint32_t ble_pbs_drhc_update(ble_pbs_t * p_pbs, uint8_t *drhc_data)
{
    if (p_pbs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
		
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = DRHC_ALLDATA_LENGTH;
		gatts_value.offset  = 0;
		gatts_value.p_value = drhc_data;

		// Update database.
		err_code = sd_ble_gatts_value_set(p_pbs->conn_handle,
																			p_pbs->drhc_handles.value_handle,
																			&gatts_value);
#if PBS_DEBUG
		printf("set value err:%04X\r\n",err_code);
#endif
		// Send value if connected and notifying.
		//if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
		if (p_pbs->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				ble_gatts_hvx_params_t hvx_params;

				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_pbs->drhc_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_INDICATION;
				//hvx_params.type   =  BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = gatts_value.offset;
				hvx_params.p_len  = &gatts_value.len;
				hvx_params.p_data = gatts_value.p_value;

				err_code = sd_ble_gatts_hvx(p_pbs->conn_handle, &hvx_params);
#if PBS_DEBUG
			printf("indicate value err:%04X\r\n",err_code);
#endif
		}
//		if (err_code == NRF_SUCCESS)
//		{
//			start_dw_flag = true;
//			printf("start download\r\n");
//		}
    return err_code;
}
uint32_t ble_pbs_esc_update(ble_pbs_t * p_pbs, uint8_t* esc_data)
{
    if (p_pbs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
		
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = ESC_ALLDATA_LENGTH;
		gatts_value.offset  = 0;
		gatts_value.p_value = esc_data;

		// Update database.
		err_code = sd_ble_gatts_value_set(p_pbs->conn_handle,
																			p_pbs->esc_handles.value_handle,
																			&gatts_value);
#if PBS_DEBUG
		printf("esc set value err:%04X\r\n",err_code);
#endif
		// Send value if connected and notifying.
		//if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
		if (p_pbs->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				ble_gatts_hvx_params_t hvx_params;

				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_pbs->esc_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_INDICATION;
				hvx_params.offset = gatts_value.offset;
				hvx_params.p_len  = &gatts_value.len;
				hvx_params.p_data = gatts_value.p_value;

				err_code = sd_ble_gatts_hvx(p_pbs->conn_handle, &hvx_params);
#if PBS_DEBUG
			printf("esc indicate value err:%04X\r\n",err_code);
#endif
		}
    return err_code;
}
uint32_t ble_pbs_bdrc_update(ble_pbs_t * p_pbs, uint8_t* bdrc_data)
{
    if (p_pbs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
		
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = BDRC_ALLDATA_LENGTH;
		gatts_value.offset  = 0;
		gatts_value.p_value = bdrc_data;

		// Update database.
		err_code = sd_ble_gatts_value_set(p_pbs->conn_handle,
																			p_pbs->bdrc_handles.value_handle,
																			&gatts_value);
#if PBS_DEBUG
		printf("bdrc set value err:%04X\r\n",err_code);
#endif
		// Send value if connected and notifying.
		//if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
		if (p_pbs->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				ble_gatts_hvx_params_t hvx_params;

				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_pbs->bdrc_handles.value_handle;
				hvx_params.type   = BLE_GATT_HVX_INDICATION;
				hvx_params.offset = gatts_value.offset;
				hvx_params.p_len  = &gatts_value.len;
				hvx_params.p_data = gatts_value.p_value;

				err_code = sd_ble_gatts_hvx(p_pbs->conn_handle, &hvx_params);
#if PBS_DEBUG
			printf("bdrc indicate value err:%04X\r\n",err_code);
#endif
		}
    return err_code;
}

static void on_pbs_cdrc_cccd_write(ble_pbs_t * p_pbs, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_pbs->evt_handler != NULL)
        {
            ble_pbs_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_PBS_EVT_NOTIFICATION_ENABLED; //evt.evt_type = BLE_PBS_EVT_INDICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_PBS_EVT_NOTIFICATION_DISABLED; //evt.evt_type = BLE_PBS_EVT_INDICATION_DISABLED;
            }

            p_pbs->evt_handler(p_pbs, &evt);
        }
    }
}
// FOR INDICATION
static void on_hvc(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_pbs->cdrc_handles.value_handle)
    {
        ble_pbs_evt_t evt;

        evt.evt_type = BLE_PBS_EVT_INDICATION_CONFIRMED;
        p_pbs->evt_handler(p_pbs, &evt);
    }
}

uint32_t ble_pbs_cdrc_update(ble_pbs_t * p_pbs, uint8_t* cdrc_data)
{
    if (p_pbs == NULL || cdrc_data == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
		
		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = CDRC_ALLDATA_MAX_LENGTH;
		gatts_value.offset  = 0;
		gatts_value.p_value = cdrc_data;

		// Update database.
		err_code = sd_ble_gatts_value_set(p_pbs->conn_handle,
																			p_pbs->cdrc_handles.value_handle,
																			&gatts_value);
#if PBS_DEBUG
		//printf("cdrc set value err:%04X\r\n",err_code);
#endif
		// Send value if connected and notifying.
		//if ((p_bas->conn_handle != BLE_CONN_HANDLE_INVALID) && p_bas->is_notification_supported)
		if (p_pbs->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				ble_gatts_hvx_params_t hvx_params;

				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_params.handle = p_pbs->cdrc_handles.value_handle;
				//hvx_params.type   = BLE_GATT_HVX_INDICATION;
			hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = gatts_value.offset;
				hvx_params.p_len  = &gatts_value.len;
				hvx_params.p_data = gatts_value.p_value;

				err_code = sd_ble_gatts_hvx(p_pbs->conn_handle, &hvx_params);
#if PBS_DEBUG
			//printf("cdrc indicate value err:%04X\r\n",err_code);
#endif
		}
    return err_code;
}

static void on_write(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
		
	// Check if BSC's flag been writen
	uint16_t tempHandle = p_ble_evt->evt.gatts_evt.params.write.handle;
	uint8_t *tempData = p_ble_evt->evt.gatts_evt.params.write.data;
#if PBS_DEBUG				
					printf("handle:%04X\r\n",tempHandle);
					printf("data[0][1]:%02X,%02X\r\n",tempData[0],tempData[1]);
#endif

//uint32_t error_code;

ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	if (tempHandle == p_pbs->tsc_handles.value_handle)
		{
			tsc_data_process(tempData, &p_pbs->tsc_s);
			printf("tsc_data\r\n");
		}
	if (tempHandle == p_pbs->bsc_handles.value_handle) // write to bsc
	{
		bsc_data_process(tempData, &p_pbs->bsc_s);
	}
	
	
	// write to esc
//		if (tempHandle == p_pbs->esc_handles.value_handle) 
//		{	
//			printf("write to esc\r\n");
//			// start download event data
//			if (p_evt_write->data[0] == 0x01 && drhc_notify_flag)
//			{
//				//esc_write_flag = true;
//				uint8_t sim_drhc_data[DRHC_ALLDATA_LENGTH] = {20,12,3,1,10,0,0,0,1};
//				ble_pbs_drhc_update(p_pbs,sim_drhc_data);
//			}
//			p_pbs->esc_s.download_control_point = p_evt_write->data[0];
//		}
	
		if(tempHandle == p_pbs->esc_handles.cccd_handle)
    {
				//CCCD been written
        if ( p_evt_write->len == 2)
				{
					esc_notify_flag = !esc_notify_flag;
//#if PBS_DEBUG
//					printf("esc_notify_flag:%d\r\n",esc_notify_flag);
//#endif
				}
		}	
		if ((tempHandle == p_pbs->drhc_handles.cccd_handle) && (p_evt_write->len == 2))
    {
			if (tempData[0] == 0)
				drhc_notify_flag = false;
			else
				drhc_notify_flag = true;
#if PBS_DEBUG
					printf("drhc_notify_flag:%d\r\n",drhc_notify_flag);
#endif	
		}
		
		if ((tempHandle == p_pbs->cdrc_handles.cccd_handle) && (p_evt_write->len == 2))
    {
			
			if (tempData[0] == 0)
				cdrc_notify_flag = false;
			else
				cdrc_notify_flag = true;
#if PBS_DEBUG
					printf("cdrc_notify_flag:%d\r\n",cdrc_notify_flag);
#endif
		}
		
		if ((tempHandle == p_pbs->bdrc_handles.cccd_handle) && (p_evt_write->len == 2))
    {
			if (tempData[0] == 0)
				bdrc_notify_flag = false;
			else
				bdrc_notify_flag = true;
		}
		if (tempHandle == p_pbs->bdc_handles.value_handle)
		{
			bdc_data_decode(tempData, &p_pbs->bdc_s);
		}
		if (tempHandle == p_pbs->bdrc_handles.value_handle)
		{
			bdrc_data_decode(tempData, &p_pbs->bdrc_s);
		}
}

void ble_pbs_on_ble_evt(ble_pbs_t * p_pbs, ble_evt_t * p_ble_evt)
{
    if (p_pbs == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
						printf("pbs_on_ble_Connected\r\n");
            on_connect(p_pbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
						printf("pbs_on_ble_Disconnected\r\n");
            on_disconnect(p_pbs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_pbs, p_ble_evt);
            break;
				
				case BLE_GATTS_EVT_HVC:
					printf("pbs_on_ble_EVT_HVC\r\n");
            on_hvc(p_pbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_pbs_init(ble_pbs_t * p_pbs)
{
		uint32_t   ser_err_code;
		uint32_t   char_err_code;
		ble_uuid_t service_uuid;
		ble_uuid_t char_uuid_temp;
		ble_uuid_t char_uuid_vendor;
		// Add service
		//BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_PIONEER_BEACON_SERVICE); // For SIG UUID

	// PBS Event Handle
		
	
	// Build GATT Profile 
		uint32_t err_code;
		ble_uuid128_t base_uuid = PBS_UUID_BASE;  // It's invert added from the array sequence, uint8_t [16] array
		//ble_uuid128_t bdc_long_uuid = PBS_BOARDING_DETECTION_REPORT_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type); // add to Nordic VS UUID table
		service_uuid.uuid = PBS_SERVICE_SHORT_UUID; // assign short UUID		
		ser_err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
#if PBS_DEBUG
		//printf("vs_add:%d\r\n",err_code);	
		printf("service_add:%d\r\n",ser_err_code);	
#endif
		//Characteristic Setting
		// The ble_gatts_char_md_t structure uses bit fields. So we reset the memory to zero.
		
		ble_gatts_attr_md_t cccd_md;
		ble_gatts_char_md_t char_md_temp;
		memset(&cccd_md, 0, sizeof(cccd_md));
		memset(&char_md_temp, 0, sizeof(char_md_temp));
		// Configure indicate
		
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		cccd_md.write_perm = p_pbs->pbs_cccd_md.cccd_write_perm;
		cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
		
		// Characteristic default setting 
    char_md_temp.char_props.read  = 0;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 0;
		char_md_temp.p_cccd_md        = NULL; //&cccd_md
    char_md_temp.p_char_user_desc = NULL;
    char_md_temp.p_char_pf        = NULL;
    char_md_temp.p_user_desc_md   = NULL;
    char_md_temp.p_sccd_md        = NULL;
	
		//p_pbs->bsc_s.flag = 0x01;  // Can not assign characteristic value here, show read-only variable can not assign
		
		// Threshold Setting Characteristic
		char_uuid_temp.uuid = PBS_THRESHOLD_SETTING_CHAR_SHORT_UUID;
		uint8_t encoded_tsc_data [TSC_ALLDATA_LENGTH];
		tsc_data_encode(encoded_tsc_data, &p_pbs->tsc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_md_temp.char_props.indicate = 0;
		char_md_temp.char_props.notify = 0;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_tsc_data,
														TSC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->tsc_handles);
#if PBS_DEBUG
		//printf("tsc_add:%d\r\n",char_err_code);
#endif	
		
		
		// Beacon Setting Characteristic
		char_uuid_temp.uuid = PBS_BEACON_SETTING_CHAR_SHORT_UUID;
		uint8_t encoded_bsc_data [BSC_ALLDATA_LENGTH];
		bsc_data_encode(encoded_bsc_data, &p_pbs->bsc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.char_props.notify = 0;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_bsc_data,
														BSC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->bsc_handles);
#if PBS_DEBUG
		//printf("bsc_add:%d\r\n",char_err_code);
#endif				
												
		// Event Storage Characteristic
		char_uuid_temp.uuid = PBS_EVENT_STORAGE_CHAR_SHORT_UUID;
		uint8_t encoded_esc_data [ESC_ALLDATA_LENGTH];
		esc_data_encode(encoded_esc_data, &p_pbs->esc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.char_props.notify = 0;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_esc_data,
														ESC_ALLDATA_LENGTH,// Can not use sizeof(esc_t), or it will be 4
														&p_pbs->pbs_attr_md,
														&p_pbs->esc_handles);
		// Notify flag
		esc_notify_flag = false;
#if PBS_DEBUG
		//printf("esc_add:%d\r\n",char_err_code);
#endif							
		// Data Report Header Characteristic
		char_uuid_temp.uuid = PBS_DATA_REPORT_HEADER_CHAR_SHORT_UUID;
		uint8_t encoded_drhc_data [DRHC_ALLDATA_LENGTH];
		drhc_data_encode(encoded_drhc_data, &p_pbs->drhc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.char_props.notify = 0;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_drhc_data,
														DRHC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->drhc_handles);
		drhc_notify_flag = false;
#if PBS_DEBUG
		//printf("drhc_add:%d\r\n",char_err_code);												
#endif
		// Cal Data Report Characteristic
		char_uuid_temp.uuid = PBS_CAL_DATA_REPORT_CHAR_SHORT_UUID;
		uint16_t CDRC_ALLDATA_LENGTH = 20;
		uint8_t encoded_cdrc_data [CDRC_ALLDATA_LENGTH];
		cdrc_data_encode(encoded_cdrc_data, &p_pbs->cdrc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 0;
		char_md_temp.char_props.notify = 1;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_cdrc_data,
														CDRC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->cdrc_handles);
		cdrc_notify_flag = false;
#if PBS_DEBUG
		//printf("cdrc_add:%d\r\n",char_err_code);
#endif
		// Raw Data Report Characteristic
		char_uuid_temp.uuid = PBS_RAW_DATA_REPORT_CHAR_SHORT_UUID;
		uint16_t RDRC_ALLDATA_LENGTH = p_pbs->rdrc_s.data_packet_length+2;
		uint8_t encoded_rdrc_data [RDRC_ALLDATA_LENGTH];
		rdrc_data_encode(encoded_rdrc_data, &p_pbs->rdrc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 0;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.char_props.notify = 0;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_rdrc_data,
														RDRC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->rdrc_handles);
		rdrc_notify_flag = false;
#if PBS_DEBUG
		//printf("rdrc_add:%d\r\n",char_err_code);
#endif				
		//ble_uuid_t venCharID;
// Boarding Detection Report Characteristic
		//ble_uuid128_t bdc_long_uuid = PBS_BOARDING_DETECTION_REPORT_UUID;  // It's invert added from the array sequence, uint8_t [16] array
		//err_code = sd_ble_uuid_vs_add(&bdc_long_uuid, &venCharID.type); // add to Nordic VS UUID table
		//printf("bdc_long_uuid_vs_add:%04X\r\n",err_code);
		char_uuid_temp.uuid = 0x9F55;
		uint8_t encoded_bdc_data [BDC_ALLDATA_LENGTH];
		bdc_data_encode(encoded_bdc_data, &p_pbs->bdc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_md_temp.char_props.indicate = 0;
		char_md_temp.char_props.notify = 0;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_temp,
														encoded_bdc_data,
														BDC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->bdc_handles);
#if PBS_DEBUG
		printf("bdc_add:%d\r\n",char_err_code);
#endif						
// Button Data Report Characteristic
		ble_uuid128_t bdrc_long_uuid = PBS_BUTTON_DATA_REPORT_UUID;  // It's invert added from the array sequence, uint8_t [16] array
		err_code = sd_ble_uuid_vs_add(&bdrc_long_uuid, &char_uuid_vendor.type); // add to Nordic VS UUID table
		printf("bdrc_long_uuid_vs_add:%04X\r\n",err_code);
		char_uuid_vendor.uuid = 0xE409;
		uint8_t encoded_bdrc_data [BDRC_ALLDATA_LENGTH];
		bdrc_data_encode(encoded_bdrc_data, &p_pbs->bdrc_s);
		char_md_temp.char_props.read  = 1;
		char_md_temp.char_props.write = 1;
		char_md_temp.char_props.indicate = 1;
		char_md_temp.char_props.notify = 0;
		char_md_temp.p_cccd_md = &cccd_md;
		char_err_code = pbs_char_add(char_md_temp,
														char_uuid_vendor,
														encoded_bdrc_data,
														BDRC_ALLDATA_LENGTH,
														&p_pbs->pbs_attr_md,
														&p_pbs->bdrc_handles);
#if PBS_DEBUG
		printf("bdrc_add:%d\r\n",char_err_code);
#endif		
	return NRF_SUCCESS;
}

