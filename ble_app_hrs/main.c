/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

//#include "main.h"
#include "lis2dh12.h"
#include "int_flash.h"
#include "nrf_drv_timer.h"
#include "button_led.h"
#include "event_detection.h"
#include "read_adc.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "bsp_btn_ble.h"
#include "ble_pbs.h"
#include "spi_flash_MXIC.h"
#include "nrf_drv_gpiote.h"

//Same definition as ble_pbs.c
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

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "BLE_PBS_V03_1"                               /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_TIMEOUT_IN_SECONDS       180000                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define DATA_HEADER_REPORT_INTERVAL      APP_TIMER_TICKS(TIMER_DURATION, APP_TIMER_PRESCALER) 

#define CAL_INTERVAL_DURATION						 50  																			// Calculation data send interval
#define CAL_DATA_INTERVAL      					 APP_TIMER_TICKS(CAL_INTERVAL_DURATION, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                        /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                          /**< Increment between each simulated battery level measurement. */

#define ADVERTISING_DEFAULT_INTERVAL     100                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define ADVERTISING_DEFAULT_PERIOD       20000
#define ADVERTISING_DEFAULT_DURATION     10000
#define ADVERTISING_DEFAULT_PERIOD_SEC       64
#define ADVERTISING_DEFAULT_DURATION_SEC     63
//#define ADV_STOP_DURATION								 60000  // Stop advertising Period, should be multiple of ADVERTISING_TIMER_DURATION
#define ADV_DURATION										 18000000 // Advertising timeout, should be multiple of ADVERTISING_TIMER_DURATION
#define ADVERTISING_TIMER_DURATION			 1000
#define ADVERTISING_TIMER_INTERVAL       APP_TIMER_TICKS(ADVERTISING_TIMER_DURATION, APP_TIMER_PRESCALER) /**< Advertising timer  (ticks). */
//#define MIN_HEART_RATE                   140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
//#define MAX_HEART_RATE                   300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
//#define HEART_RATE_INCREMENT             10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

//#define SHORT_PUSH_ON_LOCK_INTERVAL      10000
#define BUTTON_LED_TIMER_DURATION			 	 2000  // Not minisecond, need confirm
#define BUTTON_LED_INTERVAL              APP_TIMER_TICKS(BUTTON_LED_TIMER_DURATION, APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
//#define MIN_RR_INTERVAL                  100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
//#define MAX_RR_INTERVAL                  500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
//#define RR_INTERVAL_INCREMENT            1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(40, UNIT_1_25_MS)  //400         /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(100, UNIT_1_25_MS)  //650         /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define FLASH_READ								 			 1
#define DEFAULT_BLE_TX_POWER						 -8			

#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */


STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT


static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

ble_pbs_t                         m_pbs;  

/**< Structure used to identify the PBS. */
//static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */
//static ble_hrs_t                         m_hrs;                                     /**< Structure used to identify the heart rate service. */
//static bool                              m_rr_interval_enabled = true;              /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

//static sensorsim_cfg_t                   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
//static sensorsim_state_t                 m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
//static sensorsim_cfg_t                   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
//static sensorsim_state_t                 m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
//static sensorsim_cfg_t                   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
//static sensorsim_state_t                 m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

APP_TIMER_DEF(m_cal_data_id);                                                  /**< Battery timer. */
APP_TIMER_DEF(m_adv_timer_id);                                               /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_button_led_timer_id);                                              /**< RR interval timer. */                 /**< RR interval timer. */
//APP_TIMER_DEF(m_sensor_contact_timer_id);                                           /**< Sensor contact detected timer. */

extern bool esc_notify_flag;
extern bool start_dw_flag;
extern bool drhc_notify_flag;
extern bool cdrc_notify_flag;
extern bool rdrc_notify_flag;
extern bool bdrc_notify_flag;

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */
static bool sensor_data_process_trigger = false;
static bool advertising_trigger = false;
static bool button_led_trigger = false;
//static int red_led_lock = 0;
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE}};
                                   //{BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE}};
                                   //{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_ALERT_NOTIFICATION_SERVICE,         BLE_UUID_TYPE_BLE}};
#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT

//Gill Add 20160314
#define MAIN_DEBUG											1 
#define ACC_THRESHOLD                   200

static void advertising_init(uint16_t);
static uint16_t pktCounter = 0; // need change to variable
static uint16_t pktCounterCopy;
static uint16_t adv_counter = 0;
static bool 		adv_on = false;
static uint16_t previous_acc_voltage = 0;
//static uint16_t totalPktNum;
//static uint8_t p_cal_encoded_buffer_prepare[20] = {0};
// Button LED status
extern int g_led_on_countdown;  // Long push triggered
extern uint16_t r_led_pattern_push_cnt; // Short push triggered
extern float offset_xyz[3];
static bool short_push_flag = false;
extern void bsc_data_encode(uint8_t*,const bsc_t * );
//static bool acc_on = false;
//static bool                             m_pbs_esc_ind_conf_pending = false;  

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


//void flash_write_simulation(void)
//{
//	float cal_acc_test[3];	
//		cal_acc_test[0] = 50+1600;
//		cal_acc_test[1] = 30+1600;
//		cal_acc_test[2] = 100+1600;			
//		int w_cnt = 600 * 3;
//	//	flash_data_set_init();	
//		while(w_cnt--) {
//				if (w_cnt == 1000) {
//					flash_data_set_write(NULL, cal_acc_test, 1);
//				} else if (w_cnt == 500) {
//					flash_data_set_write(NULL, cal_acc_test, 2);
//				}	else {
//					flash_data_set_write(NULL, cal_acc_test,0);
//				}	
//		}
//		//nrf_delay_ms(20);
//		pktCounter = 125;
//		
////		m_pbs.esc_s.number_of_event = 1;
////		uint8_t esc_sim_data[3] = {0,0,1};
////		ble_pbs_esc_update(&m_pbs, esc_sim_data);
//		printf("flash write simulation\r\n");
//}
void drhc_offset_process(uint8_t* drhc_offset)
{
	printf("offset %f %f %f\r\n",offset_xyz[0],offset_xyz[1],offset_xyz[2]);
	uint16_t offset_x_100 = (offset_xyz[0]+16)*100;
	uint16_t offset_y_100 = (offset_xyz[1]+16)*100;
	uint16_t offset_z_100 = (offset_xyz[2]+16)*100;
	printf("offset_100 %04X %04X %04X\r\n",offset_x_100,offset_y_100,offset_z_100);
//		uint16_t data_x = (uint16_t)(offset_xyz[0]*100);
//		uint16_t data_y = (uint16_t)(offset_xyz[1]*100);
//		uint16_t data_z = (uint16_t)(offset_xyz[2]*100);
		uint16_t data_x = (uint16_t)(offset_x_100&0xFFFF);
		uint16_t data_y = (uint16_t)(offset_y_100&0xFFFF);
		uint16_t data_z = (uint16_t)(offset_z_100&0xFFFF);
		printf("offset %04X %04X %04X\r\n",data_x,data_y,data_z);
		drhc_offset[0] = (uint8_t) ((data_z& 0x0FF0)	>> 4) ;
		drhc_offset[1] = ((uint8_t)((data_z& 0x000F) << 4) & 0xF0) | (((data_y&0x0F00) >> 8)& 0x0F);
		drhc_offset[2] = (uint8_t) (data_y& 0xFF);
		drhc_offset[3] = (uint8_t) ((data_x& 0x0FF0)	>> 4) ;
		drhc_offset[4] = ((uint8_t)((data_x& 0x000F) << 4) & 0xF0);
//	for (int i=0;i<5;i++)
//		printf("%02X ",drhc_offset[i]);
//	printf("\r\n");
	
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void cal_data_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	sensor_data_process_trigger = true;
}

static void sensor_data_process(void)
{
		if (sensor_data_process_trigger)
	{
		sensor_data_process_trigger = false;
		uint32_t err_code;
	
//	if (pktCounter == 0)
//	{
//		flash_write_simulation();
//	}
	// Simulator flash read
		uint16_t data_x[4] = {0};
		uint16_t data_y[4] = {0};
		uint16_t data_z[4] = {0};
		//uint8_t len = 0;
		uint8_t p_cal_encoded_buffer[20] = {0};
		
		//uint32_t err_code;
#if FLASH_READ	
		uint32_t r_xy;
		uint32_t r_z;
		uint8_t r_event_ID;
		uint32_t r_UTC;
		uint32_t cal_r_xy;
		uint32_t cal_r_z;
		uint8_t r_sensor_interval;	
		int ret=-1;
		
		uint8_t inPktNum=0; //for 4 packet in one notify
		//printf("cdrc_notify_flag:%i,conn:%04X\r\n",cdrc_notify_flag,m_conn_handle);
		//if (ret > 0 && m_conn_handle != BLE_CONN_HANDLE_INVALID && cdrc_notify_flag== true)	// for test
		if(m_conn_handle != BLE_CONN_HANDLE_INVALID && drhc_notify_flag && cdrc_notify_flag)
		{
			ret = flash_data_set_read(&r_xy, &r_z, &cal_r_xy, &cal_r_z, &r_event_ID, &r_UTC, &r_sensor_interval);
			//nrf_delay_ms(100);
			// UART Print
			if (ret < 0)
			{
				printf("no event;");
			}
			else
			{
					printf("%i,%u,", r_event_ID, r_UTC);
					printf("%i,%f,%f,%f\n\r", ret+1,((float)((cal_r_xy&0xFFFF0000)>>16))/100-16,((float)(cal_r_xy&0x0000FFFF))/100-16, (float)cal_r_z/100-16);
					//totalPktNum = (m_pbs.bsc_s.sampling_frequency)*(m_pbs.drhc_s.recorded_event_duration); // need change

					pktCounter = (ret+1)/4;
					//printf("pktCounter : %i \r\n",pktCounter);
					// Send Data Header, use pktCounterCopy check if pktCounter recount 
					// in case that pktCounter miss the first one packet
					// Send Data Header
					if (pktCounter > pktCounterCopy)	
					{
						uint32_t tempUTC = UTC_get();
						uint8_t p_tempUTC[4]; 
						uint32_big_encode(tempUTC,p_tempUTC);
						int8_t sampling_interval = 1000/m_pbs.bsc_s.sampling_frequency;
						printf("sampling_interval:%d\r\n",sampling_interval);
						uint8_t drhc_offset[5] = {0};
						drhc_offset_process(drhc_offset);
						for (int i=0;i<5;i++)
							printf("%02X ",drhc_offset[i]);
						printf("\r\n");
						uint8_t sim_drhc_data[14] = {sampling_interval,12,3,r_event_ID,10,p_tempUTC[0],p_tempUTC[1],p_tempUTC[2],p_tempUTC[3],drhc_offset[0],drhc_offset[1],drhc_offset[2],drhc_offset[3],drhc_offset[4]};
						err_code = ble_pbs_drhc_update(&m_pbs, sim_drhc_data);
						printf("drhc_update:%04X\r\n",err_code);
					}
					pktCounterCopy = pktCounter;
					p_cal_encoded_buffer[0] = pktCounter;
					p_cal_encoded_buffer[1] = 18;
					
					pktCounter--;
					// Flash data assign to uint16_t array
					for (int i=0;i<4;i++)
					{
						data_x[i] = (uint16_t)((cal_r_xy&0xFFFF0000)>>16);
						data_y[i] = (uint16_t)(cal_r_xy&0xFFFF);
						data_z[i] = (uint16_t)(cal_r_z&0xFFFF);
						if (i!= 0)
							ret = flash_data_set_read(&r_xy, &r_z, &cal_r_xy, &cal_r_z, &r_event_ID, &r_UTC, &r_sensor_interval);
					}
						// Check data 
		//			for(int i=0;i<4;i++)
		//			{
				//		printf("%i %04X %04X %04X\r\n",i,data_x[i],data_y[i],data_z[i]);
					//}
				//	printf("\r\n");

					// assign 4 read data value to cdrc data packet
					for (int i=2;i<20;i=i+9)
					{
						p_cal_encoded_buffer[i] = (uint8_t) ((data_z[inPktNum]& 0x0FF0)	>> 4) ;
						p_cal_encoded_buffer[i+1] = ((uint8_t)((data_z[inPktNum]& 0x000F) << 4) & 0xF0) | (((data_y[inPktNum]&0x0F00) >> 8)& 0x0F);
						p_cal_encoded_buffer[i+2] = (uint8_t) (data_y[inPktNum]& 0xFF);
						p_cal_encoded_buffer[i+3] = (uint8_t) ((data_x[inPktNum]& 0x0FF0)	>> 4) ;
						p_cal_encoded_buffer[i+4] = ((uint8_t)((data_x[inPktNum]& 0x000F) << 4) & 0xF0) | (((data_z[inPktNum+1]&0x0F00) >> 8)& 0x0F);
						p_cal_encoded_buffer[i+5] = (uint8_t) (data_z[inPktNum+1]& 0xFF);
						p_cal_encoded_buffer[i+6] = (uint8_t) ((data_y[inPktNum+1]& 0x0FF0)	>> 4) ;
						p_cal_encoded_buffer[i+7] = ((uint8_t)((data_y[inPktNum+1]& 0x000F) << 4) & 0xF0) | (((data_x[inPktNum+1]&0x0F00) >> 8)& 0x0F);
						p_cal_encoded_buffer[i+8] = (uint8_t) (data_x[inPktNum+1]& 0xFF);
						inPktNum += 2;
					}
		//			printf("ret: %i,Cal data:",ret);
		//			for(int i=0;i<20;i++)
		//			{
		//				printf("%02X",p_cal_encoded_buffer[i]);
		//			}
		//			printf("\r\n");
		//			// Check Flash data read error
		//			if (p_cal_encoded_buffer[19]==0xFF &&p_cal_encoded_buffer[18]==0xFF&&p_cal_encoded_buffer[17]==0xFF && p_cal_encoded_buffer[16]==0xFF)
		//			{
		//					for(int i=19;i>=2;i--)
		//					{
		//						p_cal_encoded_buffer[i] = p_cal_encoded_buffer_prepare[i];
		//					}
		//			}
					err_code = ble_pbs_cdrc_update(&m_pbs, p_cal_encoded_buffer);
					printf("cdrc_update_err:%04X\r\n",err_code);
					
					//Prepare for flash read error
		//			for(int i=19;i>=2;i--)
		//			{
		//				p_cal_encoded_buffer_prepare[i] = p_cal_encoded_buffer[i];
		//			}
		}
		}
#endif
	} // if(ble_routine_trigger)
}



/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void advertising_handle(void)
{
	if (advertising_trigger && m_conn_handle != BLE_CONN_HANDLE_INVALID)
	{
	// Check if ACC Status Change
		// Turn On or Off
		uint16_t read_acc_in_process = read_adc_int();
		
		if ((previous_acc_voltage > ACC_THRESHOLD && read_acc_in_process < ACC_THRESHOLD) || (previous_acc_voltage < ACC_THRESHOLD && read_acc_in_process > ACC_THRESHOLD))
		{
			printf("previous_acc_voltage:%d,read_acc_in_process:%d",previous_acc_voltage,read_acc_in_process);
			printf("\r\nupdate acc_voltage\r\n");
			m_pbs.bsc_s.acc_voltage = read_acc_in_process;
			uint8_t encoded_bsc_data [13]; //BSC_DATA_LENGTH
			bsc_data_encode(encoded_bsc_data, &m_pbs.bsc_s);
			ble_pbs_bsc_update(&m_pbs,encoded_bsc_data);
			previous_acc_voltage = read_acc_in_process;
		}
	}
	
	if (advertising_trigger && m_conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		advertising_trigger = false;
		uint32_t err_code;
		
		// For Short Push Urgent status 
//		if (short_push_counter == m_pbs.bdrc_s.advertising_time)
//		{
//			adv_counter = m_pbs.bdrc_s.advertising_time;
//			printf("short_push_counter:%i\r\n",short_push_counter);
//			short_push_counter = 0;
//		}
		
		// Regular Advertising
		if (adv_counter != 0 )
			adv_counter--;
		printf("adv_counter:%i\r\n",adv_counter);
		if (adv_counter==0 && adv_on == true)
		{
			sd_ble_gap_adv_stop();
			printf("adv_stop;adv_counter:%i\r\n",adv_counter);
			adv_counter = m_pbs.bdc_s.advertising_period-m_pbs.bdc_s.advertising_duration;
			adv_on = false;
		}
		else if (adv_counter == 0 && adv_on == false )
		{
			sd_ble_gap_adv_stop();
			uint16_t adv_interval_u16 = m_pbs.bdc_s.bdc_advertising_interval/0.625;
			printf("advertising_interval:%d\r\n",adv_interval_u16);
			advertising_init(adv_interval_u16); 
			err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
			printf("adv_start;adv_err:%04X\r\n",err_code);
			adv_counter = m_pbs.bdc_s.advertising_duration;
			adv_on = true;
		}
		
		// Read ADC test
//		float adc_voltage = read_adc_voltage();
//	 uint16_t adc_value_int = read_adc_int();
//		float adc_app_value = adc_value_int * 6 * 0.6 / 1024;
//		printf("adc_voltage:%f,int:%d,app_value:%f\r\n",adc_voltage,adc_value_int,adc_app_value); 
  }
}
static void adv_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	// For advertising
	advertising_trigger = true;
}
//static int test = 0;
extern void bdrc_data_encode(uint8_t*,const bdrc_t * );


// Acquire ACC voltage temporarily here
static void button_led_process(void)
{
	if (button_led_trigger)
	{
		
		
		// Check Button Push Status
		button_led_trigger = false;
		uint32_t err_code;
		uint8_t button_status = 0;
		button_status = button_status_get();
		//printf("button_status:%02X\r\n",button_status);
		
		uint8_t encoded_bdrc_data [BDRC_ALLDATA_LENGTH] = {0};
		if (button_status != 0)
		{
			// Send Button status, then send 0
			m_pbs.bdrc_s.button_status = button_status;
			bdrc_data_encode(encoded_bdrc_data, &m_pbs.bdrc_s);
			ble_pbs_bdrc_update(&m_pbs,encoded_bdrc_data);
			//nrf_delay_ms(100);
			m_pbs.bdrc_s.button_status = 0;
			bdrc_data_encode(encoded_bdrc_data, &m_pbs.bdrc_s);
			ble_pbs_bdrc_update(&m_pbs,encoded_bdrc_data);
		}
		
		// Short push state
		if(button_status == 1)
		{
			short_push_flag = true;
			// If BLE Connected, disconnect and advertising, else adv_stop and re-advertising
			
			if (m_conn_handle != BLE_CONN_HANDLE_INVALID) 
			{
				// Set advertising interval in disconnect event
				err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
				printf("Urgent:disconnect:%04X\r\n",err_code);
			}
			else
			{
				// Set advertising interval and advertising again
				sd_ble_gap_adv_stop();
				uint16_t bdrc_adv_interval_u16 = m_pbs.bdrc_s.bdrc_advertising_interval/0.625;
				printf("Urgent;bdrc_advertising_interval:%d\r\n",bdrc_adv_interval_u16);
				advertising_init(bdrc_adv_interval_u16); 
				err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
				printf("adv_start;adv_err:%04X\r\n",err_code);
			}
		}

	}
}

static void button_led_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	button_led_trigger = true;
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void rr_interval_timeout_handler(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);

//    if (m_rr_interval_enabled)
//    {
//        uint16_t rr_interval;

//        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
//                                                      &m_rr_interval_sim_cfg);
//        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
//    }
//}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void sensor_contact_detected_timeout_handler(void * p_context)
//{
//    static bool sensor_contact_detected = false;

//    UNUSED_PARAMETER(p_context);

//    sensor_contact_detected = !sensor_contact_detected;
//    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
//}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

    err_code = app_timer_create(&m_cal_data_id,
                                APP_TIMER_MODE_REPEATED,
                                cal_data_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_adv_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                adv_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_button_led_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                button_led_timeout_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_create(&m_sensor_contact_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                sensor_contact_detected_timeout_handler);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        //APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT


static void on_pbs_evt(ble_pbs_t * p_hts, ble_pbs_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_PBS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            //temperature_measurement_send();
            break;

        case BLE_PBS_EVT_INDICATION_CONFIRMED:
            //m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
}
/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
	m_pbs.evt_handler          = on_pbs_evt;
		memset(&m_pbs, 0, sizeof(m_pbs));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_pbs.pbs_attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_pbs.pbs_attr_md.write_perm); // Orignally no access
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&m_pbs.pbs_cccd_md.cccd_write_perm);
		
		// Value setting 
		//bool utc_exist_bit = false;  // bit located in m_pbs.bsc_s.flag 0x10, TBD
		m_pbs.tsc_s.small_accident_level_x = 70;
		m_pbs.tsc_s.small_accident_level_y = 60;
		m_pbs.tsc_s.medium_accident_level = 600;
		m_pbs.tsc_s.high_accident_level = 1000;
		m_pbs.tsc_s.hard_accelaration_level = 30;
		m_pbs.tsc_s.hard_braking_level = 35;
		m_pbs.tsc_s.hard_steering_level_left = 35;
		m_pbs.tsc_s.hard_steering_level_right= 35;
	
		m_pbs.bsc_s.flag = 0x00;
		m_pbs.bsc_s.sampling_frequency = 50;
		m_pbs.bsc_s.ble_output_power = -8;
		m_pbs.bsc_s.current_utc = 0;
		m_pbs.bsc_s.ambient_sensor_value = 0;
		m_pbs.bsc_s.acc_voltage = read_adc_int();
		previous_acc_voltage = read_adc_int();
		// Event Storage Characteristic default value
		m_pbs.esc_s.download_control_point = 0x00;
		m_pbs.esc_s.number_of_event = 0x00;
		// Data Report Header Characteristic default value
		m_pbs.drhc_s.recorded_data_interval = 20;
		m_pbs.drhc_s.recorded_data_resolution = 12;
		m_pbs.drhc_s.recorded_number_of_axis = 3;
		m_pbs.drhc_s.recorded_event_id = 0;
		m_pbs.drhc_s.recorded_event_duration = 10;
		m_pbs.drhc_s.recorded_utc_of_event_start = 0;
		uint8_t default_p_sensor_data_offset[5] = {0};
		m_pbs.drhc_s.sensor_data_offset = default_p_sensor_data_offset;
		// Calibration Data Log default value
		m_pbs.cdrc_s.data_packet_id = 0;
		m_pbs.cdrc_s.data_packet_length = 18;
		uint8_t default_cdrc_data[18] = {0};
		m_pbs.cdrc_s.data_payload = default_cdrc_data;
		// Raw Data Log default value
		m_pbs.rdrc_s.data_packet_id = 0;
		m_pbs.rdrc_s.data_packet_length = 18;
		uint8_t default_rdrc_data[18] = {0};
		m_pbs.rdrc_s.data_payload = default_rdrc_data;
		
//		m_pbs.bdc_s.advertising_period = ADVERTISING_DEFAULT_PERIOD/ADVERTISING_TIMER_DURATION; 
//		m_pbs.bdc_s.advertising_duration = ADVERTISING_DEFAULT_DURATION/ADVERTISING_TIMER_DURATION; 
		m_pbs.bdc_s.advertising_period = 20;
		m_pbs.bdc_s.advertising_duration = 10;
		m_pbs.bdc_s.bdc_advertising_interval = 100;
		
		m_pbs.bdrc_s.advertising_time = 5;
		m_pbs.bdrc_s.bdrc_advertising_interval = 100;
		m_pbs.bdrc_s.button_status = 0;
    err_code = ble_pbs_init(&m_pbs);
    APP_ERROR_CHECK(err_code);
	

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
	
    err_code = app_timer_start(m_cal_data_id, CAL_DATA_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_adv_timer_id, ADVERTISING_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_button_led_timer_id, BUTTON_LED_TIMER_DURATION, NULL);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
			printf("BLE_CONN_PARAMS_EVT_FAILED\r\n");
			printf("diconnect:%04X",err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    //cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;  // BLE_GATT_HANDLE_INVALID
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
		uint32_t err_code;
    //uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
						printf("BLE_ADV_EVT_FAST\r\n");
						//err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
						//APP_ERROR_CHECK(err_code);
						if (short_push_flag == true)
						{
							adv_on = true;
							adv_counter = m_pbs.bdrc_s.advertising_time*1000/ADVERTISING_TIMER_DURATION;
							short_push_flag = false;
						}
						else
						{
							adv_on = true;
							adv_counter = m_pbs.bdc_s.advertising_duration*1000/ADVERTISING_TIMER_DURATION;
						}
            break;
        case BLE_ADV_EVT_IDLE:
						printf("BLE_ADV_EVT_IDLE\r\n");
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						printf("on_ble_Connected");
						adv_on = false;
						adv_counter = 0;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						printf("on_ble_Disconnected");
						drhc_notify_flag = false;
						cdrc_notify_flag = false;
						bdrc_notify_flag = false;
						adv_on = true;
						adv_counter = 0;
						if (short_push_flag == true)
						{
							uint16_t adv_interval_u16 = m_pbs.bdrc_s.bdrc_advertising_interval/0.625;
							printf("bdrc_advertising_interval:%d\r\n",adv_interval_u16);
							advertising_init(adv_interval_u16);
						}
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
		ble_pbs_on_ble_evt(&m_pbs, p_ble_evt);
//    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
//    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif // BLE_DFU_APP_SUPPORT
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
			printf("BSP_EVENT_SLEEP\r\n");
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
						printf("BSP_EVENT_DISCONNECT\r\n");
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
					printf("BSP_EVENT_WHITELIST_OFF\r\n");
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
// advertising_interval need to divided by 0.625 before send to advertising_init()
static void advertising_init(uint16_t advertising_interval_u16)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = advertising_interval_u16;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
//static void buttons_leds_init(bool * p_erase_bonds)
//{
//    bsp_event_t startup_event;

//    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
//                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
//                                 bsp_event_handler);
//    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
//}


/**@brief Function for the Power manager.
 */
//static void power_manage(void)
//{
//    uint32_t err_code = sd_app_evt_wait();
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
		uart_init();
    app_trace_init();
    timers_init();
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init(ADVERTISING_DEFAULT_INTERVAL);
    services_init();
    //sensor_simulator_init();
    conn_params_init();

    // Start execution.
    application_timers_start();
		
		// Set tx power before advertising
		err_code = sd_ble_gap_tx_power_set(DEFAULT_BLE_TX_POWER);
		printf("tx_power_set:%04X\r\n",err_code);
//nrf_delay_ms(1000);//application system warm up time
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		adv_on = true;
		adv_counter = ADVERTISING_DEFAULT_DURATION/ADVERTISING_TIMER_DURATION;
    APP_ERROR_CHECK(err_code);
		read_adc_init();
		button_led_init();
		lis2dh12_init();
		flash_data_set_init();
//		uart_init();
		event_sampling_interval_set(1000/DEFAULT_HZ);//set default 20 (50Hz) here
		nrf_delay_ms(1000);//application system warm up time

    // Enter main loop.
    for (;;)
    {
				event_detection_routine();
				sensor_data_process();
				advertising_handle();
				button_led_process();
        __WFI();
				
//        power_manage();
    }
}
