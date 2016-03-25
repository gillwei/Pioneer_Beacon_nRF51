#include "event_detection.h"
#include "lis2dh12.h"
#include "int_flash.h"
#include "button_led.h"
#include "spi_flash_MXIC.h"
#include "math.h"

#include "app_uart.h"
#include <stdio.h>

#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"

#include "nrf_drv_timer.h"

//#define EVENT_UART_DEBUG

//#define DUMMY_TEST

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

uint8_t current_event_ID = 0;

uint32_t tt, last_tt, cnt, Ecnt;
bool t_lock = false;

float dummy_data = 0;
uint32_t dummy_cnt = 0;

enum{
	NO_EVENT=0,  
	DANG_LEFT,
	DANG_RIGHT,
	DANG_BRAKE,
	DANG_ACCEL,
	ACCI_S,
	ACCI_M,
	ACCI_L
};
	
float acci_large = 10.0;
float acci_middle = 6.0;
float acci_small_x = 0.7;
float acci_small_y = 0.6;

float dang_acceleration = -0.3;
float dang_braking = 0.35;
float dang_right_strrting = 0.35;
float dang_left_strrting = -0.35; 

float acc_test[3] = {0};
float cal_acc_test[3] = {0};
int counter = 1;

uint8_t temp_sensor_internal_ms = 1000/DEFAULT_HZ;//set default 20 (50Hz) here
uint8_t sensor_internal_ms = 1000/DEFAULT_HZ;//set default 20 (50Hz) here

#define moving_avg_size_ms			500
#define min_interval						20		//50 Hz
float moving_avg_cal_x[moving_avg_size_ms/min_interval] = {0};
float moving_avg_cal_y[moving_avg_size_ms/min_interval] = {0};
uint8_t moving_avg_ptr = 0;
extern float offset_xyz[3];
extern int offset_roll_pitch_period_ms;

const nrf_drv_timer_t TIMER_SENSOR = NRF_DRV_TIMER_INSTANCE(2);
bool sensor_trigger = false;

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);	
}

/**
 * @brief Handler for timer events.
 */
void timer2_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE2:
						sensor_trigger = true;
            break;
        
        default:
            //Do nothing.
            break;
    }    
}

void event_sampling_interval_set(uint8_t internal_ms)
{
		uint32_t sensor_ticks;
		temp_sensor_internal_ms = internal_ms;

		nrf_drv_timer_uninit(&TIMER_SENSOR);
		nrf_drv_timer_init(&TIMER_SENSOR, NULL, timer2_event_handler);
    
    sensor_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_SENSOR, internal_ms);
    
    nrf_drv_timer_extended_compare(
         &TIMER_SENSOR, NRF_TIMER_CC_CHANNEL2, sensor_ticks, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);
    
    nrf_drv_timer_enable(&TIMER_SENSOR);
}

uint8_t accident_dangerous_detection(float Accx_offset, float Accy_offset, float Accx_avg_offset, float Accy_avg_offset)
{
		float Acc_synthesis;
		uint8_t w_event_ID;
		Acc_synthesis = fabs(Accx_offset) + 1.5*fabs(Accx_offset);
	
		if (Acc_synthesis > acci_large && current_event_ID_get() < ACCI_L) {
			w_event_ID = ACCI_L;
			large_accident_led();
		} else if (Acc_synthesis > acci_middle && current_event_ID_get() < ACCI_M) {
			w_event_ID = ACCI_M;
			middle_accident_led();
		} else if ((fabs(Accx_avg_offset) > acci_small_x || fabs(Accy_avg_offset) > acci_small_y) && current_event_ID_get() < ACCI_S) {
			w_event_ID = ACCI_S;
			small_accident_led();
		} else if (Accx_avg_offset < dang_acceleration && current_event_ID_get() == NO_EVENT)	
			w_event_ID = DANG_ACCEL;
		else if (Accx_avg_offset > dang_braking && current_event_ID_get() == NO_EVENT)	
			w_event_ID = DANG_BRAKE;
		else if (Accy_avg_offset > dang_right_strrting && current_event_ID_get() == NO_EVENT)	
			w_event_ID = DANG_RIGHT;
		else if (Accy_avg_offset < dang_left_strrting && current_event_ID_get() == NO_EVENT)	
			w_event_ID = DANG_LEFT;
		else  
			w_event_ID = NO_EVENT;

		return w_event_ID;
}
	
void event_detection_routine(void)
{
		if (sensor_trigger) {
			uint8_t w_event_ID = 0;
			
			sensor_trigger = false;
			lis2dh12_acc_data(acc_test, cal_acc_test);
//			printf("%d,%f,%f,%f,%f,%f,%f\n", cnt++, acc_test[0], acc_test[1], acc_test[2], cal_acc_test[0], cal_acc_test[1], cal_acc_test[2]); //output for SerialChart program
			
			float Accx_avg, Accy_avg = 0;
			moving_avg_cal_x[moving_avg_ptr] = cal_acc_test[0];
			moving_avg_cal_y[moving_avg_ptr] = cal_acc_test[1];
			if (++moving_avg_ptr >= moving_avg_size_ms/sensor_internal_ms) moving_avg_ptr = 0;
			
			for(int i=0; i<moving_avg_size_ms/sensor_internal_ms; i++) {
				Accx_avg += moving_avg_cal_x[i];
				Accy_avg += moving_avg_cal_y[i];
			}
			Accx_avg/=(moving_avg_size_ms/sensor_internal_ms);
			Accy_avg/=(moving_avg_size_ms/sensor_internal_ms);

			if (offset_roll_pitch_period_ms<=0)//only do the event detection after calibration (1 sec) is done
				w_event_ID = accident_dangerous_detection(cal_acc_test[0]-offset_xyz[0], cal_acc_test[1]-offset_xyz[1], Accx_avg-offset_xyz[0], Accy_avg-offset_xyz[1]);
#ifdef DUMMY_TEST
			acc_test[0] = dummy_data;
			acc_test[1] = dummy_data;
			acc_test[2] = dummy_data;
			cal_acc_test[0] = dummy_data;
			cal_acc_test[1] = dummy_data;
			cal_acc_test[2] = dummy_data;
			dummy_data += 0.01;
			if(dummy_data >= 16) dummy_data = -16;
			if (dummy_cnt++ ==50*12) {
				dummy_cnt = 50*8;
				w_event_ID = 9;// dummy event
			}
#endif			
			flash_data_set_write(acc_test, cal_acc_test, w_event_ID);
			
			if(current_event_ID_get() == NO_EVENT)
				sensor_internal_ms = temp_sensor_internal_ms;//only apply the setting if no event happening
			
#ifdef EVENT_UART_DEBUG			
		uint32_t r_xy;
		uint32_t r_z;
		uint8_t r_event_ID;
		uint32_t r_UTC;
		uint32_t cal_r_xy;
		uint32_t cal_r_z;
		uint8_t r_sensor_interval;	
		int ret=flash_data_set_read(&r_xy, &r_z, &cal_r_xy, &cal_r_z, &r_event_ID, &r_UTC, &r_sensor_interval);
		if (ret >= 0) {
			if(t_lock==false) {
				t_lock=true;
				printf("eventID: %i triggeredUTC: %u numDataSet %i\n\r", r_event_ID, r_UTC, ret+1);
			}
			if(ret==0) t_lock=false;
			//print logged raw xyz data
			printf("%f,%f,%f,%f\n\r", (float)((float)(r_sensor_interval*Ecnt++)/1000)+r_UTC-7,(float)((cal_r_xy&0xFFFF0000)>>16)/100-16,(float)(cal_r_xy&0x0000FFFF)/100-16, (float)cal_r_z/100-16);
		} else if (t_lock==false) {
			/*print calibrated xyz raw data*/
			//printf("%f,%f,%f,%f\n\r", (float)(sensor_internal_ms*cnt++)/1000, cal_acc_test[0], cal_acc_test[1], cal_acc_test[2]); //output for SerialChart program		
			/*print Gx(ave)correct and Gy(ave)correct*/
			//printf("%f,%f,%f\n\r", (float)(sensor_internal_ms*cnt++)/1000, Accx_avg-offset_xyz[0], Accy_avg-offset_xyz[1]); //output for SerialChart program		
			/*print Gx_correct and Gy_correct*/
			//printf("%f,%f,%f\n\r", (float)(sensor_internal_ms*cnt++)/1000, cal_acc_test[0]-offset_xyz[0], cal_acc_test[1]-offset_xyz[1]); //output for SerialChart program
		}
#endif		
		}		
}