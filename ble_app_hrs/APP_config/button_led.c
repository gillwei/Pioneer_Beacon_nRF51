#include "button_led.h"

#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"

#include "nrf_drv_timer.h"
const nrf_drv_timer_t TIMER_GPIO = NRF_DRV_TIMER_INSTANCE(1);

//#define G_LED_PIN				25
//#define R_LED_PIN				26
//#define BUTTON_1stPIN		27
//#define BUTTON_2ndPIN		28
#define TIMER1_TICK_MS	50

uint32_t UTC = 0;
uint8_t count_down_1s_xHz = 1*(1000/TIMER1_TICK_MS);

int button_pressing_counter = 0;
int button_releasing_counter = 0;
int g_led_on_countdown = 0;

#define CONFIRMATION_PATTERN_LENGTH	9
//uint8_t g_led_pattern_confirmation[CONFIRMATION_PATTERN_LENGTH] = {1, 1, 1, 0, 0, 0, 0, 0, 0};
uint8_t g_led_pattern_confirmation[CONFIRMATION_PATTERN_LENGTH] = {0, 0, 0, 1, 1, 1, 1, 1, 1};
uint8_t g_led_pattern_confirmation_ptr = 0;
uint16_t g_led_pattern_confirmation_cnt = 0;

#define PUSH_PATTERN_LENGTH	3
//uint8_t r_led_pattern_push[CONFIRMATION_PATTERN_LENGTH] = {1, 0, 0};
uint8_t r_led_pattern_push[CONFIRMATION_PATTERN_LENGTH] = {0, 1, 1};
uint8_t r_led_pattern_push_ptr = 0;
uint16_t r_led_pattern_push_cnt = 0;

#define L_ACCIDENT_PATTERN_LENGTH	3
//uint8_t r_led_pattern_L_accident[L_ACCIDENT_PATTERN_LENGTH] = {1, 0, 0};
uint8_t r_led_pattern_L_accident[L_ACCIDENT_PATTERN_LENGTH] = {0, 1, 1};
uint8_t r_led_pattern_L_accident_ptr = 0;
uint16_t r_led_pattern_L_accident_cnt = 0;

#define M_ACCIDENT_PATTERN_LENGTH	9
//uint8_t r_led_pattern_M_accident[M_ACCIDENT_PATTERN_LENGTH] = {1, 1, 1, 0, 0, 0, 0, 0, 0};
uint8_t r_led_pattern_M_accident[M_ACCIDENT_PATTERN_LENGTH] = {0, 0, 0, 1, 1, 1, 1, 1, 1};
uint8_t r_led_pattern_M_accident_ptr = 0;
uint16_t r_led_pattern_M_accident_cnt = 0;

#define S_ACCIDENT_PATTERN_LENGTH	40
//uint8_t r_led_pattern_S_accident[S_ACCIDENT_PATTERN_LENGTH] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
//																															 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//																															 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//																															 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};				
uint8_t r_led_pattern_S_accident[S_ACCIDENT_PATTERN_LENGTH] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
																															 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
																															 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
																															 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};	
uint8_t r_led_pattern_S_accident_ptr = 0;
uint16_t r_led_pattern_S_accident_cnt = 0;

extern int offset_roll_pitch_period_ms;

enum{
	NO_BUTTON_PUSH=0,  
	SHORT_BUTTON_PUSH,
	LONG_BUTTON_PUSH
};

uint8_t button_status;

uint8_t button_status_get(void)
{		
		uint8_t ret = button_status;
		button_status = NO_BUTTON_PUSH;	// Set the status to no button push after this function has been called since upper layer only need to be modify once
		return ret;
}

void UTC_set(uint32_t currentUTC)
{	
		UTC = currentUTC;
}
uint32_t UTC_get(void)
{	
		return UTC;
}	
																															 
/**
 * @brief Handler for timer events.
 */
void timer1_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE1:
						if (--count_down_1s_xHz == 0) {
							UTC++;
							count_down_1s_xHz = 1*(1000/TIMER1_TICK_MS);
						}
						button_pressing_counter++;
						if (button_releasing_counter++ > 1000/TIMER1_TICK_MS) button_status = NO_BUTTON_PUSH;
						
						if (g_led_on_countdown) {
							if(--g_led_on_countdown)
								nrf_drv_gpiote_out_set(G_LED_PIN);
							else
								nrf_drv_gpiote_out_clear(G_LED_PIN);
						} 
						if (g_led_pattern_confirmation_cnt) {
							if (--g_led_pattern_confirmation_cnt == 0) { nrf_drv_gpiote_out_clear(G_LED_PIN); break; } 
							if (g_led_pattern_confirmation[g_led_pattern_confirmation_ptr++])
								nrf_drv_gpiote_out_set(G_LED_PIN);
							else
								nrf_drv_gpiote_out_clear(G_LED_PIN);
							if (g_led_pattern_confirmation_ptr==CONFIRMATION_PATTERN_LENGTH) g_led_pattern_confirmation_ptr=0;
						}
						if (r_led_pattern_push_cnt) {
							if (--r_led_pattern_push_cnt == 0) { nrf_drv_gpiote_out_clear(R_LED_PIN); break; } 
							if (r_led_pattern_push[r_led_pattern_push_ptr++])
								nrf_drv_gpiote_out_set(R_LED_PIN);
							else
								nrf_drv_gpiote_out_clear(R_LED_PIN);
							if (r_led_pattern_push_ptr==PUSH_PATTERN_LENGTH) r_led_pattern_push_ptr=0;
						}
						if (r_led_pattern_L_accident_cnt) {
							if (--r_led_pattern_L_accident_cnt == 0) { nrf_drv_gpiote_out_clear(R_LED_PIN); break; } 
							if (r_led_pattern_L_accident[r_led_pattern_L_accident_ptr++])
								nrf_drv_gpiote_out_set(R_LED_PIN);
							else
								nrf_drv_gpiote_out_clear(R_LED_PIN);
							if (r_led_pattern_L_accident_ptr==L_ACCIDENT_PATTERN_LENGTH) r_led_pattern_L_accident_ptr=0;
						}	
						if (r_led_pattern_M_accident_cnt) {
							if (--r_led_pattern_M_accident_cnt == 0) { nrf_drv_gpiote_out_clear(R_LED_PIN); break; } 
							if (r_led_pattern_M_accident[r_led_pattern_M_accident_ptr++])
								nrf_drv_gpiote_out_set(R_LED_PIN);
							else
								nrf_drv_gpiote_out_clear(R_LED_PIN);
							if (r_led_pattern_M_accident_ptr==M_ACCIDENT_PATTERN_LENGTH) r_led_pattern_M_accident_ptr=0;
						}
						if (r_led_pattern_S_accident_cnt) {
							if (--r_led_pattern_S_accident_cnt == 0) { nrf_drv_gpiote_out_clear(R_LED_PIN); break; } 
							if (r_led_pattern_S_accident[r_led_pattern_S_accident_ptr++])
								nrf_drv_gpiote_out_set(R_LED_PIN);
							else
								nrf_drv_gpiote_out_clear(R_LED_PIN);
							if (r_led_pattern_S_accident_ptr==S_ACCIDENT_PATTERN_LENGTH) r_led_pattern_S_accident_ptr=0;
						}						
            break;
        
        default:
            //Do nothing.
            break;
    }    
}

void large_accident_led(void)	// Red LED blinking for 60 sec (on 50ms, off 100ms and so on)
{ 
		r_led_pattern_push_cnt = 0;
		r_led_pattern_L_accident_cnt = 60000/TIMER1_TICK_MS + 1;
		r_led_pattern_M_accident_cnt = 0;
		r_led_pattern_S_accident_cnt = 0;
}

void middle_accident_led(void)	// Red LED blinking for 60 sec (on 150ms, off 300ms and so on)
{ 
		r_led_pattern_push_cnt = 0;
		r_led_pattern_L_accident_cnt = 0;
		r_led_pattern_M_accident_cnt = 60000/TIMER1_TICK_MS + 1;
		r_led_pattern_S_accident_cnt = 0;
}

void small_accident_led(void)	// Red LED blinking for 60 sec (on 500ms, off 1500ms and so on)
{ 
		r_led_pattern_push_cnt = 0;
		r_led_pattern_L_accident_cnt = 0;
		r_led_pattern_M_accident_cnt = 0;
		r_led_pattern_S_accident_cnt = 60000/TIMER1_TICK_MS + 1;
}

void short_push_led(void)	// Red LED blinking for 1 sec (on 50ms, off 100ms and so on)
{ 
		r_led_pattern_push_cnt = 1000/TIMER1_TICK_MS + 1;
		r_led_pattern_L_accident_cnt = 0;
		r_led_pattern_M_accident_cnt = 0;
		r_led_pattern_S_accident_cnt = 0;
}

void long_push_led(void)	// Green LED ON for 1 sec while calibrating
{
		g_led_on_countdown = 1000/TIMER1_TICK_MS + 1;
		g_led_pattern_confirmation_cnt = 0;
	
		offset_roll_pitch_period_ms = 1000;//calibrate for 1 sec		
}

void confirmation_led(void)	// Green LED blinking for 10 sec (on 150ms, off 300ms and so on)
{ 
		g_led_pattern_confirmation_cnt = 10000/TIMER1_TICK_MS + 1;
		g_led_on_countdown = 0;
}

void HtoL_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		button_pressing_counter = 0;
}

void LtoH_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		if (button_pressing_counter > (1000/TIMER1_TICK_MS) *5) {
			long_push_led();
			button_status = LONG_BUTTON_PUSH;
			button_releasing_counter = 0;
		} else if (button_pressing_counter >= 50/TIMER1_TICK_MS) {
			short_push_led();
			button_status = SHORT_BUTTON_PUSH;
			button_releasing_counter = 0;
		}
}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
void button_led_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();//Tsungta this is already be called somewhere else, call twice will cause failure 
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(G_LED_PIN, &out_config);
	  APP_ERROR_CHECK(err_code);
		err_code = nrf_drv_gpiote_out_init(R_LED_PIN, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t button_LtoH_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
		nrf_drv_gpiote_in_config_t button_HtoL_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    button_LtoH_config.pull = NRF_GPIO_PIN_PULLUP;
		button_HtoL_config.pull = NRF_GPIO_PIN_PULLUP;
	
    err_code = nrf_drv_gpiote_in_init(BUTTON_1stPIN, &button_LtoH_config, LtoH_pin_handler);
		err_code = nrf_drv_gpiote_in_init(BUTTON_2ndPIN, &button_HtoL_config, HtoL_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_1stPIN, true);
		nrf_drv_gpiote_in_event_enable(BUTTON_2ndPIN, true);
		
    nrf_drv_timer_init(&TIMER_GPIO, NULL, timer1_event_handler);
    APP_ERROR_CHECK(err_code);
    
    uint32_t gpio_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_GPIO, TIMER1_TICK_MS);
    
    nrf_drv_timer_extended_compare(
    &TIMER_GPIO, NRF_TIMER_CC_CHANNEL1, gpio_ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
    
    nrf_drv_timer_enable(&TIMER_GPIO);	
		
		//Turn off LEDs
		//nrf_gpio_pin_set(R_LED_PIN);
		//nrf_gpio_pin_set(G_LED_PIN);
}

