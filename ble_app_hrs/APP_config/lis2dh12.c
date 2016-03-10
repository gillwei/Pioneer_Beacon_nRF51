#define	G_MAX		16000


#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/


/* Accelerometer Sensor Operating Mode */
#define LIS2DH_ACC_ENABLE	(0x01)
#define LIS2DH_ACC_DISABLE	(0x00)

#define	HIGH_RESOLUTION		(0x08)

#define	AXISDATA_REG		(0x28)
#define WHOAMI_LIS2DH_ACC	(0x33)	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		(0x1F)	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		(0x20)	/*	control reg 1		*/
#define	CTRL_REG2		(0x21)	/*	control reg 2		*/
#define	CTRL_REG3		(0x22)	/*	control reg 3		*/
#define	CTRL_REG4		(0x23)	/*	control reg 4		*/
#define	CTRL_REG5		(0x24)	/*	control reg 5		*/
#define	CTRL_REG6		(0x25)	/*	control reg 6		*/

#define	FIFO_CTRL_REG		(0x2E)	/*	FiFo control reg	*/

#define	INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define	INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define	INT_THS1		(0x32)	/*	interrupt 1 threshold	*/
#define	INT_DUR1		(0x33)	/*	interrupt 1 duration	*/


#define	TT_CFG			(0x38)	/*	tap config		*/
#define	TT_SRC			(0x39)	/*	tap source		*/
#define	TT_THS			(0x3A)	/*	tap threshold		*/
#define	TT_LIM			(0x3B)	/*	tap time limit		*/
#define	TT_TLAT			(0x3C)	/*	tap time latency	*/
#define	TT_TW			(0x3D)	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		(0x00)

#define LIS2DH_ACC_PM_OFF		(0x00)
#define LIS2DH_ACC_ENABLE_ALL_AXES	(0x07)


#define PMODE_MASK		(0x08)
#define ODR_MASK		(0XF0)

#define LIS2DH_ACC_ODR1	(0x10)  /* 1Hz output data rate */
#define LIS2DH_ACC_ODR10	(0x20)  /* 10Hz output data rate */
#define LIS2DH_ACC_ODR25	(0x30)  /* 25Hz output data rate */
#define LIS2DH_ACC_ODR50	(0x40)  /* 50Hz output data rate */
#define LIS2DH_ACC_ODR100	(0x50)  /* 100Hz output data rate */
#define LIS2DH_ACC_ODR200	(0x60)  /* 200Hz output data rate */
#define LIS2DH_ACC_ODR400	(0x70)  /* 400Hz output data rate */
#define LIS2DH_ACC_ODR1250	(0x90)  /* 1250Hz output data rate */



#define	IA			(0x40)
#define	ZH			(0x20)
#define	ZL			(0x10)
#define	YH			(0x08)
#define	YL			(0x04)
#define	XH			(0x02)
#define	XL			(0x01)
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	(0x40)
#define	CTRL_REG4_BDU_ENABLE	(0x80)
#define	CTRL_REG4_BDU_MASK	(0x80)
#define	CTRL_REG6_I2_TAPEN	(0x80)
#define	CTRL_REG6_HLACTIVE	(0x02)
/* */
#define NO_MASK			(0xFF)
#define INT1_DURATION_MASK	(0x7F)
#define	INT1_THRESHOLD_MASK	(0x7F)
#define TAP_CFG_MASK		(0x3F)
#define	TAP_THS_MASK		(0x7F)
#define	TAP_TLIM_MASK		(0x7F)
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			(0x20)
#define	STAP			(0x10)
#define	SIGNTAP			(0x08)
#define	ZTAP			(0x04)
#define	YTAP			(0x02)
#define	XTAZ			(0x01)


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	(0x80)

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */

#define WRITE_SINGLE_MASK		(0x00) 
#define WRITE_MULTI_MASK		(0x40) 
#define READ_SINGLE_MASK		(0x80) 
#define READ_MULTI_MASK		(0xC0) 

#include <stdio.h>

#include <lis2dh12.h>

#include "nrf_delay.h"
#include "nrf_drv_spi.h"

#include "app_util_platform.h"
#include "bsp.h"

#include <string.h>

#include <math.h>

#define AUTO_CAL//***** enable then pitch/roll change exceed threshold
#define ROLL_PITCH_CAL
#define YAW_CAL

extern uint8_t sensor_internal_ms;
int offset_roll_pitch_period_ms = 1000;
float d_pitch = 0;
float d_roll = 0;
float d_yaw = 0;

uint8_t offset_cnt = 0;
float offset_xyz[3] = {0};
	
static const nrf_drv_spi_t m_spi0_master = NRF_DRV_SPI_INSTANCE(0);
static volatile bool spi0_transfer_completed = false; /**< A flag to inform about completed transfer. */


//static void spi0_master_event_handler(nrf_drv_spi_event_t event)
static void spi0_master_event_handler(nrf_drv_spi_evt_t event)
{
    switch (event.type)
    {
        case NRF_DRV_SPI_EVENT_DONE:
						spi0_transfer_completed = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void spi0_send_recv(uint8_t * const p_tx_data,
                          uint8_t * const p_rx_data,
                          const uint16_t  len)
{
    // Start transfer.
		spi0_transfer_completed = false;
    uint32_t err_code = nrf_drv_spi_transfer(&m_spi0_master,
        p_tx_data, len, p_rx_data, len);
	  while (!spi0_transfer_completed) {}
}

bool lis2dh12_init(void)
{
		uint8_t p_tx_data[2] = {0};
    uint8_t p_rx_data[2] = {0};

    nrf_drv_spi_config_t const config =
    {
        .sck_pin  = SPIM0_SCK_PIN,
        .mosi_pin = SPIM0_MOSI_PIN,
        .miso_pin = SPIM0_MISO_PIN,
        .ss_pin   = SPIM0_SS_PIN,

        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_8M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
    nrf_drv_spi_init(&m_spi0_master, &config, spi0_master_event_handler);
		
		p_tx_data[0] = WHO_AM_I | READ_SINGLE_MASK;
		spi0_send_recv(p_tx_data, p_rx_data, 2);

		if (p_rx_data[1] != WHOAMI_LIS2DH_ACC) {
				return false;
		}

		p_tx_data[0] = CTRL_REG1;
		p_tx_data[1] = 0x67;			//200Hz, X/Y/Z axis enabled
		spi0_send_recv(p_tx_data, p_rx_data, 2);
		
		//wait for turn-on time
		nrf_delay_us(1600);
		
		p_tx_data[0] = CTRL_REG4;
		p_tx_data[1] = 0x38;			//+/-16g, High Resolution (12 bits)
		spi0_send_recv(p_tx_data, p_rx_data, 2);

		//wait for turn-on time
		nrf_delay_ms(2);
		
		return true;
}

void lis2dh_powerDown(void)
{			
		uint8_t p_tx_data[2] = {0};
    uint8_t p_rx_data[2] = {0};	

		p_tx_data[0] = CTRL_REG1;
		p_tx_data[1] = 0x00;			//power-down mode, X/Y/Z axis disabled
		spi0_send_recv(p_tx_data, p_rx_data, 1);

}

void lis2dh12_acc_data(float acc_g_xyz[3], float cal_acc_g_xyz[3])
{
		uint8_t p_tx_data[7] = {0};
    uint8_t p_rx_data[7] = {0};	
		float d2r = 0.0174532925;
		float r2d = 57.2957795;
		
		p_tx_data[0] = AXISDATA_REG | READ_MULTI_MASK;
		spi0_send_recv(p_tx_data, p_rx_data, 7);

		/*calculate the sensor xyz-axis in g*/
		for(int i=0; i<3; i++) {                   
			acc_g_xyz[i]=(int16_t)(((uint16_t)p_rx_data[2*i+2] << 8) + p_rx_data[2*i+1]) >> 4;
			if (acc_g_xyz[i] < 0x800)
				acc_g_xyz[i] = ((float)(acc_g_xyz[i] * 12)/1000);
			else
				acc_g_xyz[i] = -(1 - (float)((acc_g_xyz[i]-0x800) * 12)/1000);
		}


	if (offset_roll_pitch_period_ms==1000) {
		offset_cnt = 0;
		offset_xyz[0] = 0;
		offset_xyz[1] = 0;
		offset_xyz[2] = 0;		
#ifdef ROLL_PITCH_CAL 
		/*calculate the roll degree among 90 to -90*/
		d_roll = atan(acc_g_xyz[1]/sqrt(acc_g_xyz[0]*acc_g_xyz[0]+acc_g_xyz[2]*acc_g_xyz[2]))*r2d;
		
		/*calculate the pitch degree among 180 to -180*/
		if (acc_g_xyz[0]>=0 && acc_g_xyz[2]<0) {
			d_pitch = 180-atan(acc_g_xyz[0]/sqrt(acc_g_xyz[1]*acc_g_xyz[1]+acc_g_xyz[2]*acc_g_xyz[2]))*r2d;
		} else if (acc_g_xyz[0]<0 && acc_g_xyz[2]<0) {
			d_pitch = -180-atan(acc_g_xyz[0]/sqrt(acc_g_xyz[1]*acc_g_xyz[1]+acc_g_xyz[2]*acc_g_xyz[2]))*r2d;
		} else {		
			d_pitch = atan(acc_g_xyz[0]/sqrt(acc_g_xyz[1]*acc_g_xyz[1]+acc_g_xyz[2]*acc_g_xyz[2]))*r2d;
		}
		//printf("init_pitch: %f init_roll: %f \n\r", d_pitch, d_roll);		
#endif
	}	


		/*calculate the object xyz-axis in g*/
		float orientationvalues[3] = {0*d2r, -d_roll*d2r, d_pitch*d2r};//yaw(rotate on z-axis), roll(rotate on x-axis), pitch(rotate on y-axis)
		cal_acc_g_xyz[0] =(float) (acc_g_xyz[0]*(cos(orientationvalues[2])*cos(orientationvalues[0])+sin(orientationvalues[2])*sin(orientationvalues[1])*sin(orientationvalues[0])) + acc_g_xyz[1]*(cos(orientationvalues[1])*sin(orientationvalues[0])) + acc_g_xyz[2]*(-sin(orientationvalues[2])*cos(orientationvalues[0])+cos(orientationvalues[2])*sin(orientationvalues[1])*sin(orientationvalues[0])));
		cal_acc_g_xyz[1] =(float) (acc_g_xyz[0]*(-cos(orientationvalues[2])*sin(orientationvalues[0])+sin(orientationvalues[2])*sin(orientationvalues[1])*cos(orientationvalues[0])) + acc_g_xyz[1]*(cos(orientationvalues[1])*cos(orientationvalues[0])) + acc_g_xyz[2]*(sin(orientationvalues[2])*sin(orientationvalues[0])+ cos(orientationvalues[2])*sin(orientationvalues[1])*cos(orientationvalues[0])));
		cal_acc_g_xyz[2] =(float) (acc_g_xyz[0]*(sin(orientationvalues[2])*cos(orientationvalues[1])) + acc_g_xyz[1]*(-sin(orientationvalues[1])) + acc_g_xyz[2]*(cos(orientationvalues[2])*cos(orientationvalues[1])));

	if (offset_roll_pitch_period_ms>0) {
		offset_cnt++;
		offset_xyz[0] = ((offset_xyz[0] * (offset_cnt - 1)) + cal_acc_g_xyz[0])/offset_cnt; 
		offset_xyz[1] = ((offset_xyz[1] * (offset_cnt - 1)) + cal_acc_g_xyz[1])/offset_cnt; 
		offset_xyz[2] = ((offset_xyz[2] * (offset_cnt - 1)) + cal_acc_g_xyz[2])/offset_cnt;  		
		offset_roll_pitch_period_ms-=sensor_internal_ms;		
	}
}
