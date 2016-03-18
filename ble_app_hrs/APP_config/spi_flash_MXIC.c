
#include <spi_flash_MXIC.h>

#include "nrf_delay.h"
#include "nrf_drv_spi.h"

#include "app_util_platform.h"
#include "bsp.h"

#include <string.h>

static const nrf_drv_spi_t m_spi1_master = NRF_DRV_SPI_INSTANCE(1);
static volatile bool spi1_transfer_completed = false; /**< A flag to inform about completed transfer. */


static void spi1_master_event_handler(nrf_drv_spi_evt_t const * p_event)
{
//    switch (event)
//    {
//        case NRF_DRV_SPI_EVENT_DONE:
						spi1_transfer_completed = true;
//            break;

//        default:
//            // No implementation needed.
//            break;
//    }
}

static void spi1_send_recv(uint8_t * const p_tx_data,
                          uint8_t * const p_rx_data,
                          const uint16_t  len)
{
    // Start transfer.
		spi1_transfer_completed = false;
    uint32_t err_code = nrf_drv_spi_transfer(&m_spi1_master,
        p_tx_data, len, p_rx_data, len);
	  while (!spi1_transfer_completed) {}
}

bool spi_flash_init(void)
{
		uint8_t p_tx_data[4] = {0};
    uint8_t p_rx_data[4] = {0};
	
		uint8_t mfgId;
		uint16_t deviceID;
	
    //uint32_t * p_spi_base_address = 0;

    nrf_drv_spi_config_t const config =
    {
        .sck_pin  = SPIM1_SCK_PIN,
        .mosi_pin = SPIM1_MOSI_PIN,
        .miso_pin = SPIM1_MISO_PIN,
        .ss_pin   = SPIM1_SS_PIN,

        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_8M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
    nrf_drv_spi_init(&m_spi1_master, &config, spi1_master_event_handler);
		
    //spi_flash_writeOneByte(p_spi_base_address, CMD_POWER_UP);
		p_tx_data[0] = CMD_POWER_UP;//CMD_POWER_UP can act as dummy byte when using MXIC
		spi1_send_recv(p_tx_data, p_rx_data, 1);

		//wait for wake up
		nrf_delay_us(35);

		//spi_flash_writeOneByte(p_spi_base_address, CMD_JEDEC_ID);
		p_tx_data[0] = CMD_JEDEC_ID;
		spi1_send_recv(p_tx_data, p_rx_data, 4);
		
		mfgId = p_rx_data[1];
		deviceID = (uint16_t)(p_rx_data[2] << 8);
		deviceID |= p_rx_data[3];

		if (mfgId != MFG_ID_FLASH || deviceID != DEVICE_ID_FLASH) {
				return false;
		}
		
		return true;
}

bool spi_flash_powerDown(void)
{			
    //uint32_t * p_spi_base_address = 0;

		uint8_t p_tx_data[1] = {0};
    uint8_t p_rx_data[1] = {0};	

    //spi_flash_writeOneByte(p_spi_base_address, CMD_POWER_DOWN);
		p_tx_data[0] = CMD_POWER_DOWN;
		spi1_send_recv(p_tx_data, p_rx_data, 1);

		//wait for sleep
		nrf_delay_us(10);
		
		return true;
}

bool spi_flash_waitBusy(void)
{
		//uint8_t status;
    //uint32_t * p_spi_base_address = 0;

		uint8_t p_tx_data[2] = {0};
    uint8_t p_rx_data[2] = {0};	
		
    //spi_flash_writeOneByte(p_spi_base_address, CMD_READ_STATUS);
		//status = spi_flash_readOneByte(p_spi_base_address);
		p_tx_data[0] = CMD_READ_STATUS;
		spi1_send_recv(p_tx_data, p_rx_data, 2);
		
		if ( (p_rx_data[1] & 0x01) == 0x01 )
		{
				return true;
		} else {
				return false;
		}
}

void spi_flash_setWEL(void)
{
    //uint32_t * p_spi_base_address = 0;

		uint8_t p_tx_data[1] = {0};
    uint8_t p_rx_data[1] = {0};	
		
    //spi_flash_writeOneByte(p_spi_base_address, CMD_WRITE_ENABLE);
		p_tx_data[0] = CMD_WRITE_ENABLE;
		spi1_send_recv(p_tx_data, p_rx_data, 1);
		
}

void spi_flash_eraseCmd(uint8_t command, uint32_t address)
{

		uint8_t p_tx_data[4] = {0};
    uint8_t p_rx_data[4] = {0};	
		
		//wait busy
		while(spi_flash_waitBusy()) {};//Tsungta,
			
		//setWEL
		spi_flash_setWEL();	

		p_tx_data[0] = command;
		
		p_tx_data[1] = ((address >> 16) & 0xFF);
		p_tx_data[2] = ((address >> 8) & 0xFF);
		p_tx_data[3] = (address & 0xFF);
		
		spi1_send_recv(p_tx_data, p_rx_data, 4);

}

void spi_flash_writepage(uint32_t address, uint8_t *data, uint16_t len)
{
		//wait busy
		while(spi_flash_waitBusy()) {};
		
		//setWEL
		spi_flash_setWEL();

    //uint32_t * p_spi_base_address = 0;
    
		uint8_t p_tx_data[CMD_LENGTH+THREE_BYTE_LENGTH+len];
    uint8_t p_rx_data[CMD_LENGTH+THREE_BYTE_LENGTH+len];
		
		p_tx_data[0] = CMD_PAGE_PROG;
		
		p_tx_data[1] = ((address >> 16) & 0xFF);
		p_tx_data[2] = ((address >> 8) & 0xFF);
		p_tx_data[3] = (address & 0xFF);
		
		memcpy(&p_tx_data[4], data, len);
		spi1_send_recv(p_tx_data, p_rx_data, CMD_LENGTH+THREE_BYTE_LENGTH+len);

		return;
}

void spi_flash_readpage(uint32_t address, uint8_t *data, uint16_t len)
{

		//wait busy
		while(spi_flash_waitBusy()) {};
			
    //uint32_t * p_spi_base_address = 0;
    
		uint8_t p_tx_data[CMD_LENGTH+THREE_BYTE_LENGTH+len];
    uint8_t p_rx_data[CMD_LENGTH+THREE_BYTE_LENGTH+len];

		p_tx_data[0] = CMD_READ_DATA;
		
		p_tx_data[1] = ((address >> 16) & 0xFF);
		p_tx_data[2] = ((address >> 8) & 0xFF);
		p_tx_data[3] = (address & 0xFF);

		spi1_send_recv(p_tx_data, p_rx_data, CMD_LENGTH+THREE_BYTE_LENGTH+len);
		memcpy(data, &p_rx_data[4], len);
		
}
