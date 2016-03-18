 /* 
 *	Generated for MXIC flash
 */

#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#define MFG_ID_FLASH (0xC2)							//Tsungta, updated for MXIC
#define DEVICE_ID_FLASH (0x2813)				//Tsungta, updated for MX25R4035F

#define CMD_POWER_UP (0xAB)
#define CMD_JEDEC_ID	(0x9F)
#define CMD_POWER_DOWN	(0xB9)
#define CMD_READ_STATUS	(0x05)
#define CMD_WRITE_ENABLE (0x06)
#define CMD_PAGE_PROG (0x02)
#define CMD_READ_DATA (0x03)
#define CMD_ERASE_4K (0x20)
#define CMD_ERASE_64K (0xD8)
#define CMD_DUMMY	(0xFF)

#define	CMD_LENGTH	1
#define	THREE_BYTE_LENGTH	3
#define DEVICE_PAGE_SIZE (256)
#define DEVICE_SECTOR_SIZE (4096)
#define DEVICE_BLOCK_SIZE (65536)

bool spi_flash_init(void);
bool spi_flash_powerDown(void);

void spi_flash_eraseCmd(uint8_t command, uint32_t address);
void spi_flash_writepage(uint32_t address, uint8_t *data, uint16_t len);
void spi_flash_readpage(uint32_t address, uint8_t *data, uint16_t len);

#endif /* SPI_MASTER_H */
