#ifndef INC_WIZCHIP_H_
#define INC_WIZCHIP_H_

#include "main.h"
#include "socket.h"

extern SPI_HandleTypeDef hspi2;

typedef struct
{
	SPI_HandleTypeDef *spi_port_handler;
	struct net_buf_t
	{
		uint8_t rx[4];
		uint8_t tx[4];
	} net_buf;
	wiz_NetInfo net_info;
} w5500_t ;


void wizchip_select(void);
void wizchip_deselect(void);
uint8_t wizchip_read_byte(void);
void wizchip_write_byte(uint8_t wb);
void wizchip_readburst(uint8_t buf[], uint16_t len);
void wizchip_writeburst(uint8_t buf[], uint16_t len);
void w5500_hwrst(void);
void w5500_init(w5500_t *dev);
void tcp_client();
void tcp_server();

#endif /* INC_WIZCHIP_H_ */
