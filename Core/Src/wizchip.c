#include "wizchip.h"



void wizchip_select(void)
{
	HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void wizchip_deselect(void)
{
	HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

uint8_t wizchip_read_byte(void)
{
	uint8_t rb;
	rb = HAL_SPI_Receive(&hspi2, &rb, 1, HAL_MAX_DELAY);
	return rb;
}

void wizchip_write_byte(uint8_t wb)
{
	HAL_SPI_Transmit(&hspi2, &wb, 1, HAL_MAX_DELAY);
}

void wizchip_readburst(uint8_t buf[], uint16_t len)
{
	HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
}

void wizchip_writeburst(uint8_t buf[], uint16_t len)
{
	HAL_SPI_Transmit(&hspi2, buf, len, HAL_MAX_DELAY);
}

void w5500_hwrst(void)
{
	HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
	/*Should wait 500 us at least (10th page of datasheet)*/
	HAL_Delay(5);
	HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
}

void w5500_init(w5500_t *dev)
{
	/*Assign lib to custom functions*/
	reg_wizchip_spi_cbfunc(wizchip_read_byte, wizchip_write_byte);
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);

	wizchip_init(dev->net_buf.tx, dev->net_buf.rx);
	wizchip_setnetinfo(&dev->net_info);
	wizchip_getnetinfo(&dev->net_info);
	ctlnetwork(CN_SET_NETINFO, &dev->net_info);

//	tcp_client();
	tcp_server();
}

void tcp_client()
{
	uint8_t sockStatus;

	sockStatus = socket(0, Sn_MR_TCP, 5000, 0);
	uint8_t serverip[] = {192, 168, 1, 40};
	while(connect(0, serverip, 5000) != SOCK_OK);
}

void tcp_server()
{
	uint8_t sockStatus;

	sockStatus = socket(0, Sn_MR_TCP, 5000, 0);
	listen(0);
	while((sockStatus = getSn_SR(0)) == SOCK_LISTEN)
	{
		HAL_Delay(200);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
	uint8_t remoteIP[4];
	uint16_t remotePort;
	getsockopt(0, SO_DESTIP, remoteIP);
	getsockopt(0, SO_DESTPORT, (uint8_t*)&remotePort);
	disconnect(0);
	close(0);
}
