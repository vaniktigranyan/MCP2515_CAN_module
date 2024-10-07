#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "can.h"
#include "mcp2515.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_task_wdt.h"




#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   13

#define PIN_NUM_INTERRUPT 21


#define TAG "CAN_MODULE"
spi_device_handle_t spi;

CAN_FRAME_t can_frame_rx[1];

bool SPI_Init(void)
{
	printf("Hello from SPI_Init!\n\r");
	esp_err_t ret;
	//Configuration for the SPI bus
	spi_bus_config_t bus_cfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz = 0 // no limit
	};

	// Define MCP2515 SPI device configuration
	spi_device_interface_config_t dev_cfg = {
		.mode = 0, // (0,0)
		.clock_speed_hz = 10000000, 
		.spics_io_num = PIN_NUM_CS,
		.queue_size = 128
	};
   
	// Initialize SPI bus
	ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

    // Add MCP2515 SPI device to the bus
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &MCP2515_Object->spi);
    ESP_ERROR_CHECK(ret);

    return true;
}

void app_main(void)
{
        printf("Hello from app_main!\n");

	MCP2515_init();
	SPI_Init();
	MCP2515_reset();
	MCP2515_setBitrate(CAN_500KBPS, MCP_8MHZ);
	MCP2515_setNormalMode();

	can_frame_rx[0]->can_id = (0x12344321) | CAN_EFF_FLAG;
	can_frame_rx[0]->can_dlc = 8;
	can_frame_rx[0]->data[0] = 0x01;
	can_frame_rx[0]->data[1] = 0x02;
	can_frame_rx[0]->data[2] = 0x03;
	can_frame_rx[0]->data[3] = 0x04;
	can_frame_rx[0]->data[4] = 0x05;
	can_frame_rx[0]->data[5] = 0x06;
	can_frame_rx[0]->data[6] = 0x07;
	can_frame_rx[0]->data[7] = 0x08;
	
	while(1){
		if(MCP2515_sendMessageAfterCtrlCheck(can_frame_rx[0]) != ERROR_OK){
			ESP_LOGE(TAG, "Couldn't send message.");
		}
		vTaskDelay(1000); // check freertos tickrate for make this delay 1 second
	}
   

}