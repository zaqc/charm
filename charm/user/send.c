/*
 * send.c
 *
 *  Created on: 2024 Nov 22
 *      Author: zaqc
 */

#include <stdlib.h>
#include <stdint.h>

#include "at32f435_437.h"
#include "send.h"

uint16_t pkt_counter = 0;

void pkt_envelop(uint16_t *in_data, uint16_t *out_data, uint16_t len,
		uint8_t pack_size) {
	register uint16_t i = len;
	register uint16_t *in_ptr = in_data;
	register uint16_t *out_ptr = out_data;
	while (i-- > 0) {
		register uint16_t max = (*(int16_t*) in_ptr - (int16_t) 512) & 0x1FF;
		in_ptr++;
		register uint8_t n = pack_size;
		while (n-- > 0) {
			register uint16_t tmp = (*(int16_t*) in_ptr - (int16_t) 512)
					& 0x1FF;
			if (max < tmp)
				max = tmp;
		}
		*out_ptr = max;
		out_ptr++;
	}
}

int16_t pkt_pack(uint16_t *in_data, uint16_t *out_data, uint16_t len) {

	register uint32_t pkt_crc = 0;

	//crc_init_data_set(0xFFFFFFFF);
	CRC->idt = 0xFFFFFFFF;

	// Create Block Header
	register uint8_t *out_ptr = (uint8_t*) out_data;
	*(uint32_t*) out_ptr = PKT_MAGIC_ID;
	CRC->dt = PKT_MAGIC_ID;
	pkt_crc = CRC->dt; //crc_one_word_calculate(PKT_MAGIC_ID);
	out_ptr += sizeof(uint32_t);

	*(uint32_t*) out_ptr =  (((uint32_t) pkt_counter) << 16) | PKT_DATA_COUNT;
	CRC->dt = *(uint32_t*) out_ptr;
	pkt_crc = CRC->dt; //crc_one_word_calculate(*(uint32_t*) out_ptr);
	out_ptr += sizeof(uint32_t);
	pkt_counter++;

	*(uint32_t*) out_ptr = pkt_crc;
	CRC->dt = pkt_crc;
	pkt_crc = CRC->dt; //crc_one_word_calculate(*(uint32_t*) out_ptr);
	out_ptr += sizeof(uint32_t);

	// Decode US Data
	register uint16_t i = len;

	register uint16_t *ptr = in_data;

	while (i-- > 0) {
		register uint32_t tmp = 0xC00 | *(uint16_t*) ptr;
		tmp <<= 10;
		ptr++;
		tmp |= *(uint16_t*) ptr;
		tmp <<= 10;
		ptr++;
		tmp |= *(uint16_t*) ptr;
		pkt_crc = crc_one_word_calculate(tmp);
		ptr++;
		*(uint32_t*) out_ptr = tmp;
		out_ptr += sizeof(uint32_t);
	}

	// write final CRC32
	*(uint32_t*) out_ptr = pkt_crc;

	return (3/*hdr*/+ len + 1/*crc*/) * 2; // size in uint16_t for send by SPI (16 bit)
}

//uint16_t send_buf[4096];

void pkt_send(uint16_t *data, uint16_t len) {	// Data Stream
	gpio_init_type gpio_param;
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	gpio_default_para_init(&gpio_param);
	/* spi1 cs pin */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_5);
	/* spi1 sck pin */
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_5;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);
	/* spi1 mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_7;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);

	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL4);
	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL4, DMAMUX_DMAREQ_ID_SPI1_TX);
	dma_init_type dma_param;
	dma_default_para_init(&dma_param);
	dma_param.direction = DMA_DIR_MEMORY_TO_PERIPHERAL; // Transfer memory buffer to SPI1_TX
	dma_param.buffer_size = len;
	dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;	// 16 bit
	dma_param.memory_inc_enable = TRUE;
	dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_param.peripheral_inc_enable = FALSE;
	dma_param.priority = DMA_PRIORITY_HIGH;
	dma_param.loop_mode_enable = FALSE;
	dma_param.memory_base_addr = (uint32_t) data;
	dma_param.peripheral_base_addr = (uint32_t) &(SPI1->dt);
	dma_init(DMA1_CHANNEL4, &dma_param);

	crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
	spi_param.master_slave_mode = SPI_MODE_MASTER;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	//spi_param.clock_polarity = SPI_CLOCK_POLARITY_LOW;
	//spi_param.clock_phase = SPI_CLOCK_PHASE_2EDGE;
	spi_param.cs_mode_selection = SPI_CS_HARDWARE_MODE;
	spi_init(SPI1, &spi_param);
	spi_i2s_dma_transmitter_enable(SPI1, TRUE);
	spi_ti_mode_enable(SPI1, TRUE);
	spi_enable(SPI1, TRUE);

	dma_channel_enable(DMA1_CHANNEL4, TRUE);
}
