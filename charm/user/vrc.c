/*
 * vrc.c
 *
 *  Created on: 2024 Oct 31
 *      Author: zaqc
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

uint16_t dac_buf[2048];
uint16_t *read_buf_ptr = dac_buf;
uint16_t *fill_buf_ptr = &dac_buf[1024];
confirm_state vrc_changed = FALSE;
uint16_t vrc_max_len = 200;

/**
 * @brief	Initialize Timer8 and SPI_DAC Pins
 *			PORT_D Pin0 - CS0
 *			PORT_D Pin3 - CS1
 *			Timer8 - 1 MHz with IRQ
 */
void vrc_init(void) {
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_3);	// TMR8_EXT

	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_0 | GPIO_PINS_3;
	gpio_init(GPIOD, &gpio_param);

	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_7);

	/* spi2 sck pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_6);
	/* spi2 mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_6);

	/* DAC GPIO CS */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pins = GPIO_PINS_3;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_init(GPIOD, &gpio_param);


	crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
	spi_param.master_slave_mode = SPI_MODE_MASTER;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	spi_param.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_param.clock_phase = SPI_CLOCK_PHASE_1EDGE;
	spi_param.cs_mode_selection = SPI_CS_SOFTWARE_MODE;// HARDWARE_MODE;
	spi_init(SPI2, &spi_param);
//	spi_ti_mode_enable(SPI2, TRUE);
	spi_hardware_cs_output_enable(SPI2, FALSE); //TRUE);
//	spi_i2s_dma_transmitter_enable(SPI2, TRUE);

	//spi_software_cs_internal_level_set(SPI2, SPI_SWCS_INTERNAL_LEVEL_LOW);

	spi_enable(SPI2, TRUE);

	GPIOD->scr = GPIO_PINS_0;
	GPIOD->clr = GPIO_PINS_3;
//	dma_channel_enable(DMA1_CHANNEL3, TRUE);

	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);

	// Time 8 Initialize
	crm_periph_clock_enable(CRM_TMR8_PERIPH_CLOCK, TRUE);
	tmr_base_init(TMR8, 17, 15);	// 288MHz / 16 (15) = 18 (cntr 18 - 1 = 17)
	tmr_cnt_dir_set(TMR8, TMR_COUNT_UP);
	tmr_interrupt_enable(TMR8, TMR_OVF_INT, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR8_OVF_TMR13_IRQn, 1, 0);

	tmr_sub_mode_select(TMR8, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR8, TMR_SUB_INPUT_SEL_EXTIN);	// Sync by TMR8_EXT (PA0)

	tmr_counter_enable(TMR8, TRUE);
}

int vrc_val_ptr = 0;
//void TMR8_OVF_TMR13_IRQHandler(void) {
//	tmr_flag_clear(TMR8, TMR_OVF_FLAG);
//
//	GPIOD->scr = GPIO_PINS_0;
//	GPIOD->scr = GPIO_PINS_3;
//
//	uint16_t val = read_buf_ptr[vrc_val_ptr++];
//
//	if (vrc_val_ptr >= vrc_max_len) {
//		vrc_val_ptr = 0;
//		tmr_counter_enable(TMR8, FALSE);
//	}
//
//	GPIOD->clr = GPIO_PINS_3;
//	GPIOD->clr = GPIO_PINS_0;
//
//	spi_i2s_data_transmit(SPI2, val);
//}

void TMR8_OVF_TMR13_IRQHandler(void) {
	__disable_irq();

	tmr_flag_clear(TMR8, TMR_OVF_FLAG);

	if (vrc_val_ptr < vrc_max_len) {
		GPIOD->scr = GPIO_PINS_0;
		GPIOD->scr = GPIO_PINS_3;

		uint16_t val = read_buf_ptr[vrc_val_ptr++];

		GPIOD->clr = GPIO_PINS_0;
		GPIOD->clr = GPIO_PINS_3;

		spi_i2s_data_transmit(SPI2, val);
	} else {
		vrc_val_ptr = 0;
		tmr_counter_enable(TMR8, FALSE);

		GPIOD->scr = GPIO_PINS_0;
	}

	__enable_irq();
}

void vrc_prepare(uint32_t start_amp, uint32_t inc_amp1, uint32_t inc_amp2, uint16_t vrc_len, uint16_t amp_porge) {
	uint32_t val = start_amp;
	for(uint16_t i = 0; i < vrc_len; i++) {
		fill_buf_ptr[i] = val >> 16;
		val += inc_amp1;
	}
	for(uint16_t i = vrc_len; i < vrc_max_len; i++) {
		fill_buf_ptr[i] = val >> 16;
		val += inc_amp2;
	}
	vrc_changed = TRUE;
}

void vrc_set(void) {
	tmr_counter_enable(TMR8, FALSE);
	if (vrc_changed) {
		//NVIC_DisableIRQ();
		__disable_irq();
		if(read_buf_ptr != dac_buf) {
			read_buf_ptr = dac_buf;
			fill_buf_ptr = &dac_buf[1024];
		} else {
			read_buf_ptr = &dac_buf[1024];
			fill_buf_ptr = dac_buf;
		}
		vrc_val_ptr = 0;
		//NVIC_EnableIRQ();
		__enable_irq();
		vrc_changed = FALSE;
	}
	tmr_counter_enable(TMR8, TRUE);
}
