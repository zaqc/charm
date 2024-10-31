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
uint16_t vrc_max_len = 1024;

/**
 * @brief	Initialize Timer8 and SPI_DAC Pins
 *			PORT_D Pin0 - CS0
 *			PORT_D Pin3 - CS1
 *			Timer8 - 1 MHz with IRQ
 */
void vrc_init(void) {
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);

	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
	tmr_base_init(TMR8, 17, 15);	// 288MHz / 16 (15) = 18 (cntr 18 - 1 = 17)
	tmr_cnt_dir_set(TMR8, TMR_COUNT_UP);
	tmr_interrupt_enable(TMR8, TMR_OVF_INT, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR8_OVF_TMR13_IRQn, 1, 0);

	tmr_sub_mode_select(TMR8, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR8, TMR_SUB_INPUT_SEL_IS0);	// IS0 - TMR8 start by TMR1

	tmr_counter_enable(TMR8, TRUE);
}

int vrc_val_ptr = 0;
void TMR8_OVF_TMR13_IRQHandler(void) {
	tmr_flag_clear(TMR8, TMR_OVF_FLAG);

	GPIOD->scr = GPIO_PINS_0;
	GPIOD->scr = GPIO_PINS_3;

	uint16_t val = read_buf_ptr[vrc_val_ptr++];

	if (vrc_val_ptr >= vrc_max_len) {
		vrc_val_ptr = 0;
		tmr_counter_enable(TMR8, FALSE);
	}

	GPIOD->clr = GPIO_PINS_3;
	GPIOD->clr = GPIO_PINS_0;

	spi_i2s_data_transmit(SPI2, val);
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
