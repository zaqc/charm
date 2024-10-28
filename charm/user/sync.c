/*
 * sync.c
 *
 *  Created on: 2024 Jul 9
 *      Author: zaqc
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#include "pulse.h"

void EXINT1_IRQHandler(void) {
	if (exint_flag_get(EXINT_LINE_1) != RESET) {
		exint_flag_clear(EXINT_LINE_1);
	}
}

void init_sync(void) {
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

	gpio_init_type gpio_param = { 0 };
	gpio_param.gpio_pins = GPIO_PINS_9;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_param);

	scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);

	exint_init_type exint_param;
	exint_default_para_init(&exint_param);
	exint_param.line_enable = TRUE;
	exint_param.line_mode = EXINT_LINE_INTERRUPUT;
	exint_param.line_select = EXINT_LINE_1;
	exint_param.line_polarity = EXINT_TRIGGER_RISING_EDGE;
	exint_init(&exint_param);

	dmamux_sync_init_type  dmamux_param;
	dmamux_sync_default_para_init(&dmamux_param);
	dmamux_param.sync_request_number = 4096;
	dmamux_param.sync_signal_sel = DMAMUX_SYNC_ID_EXINT1;
	dmamux_param.sync_polarity = DMAMUX_SYNC_POLARITY_RISING;
	dmamux_param.sync_event_enable = TRUE;
	dmamux_param.sync_enable = TRUE;
	dmamux_sync_config(DMA2MUX_CHANNEL4, &dmamux_param);

	/* exint line1 interrupt nvic init */
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT1_IRQn, 1, 0);
}

void send_sync(void) {
	tmr_counter_value_set(TMR3, 0);

	tmr_counter_value_set(TMR2, 0);
	tmr_counter_value_set(TMR5, 0);


	GPIOA->scr = GPIO_PINS_2;
	__asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP");
	GPIOA->clr = GPIO_PINS_2;
	delay_us(200);
	pulse_count = 0;
}
