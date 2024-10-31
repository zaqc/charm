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

	dmamux_sync_init_type dmamux_param;
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
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	GPIOA->clr = GPIO_PINS_2;
	delay_us(200);
	pulse_count = 0;
}

void TMR1_OVF_TMR10_IRQHandler(void) {
	tmr_flag_clear(TMR1, TMR_OVF_FLAG);
}

/**
 * @brief	Initialize TMR1 as Master for synchronize Pulse (TMR2 -> (TMR5 -> TMR3)), ADC (TMR4) & VRC(DAC) TMR8
 */
void internal_sync_init() {
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_pins = GPIO_PINS_8;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);	// TMR1_CH1 PORT_A PIN_8


	// Master
	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

	tmr_base_init(TMR1, 7999, 35);	// 288MHz / 36 (35) = 8 MHz / 8000 = 1 kHz (8000 - 1 = 7999)
	tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
	tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
	tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

	tmr_output_config_type tmr_param;
	tmr_output_default_para_init(&tmr_param);
	tmr_param.oc_idle_state = TRUE;
	tmr_param.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
	tmr_param.oc_output_state = TRUE;
	tmr_param.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
	tmr_param.occ_idle_state = TRUE;
	tmr_param.occ_output_state = TRUE;
	tmr_param.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_param);
	tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, 1);

	tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);

	tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);	// Master Timer
	tmr_sub_sync_mode_set(TMR1, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);

	tmr_output_enable(TMR1, TRUE);
	tmr_counter_enable(TMR1, TRUE);
}
