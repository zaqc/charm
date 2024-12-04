/*
 * pulse.c
 *
 *  Created on: 2024 Jul 7
 *      Author: zaqc
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

volatile uint32_t pulse_count = 0;
uint32_t full_period = 114;

/**
 * @brief	Initialze TMR2, TMR5 GPIO
 * 			configure TMR3 as slave for start from master TMR1
 * 			and as master for run slaves timers TMR2 -> TMR5
 * 			TMR5_CH1 - PORTA_0 MUX_2 -> TMR5_CH4 PORTA_3 MUX2
 * 			TMR2_CH1 - PORTA_5 MUX_1
 *
 * 			TMR3 - run on 2.5 MHz (288 MHz / 115 = 2,504347826 MHz)
 * 			TMR3 - TMR_SUB_TRIGGER_MODE, TMR_SUB_INPUT_SEL_IS0
 */
void init_pulse_pio(void) {
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_pins = GPIO_PINS_3;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_2);	// PORT_A_3 MUX_2 - TMR5_CH4

	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	gpio_param.gpio_pins = GPIO_PINS_2;
	gpio_init(GPIOB, &gpio_param);
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE2, GPIO_MUX_1);	// PORT_B_2 MUX_1 - TMR2_CH4

	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_param.gpio_pins =  GPIO_PINS_2;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE2, GPIO_MUX_2);	// PORT_E_2 - MUX_2 - TMR_3_EXT

	crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
	tmr_base_init(TMR3, 114, 0);
	tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
	tmr_output_config_type tmr_param;
	tmr_output_default_para_init(&tmr_param);
	tmr_param.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
	tmr_param.oc_output_state = TRUE;
	tmr_param.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
	tmr_param.oc_idle_state = TRUE;
	tmr_param.occ_output_state = TRUE;
	tmr_param.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
	tmr_param.occ_idle_state = TRUE;

	tmr_base_init(TMR3, full_period, 0);

	tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);
	tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_2, &tmr_param);
	tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_2, 56);	// set pulse width

	tmr_sub_mode_select(TMR3, TMR_SUB_TRIGGER_MODE); // TMR_SUB_TRIGGER_MODE); External start by Sync signal
	tmr_trigger_input_select(TMR3, TMR_SUB_INPUT_SEL_EXTIN);	// IS0 - TMR1 start TMR3
	tmr_one_cycle_mode_enable(TMR3, FALSE);

	tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);

	tmr_primary_mode_select(TMR3, TMR_PRIMARY_SEL_OVERFLOW);
	tmr_sub_sync_mode_set(TMR3, TRUE);

	pulse_count = 0;
	tmr_interrupt_enable(TMR3, TMR_OVF_INT, TRUE);
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR3_GLOBAL_IRQn, 1, 0);

	tmr_output_enable(TMR3, TRUE);
	tmr_counter_enable(TMR3, TRUE);
}

void TMR3_GLOBAL_IRQHandler(void) {
	TMR3->ists = ~TMR_OVF_FLAG;

	if(++pulse_count >= 4) {
		tmr_counter_enable(TMR3, FALSE);
		tmr_counter_value_set(TMR3, 0);
		pulse_count = 0;
	}
}

void init_pulse_tmr(void) {
	crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_TMR5_PERIPH_CLOCK, TRUE);

	tmr_output_config_type tmr_param;
	tmr_output_default_para_init(&tmr_param);
	tmr_param.oc_idle_state = TRUE;
	tmr_param.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
	tmr_param.oc_output_state = TRUE;
	tmr_param.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
	tmr_param.occ_idle_state = TRUE;
	tmr_param.occ_output_state = TRUE;
	tmr_param.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;

	// Master

	uint32_t half_period = 55;
	uint32_t fill_width = 5;

	// PE3 Slave->Master
	tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
	tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &tmr_param);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, fill_width);
	tmr_base_init(TMR2, half_period /*115 / 2*/, 0);
	tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);

	tmr_sub_mode_select(TMR2, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR2, TMR_SUB_INPUT_SEL_IS2);	// TMR3 start TMR2 == IS2

	tmr_primary_mode_select(TMR2, TMR_PRIMARY_SEL_OVERFLOW);
	tmr_sub_sync_mode_set(TMR2, TRUE);

	tmr_one_cycle_mode_enable(TMR2, TRUE);
	tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_4, TMR_OUTPUT_CONTROL_PWM_MODE_A);

	// A0 Slave
	tmr_clock_source_div_set(TMR5, TMR_CLOCK_DIV1);
	tmr_output_channel_config(TMR5, TMR_SELECT_CHANNEL_4, &tmr_param);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_4, fill_width);
	tmr_base_init(TMR5, half_period /*56*/, 0);
	tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);

	tmr_sub_mode_select(TMR5, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR5, TMR_SUB_INPUT_SEL_IS0);	// TMR2 start TMR5 == IS0

//	tmr_primary_mode_select(TMR5, TMR_PRIMARY_SEL_OVERFLOW);
//	tmr_sub_sync_mode_set(TMR5, TRUE);

	tmr_one_cycle_mode_enable(TMR5, TRUE);
	tmr_output_channel_mode_select(TMR5, TMR_SELECT_CHANNEL_4, TMR_OUTPUT_CONTROL_PWM_MODE_A);

	tmr_output_enable(TMR2, TRUE);	// Pulse Pin Positive
	tmr_counter_enable(TMR2, TRUE);

	tmr_output_enable(TMR5, TRUE);	// Pulse Pin Negative
	tmr_counter_enable(TMR5, TRUE);

	tmr_output_enable(TMR3, TRUE);
	tmr_counter_enable(TMR3, TRUE);
}
