/*
 * pulse.c
 *
 *  Created on: 2024 Jul 7
 *      Author: zaqc
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

void init_pulse_tmr(void) {
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);	// TMR5_CH1
	gpio_init_type gpio_param;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_pins = GPIO_PINS_2;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE2, GPIO_MUX_2);

	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);	// TMR3_CH1
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE0, GPIO_MUX_1);

	crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
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

	uint32_t full_period = 114;

	// Master
	tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);
	tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_1, &tmr_param);
	tmr_base_init(TMR3, full_period, 0);
	tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
	tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_1, 1);

	tmr_primary_mode_select(TMR3, TMR_PRIMARY_SEL_OVERFLOW);
	tmr_sub_sync_mode_set(TMR3, TRUE);

	uint32_t half_period = 55;
	uint32_t fill_width = 5;

	// PE3 Slave->Master
	tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
	tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_1, &tmr_param);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_1, fill_width);
	tmr_base_init(TMR2, half_period /*56*/, 0);
	tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);

	tmr_sub_mode_select(TMR2, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR2, TMR_SUB_INPUT_SEL_IS2);	// TMR3 start TMR2 == IS2

	tmr_primary_mode_select(TMR2, TMR_PRIMARY_SEL_OVERFLOW);
	tmr_sub_sync_mode_set(TMR2, TRUE);

	tmr_one_cycle_mode_enable(TMR2, TRUE);
	tmr_output_channel_mode_select(TMR2, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);

	// A0 Slave
	tmr_clock_source_div_set(TMR5, TMR_CLOCK_DIV1);
	tmr_output_channel_config(TMR5, TMR_SELECT_CHANNEL_1, &tmr_param);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_1, fill_width);
	tmr_base_init(TMR5, half_period /*56*/, 0);
	tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);

	tmr_sub_mode_select(TMR5, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR5, TMR_SUB_INPUT_SEL_IS0);	// TMR2 start TMR5 == IS0

	tmr_one_cycle_mode_enable(TMR5, TRUE);
	tmr_output_channel_mode_select(TMR5, TMR_SELECT_CHANNEL_1, TMR_OUTPUT_CONTROL_PWM_MODE_A);

	//tmr_trigger_input_select(TMR20, TMR_SUB_INPUT_SEL_C1DF1);

	//tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);

	//TMR3->c1dt = 75; //20;
	//TMR8->c1dt = 25; //20;
	//TMR20->c2dt = 39; //30;

//	TMR20->ctrl2_bit.c1ios = 0;
//	TMR20->ctrl2_bit.c2ios = 0;
//
//	TMR20->cctrl_bit.c1p  = 0;
//	TMR20->cctrl_bit.c2p  = 0;

	//TMR20->cm1_output_bit.c2octrl = 2;

	tmr_output_enable(TMR2, TRUE);
	tmr_counter_enable(TMR2, TRUE);

	tmr_output_enable(TMR3, TRUE);
	tmr_counter_enable(TMR3, TRUE);

	tmr_output_enable(TMR5, TRUE);
	tmr_counter_enable(TMR5, TRUE);

//	pulse_pin = 0;
	//tmr_interrupt_enable(TMR2, TMR_OVF_INT, TRUE);
//
	//nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	//nvic_irq_enable(TMR2_GLOBAL_IRQn, 1, 0);

}
