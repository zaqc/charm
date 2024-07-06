/*
 * get_adc_data.c
 *
 *  Created on: 2024 Jul 5
 *      Author: zaqc
 */
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#include "global_param.h"

#define ADC_BUF_SIZE	(1024 * 16)

uint32_t adc_buf[ADC_BUF_SIZE];
uint32_t *adc_buf_ptr = adc_buf;
dma_init_type adc_dma_param = { 0 };

void EXINT1_IRQHandler(void) {
	if (exint_flag_get(EXINT_LINE_1) != RESET) {
		exint_flag_clear(EXINT_LINE_1);
	}
}

void init_adc_tmr(void) {
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	// ADC Pin's
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_pins = GPIO_PINS_ALL;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOF, &gpio_param);

	/* Timer4 Channel1 output pin Configuration */
	gpio_param.gpio_pins = GPIO_PINS_6;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_param);
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_2);

	/* Init TMR4 */
	crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
	tmr_base_init(TMR4, 10, 0);
	tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);

	/* channel 3 configuration in output mode */
	tmr_output_config_type tmr_param;
	tmr_output_default_para_init(&tmr_param);
	tmr_param.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
	tmr_param.oc_output_state = TRUE;
	tmr_param.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
	tmr_param.oc_idle_state = TRUE;
	tmr_param.occ_output_state = TRUE;
	tmr_param.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
	tmr_param.occ_idle_state = TRUE;
	/* channel 3 */
	tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_1, &tmr_param);
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_1, 5);	// set pulse width
}
//----------------------------------------------------------------------------

volatile uint32_t adc_dma_rdy_cntr = 0;
void DMA1_Channel2_IRQHandler(void) {
	dma_flag_clear(DMA1_FDT2_FLAG);
	adc_dma_rdy_cntr++;

	dmamux_sync_init_type  dmamux_param;
	dmamux_sync_default_para_init(&dmamux_param);
	dmamux_param.sync_request_number = 4096;
	dmamux_param.sync_signal_sel = DMAMUX_SYNC_ID_EXINT1;
	dmamux_param.sync_polarity = DMAMUX_SYNC_POLARITY_RISING;
	dmamux_param.sync_event_enable = TRUE;
	dmamux_param.sync_enable = TRUE;
	dmamux_sync_config(DMA1MUX_CHANNEL2, &dmamux_param);

	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_TMR4_CH1);
	dma_channel_enable(DMA1_CHANNEL2, TRUE);

}

void init_adc_dma(void) {
	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL2);
	adc_dma_param.buffer_size = 4096; //1024;
	adc_dma_param.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
	adc_dma_param.memory_base_addr = (uint32_t) adc_buf;
	adc_dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
	adc_dma_param.memory_inc_enable = TRUE;
	adc_dma_param.peripheral_base_addr = (uint32_t) &GPIOF->idt;
	adc_dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	adc_dma_param.peripheral_inc_enable = FALSE;
	adc_dma_param.priority = DMA_PRIORITY_VERY_HIGH;
	adc_dma_param.loop_mode_enable = FALSE;
	dma_init(DMA1_CHANNEL2, &adc_dma_param);


	/* enable tmr1 overflow dma request */
	tmr_dma_request_enable(TMR4, TMR_C1_DMA_REQUEST, TRUE);

	/* tmr1 output enable */
	tmr_output_enable(TMR4, TRUE);
	/* enable tmr1 */
	//----tmr_counter_enable(TMR4, TRUE);
	//while(dma_flag_get(DMA1_FDT2_FLAG) == RESET);

	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE0, GPIO_MUX_2);	// TMR4_EXT


	tmr_sub_mode_select(TMR4, TMR_SUB_RESET_MODE);
	tmr_trigger_input_select(TMR4, TMR_SUB_INPUT_SEL_EXTIN);

	dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(DMA1_Channel2_IRQn, 1, 0);

	/* Exint Line 1 on Pin B9 */
	//gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
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
	dmamux_sync_config(DMA1MUX_CHANNEL2, &dmamux_param);

	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_TMR4_CH1);
	dma_channel_enable(DMA1_CHANNEL2, TRUE);

	/* exint line1 interrupt nvic init */
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT1_IRQn, 1, 0);

	tmr_counter_enable(TMR4, TRUE);

}
//----------------------------------------------------------------------------

void reset_adc_dma(void) {
	if(adc_buf_ptr != adc_buf)
		adc_buf_ptr = adc_buf;
	else
		adc_buf_ptr = &adc_buf[ADC_BUF_SIZE / 2];

	adc_dma_param.buffer_size = g_AdcBufSize;
	adc_dma_param.memory_base_addr = (uint32_t) adc_buf_ptr;
	dma_init(DMA1_CHANNEL2, &adc_dma_param);
}
//----------------------------------------------------------------------------
