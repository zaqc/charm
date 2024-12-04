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

volatile uint8_t adc_dma_rdy = 0;
void DMA2_Channel4_IRQHandler(void) {
	if (dma_flag_get(DMA2_FDT4_FLAG) != RESET) {
		dma_flag_clear(DMA2_FDT4_FLAG);
		adc_dma_rdy = 1;

		//tmr_counter_enable(TMR4, FALSE);
	}
}

dma_init_type dma_init_struct;

void init_adc_tmr(void) {
	gpio_init_type gpio_init_struct = { 0 };
	dmamux_sync_init_type dmamux_sync_init_struct;

	/* enable dma2/gpioa clock */
	crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);

	/* Timer4 Channel1 output pin Configuration */
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_param.gpio_pins = GPIO_PINS_6;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init(GPIOB, &gpio_param);
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_2); // TMR4_CH1 as ADC_CLK

	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE0, GPIO_MUX_2);	// TMR4_EXT

	/* dma2 channel4 configuration */
	dma_reset(DMA2_CHANNEL4);
	dma_init_struct.buffer_size = 4096;
	dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
	dma_init_struct.memory_base_addr = (uint32_t) adc_buf;
	dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
	dma_init_struct.memory_inc_enable = TRUE;
	dma_init_struct.peripheral_base_addr = (uint32_t) &GPIOF->idt;
	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_init_struct.peripheral_inc_enable = FALSE;
	dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
	dma_init_struct.loop_mode_enable = FALSE;
	dma_init(DMA2_CHANNEL4, &dma_init_struct);

//	dmamux_sync_default_para_init(&dmamux_sync_init_struct);
//	dmamux_sync_init_struct.sync_request_number = 4096;
//	dmamux_sync_init_struct.sync_signal_sel = DMAMUX_SYNC_ID_EXINT9;
//	dmamux_sync_init_struct.sync_polarity = DMAMUX_SYNC_POLARITY_RISING;
//	dmamux_sync_init_struct.sync_event_enable = TRUE;
//	dmamux_sync_init_struct.sync_enable = TRUE;
//	dmamux_sync_config(DMA2MUX_CHANNEL4, &dmamux_sync_init_struct);

	/* enable transfer full data interrupt */
	dma_interrupt_enable(DMA2_CHANNEL4, DMA_FDT_INT, TRUE);

	/* dma2 channel4 interrupt nvic init */
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(DMA2_Channel4_IRQn, 1, 0);

	/* dmamux function enable */
	dmamux_enable(DMA2, TRUE);
	dmamux_init(DMA2MUX_CHANNEL4, DMAMUX_DMAREQ_ID_TMR4_OVERFLOW);

	/* enable dma channe4 */
	dma_channel_enable(DMA2_CHANNEL4, TRUE);

	/* tmr1 configuration */
	tmr_base_init(TMR4, 10, 0);
	tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);

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

	tmr_output_enable(TMR4, TRUE);	// TMR4_CH1 output ADC_CLK

	/* enable tmr1 overflow edam request */
	tmr_dma_request_enable(TMR4, TMR_OVERFLOW_DMA_REQUEST, TRUE);

	tmr_sub_mode_select(TMR4, TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR4, TMR_SUB_INPUT_SEL_EXTIN); // Sync by TMR4_EXT

	/* enable tmr1 */
	tmr_counter_enable(TMR4, TRUE);
}
//----------------------------------------------------------------------------

extern volatile uint32_t pulse_count;

volatile uint32_t adc_dma_rdy_cntr = 0;
void DMA1_Channel2_IRQHandler(void) {
	if (dma_flag_get(DMA1_FDT2_FLAG) != RESET) {
		dma_flag_clear(DMA1_FDT2_FLAG);
		adc_dma_rdy_cntr++;

		//tmr_counter_enable(TMR4, FALSE);
	}
}

void init_adc_dma(void) {
	crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	// ADC Pin's
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_pins = GPIO_PINS_ALL;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOF, &gpio_param);	// PORT_F as ADC_DATA

	/* Timer4 Channel1 output pin Configuration */
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	gpio_param.gpio_pins = GPIO_PINS_6;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_param);
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_2);// TMR4_CH1 as ADC_CLK

	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE0, GPIO_MUX_2);	// TMR4_EXT

	/* Exint Line 1 on Pin B9 */
	//gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_pins = GPIO_PINS_9;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_param);	// Input EXINT_LINE_9 as DMA Start

	crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
	scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOB, SCFG_PINS_SOURCE9);

	exint_init_type exint_param;
	exint_default_para_init(&exint_param);
	exint_param.line_enable = TRUE;
	exint_param.line_mode = EXINT_LINE_INTERRUPUT;
	exint_param.line_select = EXINT_LINE_9;
	exint_param.line_polarity = EXINT_TRIGGER_RISING_EDGE;
	exint_init(&exint_param);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT9_5_IRQn, 1, 0);

	// Init DMA
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

//	dmamux_sync_init_type dmamux_sync;
//	dmamux_sync_default_para_init(&dmamux_sync);
//	dmamux_sync.sync_request_number = 1024;
//	dmamux_sync.sync_signal_sel = DMAMUX_SYNC_ID_EXINT9;
//	dmamux_sync.sync_polarity = DMAMUX_SYNC_POLARITY_RISING;
//	dmamux_sync.sync_event_enable = TRUE;
//	dmamux_sync.sync_enable = TRUE;
//	dmamux_sync_config(DMA1MUX_CHANNEL2, &dmamux_sync);

	dma_interrupt_enable(DMA1_CHANNEL2, DMA_FDT_INT, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(DMA1_Channel2_IRQn, 1, 0);

	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_TMR4_OVERFLOW);

	dma_channel_enable(DMA1_CHANNEL2, TRUE);

	/* Init TMR4 */
	crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
	tmr_base_init(TMR4, 10, 0);
	tmr_cnt_dir_set(TMR4, TMR_COUNT_UP);
	tmr_one_cycle_mode_enable(TMR4, FALSE);

	tmr_sub_mode_select(TMR4, TMR_SUB_RESET_MODE); //TMR_SUB_TRIGGER_MODE);
	tmr_trigger_input_select(TMR4, TMR_SUB_INPUT_SEL_EXTIN); // Sync by TMR4_EXT

	tmr_output_enable(TMR4, TRUE);	// TMR4_CH1 output ADC_CLK

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

	tmr_dma_request_enable(TMR4, TMR_OVERFLOW_DMA_REQUEST, TRUE);
	tmr_counter_enable(TMR4, TRUE);
}
//----------------------------------------------------------------------------

void reset_adc_dma(void) {
	if (adc_buf_ptr != adc_buf)
		adc_buf_ptr = adc_buf;
	else
		adc_buf_ptr = &adc_buf[ADC_BUF_SIZE / 2];

	//dma_reset(DMA2_CHANNEL4);
	DMA2_CHANNEL4->ctrl_bit.chen = 0;
	DMA2_CHANNEL4->ctrl = 0;
	//DMA2_CHANNEL4->dtcnt = 0;
	//DMA2_CHANNEL4->paddr = 0;
	//DMA2_CHANNEL4->maddr = 0;
	DMA2->clr |= 0xF000;

//	dma_init_struct.buffer_size = 4096;
//	dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
//	dma_init_struct.memory_base_addr = (uint32_t) adc_buf;
//	dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
//	dma_init_struct.memory_inc_enable = TRUE;
//	dma_init_struct.peripheral_base_addr = (uint32_t) &GPIOF->idt;
//	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
//	dma_init_struct.peripheral_inc_enable = FALSE;
//	dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
//	dma_init_struct.loop_mode_enable = FALSE;
//	dma_init(DMA2_CHANNEL4, &dma_init_struct);

//	DMA2_CHANNEL4->ctrl &= 0xbfef;
//	DMA2_CHANNEL4->ctrl |= dma_init_struct->direction;	// DMA_DIR_PERIPHERAL_TO_MEMORY == 0

//	DMA2_CHANNEL4->ctrl_bit.chpl = DMA_PRIORITY_VERY_HIGH;//				dma_init_struct->priority;
//	DMA2_CHANNEL4->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_HALFWORD;//		dma_init_struct->memory_data_width;
//	DMA2_CHANNEL4->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;//	dma_init_struct->peripheral_data_width;
//	DMA2_CHANNEL4->ctrl_bit.mincm = TRUE;//									dma_init_struct->memory_inc_enable;
//	DMA2_CHANNEL4->ctrl_bit.pincm = FALSE;//								dma_init_struct->peripheral_inc_enable;
//	DMA2_CHANNEL4->ctrl_bit.lm = FALSE;	// 									dma_init_struct->loop_mode_enable;

	DMA2_CHANNEL4->ctrl = (DMA_PRIORITY_VERY_HIGH << 12) | (DMA_MEMORY_DATA_WIDTH_HALFWORD << 10) |
			DMA_PERIPHERAL_DATA_WIDTH_HALFWORD << 8 | (1 << 7);

	DMA2_CHANNEL4->dtcnt_bit.cnt = 4096;// 									dma_init_struct->buffer_size;
	DMA2_CHANNEL4->paddr = (uint32_t) &GPIOF->idt;//						dma_init_struct->peripheral_base_addr;
	DMA2_CHANNEL4->maddr = (uint32_t) adc_buf;//							dma_init_struct->memory_base_addr;

	//dma_interrupt_enable(DMA2_CHANNEL4, DMA_FDT_INT, TRUE);
	DMA2_CHANNEL4->ctrl |= DMA_FDT_INT;

	/* dmamux function enable */
	//dmamux_enable(DMA2, TRUE);
	DMA2->muxsel_bit.tblsel = 1;
	//dmamux_init(DMA2MUX_CHANNEL4, DMAMUX_DMAREQ_ID_TMR4_OVERFLOW);
	DMA2MUX_CHANNEL4->muxctrl_bit.reqsel = DMAMUX_DMAREQ_ID_TMR4_OVERFLOW;

//	/* enable dma channe4 */
//	dma_channel_enable(DMA2_CHANNEL4, TRUE);
//
//	/* enable tmr1 overflow edam request */
//	tmr_dma_request_enable(TMR4, TMR_OVERFLOW_DMA_REQUEST, TRUE);
}
//----------------------------------------------------------------------------
