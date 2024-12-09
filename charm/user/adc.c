/*
 * get_adc_data.c
 *
 *  Created on: 2024 Jul 5
 *      Author: zaqc
 */
#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#include "global_param.h"
#include "adc.h"

uint16_t adc_buf[ADC_SLOT_SIZE * 2];
uint16_t *adc_put_ptr = adc_buf;
uint16_t *adc_get_ptr = adc_buf;

volatile uint8_t adc_dma_done_cntr = 0;

void DMA2_Channel4_IRQHandler(void) {
	if (dma_flag_get(DMA2_FDT4_FLAG) != RESET) {
		dma_flag_clear(DMA2_FDT4_FLAG);
		++adc_dma_done_cntr;
		adc_dma_busy = 0;
	}
}

void init_adc_tmr4_dma2ch4(void) {
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
	dma_init_type dma_init_struct;
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

void reset_adc_dma(void) {
//	if (adc_buf_ptr != adc_buf)
//		adc_buf_ptr = adc_buf;
//	else
//		adc_buf_ptr = &adc_buf[ADC_BUF_SIZE / 2];

	dma_reset(DMA2_CHANNEL4);
	//DMA2_CHANNEL4->ctrl_bit.chen = 0;
	//DMA2_CHANNEL4->ctrl = 0;
	//DMA2_CHANNEL4->dtcnt = 0;
	//DMA2_CHANNEL4->paddr = 0;
	//DMA2_CHANNEL4->maddr = 0;
	//DMA2->clr |= 0xF000;

	dma_init_type dma_init_struct;
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

//	DMA2_CHANNEL4->ctrl &= 0xbfef;
//	DMA2_CHANNEL4->ctrl |= dma_init_struct->direction;	// DMA_DIR_PERIPHERAL_TO_MEMORY == 0

//	DMA2_CHANNEL4->ctrl_bit.chpl = DMA_PRIORITY_VERY_HIGH;//				dma_init_struct->priority;
//	DMA2_CHANNEL4->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_HALFWORD;//		dma_init_struct->memory_data_width;
//	DMA2_CHANNEL4->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;//	dma_init_struct->peripheral_data_width;
//	DMA2_CHANNEL4->ctrl_bit.mincm = TRUE;//									dma_init_struct->memory_inc_enable;
//	DMA2_CHANNEL4->ctrl_bit.pincm = FALSE;//								dma_init_struct->peripheral_inc_enable;
//	DMA2_CHANNEL4->ctrl_bit.lm = FALSE;	// 									dma_init_struct->loop_mode_enable;

//	DMA2_CHANNEL4->ctrl = (DMA_PRIORITY_VERY_HIGH << 12) | (DMA_MEMORY_DATA_WIDTH_HALFWORD << 10) |
//			DMA_PERIPHERAL_DATA_WIDTH_HALFWORD << 8 | (1 << 7);
//
//	DMA2_CHANNEL4->dtcnt_bit.cnt = 4096;// 									dma_init_struct->buffer_size;
//	DMA2_CHANNEL4->paddr = (uint32_t) &GPIOF->idt;//						dma_init_struct->peripheral_base_addr;
//	DMA2_CHANNEL4->maddr = (uint32_t) adc_buf;//							dma_init_struct->memory_base_addr;

	dma_interrupt_enable(DMA2_CHANNEL4, DMA_FDT_INT, TRUE);
//	DMA2_CHANNEL4->ctrl |= DMA_FDT_INT;

	/* dmamux function enable */
	dmamux_enable(DMA2, TRUE);
//	DMA2->muxsel_bit.tblsel = 1;
	dmamux_init(DMA2MUX_CHANNEL4, DMAMUX_DMAREQ_ID_TMR4_OVERFLOW);
//	DMA2MUX_CHANNEL4->muxctrl_bit.reqsel = DMAMUX_DMAREQ_ID_TMR4_OVERFLOW;

	/* enable dma channe4 */
	dma_channel_enable(DMA2_CHANNEL4, TRUE);

	/* enable tmr1 overflow edam request */
	tmr_dma_request_enable(TMR4, TMR_OVERFLOW_DMA_REQUEST, TRUE);
}
//----------------------------------------------------------------------------
