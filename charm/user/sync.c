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

void reset_adc_dma(void);
extern volatile uint8_t adc_dma_rdy;
volatile uint8_t tmr_rst;
void send_sync(void) {
	__disable_irq();
	tmr_rst = 0;
	__enable_irq();
	GPIOE->clr = GPIO_PINS_1;
	while (1) {
		__disable_irq();
		uint8_t tmr_rst_flag = tmr_rst;
		__enable_irq();
		if (tmr_rst_flag)
			break;
		__asm("NOP");
	}
	GPIOE->scr = GPIO_PINS_1;
	//delay_us(200);
	//pulse_count = 0;
}

volatile uint8_t send_sync_flag = 0;
void TMR1_OVF_TMR10_IRQHandler(void) {
	tmr_flag_clear(TMR1, TMR_OVF_FLAG);
	send_sync_flag = 1;
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
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);// TMR1_CH1 PORT_A PIN_8

	// Master
	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

	tmr_base_init(TMR1, 7999, 35);// 288MHz / 36 (35) = 8 MHz / 8000 = 1 kHz (8000 - 1 = 7999)
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

	tmr_output_channel_mode_select(TMR1, TMR_SELECT_CHANNEL_1,
			TMR_OUTPUT_CONTROL_PWM_MODE_A);

	tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);	// Master Timer
	tmr_sub_sync_mode_set(TMR1, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);

	tmr_output_enable(TMR1, TRUE);
	tmr_counter_enable(TMR1, TRUE);
}

//extern void reset_adc_dma(void);
volatile uint32_t exint9_cntr = 0;

#define ADC_BUF_SIZE	(1024 * 16)
extern uint32_t adc_buf[ADC_BUF_SIZE];
extern uint32_t *adc_buf_ptr;

void EXINT15_10_IRQHandler(void) {
	if(EXINT->intsts & EXINT_LINE_13) {
		EXINT->intsts = EXINT_LINE_13;
		exint9_cntr++;

		TMR3->cval = 0;

		TMR2->cval = 0;
		TMR5->cval = 0;

		__disable_irq();
		uint8_t flag = adc_dma_rdy;
		adc_dma_rdy = 0;
		__enable_irq();
		if (flag) {
			if (adc_buf_ptr != adc_buf)
				adc_buf_ptr = adc_buf;
			else
				adc_buf_ptr = &adc_buf[4096];

			DMA2_CHANNEL4->ctrl_bit.chen = 0;
			DMA2_CHANNEL4->ctrl = 0;
			DMA2->clr |= 0xF000;

			DMA2_CHANNEL4->ctrl = (DMA_PRIORITY_VERY_HIGH << 12) | (DMA_MEMORY_DATA_WIDTH_HALFWORD << 10) |
					DMA_PERIPHERAL_DATA_WIDTH_HALFWORD << 8 | (1 << 7);

			DMA2_CHANNEL4->dtcnt_bit.cnt = 4096;
			DMA2_CHANNEL4->paddr = (uint32_t) &GPIOF->idt;
			DMA2_CHANNEL4->maddr = (uint32_t) adc_buf;

			DMA2_CHANNEL4->ctrl |= DMA_FDT_INT;

			DMA2->muxsel_bit.tblsel = 1;
			DMA2MUX_CHANNEL4->muxctrl_bit.reqsel = DMAMUX_DMAREQ_ID_TMR4_OVERFLOW;
		}

		TMR4->ctrl1_bit.tmren = 0;
		TMR4->cval = 0;

		DMA2_CHANNEL4->ctrl_bit.chen = TRUE;

		TMR4->iden |= TMR_OVERFLOW_DMA_REQUEST;

		tmr_rst = 1;
	}
}

void init_sync_pin(void) {
	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOE, &gpio_param);

	GPIOE->scr = GPIO_PINS_1;	// HI Level

	/* config pa1 for input mode */
	gpio_param.gpio_pins = GPIO_PINS_13;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOE, &gpio_param);

	scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOE, SCFG_PINS_SOURCE13);

	exint_init_type exint_param;
	exint_default_para_init(&exint_param);
	exint_param.line_enable = TRUE;
	exint_param.line_mode = EXINT_LINE_INTERRUPUT;
	exint_param.line_select = EXINT_LINE_13;
	exint_param.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
	exint_init(&exint_param);

	/* exint line1 interrupt nvic init */
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT15_10_IRQn, 1, 0);

}

void sync_test(void) {
	init_sync_pin();
	internal_sync_init();
	while (1) {
		__disable_irq();
		uint8_t flag = send_sync_flag;
		send_sync_flag = 0;
		__enable_irq();
		if (flag)
			send_sync();
		//else
			//delay_us(1);
		__asm("NOP");
	}
}
