/*
 * sync.c
 *
 *  Created on: 2024 Jul 9
 *      Author: zaqc
 */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

#include "global_param.h"
#include "pulse.h"
#include "adc.h"

volatile uint8_t adc_dma_busy = 0;
volatile int32_t sync_overrun = 0;

/**
 * @brief	IRQ Handler
 * 			IRQ from PORT_E_13
 * 			triggered by falling edge of LINE_13
 * 			reset all timers and ADC_DMA buffer
 * 			for start all processes on rising edge of same signal
 */
void EXINT15_10_IRQHandler(void) {
	if (exint_flag_get(EXINT_LINE_13) == SET) {
		exint_flag_clear(EXINT_LINE_13);

		__disable_irq();
		uint8_t busy = adc_dma_busy;
		if(0 != adc_dma_busy)
			sync_overrun++;
		__enable_irq();

		if (0 == busy) {
			GPIOE->clr = GPIO_PINS_1;	// Clear Sync Pin

			TMR3->cval = 0;

			TMR2->cval = 0;
			TMR5->cval = 0;

			DMA2_CHANNEL4->ctrl_bit.chen = 0;
			DMA2_CHANNEL4->ctrl = 0;
			DMA2->clr |= 0xF000;

			TMR4->ctrl1_bit.tmren = 0;	// TMR4 Enable Set FALSE
			TMR4->cval = 0;

			DMA2_CHANNEL4->ctrl = (DMA_PRIORITY_VERY_HIGH << 12)
					| (DMA_MEMORY_DATA_WIDTH_HALFWORD << 10)
					| DMA_PERIPHERAL_DATA_WIDTH_HALFWORD << 8 | (1 << 7);

			DMA2_CHANNEL4->dtcnt_bit.cnt = 4096;
			DMA2_CHANNEL4->paddr = (uint32_t) &GPIOF->idt;
			DMA2_CHANNEL4->maddr = (uint32_t) adc_put_ptr;

			if(adc_put_ptr != adc_buf)
				adc_put_ptr = adc_buf;
			else
				adc_put_ptr = &adc_buf[ADC_SLOT_SIZE];

			DMA2_CHANNEL4->ctrl |= DMA_FDT_INT;

			DMA2->muxsel_bit.tblsel = 1;
			DMA2MUX_CHANNEL4->muxctrl_bit.reqsel =
					DMAMUX_DMAREQ_ID_TMR4_OVERFLOW;

			DMA2_CHANNEL4->ctrl_bit.chen = TRUE;

			TMR4->iden |= TMR_OVERFLOW_DMA_REQUEST;

			__disable_irq();
			adc_dma_busy = 1;
			__enable_irq();

			GPIOE->scr = GPIO_PINS_1;	// Set Sync Pin (run all timers and DMA)
		}
	}
}

/**
 * @brief	Initialize PORT_E PIN_13 as input (LVDS INT Signal)
 * 			And for generate IRQ EXINT_LINE_13 by falling edge
 * 			This is use for reset timers and DMA in IRQ handler
 * 			Initialize PORT_E PIN_1 as output for send START signal
 * 			This pin is connected with TMR3_EXT(Pulse), TMR4_EXT(ADC_DMA), TMR8_EXT(DAC)
 */
void init_sync_pin(void) {
	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	/* configure pe1 for output mode */
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOE, &gpio_param);

	/* configure pe13 for input mode */
	gpio_param.gpio_pins = GPIO_PINS_13;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOE, &gpio_param);

	/* pe13 as EXINT_LINE_13 */
	crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
	scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOE, SCFG_PINS_SOURCE13);

	exint_init_type exint_param;
	exint_default_para_init(&exint_param);
	exint_param.line_enable = TRUE;
	exint_param.line_mode = EXINT_LINE_INTERRUPUT;
	exint_param.line_select = EXINT_LINE_13;
	exint_param.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
	exint_init(&exint_param);

	/* EXINT line1 interrupt nvic init */
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT15_10_IRQn, 1, 0);
}

/**
 * @brief	TMR1_OVERFLOW IRQ Handler
 * 			triggers send_sync_flag
 * 			event frequency is 1 kHz
 */
volatile uint8_t send_sync_flag = 0;
void TMR1_OVF_TMR10_IRQHandler(void) {
	tmr_flag_clear(TMR1, TMR_OVF_FLAG);

	GPIOB->clr = GPIO_PINS_9;
	delay_us(1);
	GPIOB->scr = GPIO_PINS_9;
}

/**
 *	@brief	This procedure initialize PORT_B PIN_9 and toggles it
 *			This is using only for testing purpose
 *			The signal on this PIN is toggled by TMR1_OVERFLOW event
 *			This signal falling down around 1 uSec
 *			On falling edge of this signal, an IRQ generated for clear all timers and adc_dma
 *			On rising edge all timers and ADC DMA are start
 */
void sync_test(void) {
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

	gpio_init_type gpio_param = { 0 };
	gpio_param.gpio_pins = GPIO_PINS_9;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_param);

	GPIOB->scr = GPIO_PINS_9;

	/* configure TMR1 for generate 1 KHz event */
	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

	tmr_base_init(TMR1, 7999, 35);// 288MHz / 36 (35) = 8 MHz / 8000 = 1 kHz (8000 - 1 = 7999)
	tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
	tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
	tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);

	tmr_counter_enable(TMR1, TRUE);

	while (1) {
//		__disable_irq();
//		uint8_t flag = send_sync_flag;
//		send_sync_flag = 0;
//		__enable_irq();
//		if (flag) {
//			__disable_irq();
//			one_time_int = 0;
//			__enable_irq();
//			GPIOB->clr = GPIO_PINS_9;
//			delay_us(1);
//			GPIOB->scr = GPIO_PINS_9;
//		}
//
		__asm("NOP");
	}
}
