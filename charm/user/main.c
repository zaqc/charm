/**
  **************************************************************************
  * @file     main.c
  * @version  v2.1.0
  * @date     2022-08-16
  * @brief    main program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "at32f435_437_board.h"
#include "at32f435_437_clock.h"

/** @addtogroup AT32F437_periph_template
  * @{
  */

/** @addtogroup 437_LED_toggle LED_toggle
  * @{
  */

#define DELAY                            100
#define FAST                             1
#define SLOW                             4

uint8_t g_speed = FAST;

void button_exint_init(void);
void button_isr(void);

/**
  * @brief  configure button exint
  * @param  none
  * @retval none
  */
void button_exint_init(void)
{
//  exint_init_type exint_init_struct;
//
//  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
//  scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOA, SCFG_PINS_SOURCE0);
//
//  exint_default_para_init(&exint_init_struct);
//  exint_init_struct.line_enable = TRUE;
//  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
//  exint_init_struct.line_select = EXINT_LINE_0;
//  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
//  exint_init(&exint_init_struct);
//
//  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
//  nvic_irq_enable(EXINT0_IRQn, 0, 0);
}

/**
  * @brief  button handler function
  * @param  none
  * @retval none
  */
void button_isr(void)
{
  /* delay 5ms */
  delay_ms(5);

  /* clear interrupt pending bit */
  exint_flag_clear(EXINT_LINE_0);

  /* check input pin state */
  if(SET == gpio_input_data_bit_read(USER_BUTTON_PORT, USER_BUTTON_PIN))
  {
    if(g_speed == SLOW)
      g_speed = FAST;
    else
      g_speed = SLOW;
  }
}

/**
  * @brief  exint0 interrupt handler
  * @param  none
  * @retval none
  */
//void EXINT0_IRQHandler(void)
//{
//  button_isr();
//}

int16_t spi_tx_buf[4096];	// 4096

void test_spi(void) {
	gpio_init_type gpio_param;
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	gpio_default_para_init(&gpio_param);
	/* spi2 cs pin */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_7);
	/* spi2 sck pin */
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_6);
	/* spi2 miso pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_2;
	gpio_init(GPIOC, &gpio_param);
	gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE2, GPIO_MUX_5);
	/* spi2 mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_6);

	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL1);
	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_SPI2_TX);
	dma_init_type dma_param;
	dma_default_para_init(&dma_param);
	dma_param.direction = DMA_DIR_MEMORY_TO_PERIPHERAL; // Transfer memory buffer to SPI2_TX
	dma_param.buffer_size = 1024;
	dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;	// 16 bit
	dma_param.memory_inc_enable = TRUE;
	dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_param.peripheral_inc_enable = FALSE;
	dma_param.priority = DMA_PRIORITY_HIGH;
	dma_param.loop_mode_enable = FALSE;
	dma_param.memory_base_addr = (uint32_t)spi_tx_buf;
	dma_param.peripheral_base_addr = (uint32_t)&(SPI2->dt);
	dma_init(DMA1_CHANNEL1, &dma_param);

	crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
	spi_param.master_slave_mode = SPI_MODE_MASTER;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	spi_param.clock_polarity = SPI_CLOCK_POLARITY_LOW;
	spi_param.clock_phase = SPI_CLOCK_PHASE_2EDGE;
	spi_param.cs_mode_selection = SPI_CS_HARDWARE_MODE;
	spi_init(SPI2, &spi_param);
	spi_i2s_dma_transmitter_enable(SPI2, TRUE);
	spi_ti_mode_enable(SPI2, TRUE);
	spi_enable(SPI2, TRUE);

	dma_channel_enable(DMA1_CHANNEL1, TRUE);

	while(dma_flag_get(DMA1_HDT1_FLAG) == RESET);

	delay_us(10);
}

void test_adc(void) {
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	// ADC Pin's
	gpio_default_para_init(&gpio_param);
	gpio_param.gpio_pins = GPIO_PINS_ALL;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN;
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

	crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
	tmr_base_init(TMR4, 9, 0);
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
	tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_1, 7);	// set pulse width

	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL2);
	dma_init_type dma_param;
	dma_param.buffer_size = 1024;
	dma_param.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
	dma_param.memory_base_addr = (uint32_t) spi_tx_buf;
	dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
	dma_param.memory_inc_enable = TRUE;
	dma_param.peripheral_base_addr = (uint32_t)&GPIOF->idt;
	dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_param.peripheral_inc_enable = FALSE;
	dma_param.priority = DMA_PRIORITY_VERY_HIGH;
	dma_param.loop_mode_enable = FALSE;
	dma_init(DMA1_CHANNEL2, &dma_param);

	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL2, DMAMUX_DMAREQ_ID_TMR4_CH1);
	dma_channel_enable(DMA1_CHANNEL2, TRUE);

	/* enable tmr1 overflow dma request */
	tmr_dma_request_enable(TMR4, TMR_C1_DMA_REQUEST, TRUE);

	/* tmr1 output enable */
	tmr_output_enable(TMR4, TRUE);
	/* enable tmr1 */
	tmr_counter_enable(TMR4, TRUE);

	while(dma_flag_get(DMA1_FDT2_FLAG) == RESET);

	//tmr_counter_enable(TMR4, FALSE);

//	for (int i = 0; i < 1024; i++)
//		spi_tx_buf[i] =  512 - (0x3FF & spi_tx_buf[i]);

	//delay_us(10);
}

uint16_t dac_tx_buf[1024];	// 4096

void test_dac(void) {
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	/* spi2 cs pin */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_7);
	/* spi2 sck pin */
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_6);
	/* spi2 mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_6);

	/* DAC GPIO CS */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pins = GPIO_PINS_3;
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOD, &gpio_param);
	//gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE3, GPIO_MUX_0);

	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL3);
	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL3, DMAMUX_DMAREQ_ID_SPI2_TX);
	dma_init_type dma_param;
	dma_default_para_init(&dma_param);
	dma_param.direction = DMA_DIR_MEMORY_TO_PERIPHERAL; // Transfer memory buffer to SPI2_TX
	dma_param.buffer_size = 256;
	dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;	// 16 bit
	dma_param.memory_inc_enable = TRUE;
	dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_param.peripheral_inc_enable = FALSE;
	dma_param.priority = DMA_PRIORITY_MEDIUM;
	dma_param.loop_mode_enable = FALSE;
	dma_param.memory_base_addr = (uint32_t)dac_tx_buf;
	dma_param.peripheral_base_addr = (uint32_t)&(SPI2->dt);
	dma_init(DMA1_CHANNEL3, &dma_param);

	crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
	spi_param.master_slave_mode = SPI_MODE_MASTER;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_16;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	spi_param.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_param.clock_phase = SPI_CLOCK_PHASE_1EDGE;
	spi_param.cs_mode_selection = SPI_CS_HARDWARE_MODE;
	spi_init(SPI2, &spi_param);
	spi_ti_mode_enable(SPI2, TRUE);
	spi_hardware_cs_output_enable(SPI2, TRUE);
	spi_i2s_dma_transmitter_enable(SPI2, TRUE);
	spi_enable(SPI2, TRUE);

	GPIOD->clr = GPIO_PINS_3;
	dma_channel_enable(DMA1_CHANNEL3, TRUE);

	while(dma_flag_get(DMA1_FDT3_FLAG) == RESET);
	GPIOD->scr = GPIO_PINS_3;

	delay_us(10000);

}

void test_dac_swcs(void) {
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	/* spi2 cs pin */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_9;
	//gpio_init(GPIOB, &gpio_param);
	//gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_5);

	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_NONE;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_7);

	/* spi2 sck pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE1, GPIO_MUX_6);
	/* spi2 mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOD, &gpio_param);
	gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE4, GPIO_MUX_6);

	/* DAC GPIO CS */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pins = GPIO_PINS_3;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_init(GPIOD, &gpio_param);
	//gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE3, GPIO_MUX_0);

//	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
//	dma_reset(DMA1_CHANNEL3);
//	dmamux_enable(DMA1, TRUE);
//	dmamux_init(DMA1MUX_CHANNEL3, DMAMUX_DMAREQ_ID_SPI2_TX);
//	dma_init_type dma_param;
//	dma_default_para_init(&dma_param);
//	dma_param.direction = DMA_DIR_MEMORY_TO_PERIPHERAL; // Transfer memory buffer to SPI2_TX
//	dma_param.buffer_size = 256;
//	dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;	// 16 bit
//	dma_param.memory_inc_enable = TRUE;
//	dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
//	dma_param.peripheral_inc_enable = FALSE;
//	dma_param.priority = DMA_PRIORITY_MEDIUM;
//	dma_param.loop_mode_enable = FALSE;
//	dma_param.memory_base_addr = (uint32_t)dac_tx_buf;
//	dma_param.peripheral_base_addr = (uint32_t)&(SPI2->dt);
//	dma_init(DMA1_CHANNEL3, &dma_param);

	crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
	spi_param.master_slave_mode = SPI_MODE_MASTER;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	spi_param.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_param.clock_phase = SPI_CLOCK_PHASE_1EDGE;
	spi_param.cs_mode_selection = SPI_CS_HARDWARE_MODE;
	spi_init(SPI2, &spi_param);
//	spi_ti_mode_enable(SPI2, TRUE);
	spi_hardware_cs_output_enable(SPI2, TRUE);
//	spi_i2s_dma_transmitter_enable(SPI2, TRUE);

	//spi_software_cs_internal_level_set(SPI2, SPI_SWCS_INTERNAL_LEVEL_LOW);

	spi_enable(SPI2, TRUE);

	GPIOD->clr = GPIO_PINS_3;
//	dma_channel_enable(DMA1_CHANNEL3, TRUE);

	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOD, &gpio_param);

	for(int i = 0; i < 1024; i++) {

//		gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
//		gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
//		gpio_param.gpio_mode = GPIO_MODE_MUX;
//		gpio_param.gpio_pull = GPIO_PULL_NONE;
//		gpio_param.gpio_pins = GPIO_PINS_0;
//		gpio_init(GPIOD, &gpio_param);
//		gpio_pin_mux_config(GPIOD, GPIO_PINS_SOURCE0, GPIO_MUX_7);

		//delay_us(1);
	    while(spi_i2s_flag_get(SPI2, SPI_I2S_TDBE_FLAG) == RESET);
		spi_i2s_data_transmit(SPI2, dac_tx_buf[i]);
	}

	//GPIOD->scr = GPIO_PINS_3;

	//delay_us(10);

}

int nn = 0;
void TMR1_OVF_TMR10_IRQHandler(void) {
	tmr_flag_clear(TMR1, TMR_OVF_FLAG);


	GPIOD->scr = GPIO_PINS_0;
	//delay_us(1);

	GPIOD->scr = GPIO_PINS_3;
	//delay_us(1);

	uint16_t val = dac_tx_buf[nn];

	nn += nn > 1 ? 6 : 1;

	if(nn >= 1024) nn = 0;

	GPIOD->clr = GPIO_PINS_3;
	//delay_us(1);

    GPIOD->clr = GPIO_PINS_0;
    //delay_us(1);


    spi_i2s_data_transmit(SPI2, val);
}

void test_timer(void) {
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
	tmr_base_init(TMR1, 17, 15);	// 288MHz / 16 (15) = 18 (cntr 18 - 1 = 17)
	tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
	tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 0);

	tmr_counter_enable(TMR1, TRUE);

	//while(1);
}

uint16_t send_buf[4096];

void test_spi_ti_send(){	// Data Stream
	gpio_init_type gpio_param;
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	gpio_default_para_init(&gpio_param);
	/* spi1 cs pin */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE4, GPIO_MUX_5);
	/* spi1 sck pin */
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_5;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE5, GPIO_MUX_5);
	/* spi1 mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_UP;
	gpio_param.gpio_pins = GPIO_PINS_7;
	gpio_init(GPIOA, &gpio_param);
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_5);

	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL4);
	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL4, DMAMUX_DMAREQ_ID_SPI1_TX);
	dma_init_type dma_param;
	dma_default_para_init(&dma_param);
	dma_param.direction = DMA_DIR_MEMORY_TO_PERIPHERAL; // Transfer memory buffer to SPI1_TX
	dma_param.buffer_size = 1024;
	dma_param.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;	// 16 bit
	dma_param.memory_inc_enable = TRUE;
	dma_param.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_param.peripheral_inc_enable = FALSE;
	dma_param.priority = DMA_PRIORITY_HIGH;
	dma_param.loop_mode_enable = FALSE;
	dma_param.memory_base_addr = (uint32_t)send_buf;
	dma_param.peripheral_base_addr = (uint32_t)&(SPI1->dt);
	dma_init(DMA1_CHANNEL4, &dma_param);

	crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_HALF_DUPLEX_TX;
	spi_param.master_slave_mode = SPI_MODE_MASTER;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	//spi_param.clock_polarity = SPI_CLOCK_POLARITY_LOW;
	//spi_param.clock_phase = SPI_CLOCK_PHASE_2EDGE;
	spi_param.cs_mode_selection = SPI_CS_HARDWARE_MODE;
	spi_init(SPI1, &spi_param);
	spi_i2s_dma_transmitter_enable(SPI1, TRUE);
	spi_ti_mode_enable(SPI1, TRUE);
	spi_enable(SPI1, TRUE);

	dma_channel_enable(DMA1_CHANNEL4, TRUE);

	while(dma_flag_get(DMA1_FDT4_FLAG) == RESET);

	//delay_us(10);
}

volatile int irq_cntr = 0;
void EXINT15_10_IRQHandler(void) {
	if (exint_flag_get(EXINT_LINE_13) != RESET) {
		irq_cntr = 1;
		exint_flag_clear(EXINT_LINE_13);
	}
}

void get_gpio_sync(void) {
	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	gpio_init_type gpio_param;
	gpio_default_para_init(&gpio_param);
	/* sync pin PE13 */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_mode = GPIO_MODE_INPUT;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_13;
	gpio_init(GPIOE, &gpio_param);

	//EXINT0_IRQn
	crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);
	scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOE, SCFG_PINS_SOURCE13);
	exint_init_type exint_param;
	exint_default_para_init(&exint_param);
	exint_param.line_enable = TRUE;
	exint_param.line_mode = EXINT_LINE_INTERRUPUT;
	exint_param.line_polarity = EXINT_TRIGGER_RISING_EDGE;
	exint_param.line_select = EXINT_LINE_13;
	exint_init(&exint_param);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT15_10_IRQn, 1, 0);
}


uint16_t r_buf[1024];
volatile uint32_t r_ptr = 0;
void SPI4_IRQHandler(void) {
	if (spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) != RESET) {
		uint16_t val = spi_i2s_data_receive(SPI4);
		r_buf[r_ptr++] = val;
		if(r_ptr >= 1024) {
			r_ptr = 0;
		}
	}
}

static void rcv_cmd_spi4(void) {
	gpio_init_type gpio_param;
	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	/* master cs pin */
	gpio_param.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_mode = GPIO_MODE_MUX;
	gpio_param.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_param.gpio_pins = GPIO_PINS_11;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE11, GPIO_MUX_5);
	/* slave sck pin */
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_12;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE12, GPIO_MUX_5);
	/* master mosi pin */
	gpio_param.gpio_pull = GPIO_PULL_DOWN;
	gpio_param.gpio_pins = GPIO_PINS_14;
	gpio_init(GPIOE, &gpio_param);
	gpio_pin_mux_config(GPIOE, GPIO_PINS_SOURCE14, GPIO_MUX_5);

	crm_periph_clock_enable(CRM_SPI4_PERIPH_CLOCK, TRUE);
	spi_init_type spi_param;
	spi_default_para_init(&spi_param);
	spi_param.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
	spi_param.master_slave_mode = SPI_MODE_SLAVE;
	spi_param.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_param.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_param.frame_bit_num = SPI_FRAME_16BIT;
	spi_param.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_param.clock_phase = SPI_CLOCK_PHASE_1EDGE;
	spi_param.cs_mode_selection = SPI_CS_SOFTWARE_MODE;

	spi_init(SPI4, &spi_param);

	nvic_irq_enable(SPI4_IRQn, 0, 0);
	spi_i2s_interrupt_enable(SPI4, SPI_I2S_RDBF_INT, TRUE);

	spi_ti_mode_enable(SPI4, TRUE);

	spi_enable(SPI4, TRUE);
}

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void) {
	system_clock_config();

	at32_board_init();

	button_exint_init();

	rcv_cmd_spi4();

	get_gpio_sync();

	dac_tx_buf[0] = 0x1100;
	dac_tx_buf[1] = 0x5100;
	for(int i = 2; i < 1024; i++) {
		dac_tx_buf[i] = i < 128 ? 0x1000 + i * 8 : 0x5000 + (i - 512) * 8;
	}

//	for(int i = 0; i < 1024; i += 2) {
//		dac_tx_buf[i] = 0x1000 + 1500;
//		dac_tx_buf[i + 1] = 0x5000 + 1500;
//	}

	test_dac_swcs();	// init VRC (DAC SPI)

	test_timer();		// Send VRC from Timer Interrupt
	//tmr_counter_enable(TMR1, TRUE);
	//while(1);

	uint16_t kk = 0;
	while(1) {
		nn = 100;
		tmr_counter_enable(TMR1, TRUE);
		test_adc();
		tmr_counter_enable(TMR1, FALSE);

		for(int i = 0; i < 1024; i++) {
			send_buf[i] = spi_tx_buf[i];
			//send_buf[i] = i + kk;
		}
		kk++;
		while(!irq_cntr);
		irq_cntr = 0;
		test_spi_ti_send();
	}

	for (int i = 0; i < 128; i++)
		spi_tx_buf[i] = 0x55AA;

	while (1) {
		test_adc();
	}

	while (1) {
	  test_spi();
//    at32_led_toggle(LED2);
//    delay_ms(g_speed * DELAY);
//    at32_led_toggle(LED3);
//    delay_ms(g_speed * DELAY);
//    at32_led_toggle(LED4);
//    delay_ms(g_speed * DELAY);
	}
}

/**
  * @}
  */

/**
  * @}
  */
