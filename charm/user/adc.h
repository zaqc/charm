/*
 * adc.h
 *
 *  Created on: 2024 Jul 6
 *      Author: zaqc
 */

#ifndef USER_ADC_H_
#define USER_ADC_H_

#define ADC_SLOT_SIZE	8192

extern uint16_t adc_buf[ADC_SLOT_SIZE * 2];
extern uint16_t *adc_put_ptr;
extern uint16_t *adc_get_ptr;

extern volatile uint8_t adc_dma_busy;
extern volatile uint8_t adc_dma_done_cntr;

void init_adc_tmr4_dma2ch4(void);

#endif /* USER_ADC_H_ */
