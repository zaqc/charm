/*
 * global_param.c
 *
 *  Created on: 2024 Jul 6
 *      Author: zaqc
 */

#include "at32f435_437_board.h"

uint32_t g_AdcBufSize = 4096;	// Size of buffer for DMA to fill with ADC data

uint32_t g_PulseTickCount = 2;	// Count of Impulses in Probing Pulse
uint32_t g_PulseWidth = 50;		// Width of "1" Probing Pulse (5-50)
uint32_t g_PulseFreq = 114;		// 114 ~ 2.5 MHz
