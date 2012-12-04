/*
 * (c) 2004 Trevor Blackwell <tlb@tlb.org>
 *
 */

#ifndef _ADC_H_
#define _ADC_H_

#include <inttypes.h>

#define ADC_HIST 16

extern void adc_init( void );
uint8_t adc_collect_samples(uint16_t *dst, uint8_t *last_s0);
void dump_adc_log();

#endif
