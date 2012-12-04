/*
  COPYRIGHT
*/

#ifndef _ADC_H_
#define _ADC_H_

#include <inttypes.h>
#include "hwdefs.h"

#define ADC_HIST 8

void adc_init();
void adc_disable();
uint8_t adc_collect_samples(uint16_t *dst, uint8_t *last_samplecount, uint8_t adci);
void dump_adc_log();
void adc_stabilize();

extern volatile uint8_t adc_samplecount[ADCNO_END];

#endif
