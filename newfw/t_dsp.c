#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <math.h>

#include <iobits.h>
#include "uart.h"
#include "string.h"
#include "dsp.h"

float randfloat_x;
float randfloat_y;
int32_t randint_x;
int32_t randint_y;
int32_t save_16dot16;
float save_float;

void
time_dsp_ops()
{
  {
    uint16_t start_tcnt3 = TCNT3;
    save_16dot16=dspmul(randint_x, randint_y);
    uint16_t elapsed = TCNT3-start_tcnt3;
  
    uart_tx_str_P(PSTR("elapsed: dspmul +,+: "));
    uart_tx_uint16_hex(elapsed);
    uart_tx_uint8('\n');
  }

  {
    uint16_t start_tcnt3 = TCNT3;
    save_16dot16=dspmul(randint_x, -randint_y);
    uint16_t elapsed = TCNT3-start_tcnt3;
  
    uart_tx_str_P(PSTR("elapsed: dspmul +,-: "));
    uart_tx_uint16_hex(elapsed);
    uart_tx_uint8('\n');
  }

  {
    uint16_t start_tcnt3 = TCNT3;
    save_16dot16=dspadd(randint_x, randint_y);
    uint16_t elapsed = TCNT3-start_tcnt3;
  
    uart_tx_str_P(PSTR("elapsed: dspadd: "));
    uart_tx_uint16_hex(elapsed);
    uart_tx_uint8('\n');
  }

  {
    uint16_t start_tcnt3 = TCNT3;
    save_float=randfloat_x*randfloat_y;
    uint16_t elapsed = TCNT3-start_tcnt3;
  
    uart_tx_str_P(PSTR("elapsed: mul float: "));
    uart_tx_uint16_hex(elapsed);
    uart_tx_uint8('\n');
  }


  {
    uint16_t start_tcnt3 = TCNT3;
    save_float=randfloat_x+randfloat_y;
    uint16_t elapsed = TCNT3-start_tcnt3;
  
    uart_tx_str_P(PSTR("elapsed: add float: "));
    uart_tx_uint16_hex(elapsed);
    uart_tx_uint8('\n');
  }
}

void
log_dsp_mul()
{
  int32_t ops[12]={0x00000001,
                  0x00000100,
                  0x00010000,
                  0x00700000,
                  0x00f00000,
                  0x02000000,
                  0x05000000,
                  0x20000000,
                  0xffffffff,
                  0xffffff00,
                  0xffff0000,
                  0xff810000};
  for (uint8_t i=0; i<12; i++) {
    for (uint8_t j=0; j<12; j++) {
      int32_t x=ops[i];
      int32_t y=ops[j];
      int32_t mul_result=dspmul(x, y);

      uart_tx_str_P(PSTR("mul: "));
      uart_tx_uint32_hex(x);
      uart_tx_uint8('*');
      uart_tx_uint32_hex(y);
      uart_tx_uint8('=');
      uart_tx_uint32_hex(mul_result);
      uart_tx_uint8('\n');      
    }
  }

  for (int iter=0; iter<500; iter++) {
    dsp_t x=rand() + ((dsp_t)rand()<<12);
    dsp_t y=rand() + ((dsp_t)rand()<<12);
    dsp_t mul_result=dspmul(x, y);
    
    uart_tx_str_P(PSTR("mul: "));
    uart_tx_uint32_hex(x);
    uart_tx_uint8('*');
    uart_tx_uint32_hex(y);
    uart_tx_uint8('=');
    uart_tx_uint32_hex(mul_result);
    uart_tx_uint8('\n');      
  }
}

void
log_div()
{
  dsp_t z;

  uart_tx_str_P(PSTR("div64: "));
  z=randint_x/64L;
  uart_tx_uint32_hex(randint_x);
  uart_tx_uint8('=');
  uart_tx_uint32_hex(z);
  uart_tx_uint8(',');

  z=-randint_x/64L;
  uart_tx_uint32_hex(-randint_x);
  uart_tx_uint8('=');
  uart_tx_uint32_hex(z);
  uart_tx_uint8(',');

  z=randint_y/64L;
  uart_tx_uint32_hex(randint_y);
  uart_tx_uint8('=');
  uart_tx_uint32_hex(z);
  uart_tx_uint8(',');

  z=-randint_y/64L;
  uart_tx_uint32_hex(-randint_y);
  uart_tx_uint8('=');
  uart_tx_uint32_hex(z);

  uart_tx_uint8('\n');
}

void
log_dsp_op(int32_t x, int32_t y)
{
  int32_t mul_result=dspmul(x, y);
  int32_t add_result=dspadd(x, y);
  int32_t sub_result=dspsub(x, y);

  uart_tx_str_P(PSTR("dsp: "));

  uart_tx_uint32_hex(x);
  uart_tx_uint8(',');
  uart_tx_uint32_hex(y);
  uart_tx_uint8('=');
  uart_tx_uint32_hex(mul_result);
  uart_tx_uint8(',');
  uart_tx_uint32_hex(add_result);
  uart_tx_uint8(',');
  uart_tx_uint32_hex(sub_result);
  uart_tx_uint8('\n');

  while (!uart_tx_empty()) {}
}

void
log_dsp_suite(int32_t x, int32_t y)
{
  log_dsp_op(x, y);
  log_dsp_op(-x, y);
  log_dsp_op(x, -y);
  log_dsp_op(-x, -y);
  uart_tx_uint8('\n');
}

void foo()
{
  randint_x=DSPNUM(0.05);
}

int main( void )
{
  uart_init_tx();
  uart_init_rx();

  TCCR3A = 0;
  TCCR3B = 0 |
    (0<<CS32) | (0<<CS31) | (1<<CS30); // prescale clk/1

  sei();

  randfloat_x=4.2394830948;
  randfloat_y=1.34923948;

  randint_x=0x00040000;
  randint_y=0x00017238;

  if (1) {
    time_dsp_ops();
  }
  if (1) {
    log_div();
  }
  if (1) {
    log_dsp_mul();
  }
  foo();
  if (1) {
    log_dsp_suite(0L,0L);
    log_dsp_suite(0x010000L,0x010000L);
    log_dsp_suite(0x030000L,0x050000L);
    log_dsp_suite(0x008500L,0x050000L);
  }


  while(1) {

  }
}
