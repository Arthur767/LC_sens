#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>


#define F_CPU 32000000

void _delay_cycles_n3(uint32_t n3)
{
  while(n3--)
  {
    asm("");
  }
}

#define _delay_us(A)\
  _delay_cycles_n3( (uint32_t) (( (double)(F_CPU) *((A)/1000000.0))/3.0+0.5))

#define _delay_ms(A)\
  _delay_cycles_n3( (uint32_t) (( (double)(F_CPU) *((A)/1000.0))/3.0+0.5))

#define _delay_s(A)\
  _delay_cycles_n3( (uint32_t) (( (double)(F_CPU) *((A)/1.0))/3.0+0.5))

      
#endif