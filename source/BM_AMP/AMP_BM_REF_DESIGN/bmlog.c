/*
 *
 * Copyright (C) 2014 Embest Technology Co., Ltd. <http://www.embest-tech.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of Embest Technology Co., Ltd. nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS", 
 * it is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <types.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <amp_config.h>
#include <bmlog.h>
#include <regs.h>

extern void* usr_stack_end;

//debug only!
#define UART_LSR_THRE   0x20            /* Xmit holding register empty */
void serial0_putc(char ch)
{
        u32 uart0_va_base;
        uart0_va_base = SOCFPGA_UART0_ADDRESS;
        while ((*(volatile u32 *)(((u32)uart0_va_base)+(5<<2)) & UART_LSR_THRE) == 0);
                *(volatile u32 *)uart0_va_base = ch;
}

void uartputs(char *s)
{
	while(*s != '\0')
		serial0_putc(*s++);

	serial0_putc('$');
}


caddr_t _sbrk(int incr){
	
  static char *heap_end = NULL;
  char *prev_heap_end;

  if (heap_end == 0) {
    heap_end = usr_stack_end;
	/* give 2KB area for stacks and use the rest of memory for heap*/
	heap_end += 2048;
  }
  prev_heap_end = heap_end;
  heap_end += incr;
  return (caddr_t) prev_heap_end;
}


int vscnprintf(char *buf, size_t size, const char *fmt, va_list args)
{
	int i;

	i = vsnprintf(buf, size, fmt, args);

	if (i < size)
		return i;
	if (size != 0)
		return size - 1;
	return 0;
}


void bmlog(const char *fmt, ...)
{
	int len;
	
	va_list args;
	
	va_start(args, fmt);
	len = vscnprintf(asp->logbuf + asp->logindex, LOG_LEN, fmt, args);
	asp->logindex += len;
	if(asp->logindex > (LOG_LEN-1) )
	{
		memcpy(asp->logbuf, asp->logbuf + LOG_LEN, asp->logindex - LOG_LEN + 1);
		asp->logindex &= (LOG_LEN - 1);
		asp->logbuf_overlap = 1;
	}
	va_end(args);
}

static char debugbuf[128];

void uartprintf(const char *fmt, ...)
{
	int len;
	int index = 0;
	
	va_list args;
	
	va_start(args, fmt);
	len = vscnprintf(debugbuf + index, 128 - index, fmt, args);
	index += len;
	va_end(args);

	if(index > 0)
	{
		debugbuf[index] = '\0';
		uartputs(debugbuf);
	}
}

