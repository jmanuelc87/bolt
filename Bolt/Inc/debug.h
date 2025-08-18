#ifndef BOLT_DEBUG_H
#define BOLT_DEBUG_H
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "usart.h"

#define DEBUG_ENABLED 1

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

void flush();

#endif // __DEBUG_H__

#endif /* BOLT_DEBUG_H */
