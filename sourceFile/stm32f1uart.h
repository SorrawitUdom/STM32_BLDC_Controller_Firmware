#ifndef UART_H
#define UART_H

#include "stm32f10x.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"

void seriailInit(unsigned long F_CPU,unsigned long baud);
void serialprintf(const char *stream ,...);

#endif