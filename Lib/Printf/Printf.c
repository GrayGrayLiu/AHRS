//
// Created by Gray on 2026/1/7.
//

#include "Printf.h"
#include "usart.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    while ((USART1->ISR & 0X40U) == 0);
    USART1->TDR = (uint8_t) ch;
    return ch;
}
