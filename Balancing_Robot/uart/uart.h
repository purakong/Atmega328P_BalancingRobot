#ifndef _UART_H_
#define _UART_H_

#include <inttypes.h>
#include <stdint.h>
#include <avr/io.h>
#define BAUD 9600

// initialize UART
void uart_init();
unsigned char UART_receive(void);
unsigned char UART_transmit(unsigned char data);
void UART_printString(char *str);
void UART_printUnsigned8bitNumber(uint8_t data);
void UART_printUnsigned16bitNumber(uint16_t data);
void UART_printSigned16bitNumber(int16_t data);
void UART_printDouble(double data, int len);

#endif