#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#include "uart.h"


// initialize UART
void uart_init() {
  
  //2배 모드 
  UCSR0A |= _BV(U2X0);
  
  //9600BPS
  if(BAUD == 9600)
  {
	  UBRR0H = 0x00;
	  UBRR0L = 207;  
  }
  else if(BAUD == 115200)
  {
	  UBRR0H = 0x00;
	  UBRR0L = 16;
  }

  //Set the number of data bits in a frame the receiver and transmitter use
  // UCSZ01, UCSZ00 : 1 1 : 8bit
  UCSR0C = 0x06;
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX
}

//Byte 단위로 수신하는 함수
unsigned char UART_receive(void)
{
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}

//Byte 단위로 송신하는 함수
unsigned char UART_transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0))){}
	UDR0 = data;
}

//문자열을 출력하는 함수
void UART_printString(char *str)
{
	//'\0' 문자를 만날 때 까지 반복
	for(int i=0; str[i]; i++)
	{
		UART_transmit(str[i]);
	}
}

void UART_printUnsigned8bitNumber(uint8_t data)
{
	//8bit는 255가 최대값이므로, 널문자를 포함한 4Byte 선언
	char numString[4] = "0";
	int i, index = 0;
	if(data>0)
	{
		for(i = 0; data !=0; i++)
		{
			numString[i] = data % 10 + '0';
			data = data/10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i>=0; i--)
	{
		UART_transmit(numString[i]);
	}
}

void UART_printUnsigned16bitNumber(uint16_t data)
{
	char numString[6] = "0";
	int i, index = 0;
	if(data>0)
	{
		for(i = 0; data !=0; i++)
		{
			numString[i] = data % 10 + '0';
			data = data/10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i>=0; i--)
	{
		UART_transmit(numString[i]);
	}
}

void UART_printSigned16bitNumber(int16_t data)
{
	char numString[7] = "0";
	int i, index = 0;
	unsigned short u_data;
	if(data>0)
	{
		for(i = 0; data !=0; i++)
		{
			numString[i] = data % 10 + '0';
			data = data/10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	else if(data<0)
	{
		u_data = -data;
		for(i = 0; u_data !=0; i++)
		{
			numString[i] = u_data % 10 + '0';
			u_data = u_data/10;
		}
		numString[i] = '-';
		numString[++i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i>=0; i--)
	{
		UART_transmit(numString[i]);
	}
}

void UART_printUnsigned32bitNumber(uint32_t data)
{
	char numString[11] = "0";
	int i, index = 0;
	if(data>0)
	{
		for(i = 0; data !=0; i++)
		{
			numString[i] = data % 10 + '0';
			data = data/10;
		}
		numString[i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i>=0; i--)
	{
		UART_transmit(numString[i]);
	}
}

//실수형 변수 출력
//data 출력값, len: 소수점 이하 자릿수
void UART_printDouble(double data, int len)
{
	char numString[20] = "0";
	int i = 0;
	int j = 0;
	int index = 0;
	long long_data = (long)data;
	long decimal_data = (long)((data - (double)long_data)*pow(10.0,(double)len));
	unsigned long u_long_data;
	if(data > 0)
	{
		for(j = 0; j <len; j++)
		{
			numString[i] = decimal_data % 10 + '0';
			decimal_data = decimal_data/10;
			i++;
		}
		
		// 46의 아스키코드값은 '.'이다.
		numString[i] = 46;
		i++;
		
		for(i; long_data !=0; i++)
		{
			numString[i] = long_data % 10 + '0';
			long_data = long_data/10;
		}
		
		numString[i] = '\0';
		index = i - 1;
	}
	
	else if(data < 0)
	{
		decimal_data = -decimal_data;
		for(j = 0; j <len; j++)
		{
			numString[i] = decimal_data % 10 + '0';
			decimal_data = decimal_data/10;
			i++;
		}
		
		// 46의 아스키코드값은 '.'이다.
		numString[i] = 46;
		i++;
		
		u_long_data = -long_data;
		for(i; u_long_data !=0; i++)
		{
			numString[i] = u_long_data % 10 + '0';
			u_long_data = u_long_data/10;
		}
		numString[i] = '-';
		numString[++i] = '\0';
		index = i - 1;
	}
	
	for(i = index; i>=0; i--)
	{
		UART_transmit(numString[i]);
	}
	
}




