/*
 * uart.c
 *
 *  Created on: 05.08.2017
 *      Author: Mariusz
 */


#include "stdlib.h"


#include "DISCOVERY/uart.h"

volatile uint8_t ascii_line; // zmienna mowiaca ile jest pelnych lini danych jest w buforze

static void (*uart_rx_str_event_callback)(char *pBuf); // wskaznik na funkcje ktora bedzie wywolana w chwili zajscia zdarzenia UART_Rx_STR_Event()


#define USART_X USART1 // trzeba dodatkowo pozmieniac w funkcji inicjalizacyjnej

// funkcje:
void uart_init(void)
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ********************************************   ODBIORCZY BUFOR CYKLICZNY
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define UART_RX_BUF_SIZE 32						// rozmiar bufora ( nusi to byc 2^n)
#define UART_RX_BUF_MASK (UART_RX_BUF_SIZE - 1) // maska (przydatna przy zapetlaniu bufora)

volatile char UART_Rx_Buf[UART_RX_BUF_SIZE]; // definicja bufora

// indeksy okreslajace ilosc danych w buforze
volatile uint8_t Uart_RxHead;
volatile uint8_t Uart_RxTail;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ********************************************   NADAWCZY BUFOR CYKLICZNY
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define UART_TX_BUF_SIZE 64
#define UART_TX_BUF_MASK (UART_TX_BUF_SIZE - 1)

volatile char UART_Tx_Buf[UART_TX_BUF_SIZE];

// indeksy okreslajace ilosc danych w buforze
volatile uint8_t Uart_TxHead;
volatile uint8_t Uart_TxTail;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ********************************************   FUNKCJE DO OBSLUGI BUFOROW
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char uart_getchar(void)
{
	/*
	if(Uart_RxHead == Uart_RxTail) return -1;
	Uart_RxTail = (Uart_RxTail + 1) & UART_RX_BUF_MASK;
	*/
	return UART_Rx_Buf[Uart_RxTail];
}

char* uart_getstring(char *buf)
{
	char c;
	char *wsk = buf;

	while( (c = uart_getchar()) )
	{
		if( c==13 || c<0 ) break;
		*buf++ = c;
	}
	*buf=0;
	ascii_line--;

	return wsk;

}

void uart_putchar(char c)
{
	/*uint8_t tmp_head;

	tmp_head = (Uart_TxHead + 1) & UART_TX_BUF_MASK;
	while(tmp_head == Uart_TxTail){}

	UART_Tx_Buf[tmp_head] = c;
	Uart_TxHead = tmp_head;
*/

}

void uart_putstring(char *str)
{
	register char c;
	while( (c=*str++) ) uart_putchar(c);

}

void uart_putint(uint32_t liczba, uint8_t radix)
{
	char buf[17];
	itoa(liczba, buf, radix);
	uart_putstring(buf);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ********************************************   OBSLUGA PRZERWANIA (Wspolna dla bufora odbiorczego i nadawczego)
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ********************************************   Komendy AT
 */
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void register_uart_rx_str_event_callback( void (*callback)(char *pBuf) ) // funkcja rejestrujaca callbacka
{
	uart_rx_str_event_callback = callback;
}


void UART_Rx_STR_Event( char *rbuf )
{
	if(ascii_line)
	{
		if(uart_rx_str_event_callback)
		{
			uart_getstring( rbuf );
			(*uart_rx_str_event_callback)(rbuf);
		}
	}
}
