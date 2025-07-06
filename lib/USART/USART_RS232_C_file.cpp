/*
 * USART_RS232_C_file.c
 * http://www.electronicwings.com
 *
 */ 
#include "param.h"
#include "USART_RS232_H_file.h"							/* Include USART header file */

/* USART initialize function */
void USART_Init(void)
{
#ifdef DOUBLE_SPEED_MODE
	UCSRA |=(1 << U2X);
#endif
	UCSRB |= (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);	/* Enable USART transmitter and receiver */
	UCSRC |= (1 << URSEL)| (1 << UCSZ0) | (1 << UCSZ1);	/* Write USCRC for 8 bit data and 1 stop bit */ 
	UBRRL = BAUD_PRESCALE;								/* Load UBRRL with lower 8 bit of prescale value */
	UBRRH = (BAUD_PRESCALE >> 8);						/* Load UBRRH with upper 8 bit of prescale value */
}

unsigned char USART_RxChar()										/* Data receiving function */
{
	while (!(UCSRA & (1 << RXC)));						/* Wait until new data receive */
	return(UDR);										/* Get and return received data */ 
}

void USART_TxChar(unsigned char data)							/* Data transmitting function */
{
	UDR = data;											/* Write data to be transmitting in UDR */
	while (!(UCSRA & (1<<UDRE)));						/* Wait until data transmit and buffer get empty */
}

static void transmit_hex_byte(uint8_t byte) {
    const char hex[] = "0123456789ABCDEF";
    USART_TxChar(hex[(byte >> 4) & 0x0F]);
    USART_TxChar(hex[byte & 0x0F]);
}

void USART_SendString(const char *str)					/* Send string of USART data function */ 
{
	int i=0;															
	while (str[i]!=0)
	{
		USART_TxChar(str[i]);							/* Send each char of string till the NULL */
		i++;
	} 
}
void USART_SendHexByte(uint8_t* hex_bytes_arr, int size){
	// Transmit hex bytes as ASCII
	for (uint8_t i = 0; i < size; i++) {
		transmit_hex_byte(hex_bytes_arr[i]);
		USART_TxChar(' '); // Space between bytes
	} 
}


char* USART_GetStringBuffer(){
	return string_buffer;
}

void USART_SetStringBuffer(int index, char ch){
	if(index >=0 && index <STRING_BUFFER_SIZE){
		string_buffer[index] = ch; 
	}
}

void USART_ClearStringBuffer(){
	string_buffer_index = 0;
}

uint8_t USART_IsStringReady(){
	return string_ready;
}

void USART_SetStringReady(uint8_t ready)
{
	string_ready = ready;
}

uint8_t USART_GetStringBufferIndex()
{
	return string_buffer_index;
}

void USART_SetStringBufferIndex(uint8_t index)
{
	string_buffer_index = index;
}


uint8_t* USART_GetHexBuffer(){
	return hex_byte_buffer;
}

uint8_t USART_GetHexBufferSize(){
	return hex_byte_count;
}

void USART_SetHexBuffer(int index, uint8_t hex_byte){
	if(index >=0 && index < HEX_BYTE_BUFFER_SIZE){
		hex_byte_buffer[index] = hex_byte;
	}
}

void USART_ClearHexBuffer(){
	hex_byte_count = 0;
	hex_state = 0;
}

uint8_t USART_IsHexReady(){
	return hex_byte_ready;
}

void USART_SetHexReady(uint8_t ready)
{
	hex_byte_ready = ready;
}

uint8_t USART_GetHexState()
{
	return hex_state;
}

void USART_SetHexState(uint8_t state)
{
	hex_state = state;
}

uint8_t USART_GetHexNibble()
{
	return hex_nibble;
}

void USART_SetHexNibble(uint8_t nibble)
{
	hex_nibble = nibble;
}