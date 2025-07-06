/*
 * USART_RS232_H_file.h
 * http://www.electronicwings.com
 *
 */ 


#ifndef USART_RS232_H_FILE_H_				/* Define library H file if not defined */
#define USART_RS232_H_FILE_H_

#ifdef DOUBLE_SPEED_MODE
	#define BAUD_PRESCALE (int)round(((((double)F_CPU / ((double)BAUDRATE * 8.0))) - 1.0))	/* Define prescale value */
#else
	#define BAUD_PRESCALE (int)round(((((double)F_CPU / ((double)BAUDRATE * 16.0))) - 1.0))	/* Define prescale value */
#endif

void USART_Init(void);						/* USART initialize function */
unsigned char USART_RxChar();						/* Data receiving function */
void USART_TxChar(unsigned char);					/* Data transmitting function */
void USART_SendString(const char*);			/* Send string of USART data function */
void USART_SendHexByte(uint8_t* hex_bytes_arr, int size);
char* USART_GetStringBuffer();
void USART_SetStringBuffer(int index, char ch);
void USART_ClearStringBuffer();
uint8_t USART_IsStringReady();
void USART_SetStringReady(uint8_t ready);
uint8_t USART_GetStringBufferIndex();
void USART_SetStringBufferIndex(uint8_t index);
uint8_t* USART_GetHexBuffer();
uint8_t USART_GetHexBufferSize();
void USART_SetHexBuffer(int index, uint8_t hex_byte);
void USART_ClearHexBuffer();
uint8_t USART_IsHexReady();
void USART_SetHexReady(uint8_t ready);
uint8_t USART_GetHexState();
void USART_SetHexState(uint8_t state);
uint8_t USART_GetHexNibble();
void USART_SetHexNibble(uint8_t nibble);

#endif										/* USART_RS232_H_FILE_H_ */ 
