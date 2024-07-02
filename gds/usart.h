#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include <stdint.h>

/*! The rx buffer size for USART */
#define USART_CTRL_RX_BUFFER_SIZE 256
#define USART_CTRL_TX_BUFFER_SIZE 512

extern volatile int USART_CTRL_tx_busy;

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(void);
int uart_recv(uint8_t *buf, int nbytes);
void uart_send_char(uint8_t c);
void uart_flush_input(void);
void uart_flush_output(void);

#ifdef __cplusplus
};
#endif

#endif
