/*=============================================================================
  *
  * @file     : uart.h
  * @author        : JackABK
  * @data       : 2014/2/24
  * @brief   : uart.c header file
  *
  *============================================================================*/
#ifndef __UART_H__
#define __UART_H__



/*the usart acept the command from RX when RX interrupt is trigger*/
unsigned char Receive_data;
unsigned char Receive_data2;

/*Setting the USART MAX string lenth */
#define MAX_STRLEN 50 // this is the maximum string length of our string in characters

volatile unsigned char received_string[MAX_STRLEN]; // this will hold the recieved string
volatile unsigned char received_string2[MAX_STRLEN]; // this will hold the recieved string
volatile unsigned char alarm_message;
volatile int message_counter2;

extern void init_USART3(uint32_t baurate);
extern void USART3_IRQHandler(void);
extern void USART_puts(USART_TypeDef* USARTx, volatile char *s);

extern void init_USART6(uint32_t baurate);
extern void USART6_IRQHandler(void);

// usart6
#define USARTy                           USART6
#define USARTy_CLK                       RCC_APB2Periph_USART6
#define USARTy_CLK_INIT                  RCC_APB2PeriphClockCmd
#define USARTy_IRQn                      USART6_IRQn
#define USARTy_IRQHandler                USART6_IRQHandler

#define USARTy_TX_PIN                    GPIO_Pin_6                
#define USARTy_TX_GPIO_PORT              GPIOC                       
#define USARTy_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTy_TX_SOURCE                 GPIO_PinSource6
#define USARTy_TX_AF                     GPIO_AF_USART6

#define USARTy_RX_PIN                    GPIO_Pin_7                
#define USARTy_RX_GPIO_PORT              GPIOC                    
#define USARTy_RX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTy_RX_SOURCE                 GPIO_PinSource7
#define USARTy_RX_AF                     GPIO_AF_USART6


#endif /* __UART_H__ */
