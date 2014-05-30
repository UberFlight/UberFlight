/*
 * pin.h
 *
 *  Created on: 29 mai 2014
 *      Author: treym
 */



#if defined(NAZEPRO)

#define BEEP_PIN    GPIO_Pin_10
#define BEEP_GPIO   GPIOB

#define LED0_PIN    GPIO_Pin_12
#define LED0_GPIO   GPIOB

#define LED1_PIN    GPIO_Pin_12
#define LED1_GPIO   GPIOB

#define MASTER_SPEKTRUM_UART_PIN       GPIO_Pin_11
#define MASTER_SPEKTRUM_UART_GPIO      GPIOB
#define MASTER_SPEKTRUM_UART_PINSOURCE GPIO_PinSource11

#define UART1_TX_PIN GPIO_Pin_9
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_GPIO GPIOA
#define UART1_TX_PINSOURCE GPIO_PinSource9
#define UART1_RX_PINSOURCE GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_3
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_TX_GPIO       GPIOB
#define UART2_RX_GPIO       GPIOA
#define UART2_TX_PINSOURCE  GPIO_PinSource3
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#endif

//-------

// Afroflight32


#if defined(NAZE)
#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Buzzer)
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

// Fixme
#define MASTER_SPEKTRUM_UART_PIN       GPIO_Pin_11
#define MASTER_SPEKTRUM_UART_GPIO      GPIOB
#define MASTER_SPEKTRUM_UART_PINSOURCE GPIO_PinSource11
#define UART1_TX_PIN GPIO_Pin_9
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_GPIO GPIOA
#define UART1_TX_PINSOURCE GPIO_PinSource9
#define UART1_RX_PINSOURCE GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_3
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_TX_GPIO       GPIOB
#define UART2_RX_GPIO       GPIOA
#define UART2_TX_PINSOURCE  GPIO_PinSource3
#define UART2_RX_PINSOURCE  GPIO_PinSource15
#endif

// #define PROD_DEBUG





