
#include "board.h"

// Using RX DMA disables the use of receive callbacks
#define USE_USART1_RX_DMA
#define USE_USART2_RX_DMA
#define USE_USART2_TX_DMA

static uartPort_t uartPort1;
static uartPort_t uartPort2;
static uartPort_t uartPort3;

void uartStartTxDMA(uartPort_t *s);

uartPort_t *serialUSART1(uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s;
    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    s = &uartPort1;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx1Buffer;
    s->port.txBuffer = tx1Buffer;
    s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART1_TX_BUFFER_SIZE;
//#ifdef USE_USART1_RX_DMA
//    s->rxDMAChannel = DMA1_Channel5;
//#endif
//    s->txDMAChannel = DMA1_Channel4;

    s->USARTx = USART1;
//
//    s->rxDMAPeripheralBaseAddr = (uint32_t) & s->USARTx->RDR;
//    s->txDMAPeripheralBaseAddr = (uint32_t) & s->USARTx->TDR;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    if (mode & MODE_TX) {
        GPIO_InitStructure.GPIO_Pin = UART1_TX_PIN;
        GPIO_PinAFConfig(UART1_TX_GPIO, UART1_TX_PINSOURCE, GPIO_AF_USART1);
        GPIO_Init(UART1_TX_GPIO, &GPIO_InitStructure);
    }

    if (mode & MODE_RX) {
        GPIO_InitStructure.GPIO_Pin = UART1_RX_PIN;
        GPIO_PinAFConfig(UART1_RX_GPIO, UART1_RX_PINSOURCE, GPIO_AF_USART1);
        GPIO_Init(UART1_RX_GPIO, &GPIO_InitStructure);
    }

//    // DMA TX Interrupt
//    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//
//#ifndef USE_USART1_RX_DMA

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//#endif

    return s;
}

uartPort_t *serialUSART2(uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s;
    static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
    static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    s = &uartPort2;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART2_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx2Buffer;
    s->port.txBuffer = tx2Buffer;

    s->USARTx = USART2;
//#ifdef USE_USART2_RX_DMA
//    s->rxDMAChannel = DMA1_Channel6;
//    s->rxDMAPeripheralBaseAddr = (uint32_t) & s->USARTx->RDR;
//#endif
//#ifdef USE_USART2_TX_DMA
//    s->txDMAChannel = DMA1_Channel7;
//    s->txDMAPeripheralBaseAddr = (uint32_t) & s->USARTx->TDR;
//#endif

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    if (mode & MODE_TX) {
        GPIO_InitStructure.GPIO_Pin = UART2_TX_PIN;
        GPIO_PinAFConfig(UART2_TX_GPIO, UART2_TX_PINSOURCE, GPIO_AF_USART2);
        GPIO_Init(UART2_TX_GPIO, &GPIO_InitStructure);
    }

    if (mode & MODE_RX) {
        GPIO_InitStructure.GPIO_Pin = UART2_RX_PIN;
        GPIO_PinAFConfig(UART2_RX_GPIO, UART2_RX_PINSOURCE, GPIO_AF_USART2);
        GPIO_Init(UART2_RX_GPIO, &GPIO_InitStructure);
    }

//#ifdef USE_USART2_TX_DMA
//    // DMA TX Interrupt
//    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//#endif
//
//#ifndef USE_USART2_RX_DMA
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//#endif

    return s;
}

//static void handleUsartTxDma(uartPort_t *s)
//{
//    DMA_Cmd(s->txDMAChannel, DISABLE);
//
//    if (s->port.txBufferHead != s->port.txBufferTail)
//        uartStartTxDMA(s);
//    else
//        s->txDMAEmpty = true;
//}
//
//// USART1 Tx DMA Handler
//void DMA1_Channel4_IRQHandler(void)
//{
//    uartPort_t *s = &uartPort1;
//    DMA_ClearITPendingBit(DMA1_IT_TC4);
//    DMA_Cmd(DMA1_Channel4, DISABLE);
//    handleUsartTxDma(s);
//}
//
//// USART1 Rx DMA Handler
//void DMA1_Channel5_IRQHandler(void)
//{
//    uartPort_t *s = &uartPort1;
//    DMA_ClearITPendingBit(DMA1_IT_TC5);
//    DMA_Cmd(DMA1_Channel5, DISABLE);
//    handleUsartTxDma(s);
//}
//
//// USART2 Rx DMA Handler
//void DMA1_Channel6_IRQHandler(void)
//{
//    uartPort_t *s = &uartPort2;
//    DMA_ClearITPendingBit(DMA1_IT_TC6);
//    DMA_Cmd(DMA1_Channel6, DISABLE);
//    handleUsartTxDma(s);
//}
//
//
//// USART2 Tx DMA Handler
//void DMA1_Channel7_IRQHandler(void)
//{
//    uartPort_t *s = &uartPort2;
//    DMA_ClearITPendingBit(DMA1_IT_TC7);
//    DMA_Cmd(DMA1_Channel7, DISABLE);
//    handleUsartTxDma(s);
//}


void usartIrqHandler(uartPort_t *s)
{
    uint32_t ISR = s->USARTx->SR;

    if (ISR & USART_FLAG_RXNE) {
        if (s->port.callback) {
            s->port.callback(s->USARTx->DR);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = s->USARTx->DR;
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    if (ISR & USART_FLAG_TXE) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (ISR & USART_FLAG_ORE) {
        USART_ClearITPendingBit(s->USARTx, USART_IT_ORE);
    }
}


uartPort_t *serialUSART3(uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s;
    static volatile uint8_t rx3Buffer[UART3_RX_BUFFER_SIZE];
    static volatile uint8_t tx3Buffer[UART3_TX_BUFFER_SIZE];
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    s = &uartPort3;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART3_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART3_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx3Buffer;
    s->port.txBuffer = tx3Buffer;

    s->USARTx = USART3;

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    if (mode & MODE_TX) {
         GPIO_InitStructure.GPIO_Pin = UART3_TX_PIN;
         GPIO_PinAFConfig(UART3_TX_GPIO, UART3_TX_PINSOURCE, GPIO_AF_USART2);
         GPIO_Init(UART3_TX_GPIO, &GPIO_InitStructure);
     }

     if (mode & MODE_RX) {
         GPIO_InitStructure.GPIO_Pin = UART3_RX_PIN;
         GPIO_PinAFConfig(UART3_RX_GPIO, UART3_RX_PINSOURCE, GPIO_AF_USART2);
         GPIO_Init(UART3_RX_GPIO, &GPIO_InitStructure);
     }




    return s;
}

void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;

    usartIrqHandler(s);
}

void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;

    usartIrqHandler(s);
}

void USART3_IRQHandler(void)
{
    uartPort_t *s = &uartPort3;

    usartIrqHandler(s);
}

