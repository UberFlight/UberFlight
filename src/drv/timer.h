#pragma once

typedef enum {
    Pin_0 = 0x0001,
    Pin_1 = 0x0002,
    Pin_2 = 0x0004,
    Pin_3 = 0x0008,
    Pin_4 = 0x0010,
    Pin_5 = 0x0020,
    Pin_6 = 0x0040,
    Pin_7 = 0x0080,
    Pin_8 = 0x0100,
    Pin_9 = 0x0200,
    Pin_10 = 0x0400,
    Pin_11 = 0x0800,
    Pin_12 = 0x1000,
    Pin_13 = 0x2000,
    Pin_14 = 0x4000,
    Pin_15 = 0x8000,
    Pin_All = 0xFFFF
} GPIO_Pin;


#define USABLE_TIMER_CHANNEL_COUNT 14


typedef uint16_t captureCompare_t;

typedef void timerCCCallbackPtr(uint8_t port, captureCompare_t capture);

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *gpio;
    uint32_t pin;
    uint8_t channel;
    uint8_t irq;
    uint8_t outputEnable;
    GPIO_Mode gpioInputMode;
} timerHardware_t;

extern const timerHardware_t timerHardware[];

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);
void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz);
void timerNVICConfigure(uint8_t irq);

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel);
void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback);
void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback);
