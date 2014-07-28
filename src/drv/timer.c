#include "board.h"
#include "timer.h"


#if defined(NAZE)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn, 0, Mode_IPD},          // PWM1
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn, 0, Mode_IPD},          // PWM2
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 0, Mode_IPD},          // PWM3
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn, 0, Mode_IPD},          // PWM4
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn, 0, Mode_IPD},          // PWM5
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn, 0, Mode_IPD},          // PWM6
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 0, Mode_IPD},          // PWM7
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 0, Mode_IPD},          // PWM8
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 1, Mode_IPD},       // PWM9
    { TIM1, GPIOA, Pin_11, TIM_Channel_4, TIM1_CC_IRQn, 1, Mode_IPD},      // PWM10
    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn, 0, Mode_IPD},          // PWM11
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn, 0, Mode_IPD},          // PWM12
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 0, Mode_IPD},          // PWM13
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 0, Mode_IPD}           // PWM14
};

#define MAX_TIMERS 4 // TIM1..TIM4

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
    TIM1, TIM2, TIM3, TIM4
};
#endif

#if defined(NAZEPRO)
const timerHardware_t timerHardware[] = {
    {   TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn, 1,  Mode_AF_PP_PD},
    {   TIM1,  GPIOA, Pin_9,  TIM_Channel_2, TIM1_CC_IRQn, 1,  Mode_AF_PP_PD},
    {   TIM1,  GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 1,  Mode_AF_PP_PD},
    {   TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,    0,  Mode_AF_PP_PD},
    {   TIM4,  GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn, 0, Mode_AF_PP_PD},
    {   TIM4,  GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn, 0, Mode_AF_PP_PD},
    {   TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn, 0, Mode_AF_PP_PD},
    {   TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn, 0, Mode_AF_PP_PD},

    {   TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn, 0, Mode_AF_PP},
    {   TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP},
    {   TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP_PD},
    {   TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn, 1, Mode_AF_PP_PD},
    {   TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn, 1, Mode_AF_PP_PD},
    {   TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP_PD},
    {   TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn, 0, Mode_AF_PP_PD}               // flex port

};
#define MAX_TIMERS 7

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
TIM1, TIM2, TIM3, TIM4, TIM15, TIM16 , TIM17 };

#endif

#if defined(QUANTOM)
const timerHardware_t timerHardware[] = {
    {   TIM1,  GPIOA, Pin_10, TIM_Channel_3, TIM1_CC_IRQn, 1,  Mode_AF_PP_PD},
    {   TIM8,  GPIOC, Pin_6,  TIM_Channel_1, TIM8_CC_IRQn, 1,  Mode_AF_PP_PD},
    {   TIM8,  GPIOC, Pin_10, TIM_Channel_2, TIM8_CC_IRQn, 1,  Mode_AF_PP_PD},
    {   TIM8,  GPIOC, Pin_4,  TIM_Channel_3, TIM8_CC_IRQn,    0,  Mode_AF_PP_PD},
    {   TIM2,  GPIOA, Pin_15, TIM_Channel_1, TIM2_IRQn, 0, Mode_AF_PP_PD},
    {   TIM2,  GPIOB, Pin_3,  TIM_Channel_2, TIM2_IRQn, 0, Mode_AF_PP_PD},
    {   TIM5,  GPIOA, Pin_0,  TIM_Channel_1, TIM5_IRQn, 0, Mode_AF_PP_PD},
    {   TIM5,  GPIOA, Pin_1,  TIM_Channel_2, TIM5_IRQn, 0, Mode_AF_PP_PD},

    {   TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn, 0, Mode_AF_PP},
    {   TIM3,  GPIOB, Pin_5,  TIM_Channel_2, TIM3_IRQn, 0, Mode_AF_PP},
    {   TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn, 1, Mode_AF_PP_PD},
    {   TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn, 1, Mode_AF_PP_PD},
    {   TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 1, Mode_AF_PP_PD},
    {   TIM10, GPIOB, Pin_8,  TIM_Channel_1, TIM1_UP_TIM10_IRQn, 1, Mode_AF_PP_PD},
    {   TIM11, GPIOB, Pin_9,  TIM_Channel_1, TIM1_TRG_COM_TIM11_IRQn, 0, Mode_AF_PP_PD}               // flex port

};
#define MAX_TIMERS 8

static const TIM_TypeDef const *timers[MAX_TIMERS] = {
TIM1, TIM2, TIM3, TIM5, TIM8, TIM10 , TIM11, TIM12 };

#endif

#define CC_CHANNELS_PER_TIMER 4 // TIM_Channel_1..4
static const uint16_t channels[CC_CHANNELS_PER_TIMER] = {
    TIM_Channel_1, TIM_Channel_2, TIM_Channel_3, TIM_Channel_4
};

typedef struct timerConfig_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    timerCCCallbackPtr *callback;
    uint8_t reference;
} timerConfig_t;

static timerConfig_t timerConfig[MAX_TIMERS * CC_CHANNELS_PER_TIMER];

static uint8_t lookupTimerIndex(const TIM_TypeDef *tim)
{
    uint8_t timerIndex = 0;
    while (timers[timerIndex] != tim) {
        timerIndex++;
    }
    return timerIndex;
}

static uint8_t lookupChannelIndex(const uint16_t channel)
{
    uint8_t channelIndex = 0;
    while (channels[channelIndex] != channel) {
        channelIndex++;
    }
    return channelIndex;
}

static uint8_t lookupTimerConfigIndex(TIM_TypeDef *tim, const uint16_t channel)
{
    return lookupTimerIndex(tim) + (MAX_TIMERS * lookupChannelIndex(channel));
}

void configureTimerChannelCallback(TIM_TypeDef *tim, uint8_t channel, uint8_t reference, timerCCCallbackPtr *callback)
{
    assert_param(IS_TIM_CHANNEL(channel));

    uint8_t timerConfigIndex = lookupTimerConfigIndex(tim, channel);

    if (timerConfigIndex >= MAX_TIMERS * CC_CHANNELS_PER_TIMER) {
        return;
    }

    timerConfig[timerConfigIndex].callback = callback;
    timerConfig[timerConfigIndex].channel = channel;
    timerConfig[timerConfigIndex].reference = reference;
}

void configureTimerInputCaptureCompareChannel(TIM_TypeDef *tim, const uint8_t channel)
{
    switch (channel) {
        case TIM_Channel_1:
            TIM_ITConfig(tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

void configureTimerCaptureCompareInterrupt(const timerHardware_t *timerHardwarePtr, uint8_t reference, timerCCCallbackPtr *callback)
{
    configureTimerChannelCallback(timerHardwarePtr->tim, timerHardwarePtr->channel, reference, callback);
    configureTimerInputCaptureCompareChannel(timerHardwarePtr->tim, timerHardwarePtr->channel);
}

void timerNVICConfigure(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void configTimeBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1; // AKA TIMx_ARR

    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / ((uint32_t)mhz * 1000000)) - 1;


    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void timerConfigure(const timerHardware_t *timerHardwarePtr, uint16_t period, uint8_t mhz)
{
    configTimeBase(timerHardwarePtr->tim, period, mhz);
    TIM_Cmd(timerHardwarePtr->tim, ENABLE);
    timerNVICConfigure(timerHardwarePtr->irq);
}

timerConfig_t *findTimerConfig(TIM_TypeDef *tim, uint16_t channel)
{
    uint8_t timerConfigIndex = lookupTimerConfigIndex(tim, channel);
    return &(timerConfig[timerConfigIndex]);
}

static void timCCxHandler(TIM_TypeDef *tim)
{
    captureCompare_t capture;
    timerConfig_t *tConfig;

    uint8_t channelIndex = 0;
    for (channelIndex = 0; channelIndex < CC_CHANNELS_PER_TIMER; channelIndex++) {
        uint8_t channel = channels[channelIndex];

        if (channel == TIM_Channel_1 && TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC1);

            tConfig = findTimerConfig(tim, TIM_Channel_1);
            capture = TIM_GetCapture1(tim);
        } else if (channel == TIM_Channel_2 && TIM_GetITStatus(tim, TIM_IT_CC2) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC2);

            tConfig = findTimerConfig(tim, TIM_Channel_2);
            capture = TIM_GetCapture2(tim);
        } else if (channel == TIM_Channel_3 && TIM_GetITStatus(tim, TIM_IT_CC3) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC3);

            tConfig = findTimerConfig(tim, TIM_Channel_3);
            capture = TIM_GetCapture3(tim);
        } else if (channel == TIM_Channel_4 && TIM_GetITStatus(tim, TIM_IT_CC4) == SET) {
            TIM_ClearITPendingBit(tim, TIM_IT_CC4);

            tConfig = findTimerConfig(tim, TIM_Channel_4);
            capture = TIM_GetCapture4(tim);
        } else {
            continue; // avoid uninitialised variable dereference
        }

        if (!tConfig->callback) {
            continue;
        }
        tConfig->callback(tConfig->reference, capture);
    }
}

void TIM1_CC_IRQHandler(void)
{
    timCCxHandler(TIM1);
}

void TIM2_IRQHandler(void)
{
    timCCxHandler(TIM2);
}

void TIM3_IRQHandler(void)
{
    timCCxHandler(TIM3);
}

void TIM4_IRQHandler(void)
{
    timCCxHandler(TIM4);
}


void TIM8_CC_IRQHandler(void)
{
    timCCxHandler(TIM8);
}

#if !defined(QUANTOM)
void TIM1_BRK_TIM15_IRQHandler(void)
{
    timCCxHandler(TIM15);
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    timCCxHandler(TIM16);
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    timCCxHandler(TIM17);
}

#endif
