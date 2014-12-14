#include "board.h"
#include "mw.h"
#include "align.h"
#include "telemetry_common.h"

core_t core;

extern rcReadRawDataPtr rcReadRawFunc;
extern uint16_t pwmReadRawRC(uint8_t chan);

int main(void)
{
    checkFirstTime(false);          // check the need to load default config
    loadAndActivateConfig();        // load master and profile configuration

    systemInit(mcfg.emf_avoidance); // start board

//    // configure power ADC
//    if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9))
//        adc_params.powerAdcChannel = mcfg.power_adc_channel;
//    else {
//        adc_params.powerAdcChannel = 0;
//        mcfg.power_adc_channel = 0;
//    }
//    adcInit(&adc_params);

    // Check battery type/voltage
//    if (feature(FEATURE_VBAT))
//        batteryInit();

    boardAlignmentInit();

    sensorsSet(SENSORS_SET);    // we have these sensors; SENSORS_SET defined in board.h depending on hardware platform

#if !(defined(NAZE))

#if defined(NAZEPRO)
    if (feature(FEATURE_I2C))
        i2cInit(BOARD_I2C_PORT);

#endif
//

    core.mainport = usbInit();      // start serial
    sensorsAutodetect();            // drop out any sensors that don't seem to work, init all the others.

    if (!sensors(SENSOR_GYRO))// if gyro was not detected, we give up now.
    failureMode(3);

#else
    core.mainport = uartOpen(USART1, NULL, mcfg.serial_baudrate, MODE_RXTX,SERIAL_NOT_INVERTED);
#endif



    {

        int i;
        // configure PWM/CPPM read function and max number of channels. spektrum or sbus below will override both of these, if enabled
        for (i = 0; i < RC_CHANS; i++)
            rcData[i] = 1502;
        rcReadRawFunc = pwmReadRawRC;
        core.numRCChannels = MAX_INPUTS;

        if (feature(FEATURE_SERIALRX)) {
            switch (mcfg.serialrx_type) {
                case SERIALRX_SPEKTRUM1024:
                case SERIALRX_SPEKTRUM2048:
                    spektrumInit(&rcReadRawFunc);
                    break;
                case SERIALRX_SBUS:
                    sbusInit(&rcReadRawFunc);
                    break;
                case SERIALRX_SUMD:
                    sumdInit(&rcReadRawFunc);
                    break;
                case SERIALRX_MSP:
                    mspRxInit(&rcReadRawFunc);
                    break;
            }
        }

    }

    {
        // now init other serial
        if (feature(FEATURE_TELEMETRY)) {
            initTelemetry(UART_HEADER_RXTX);
        }

        if (feature(FEATURE_GPS)) {
            USART_TypeDef *gpsUSARTx = UART_HEADER_RXTX;
            if (core.telemport) {
                gpsUSARTx = UART_HEADER_FLEX;
            }
            if (core.rcvrport) {
                gpsUSARTx = UART_HEADER_RC;
            }
            gpsInit(gpsUSARTx, mcfg.gps_baudrate);
        }

    }

    mspInit();                  // this will configure the aux box based on features and detected sensors

    mixerInit();                // this will set core.useServo var depending on mixer type
    imuInit();                  // set initial flight constant like gravitation or other user defined setting

    {
        // TODO fixme configure power ADC
        drv_adc_config_t adc_params;
        if (mcfg.power_adc_channel > 0 && (mcfg.power_adc_channel == 1 || mcfg.power_adc_channel == 9))
            adc_params.powerAdcChannel = mcfg.power_adc_channel;
        else {
            adc_params.powerAdcChannel = 0;
            mcfg.power_adc_channel = 0;
        }
        adcInit(&adc_params);
    }

    {
        drv_pwm_config_t pwm_params;

        // when using airplane/wing mixer, servo/motor outputs are remapped
        if (mcfg.mixerConfiguration == MULTITYPE_AIRPLANE || mcfg.mixerConfiguration == MULTITYPE_FLYING_WING)
            pwm_params.airplane = true;
        else
            pwm_params.airplane = false;

        pwm_params.useRcUART = feature(FEATURE_GPS) && feature(FEATURE_SERIALRX) && core.telemport;
        pwm_params.useAf = feature(FEATURE_AF);
        pwm_params.noPwmRx = feature(FEATURE_PPM) || feature(FEATURE_SERIALRX);
        pwm_params.useSerialrx = feature(FEATURE_SERIALRX);
        pwm_params.useI2c = feature(FEATURE_I2C);
        pwm_params.useCamStab = feature(FEATURE_SERVO_TILT) || (mcfg.mixerConfiguration == MULTITYPE_GIMBAL);
    //    pwm_params.notorsNumber = core.notorsNumber;
        pwm_params.useTri = mcfg.mixerConfiguration == MULTITYPE_TRI;
        pwm_params.motorPwmRate = mcfg.motor_pwm_rate;
        pwm_params.servoPwmRate = mcfg.servo_pwm_rate;
        pwm_params.idlePulse = PULSE_1MS;                  // standard PWM for brushless ESC (default, overridden below)
        if (feature(FEATURE_3D))
            pwm_params.idlePulse = mcfg.neutral3d;
        if (pwm_params.motorPwmRate > 500)
            pwm_params.idlePulse = 0;                  // brushed motors
        pwm_params.servoCenterPulse = mcfg.midrc;
        pwm_params.failsafeThreshold = cfg.failsafe_detect_threshold;

        // todo assign a pin adc + rssi
//        switch (mcfg.power_adc_channel) {
//            case 1:
//                pwm_params.adcChannel = PWM2;
//                break;
//            case 9:
//                pwm_params.adcChannel = PWM8;
//                break;
//            default:
//                pwm_params.adcChannel = 0;
//                break;
//        }

        pwmInit(&pwm_params);
    }

    previousTime = micros();
    if (mcfg.mixerConfiguration == MULTITYPE_GIMBAL)
        calibratingA = CALIBRATING_ACC_CYCLES;
    calibratingG = CALIBRATING_GYRO_CYCLES;
    calibratingB = CALIBRATING_BARO_CYCLES;             // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
    f.SMALL_ANGLE = 1;

    while (1) {
        loop();
    }

}

void HardFault_Handler(void)
{
// fall out of the sky
    writeAllMotors(mcfg.mincommand);
    LED0_ON
    BEEP_ON
    while (1)
        ; // Keep buzzer on
}

