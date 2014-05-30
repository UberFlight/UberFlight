/*
 * telemetry_common.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#ifndef TELEMETRY_COMMON_H_
#define TELEMETRY_COMMON_H_

// telemetry
void initTelemetry(USART_TypeDef *USARTx);
void checkTelemetryState(void);
void handleTelemetry(void);

#endif /* TELEMETRY_COMMON_H_ */
