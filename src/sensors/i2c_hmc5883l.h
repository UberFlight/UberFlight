#pragma once

bool hmc5883lDetectI2c(sensor_t *mag);
void hmc5883lInit(sensor_align_e align);
bool hmc5883lRead(int16_t *magData);
