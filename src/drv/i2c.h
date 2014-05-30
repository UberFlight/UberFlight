#pragma once

void i2cInit();
bool i2cWriteBuffer(I2C_TypeDef* I2Cx, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(I2C_TypeDef* I2Cx, uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cRead(I2C_TypeDef* I2Cx, uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
uint16_t i2cGetErrorCounter(I2C_TypeDef* I2Cx);
