#ifndef _MPU6500_H_
#define _MPU6500_H_

#include "mpu6500_regs.h"
#include "main.h"

//twi helper functions
//I2C Register Read-Write

uint8_t twiSendByte(uint8_t addr, uint8_t byte);
uint8_t twiSendBytes(uint8_t addr, uint8_t* buf, uint16_t len);
uint8_t twiRecvByte(uint8_t addr);
uint16_t twiRecvBytes(uint8_t addr, uint8_t* buf, uint16_t len);
uint8_t twiWriteReg8(uint8_t addr, uint8_t reg, uint8_t val);
uint8_t twiWriteRegs8(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* buf);
uint8_t twiReadReg8(uint8_t addr, uint8_t reg, uint8_t* buf);
uint8_t twiReadRegs8(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* buf);

// ##########################
// mpu6500 specific

void initMPU6500(I2C_HandleTypeDef* i2ch);

uint8_t imuCheckPresence();
uint8_t imuSelfTest();
void imuSetOffsets();
float imuGetTemp();
uint64_t imuGetAccelData();
uint64_t imuGetGyroData();


// ########################
// privates

uint8_t imuEvalSelfTest(float low, float high, float value, const char* string);


#endif //_MPU6500_H_
