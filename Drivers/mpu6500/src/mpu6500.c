#include "mpu6500.h"

const uint16_t mpu6500StTb[256] = {
  2620,2646,2672,2699,2726,2753,2781,2808, //7
  2837,2865,2894,2923,2952,2981,3011,3041, //15
  3072,3102,3133,3165,3196,3228,3261,3293, //23
  3326,3359,3393,3427,3461,3496,3531,3566, //31
  3602,3638,3674,3711,3748,3786,3823,3862, //39
  3900,3939,3979,4019,4059,4099,4140,4182, //47
  4224,4266,4308,4352,4395,4439,4483,4528, //55
  4574,4619,4665,4712,4759,4807,4855,4903, //63
  4953,5002,5052,5103,5154,5205,5257,5310, //71
  5363,5417,5471,5525,5581,5636,5693,5750, //79
  5807,5865,5924,5983,6043,6104,6165,6226, //87
  6289,6351,6415,6479,6544,6609,6675,6742, //95
  6810,6878,6946,7016,7086,7157,7229,7301, //103
  7374,7448,7522,7597,7673,7750,7828,7906, //111
  7985,8065,8145,8227,8309,8392,8476,8561, //119
  8647,8733,8820,8909,8998,9088,9178,9270,
  9363,9457,9551,9647,9743,9841,9939,10038,
  10139,10240,10343,10446,10550,10656,10763,10870,
  10979,11089,11200,11312,11425,11539,11654,11771,
  11889,12008,12128,12249,12371,12495,12620,12746,
  12874,13002,13132,13264,13396,13530,13666,13802,
  13940,14080,14221,14363,14506,14652,14798,14946,
  15096,15247,15399,15553,15709,15866,16024,16184,
  16346,16510,16675,16842,17010,17180,17352,17526,
  17701,17878,18057,18237,18420,18604,18790,18978,
  19167,19359,19553,19748,19946,20145,20347,20550,
  20756,20963,21173,21385,21598,21814,22033,22253,
  22475,22700,22927,23156,23388,23622,23858,24097,
  24338,24581,24827,25075,25326,25579,25835,26093,
  26354,26618,26884,27153,27424,27699,27976,28255,
  28538,28823,29112,29403,29697,29994,30294,30597,
  30903,31212,31524,31839,32157,32479,32804,33132
};

// necesito una global donde se configure el puerto a usar;
static I2C_HandleTypeDef* privateI2CHandler = NULL;

//necesario para que esta biblioteca tome nocion
//del I2C_HandleTypeDef que se esta usando.
void initMPU6500(I2C_HandleTypeDef* i2ch) {
	privateI2CHandler = i2ch;
}

//Manda un byte a una direccion I2C
uint8_t twiSendByte(uint8_t addr, uint8_t byte) {

	if (privateI2CHandler == NULL) return 1;
	addr = addr << 1;
	return (uint8_t)HAL_I2C_Master_Transmit(privateI2CHandler, addr, &byte, sizeof(uint8_t), HAL_MAX_DELAY);

}

//Manda una serie de bytes a una direccion I2C
uint8_t twiSendBytes(uint8_t addr, uint8_t* buf, uint16_t len) {

	if (privateI2CHandler == NULL) return 1;
	addr = addr << 1;
	return (uint8_t)HAL_I2C_Master_Transmit(privateI2CHandler, addr, buf, len, HAL_MAX_DELAY);

}

//Recibe un byte de una dirección de I2C
uint8_t twiRecvByte(uint8_t addr) {

	uint8_t buf = 0;
	if (privateI2CHandler == NULL) return buf;
	addr = addr << 1;
	HAL_I2C_Master_Receive(privateI2CHandler, addr, &buf, sizeof(uint8_t), HAL_MAX_DELAY);
	return buf;

}

//Recibe varios bytes de una dirección I2C
uint16_t twiRecvBytes(uint8_t addr, uint8_t* buf, uint16_t len) {

	if (privateI2CHandler == NULL) return 1;
	addr = addr << 1;
	HAL_I2C_Master_Receive(privateI2CHandler, addr, buf, len, HAL_MAX_DELAY);
	//supuestamente la interface bloquea hasta no recibir len bytes;
	//no es como la de arduino que puede devolver de menos
	return len;

}

//Escribe un registro de 8 bits en un dispo I2C
uint8_t twiWriteReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = val;
  return twiSendBytes(addr, buf, 2);
}

//Escribe una serie de registros contiguos en un dispo I2C
uint8_t twiWriteRegs8(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* buf) {

	uint8_t auxBuf[len + 1];

	for (uint32_t i = 0 ; i < len; i++) auxBuf[i+1] = buf[i];
	auxBuf[0] = reg;

	if (privateI2CHandler == NULL) return 1;
	addr = addr << 1;
	return (uint8_t)HAL_I2C_Master_Transmit(privateI2CHandler, addr, auxBuf, len + 1, HAL_MAX_DELAY);

}

//Lee el valor de un registro de 8 bits en un dispo I2C
uint8_t twiReadReg8(uint8_t addr, uint8_t reg, uint8_t* buf) {

	uint8_t ret;
	ret = twiSendByte(addr, reg);
	if (ret) return ret;
	*buf = twiRecvByte(addr);
	return 0;

}

//Lee el valor de varios registros contiguos de un dispo I2C
uint8_t twiReadRegs8(uint8_t addr, uint8_t reg, uint16_t len, uint8_t* buf) {

  uint8_t ret;
  ret = twiSendByte(addr, reg);
  if (ret) return ret;
  if (len == twiRecvBytes(addr, buf, len)) return 0;
  return (uint8_t)(-1);

}

//Verifica la presencia de un sensor MPU6500
//Leyendo el registro Who am I y verificando que sea igual a 0x70.
uint8_t imuCheckPresence() {
  uint8_t ret = 0, val = 0;
  ret = twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_WHO_AM_I, &val);
  if (ret != 0 || val != MPU6500_WHO_AM_I_BYTE) return 0;
  return 1;
}

//Comprueba los resultados de los self-test
uint8_t imuEvalSelfTest(float low, float high, float value, const char* string) {
  
  uint8_t pass = 0;
  
  // gyros only have lower limit
  // this is a hack to use same evaluation function
  if (low > high) {
    if (value >= low) pass = 1;
  } else {
    if (value >= low || value <= high) pass = 1;
  }

//  #ifdef DEBUG_PRINTF
//    Serial.printf("Self test %s %s. low: %0.2f, high: %0.2f, measured: %0.2f\n",
//    string, pass ? "[PASS]" : "[FAIL]" , low, high, value);
//  #endif

  return pass;

}

//Realiza el self-test del sensor
//Retorna un valor que describe el resultado como una combinacion de flags
// 0: todo ok
// 1: no paso el giro X
// 2: no paso el giro Y
// 4: no paso el giro Z
// 8: no paso el accel X
// 16: no paso el accel Y
// 32: no paso el accel Z
uint8_t imuSelfTest() {
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t saveReg[5];
  uint8_t selfTest[6];
  int32_t gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
  uint16_t factoryTrim[6];
  float aDiff[3], gDiff[3];
  int i;

  uint8_t testStatus = 0;

  // Save old configuration
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_SMPLRT_DIV, &saveReg[0]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_CONFIG, &saveReg[1]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, &saveReg[2]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG_2, &saveReg[3]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, &saveReg[4]);
  // Write test configuration
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, MPU6500_GYRO_FS_250); // Set full scale range for the gyro to 250 dps
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, MPU6500_ACCEL_FS_2); // Set full scale range for the accelerometer to 2 g

  for(i = 0; i < 200; i++)
  {
    // get average current values of gyro and acclerometer
    twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
    gAvg[2] += (int16_t)((int16_t)rawData[4] << 8) | rawData[5];
  }

  for (i = 0; i < 3; i++)
  { // Get average of 200 values and store as average current readings
    aAvg[i] /= 200;
    gAvg[i] /= 200;
  }

  // Configure the accelerometer for self-test
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  HAL_Delay(25); // Delay a while to let the device stabilize

  for(i = 0; i < 200; i++)
  {
    // get average self-test values of gyro and acclerometer
    twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (i =0; i < 3; i++)
  { // Get average of 200 values and store as average self-test readings
    aSTAvg[i] /= 200;
    gSTAvg[i] /= 200;
  }

   // Configure the gyro and accelerometer for normal operation
   twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, 0x00);
   twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, 0x00);
   HAL_Delay(25); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ST_X_ACCEL, &selfTest[0]); // X-axis accel self-test results
   twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ST_Y_ACCEL, &selfTest[1]); // Y-axis accel self-test results
   twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ST_Z_ACCEL, &selfTest[2]); // Z-axis accel self-test results
   twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ST_X_GYRO, &selfTest[3]); // X-axis gyro self-test results
   twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ST_Y_GYRO, &selfTest[4]); // Y-axis gyro self-test results
   twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ST_Z_GYRO, &selfTest[5]); // Z-axis gyro self-test results

   for (i = 0; i < 6; i++)
   {
      if (selfTest[i] != 0)
      {
        factoryTrim[i] = *(mpu6500StTb + selfTest[i] - 1);
      }
      else
      {
        factoryTrim[i] = 0;
      }
      // printf("[Self-test] idx #%d, st-code: %d, codeindex: %p, st-otp: %d\n", i, selfTest[i], mpu6500StTb + selfTest[i] - 1, factoryTrim[i]);
    }

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (i = 0; i < 3; i++)
  {
    // aDiff[i] = 100.0f*((float)((aSTAvg[i] - aAvg[i]) - factoryTrim[i]))/factoryTrim[i]; // Report percent differences
    // gDiff[i] = 100.0f*((float)((gSTAvg[i] - gAvg[i]) - factoryTrim[i+3]))/factoryTrim[i+3]; // Report percent differences
    
    aDiff[i] = 100.0f*((float)((aSTAvg[i] - aAvg[i])))/factoryTrim[i]; // Report percent differences
    gDiff[i] = 100.0f*((float)((gSTAvg[i] - gAvg[i])))/factoryTrim[i+3]; // Report percent differences
    
    // printf("a[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, aAvg[i], aSTAvg[i], aSTAvg[i] - aAvg[i], factoryTrim[i], aDiff[i]);
    // printf("g[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, gAvg[i], gSTAvg[i], gSTAvg[i] - gAvg[i], factoryTrim[i+3], gDiff[i]);
  }

  // Restore old configuration
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_SMPLRT_DIV, saveReg[0]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_CONFIG, saveReg[1]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, saveReg[2]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG_2, saveReg[3]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, saveReg[4]);

   // Check result
  testStatus |= imuEvalSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[0], "gyro X") ? 0x00: 0x01;
  testStatus |= imuEvalSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[1], "gyro Y") ? 0x00 : 0x02;
  testStatus |= imuEvalSelfTest(MPU6500_ST_GYRO_LOW, MPU6500_ST_GYRO_HIGH, gDiff[2], "gyro Z") ? 0x00 : 0x04;
  testStatus |= imuEvalSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[0], "acc X") ? 0x00 : 0x08;
  testStatus |= imuEvalSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[1], "acc Y") ? 0x00 : 0x10;
  testStatus |= imuEvalSelfTest(MPU6500_ST_ACCEL_LOW, MPU6500_ST_ACCEL_HIGH, aDiff[2], "acc Z") ? 0x00 : 0x20;

  return testStatus;
}

// Setea los registros de corrección de offsets.
// De acelerometro y giroscopos.
void imuSetOffsets() {
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t saveReg[5];
  uint8_t offsetsA[6] = {}, offsetsG[6] = {};
  int32_t gAvg[3]={0}, aAvg[3]={0};
  int i;

  // Save old configuration
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_SMPLRT_DIV, &saveReg[0]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_CONFIG, &saveReg[1]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, &saveReg[2]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG_2, &saveReg[3]);
  twiReadReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, &saveReg[4]);
  // Write test configuration
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, MPU6500_GYRO_FS_250); // Set full scale range for the gyro to 250 dps
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, MPU6500_ACCEL_FS_2); // Set full scale range for the accelerometer to 2 g

  for(i = 0; i < 200; i++)
  {
    // get average current values of gyro and acclerometer
    twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
    gAvg[2] += (int16_t)((int16_t)rawData[4] << 8) | rawData[5];
  }

  for (i = 0; i < 3; i++)
  { // Get average of 200 values and store as average current readings
    aAvg[i] /= 200;
    gAvg[i] /= 200;

    offsetsA[i*2] = ((aAvg[i] >> 3) & 0x0000ff00) >> 8;
    offsetsA[(i*2) + 1] = (aAvg[i] >> 3) & 0x000000fe;

    offsetsG[i*2] = ((gAvg[i] >> 3) & 0x0000ff00) >> 8;
    offsetsG[(i*2) + 1] = (gAvg[i] >> 3) & 0x000000fe;
  }

  //Write offsets
  twiWriteRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_XG_OFFS_USRH, 6, offsetsG);
  //twiWriteRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_XA_OFFSET_H, 6, offsetsA);

  //Restore registers
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_SMPLRT_DIV, saveReg[0]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_CONFIG, saveReg[1]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_CONFIG, saveReg[2]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG_2, saveReg[3]);
  twiWriteReg8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_CONFIG, saveReg[4]);
  
}

//Retorna la temperatura en grados Celscius del integrado
float imuGetTemp() {
  uint8_t buf[2];
  uint16_t temp;
  twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_TEMP_OUT_H, 2, buf);
  temp = buf[0] << 8 | buf[0];
  return ((float)temp / 333.87f) + 21.0f;
}

//Retorna en un uint64 los datos de los 3 ejes del acelerometro.
//Formato: IIXXYYZZ (donde cada letra es un grupo de 8 bits. I: Ignorados)
uint64_t imuGetAccelData() {
  uint8_t buf[6];
  uint64_t ret = 0;
  twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_ACCEL_XOUT_H, 6, buf);
  for (int i = 0; i < 3; i++) {
	  //Esta operativa me carga en el uint64 los datos de x, y & z.
	  //Es posible que sea mas legible y eficiente si unrolleo el loop;
	  ret |= ((uint64_t)buf[i*2]) << (8 + (2-i) * 16);
	  ret |= ((uint64_t)buf[(i*2)+1]) << (2-i) * 16;

  }
  return ret;
}

//Retorna en un uint64 los datos de los 3 ejes del giroscopo
//Formato: IIXXYYZZ (donde cada letra es un grupo de 8 bits. I: Ignorados)
uint64_t imuGetGyroData() {
  uint8_t buf[6];
  uint64_t ret = 0;
  twiReadRegs8(MPU6500_DEFAULT_ADDRESS, MPU6500_RA_GYRO_XOUT_H, 6, buf);
  for (int i = 0; i < 3; i++) {
	  //Esta operativa me carga en el uint64 los datos de x, y & z.
	  //Es posible que sea mas legible y eficiente si unrolleo el loop;
	  ret |= ((uint64_t)buf[i*2]) << (8 + (2-i) * 16);
	  ret |= ((uint64_t)buf[(i*2)+1]) << (2-i) * 16;
  }
  return ret;
}
