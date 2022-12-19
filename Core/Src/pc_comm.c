#include "pc_comm.h"

void vPcRead (float* dt, float* gir_x, float*gir_y, float* gir_z, float* acc_x, float* acc_y, float* acc_z) {
	uint8_t pucBuffer [sizeof(float)*7];
	raw_t xBuffer[7];
	uint8_t i = 0;

	HAL_UART_Receive(&huart2, pucBuffer, sizeof(pucBuffer), HAL_MAX_DELAY);

	for (i=0; i<7; i++) {
		xBuffer[i].ul = ( pucBuffer[0 + i*sizeof(float)] << 24) |
				(pucBuffer[1 + i*sizeof(float)] << 16) |
				(pucBuffer[2 + i*sizeof(float)] << 8) |
				pucBuffer[3 +i*sizeof(float)];
	}
	*dt = xBuffer[0].f;
	*acc_x = xBuffer[1].f;
	*acc_y = xBuffer[2].f;
	*acc_z = xBuffer[3].f;
	*gir_x = xBuffer[4].f;
	*gir_y = xBuffer[5].f;
	*gir_z = xBuffer[6].f;
}

void vPcWrite (float value) {
	raw_t xBuffer;
	uint8_t pucBuffer [sizeof(float)];
	xBuffer.f = value;

	pucBuffer[0] = (xBuffer.ul >> 24) 	&	0xFF;
	pucBuffer[1] = (xBuffer.ul >> 16) 	& 	0xFF;
	pucBuffer[2] = (xBuffer.ul >> 8)	& 	0xFF;
	pucBuffer[3] = xBuffer.ul 			& 	0xFF;

	HAL_UART_Transmit(&huart2, pucBuffer, sizeof(pucBuffer), HAL_MAX_DELAY);

}
