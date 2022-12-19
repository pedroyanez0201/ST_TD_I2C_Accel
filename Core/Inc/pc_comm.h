#ifndef INC_PC_COMM_H_
#define INC_PC_COMM_H_

#include "main.h"

typedef union {
	float f;
	uint32_t ul;
} raw_t;


void vPcRead (float* dt, float* gir_x, float*gir_y, float* gir_z, float* acc_x, float* acc_y, float* acc_z);
void vPcWrite (float value);


#endif /* INC_PC_COMM_H_ */
