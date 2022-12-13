/*
 * funciones.h
 *
 *  Created on: 1 dic. 2022
 *      Author: franb
 */

#ifndef INC_FUNCIONES_H_
#define INC_FUNCIONES_H_

#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "timer.h"

#define Ivth 	 			-1.2
#define Iazth 	 			11.6
#define d_Hth 	 			-0.31
#define Ivavg_th 			-0.4
#define theta_g_th			20
#define Tmax_aceleracion 	800000 //useg
#define	Tmax_dh				5000000//useg
#define Tmax_Ivavg			500000 //useg
#define Tmax_thetag			3000000 //useg
#define T_MAX_H				5000000
#define TRUE 				1
#define FALSE 				0
#define RESET  				1
#define ON 					0

enum{
	velocidad = 0,
	aceleracion,
	altura,
	velocidad_avg,
	theta_g
};

float Aceleracion_vertical(float * zn, float * ya, arm_matrix_instance_f32 * R_b_g);
float Calculo_theta(float * zn);
uint32_t Promedio_Vel(float *xnn,uint8_t flag);
void Detector_caida(float *xnn,float h_pasado, float theta_gravedad, float ian);

#endif /* INC_FUNCIONES_H_ */
