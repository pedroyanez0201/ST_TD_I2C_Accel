/*
 * funciones.c
 *
 *  Created on: 1 dic. 2022
 *      Author: franb
 */

#include "funciones.h"

extern uint8_t dt;

extern float G;
uint8_t flag_detecto_caida;//provisoriamente para compilar

float Aceleracion_vertical(float * zn, float * ya, arm_matrix_instance_f32 * R_b_g)
{
	//inicio ecuacion 38 Ian = aceleracion vertical
	arm_matrix_instance_f32 Ian,Ian_aux;
	float ian[3]={0,0,0},ian_aux[3]={0,0,0};

	arm_mat_init_f32(&Ian, 3, 1, ian);
	arm_mat_init_f32(&Ian_aux, 3, 1, ian_aux);

	ian_aux[0] = ya[0] - G * zn[0];
	ian_aux[1] = ya[1] - G * zn[1];
	ian_aux[2] = ya[2] - G * zn[2];

	//arm_mat_mult_f32(&Ian, R_b_g, &Ian_aux);//obtengo la aceleracion vertical
	arm_mat_mult_f32(R_b_g, &Ian_aux, &Ian);
	//fin ecuacion 38
	return ian[2];
}

float Calculo_theta(float * zn) //ecuacion 44
{
	float aux_zn =zn[2];
	if (aux_zn>1.0 && aux_zn-1.0<0.09)
	{
		aux_zn=1.0;
	}

	if (aux_zn<-1.0 && aux_zn+1.0>-0.09)
	{
		aux_zn=-1.0;
	}

	return (float)acosf(aux_zn);
}

uint32_t Promedio_Vel(float *xnn,uint8_t flag)
{
	//calculo promedio de velocidad
	static uint32_t contador = 1;
	static float Ivavg = 0;


	Ivavg = Ivavg + (xnn[1]-Ivavg)/ contador; //calculo el promedio

	contador++;
	if(flag == 1)
	{
		Ivavg = 0;
		contador = 1;
	}
	//fin del calculo
	return Ivavg;
}

void Detector_caida(float *xnn,float h_pasado, float theta_gravedad, float ian)
{
	//inicio deteccion (podria ser una tarea aparte)

	//maquina de estados
	static uint8_t estado = 0;
	static float Ivavg=0;



	switch (estado) {
		case velocidad:
			if(xnn[1] < Ivth)
			{
				estado = aceleracion;
				start_timer_3();//falta configurar el timer en el .ioc
			}
			break;
		case aceleracion:
			if(valor_timer_3() >= Tmax_aceleracion)
			{
				stop_timer_3();
				estado = velocidad;
			}
			if(ian < Iazth)
			{
				stop_timer_3();
				start_timer_3();
				estado = altura;
			}
			break;
		case altura:
			if(valor_timer_3() >= Tmax_dh)
			{
				estado = velocidad;
				stop_timer_3();
			}
			if((xnn[0]-h_pasado)< d_Hth)
			{
				stop_timer_3();
				start_timer_3();
				estado = 4;
			}
			break;
		case velocidad_avg:
			if(valor_timer_3() >= Tmax_Ivavg)
			{
				estado = velocidad;
				stop_timer_3();
				Ivavg = Promedio_Vel(xnn,Reseteo);
			}
			else
			{
				Ivavg = Promedio_Vel(xnn,ON);
			}
			if( Ivavg < Ivavg_th)
			{
				stop_timer_3();
				start_timer_3();
				estado = theta_g;
			}
			break;
		case theta_g:
			if(valor_timer_3() >= Tmax_thetag)
			{
				estado = velocidad;
				stop_timer_3();
			}
			if( theta_gravedad < theta_g_th)
			{
				stop_timer_3();
				flag_detecto_caida = TRUE;//falta definir esto
				estado = velocidad;
			}
			break;
		default:
			break;
	}

}
