/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*
 * # Instrucciones y breve descripcion
 *
 * La biblioteca desarrollada para controlar la mpu esta en
 * "./Drivers/mpu6500". Ahi esta el driver que desarrollé bajo
 * el marco de arduino adaptado para STM32. Deberia funcionar.
 *
 * El procedimiento antes de hacer nada con la IMU es el siguiente:
 *
 * 1 - Llamar a initMPU6500 con el handler de i2c correspondiente
 * al puerto i2c a usar. (OJO! hacerlo DESPUES DE EL HAL_I2C_INIT)
 *
 * 2 - Checkear la "presencia" de la imu usando imuCheckPresence();
 *
 * 3 - Setear los offsets de gyros y acelerometros con imuSetOffsets();
 *
 * 4 - Correr el self-test con imuSelfTest() y verificar si pasa o no
 * pasan los valores (ver la funcion en el .c para una descripcion
 * mas detallada de los valores de retorno).
 *
 * 5 - Ya esta lista para usar:
 *
 * Temperatura en grados centigrados:
 * float imuGetTemp();
 *
 * Datos de acelerometro (IIXXYYZZ, cada letra es un byte. las I
 * son de "ignorado"):
 * uint64_t imuGetAccelData();
 *
 * Datos de giroscopo (IIXXYYZZ, cada letra es un byte. las I
 * son de "ignorado"):
 * uint64_t imuGetGyroData();
 *
 * PD: Los return de las funciones de acelerometro y giroscopo
 * los hice con un uint64 para ahorrarme de pasar un buffer por
 * referencia a la funcion, pero puede modificarse la biblioteca
 * para que funcione de esa otra forma.
 *
 * PD2: El archivo .c del driver tiene comentarios en ciertas funciones
 * y procesos que pueden ayudar a entender un poco mejor.
 *
 */




/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mpu6500.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "arm_math.h"
#include "arm_const_structs.h"
#include "timer.h"
#include "funciones.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;



/* USER CODE BEGIN PV */
uint64_t accel;
uint64_t gyros;

float dt=0.002;

float G = 9.81;
float Ca = 0.1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float acc_x;
float acc_y;
float acc_z;
float gir_x;
float gir_y;
float gir_z;
float ax[10];
float ay[10];
float az[10];
float gx[10];
float gy[10];
float gz[10];
float amx;
float gmx;
float amy;
float gmy;
float amz;
float gmz;
float vax;
float vgx;
float vay;
float vgy;
float vaz;
float vgz;

float escala_acc = 4.0*9.81/65536.0;
float escala_gir = 500.0/65536.0;

arm_matrix_instance_f32 I;
float identidad [9] = {1,0,0,0,1,0,0,0,1};//vector de datos de la matriz identidad
float ceros [9] = {0,0,0,0,0,0,0,0,0};//vector nulo



//de la tarea

arm_matrix_instance_f32 ap,Ea;
arm_matrix_instance_f32 Znn,Eg;

float a_p[3]; //aceleracion externa pasada
float cov_a[3];//covarianza del acelerometro
float cov_g[3];//covarianza giroscopo
float yg[3];
float znn[3];


void Calculo_F(float yg[], arm_matrix_instance_f32* F)
{

	arm_matrix_instance_f32 aux,Ygx;
	float ygx[9];
	float ax[9];

	arm_mat_init_f32(&aux, 3, 3, ax);//auxialiar para ir guardando operaciones



	ygx[0]=0;
	ygx[1]=-yg[2];
	ygx[2]=yg[1];
	ygx[3]=yg[2];
	ygx[4]=0;
	ygx[5]=-yg[0];
	ygx[6]=-yg[1];
	ygx[7]=yg[0];
	ygx[8]=0;

	arm_mat_init_f32(&Ygx, 3, 3, ygx);

	arm_mat_scale_f32(&Ygx, (-dt), &aux);//primera parte de la ecuacion 24 (-delta_t*[yg]x)

	arm_mat_add_f32(&I, &aux, F);//segunda parte de ecuacion 24 I-aux
}

void Calculo_Q(float znn[], arm_matrix_instance_f32* Q,arm_matrix_instance_f32* Eg )
{

	float d_zx[9];//covarianza del giroscopo d_zx sera donde guarde guarde los datos de la matriz producto vectorial de znn
	float kq;
	float ax_1[9];
	float ax_2[9];
	arm_matrix_instance_f32 aux_1,aux_2,Zx;

	kq=(-dt)*dt;//escalar que se utiliza luego en el calculo de Q

	arm_mat_init_f32(&aux_1, 3, 3, ax_1);
	arm_mat_init_f32(&aux_2, 3, 3, ax_2);


	d_zx[0]=0;
	d_zx[1]=-znn[2];
	d_zx[2]=znn[1];
	d_zx[3]=znn[2];
	d_zx[4]=0;
	d_zx[5]=-znn[0];
	d_zx[6]=-znn[1];
	d_zx[7]=znn[0];
	d_zx[8]=0;

	arm_mat_init_f32(&Zx, 3, 3, d_zx);//matriz producto vectorial de Znn

	arm_mat_mult_f32(Eg,&Zx,&aux_1);//primer producto vectorial de ec 26 (Eg.[Znn]x)
	arm_mat_mult_f32(&Zx,&aux_1,&aux_2);//segundo producto vectorial de ec26
	arm_mat_scale_f32(&aux_2, kq, Q);//escalamiento final de Q ec 26


}

void Calculo_H(arm_matrix_instance_f32* H)
{
	arm_mat_scale_f32(&I, G, H);// H = I*g ecuacion 31
}

void Calculo_Rn(arm_matrix_instance_f32* Ea, arm_matrix_instance_f32* Rn, float a_p[])
{
	arm_matrix_instance_f32 Ee;
	float d_Ee[9];
	float norma_ap;
	float ca=0.1;
	float k=0;
	float a=0.333333333;
	arm_mat_init_f32(&Ee, 3, 3, d_Ee);//solo primea vez

	norma_ap= a_p[0]*a_p[0] + a_p[1]*a_p[1] + a_p[2]*a_p[2];

	k = norma_ap*ca*ca;//calculo del escalar que multiplica a identidad en ecuacion 35
	k=k*a;
	arm_mat_scale_f32(&I, k, &Ee);//Ee = I*k ecuacion 35

	arm_mat_add_f32(&Ee, Ea, Rn); //ecuacion 33
}

//chequear pasajes de argumentos
void Calculo_matriz_Rb(arm_matrix_instance_f32* R_b_g, float *zn)
{
	//inicio ecuacion 37
	arm_matrix_instance_f32 R_aux1,R_aux2; //R que depende de beta y gamma
	float r_aux1[9]={0,0,0,0,1,0,0,0,0},r_aux2[9]={1,0,0,0,0,0,0,0,0},cos_b=0,sen_b=0,cos_g=0,sen_g=0;
	double gamma=0,beta = 0;

	arm_mat_init_f32(&R_aux1, 3, 3, r_aux1);
	arm_mat_init_f32(&R_aux2, 3, 3, r_aux2);

	//chequear que no se pasen los valores mucho de uno

	if (zn[0]>1.0 && zn[0]-1.0<0.09)
	{
		zn[0]=1.0;
	}

	if (zn[0]<-1.0 && zn[0]+1.0>-0.09)
	{
		zn[0]=-1.0;
	}

	if (zn[1]>1.0 && zn[1]-1.0<0.09)
	{
		zn[1]=1.0;
	}

	if (zn[1]<-1.0 && zn[1]+1.0>-0.09)
	{
		zn[1]=-1.0;
	}

	if (zn[2]>1.0 && zn[2]-1.0<0.09)
	{
		zn[2]=1.0;
	}

	if (zn[2]<-1.0 && zn[2]+1.0>-0.09)
	{
		zn[2]=-1.0;
	}


	//calculo beta y gamma
	gamma = atan((double) zn[1]/zn[2]);

	sen_g= sin(gamma);
	cos_g= cos(gamma);

	beta = atan(zn[0]*sen_g/zn[2]);

	//para que solo se utilize una ves las funciones matematicas las cargo en variables.
	cos_b= cos(beta);
	sen_b= sin(beta);
	//sen_g= sin(gamma);
	//cos_g= cos(gamma);


	//cargo los valores de la 1ra matriz auxiliar
	r_aux1[0] = (float)cos_b;
	r_aux1[2] = (float)sen_b;
	r_aux1[6] = (float)-sen_b;
	r_aux1[8] = (float)cos_b;

	//cargo los valores de la 2da matriz auxiliar
	r_aux2[4] = (float)cos_g;
	r_aux2[5] = (float)-sen_g;
	r_aux2[7] = (float)sen_g;
	r_aux2[8] = (float)cos_g;

	//arm_mat_mult_f32(R_b_g, &R_aux1, &R_aux2);//calculo la matriz de rotacion
	arm_mat_mult_f32(&R_aux1, &R_aux1, R_b_g);
	//fin ecuacion 37
}

void Calculo_Xnn(arm_matrix_instance_f32 * Xnn, float * h_pasado, float iaz)//xnn tiene 2 posiciones, iaz es el valor devuelto por aceleracion vertical
{
	//inicio ecuacion 39 --> calculo del Xnn (vector de estado del sistema)
	arm_matrix_instance_f32 Xnp,Mat_aux,T_aux1;
	static float xnp[2]={0,0},aux = 0;//Xn;n-1
	static float h_dif = 0;
	float t_aux2[2]={0.5*dt*dt,dt},t_aux1[4]={1,dt,0,1},mat_aux[2]={0,0};

	arm_mat_init_f32(&T_aux1, 2, 2, t_aux1);

	//estas definiciones no se usan, pero ya dejo incializadas las matrices.
	arm_mat_init_f32(&Xnp, 2, 1, xnp);
	arm_mat_init_f32(&Mat_aux, 2, 1, mat_aux);
	//1ra parte de la suma
	//arm_mat_mult_f32(&Mat_aux, &T_aux1, &Xnp);
	arm_mat_mult_f32(&T_aux1, &Xnp, &Mat_aux);

	//calculo final --> suma de matrices 2x1
	Xnn->pData[0]= mat_aux[0]+ t_aux2[0] * iaz;
	Xnn->pData[1]= mat_aux[1]+ t_aux2[1] * iaz;

	//guardo el valor actual (proximo pasado)
	if(aux == 0)
	{
		start_timer_4();
		aux = 1;
	}
	if(valor_timer_4() >= T_MAX_H)
	{
		stop_timer_4();
		h_dif = Xnn->pData[0];
		start_timer_4();
	}
	xnp[0] = Xnn->pData[0];
	xnp[1] = Xnn->pData[1];

	*h_pasado = h_dif;//devuelve el valor pasado
	//fin ecuacion 39
}

void fil_kal(float* yg, float* ya, arm_matrix_instance_f32* Ap, arm_matrix_instance_f32* Xaa, arm_matrix_instance_f32* Xap, arm_matrix_instance_f32* Paa, arm_matrix_instance_f32* Pap, uint16_t primero, float cov_a[],float cov_g[])
{

	//matrices
	arm_matrix_instance_f32 F;
	arm_matrix_instance_f32 Q;
	arm_matrix_instance_f32 H;
	arm_matrix_instance_f32 Rn;
	arm_matrix_instance_f32 Ea;
	arm_matrix_instance_f32 Eg;
	arm_matrix_instance_f32 a;
	arm_matrix_instance_f32 b;
	arm_matrix_instance_f32 c;
	arm_matrix_instance_f32 d;
	arm_matrix_instance_f32 e;
	arm_matrix_instance_f32 ax;//unas deben ser 3x3 los otros 3x1 (c y d)
	arm_matrix_instance_f32 Ht;
	arm_matrix_instance_f32 Zn;
	arm_matrix_instance_f32 Kn;
	arm_matrix_instance_f32 Ya;
	//arm_matrix_instance_f32 Ap;
	//acoplado
	arm_matrix_instance_f32 R_b_g,Xnn;
	static float  h_pasado = 0;
	float iaz = 0;

	float h[9], g[9], f[9], q[9], A[9], B[9], E[9], AX[9], C[3], D[3],ht[9],zn[3],kn[9];
	float xnn[2]={0,0},r_b_g[9]={0,0,0,0,0,0,0,0,0};

	arm_mat_init_f32(&I, 3, 3,identidad);


	arm_mat_init_f32(&H, 3, 3, h);
	arm_mat_init_f32(&F, 3, 3, f);
	arm_mat_init_f32(&Rn, 3, 3, g);
	arm_mat_init_f32(&Q, 3, 3, q);
	arm_mat_init_f32(&Ea, 3, 3, cov_a);
	arm_mat_init_f32(&Eg, 3, 3, cov_g);
	arm_mat_init_f32(&Ya, 3, 1, ya);
	arm_mat_init_f32(&a, 3, 3, A);
	arm_mat_init_f32(&b, 3, 3, B);
	arm_mat_init_f32(&e, 3, 3, E);
	arm_mat_init_f32(&ax, 3, 3, AX);
	arm_mat_init_f32(&c, 3, 1, C);
	arm_mat_init_f32(&d, 3, 1, D);
	arm_mat_init_f32(&Ht, 3, 3, ht);
	arm_mat_init_f32(&Zn, 3, 1, zn);
	arm_mat_init_f32(&Kn, 3, 3, kn);
	//arm_mat_init_f32(&Ap, 3, 1, a_p);
	//acoplado
	arm_mat_init_f32(&Xnn, 2, 1, xnn);
	arm_mat_init_f32(&R_b_g, 3, 3, r_b_g);

	if (primero==0)
	{
		Calculo_H(&H);

		Calculo_Rn(&Ea, &Rn, Ap->pData);


		//Kn = P_a_p.Ht.(H.P_a_p.Ht + Rn)-1

		arm_mat_trans_f32(&H, &Ht);
		arm_mat_mult_f32(Pap, &Ht, &b);
		arm_mat_mult_f32(&H, &b, &a);
		arm_mat_add_f32(&a, &Rn, &b);
		arm_mat_inverse_f32(&b, &a);
		arm_mat_mult_f32(&Ht, &a, &b);
		arm_mat_mult_f32(Pap,&b,&Kn);

		//zn = ya -Ca*(a_p_p)

		arm_mat_scale_f32(Ap, -Ca, &c);
		//arm_mat_scale_f32(&Ya, G, &d);
		arm_mat_add_f32(&Ya, &c, &Zn);

		//x_a_a = x_a_p + Kn.(zn - H.x_a_p)

		arm_mat_mult_f32(&H, Xap, &c);
		arm_mat_scale_f32(&c, -1, &d);
		arm_mat_add_f32(&Zn, &d, &c);
		arm_mat_mult_f32(&Kn, &c, &d);
		arm_mat_add_f32(Xap, &d, Xaa);


		//P_a_a = (I - Kn.H).P_a_p(I - KnH)t + Kn.Rn.Knt

		arm_mat_trans_f32(&Kn, &a);
		arm_mat_mult_f32(&Rn, &a, &b);
		arm_mat_mult_f32(&Kn, &b, &e);

		arm_mat_mult_f32(&Kn, &H, &a);
		arm_mat_scale_f32(&a, -1, &b);
		arm_mat_add_f32(&I, &b, &ax);


		arm_mat_mult_f32(&Kn, &H, &a);
		arm_mat_scale_f32(&a, -1, &b);
		arm_mat_add_f32(&I, &b, &a);
		arm_mat_trans_f32(&a, &b);

		arm_mat_mult_f32(Pap, &b, &a);
		arm_mat_mult_f32(&ax, &a, &b);

		arm_mat_add_f32(&b, &e, Paa);

		//aceleracion externa
		arm_mat_scale_f32(Xaa, -G, &c);
		arm_mat_add_f32(&Ya, &c, Ap);

		}



	Calculo_F(yg, &F);


	Calculo_Q(Xaa->pData, &Q, &Eg);
	//x_f_a = F.X_a_a    En la siguiente sera x_a_p
	arm_mat_mult_f32(&F, Xaa, Xap); //esto es 3x1

	//P_f_a = F.P_a_a.Ft + Q

	arm_mat_trans_f32(&F, &a);
	arm_mat_mult_f32(Paa, &a, &b);
	arm_mat_mult_f32(&F, &b, &a);
	arm_mat_add_f32(&a, &Q, Pap);//chequear ordenes de matrices me parece que p tendria que ser 3x3
	//ojo que estos valores deben seguir por fuera de la funcion


	//acoplado
	Calculo_matriz_Rb(&R_b_g, Xaa->pData);
	iaz = Aceleracion_vertical(Xaa->pData,ya,&R_b_g);
	Calculo_Xnn(&Xnn, &h_pasado, iaz);
	Detector_caida(xnn,h_pasado,Calculo_theta(Xaa->pData),iaz);//maquina de estados


//primera pasada no pasan por aca ni las de arriba



// aceleracion externa

/*arm_mat_scale_f32(Xaa, -G, &c);
arm_mat_add_f32(&Ya, &c, Ap);*/
//arm_mat_add_f32(&Ya, &c, &d);

/*arm_mat_scale_f32(Xaa, -1, &c);
arm_mat_add_f32(&Ya, &c, &d);
arm_mat_scale_f32(&d, G, Ap);*/


}







void getData() {
	accel = imuGetAccelData();
	gyros = imuGetGyroData();

	/*acc_z = escala_acc*(int16_t)(accel & 0x0000000000000000FFFF);
	gir_z = escala_gir*(int16_t)(gyros & 0x0000000000000000FFFF);

	acc_y = escala_acc*(int16_t)((accel >> 16) & 0x0000000000000000FFFF);
	gir_y = escala_gir*(int16_t)((gyros >> 16) & 0x0000000000000000FFFF);

	acc_x = escala_acc*(int16_t)((accel >> 32) & 0x0000000000000000FFFF);
	gir_x = escala_gir*(int16_t)((gyros >> 32) & 0x0000000000000000FFFF);*/
	acc_z = escala_acc*(int16_t)(accel );
	gir_z = escala_gir*(int16_t)(gyros);

	acc_y = escala_acc*(int16_t)((accel >> 16));
	gir_y = escala_gir*(int16_t)((gyros >> 16));

	acc_x = escala_acc*(int16_t)((accel >> 32));
	gir_x = escala_gir*(int16_t)((gyros >> 32));

}

void buttonLoop() {
	static uint8_t debounce = 0;
	static uint8_t status = 0;
	static uint32_t tick = 0;

	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != status) {
		if (HAL_GetTick() - tick > 150) {
			tick = HAL_GetTick();
			debounce++;
		}
		if (debounce > 3) {
			debounce = 0;
			tick = HAL_GetTick();
			status = !status;
			if (status) {
				getData();
			}
		}
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  uint8_t ret = 0;
  ret++;

  initMPU6500(&hi2c1);
  ret = imuCheckPresence();
  imuSetOffsets();
  ret = imuSelfTest();


int i;


for (i=0;i<10;i++)
{
	getData();

	az[i] = acc_z;
	gz[i] = gir_z;

	ay[i] = acc_y;
	gy[i] = gir_y;

	ax[i] = acc_x;
	gx[i] = gir_x;

}


amx=0;
gmx=0;

amy=0;
gmy=0;

amz=0;
gmz=0;

for (i=0;i<10;i++)
{
	amx = amx + ax[i]/10;
	gmx = gmx + gx[i]/10;

	amy = amy + ay[i]/10;
	gmy = gmy + gy[i]/10;

	amz = amz + az[i]/10;
	gmz = gmz + gz[i]/10;

}



vax=0;
vgx=0;

vay=0;
vgy=0;

vaz=0;
vgz=0;

for (i=0;i<10;i++)
{

	vax = (ax[i]-amx)*(ax[i]-amx)/10;
	vgx = (gx[i]-gmx)*(gx[i]-gmx)/10;

	vay = (ay[i]-amy)*(ay[i]-amy)/10;
	vgy = (gy[i]-gmy)*(gy[i]-gmy)/10;

	vaz = (az[i]-amz)*(az[i]-amz)/10;
	vgz = (gz[i]-gmz)*(gz[i]-gmz)/10;


}






//int tarea_filtro (void)
//{
	arm_matrix_instance_f32 Ap, Ea,Eg,Xaa,Paa,Xap,Pap;


	float a_p[3]; //aceleracion externa pasada
	float cov_a[] = {0,0,0,0,0,0,0,0,0};//covarianza del acelerometro
	float cov_g[]= {0,0,0,0,0,0,0,0,0}  ;//covarianza giroscopo
	float yg[3];
	float ya[3];
	float xaa[3];
	float xap[3];
	float paa[9];
	float pap[9]; //chequear valores iniciales //covariazas que arranquen en 0

	a_p[0]=0;
	a_p[1]=0;
	a_p[2]=0;

	xaa[0] = 0;
	xaa[1] = 0;
	xaa[2] = 1;
	for (i=0;i<9;i++)
	{
		if (i==0)
		{
			cov_a[i] = vax;


			cov_g[i] = vgx;


		}else if (i==4)
		{
			cov_a[i] = vay;
			cov_g[i] = vgy;


		}else if(i==8)
		{
			cov_a[i] = vaz;
			cov_g[i] = vgz;

		}else
		{
			cov_a[i] = 0;
			cov_g[i] = 0;


		}
	}


	/*cov_a[0] = 0.001;
	cov_a[4] = 0.001;
	cov_a[8] = 0.001;

	cov_g[0] = 0.1;
	cov_g[4] = 0.1;
	cov_g[8] = 0.1;*/



	for ( i=0; i<9;i++)
	{
		paa[i] = 0;
	}

	arm_mat_init_f32(&Ea, 3, 3, cov_a);
	arm_mat_init_f32(&Eg, 3, 3, cov_g);
	arm_mat_init_f32(&Xaa, 3, 1, xaa);
	arm_mat_init_f32(&Xap, 3, 1, xap);
	arm_mat_init_f32(&Paa, 3, 3, paa);
	arm_mat_init_f32(&Pap, 3, 3, pap);
	arm_mat_init_f32(&Ap, 3, 1, a_p);

	uint16_t primero=1;


	while(1)
	{
		//recibo datos de una cola

		getData();

		yg[0] = gir_x;
		yg[1] = gir_y;
		yg[2] = gir_z;

		ya[0] = acc_x;
		ya[1] = acc_y;
		ya[2] = acc_z;



		fil_kal(yg, ya, &Ap, &Xaa, &Xap, &Paa, &Pap, primero, cov_a, cov_g);

		primero = 0;



		//deteccion caida libre pense que devuelva un 1 si hay y un 0 sino.
		//lo unico que habria que pasarle es el vector con Xaa y los datos anteriores de altura y velocidad (ver como mantener estos datos porque si solo estan la funcion mueren cuando termina
		//aplicar ecuaciones 15 y 16
		//luego 37 38
		//por ultimo 39 y 40
		//luego hay que empezar las comparaciones del cuadro para que ver si hay caida libre o no
	}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  //buttonLoop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
