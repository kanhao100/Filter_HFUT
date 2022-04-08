/*
 * Filter_HFUT.h
 *
 *  Created on: 2022年3月17日
 *      Author: WeijuWU
 */

#ifndef CODE_FILTER_HFUT_H_
#define CODE_FILTER_HFUT_H_


#define Sliding_Windows_Filter_Num 5//窗口所存的数据量
#define Sliding_Windows_Filter 1
//修改宏定义以编译指定部分代码,即使用指定滤波
//#define Complementary_filter 1
#define Kalman_filter 1 //依赖于互补滤波
//#define Mahony_6_filter 1
//#define Madgwick_6_filter 1



#if Sliding_Windows_Filter
float Sliding_Windows_Update(float data,int N);
#endif

float RC_Filter_update(float data, float f_s, float dt);

#if Complementary_filter || Kalman_filter

void Gyro_Raw_Update(float gx, float gy, float gz, float ax, float ay, float az,float dt, float f_s);
void Gyro_Complementary_Update(float gx, float gy, float gz,float ax, float ay, float az,float Ts, int f_s);
float *GetGryo_Integral(void);
float *GetGryo_Acc(void);
float *GetGryo_Acc_RC(void);
float *GetGryo_Complementary(void);

#endif

#if Kalman_filter

void Kalman_Init(float Q_a, float Q_g, float R_a);
void Kalman_Filter_Update_Pitch(float gx, float gy, float gz, int samplerate);
void Kalman_Filter_Update_Roll(float gx, float gy, float gz, int samplerate);
float *GetGryo_Kalman(void);
#endif

#if Mahony_6_filter

extern float q0, q1, q2, q3;      //四元数
void Mahony_Init(float loop_ms);
void Mahony_Imu_Update(float gx, float gy, float gz, float ax, float ay, float az);
float *GetGryo_Mahony(void);

#endif



#if Madgwick_6_filter

extern float q0, q1, q2, q3;     //四元数
void Madgwick_Init(float loop_ms);
float invSqrt(float x);
void MadgwickAHRSupdate_6(float gx, float gy, float gz, float ax, float ay, float az);
float *GetGryo_Madgwick(void);

#endif

float First_Derivative (float data, float dt);
float Second_Derivative (float data, float dt);


#endif /* CODE_FILTER_HFUT_H_ */
