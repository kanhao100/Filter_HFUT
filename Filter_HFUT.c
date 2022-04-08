/*
 * Filter_HFUT.cpp
 *
 *  Created on: 2022��3��17��
 *      Author: WeijuWU
 */

//---------------------------------------------------------------------------------------------------

//TO DO:
//���ٶȼƵı궨
//ʺɽ�����Ż�,�ĳɽṹ����ʽ,��������WARNNING
//���׵�ͨ�˲�
//������������R�Ĳ���
//EKF,UKF

// Header files
#include <math.h>
#include "Filter_HFUT.h"
#include <stdio.h>

#if Complementary_filter || Kalman_filter // �������˲������ڻ����˲�����Ĳ��ֺ��������

#define G 9.7947f  // m/s^2 �Ϸ��������ٶ�
#define PI 3.141592653589793
float Pitch_integral = 0.0,Roll_integral = 0.0,Yaw_integral = 0.0;
float Pitch_acc = 0.0,Roll_acc = 0.0;//Yaw �޷��ü��ٶȼƽ���
float Pitch_acc_RC = 0.0,Roll_acc_RC = 0.0;
float Pitch_Complementary=0.0,Roll_Complementary=0.0,Yaw_Complementary=0.0;//Yawֻ��Դ�������ǻ���
float Pitch_Complementary_last=0.0,Roll_Complementary_last=0.0,Yaw_Complementary_last=0.0;
float norm_acc;
float tao;


//SHIT CODE WARNING!!!
float gyro_Acc[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Integral[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Acc_RC[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Complementary[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Kalman[] = {0.0, 0.0, 0.0};//Pitch Roll Yaw һ��΢�� ����΢��
float gyro_Mahony[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Madgwick[] = {0.0,0.0,0.0};//Pitch Roll Yaw


#if Sliding_Windows_Filter

float wf[Sliding_Windows_Filter_Num];
int wf_len = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���������˲��㷨
//  @param      N        ���ڰ��������ݸ���
//  @param      data     ��Ҫ�˲�������
//  @return     data     ��ʼ������ʱ,����ԭʼֵ
//  @return     sum/N    ��ʼ����ɺ�,�����˲����ֵ
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float Sliding_Windows_Update(float data,int N)
{
    if (wf_len <= N-1)//���ڳ�ʼ��
    {
        //printf("wf_len%d:%f ",wf_len,wf[wf_len]);
        wf[wf_len] = data;
        wf_len++;
        return data;
    }
    else
    {
        //�󴰿ھ�ֵ
        float sum = 0;
        for (int i=0;i<=N-1;i++)
        {
            sum += wf[i];
            //printf("i%d:%f ",i,wf[i]);
        }
        //��������
        for (int i=0;i<=N-2;i++)
        {
            wf[i]=wf[i+1];
        }
        wf[N-1] = data;
        return sum/N;
    }
}

#endif

float RCF_data_last = 0, RCF_data = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      һ��RC��ͨ�˲�
//  @param      data    ��Ҫ�˲�������
//  @param      f_s     ��ֹƵ��
//  @param      dt      ��������֮���ʱ���
//  @return     float   �˲��������
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float RC_Filter_update(float data, float f_s, float dt)
{
    tao = 1.0/(2.0*PI*f_s);
    //            1
    // tao = ----------   ��ֹƵ��
    //         2*pi*f_s
    RCF_data = RCF_data_last -(dt/(dt+tao))*RCF_data_last + (dt/(dt+tao))*data;
    RCF_data_last = RCF_data;


    return RCF_data;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ������ٶȼƺ������Ǽ������ԭʼֵ
//  @param      gx    x����ٶ�
//  @param      gy    y����ٶ�
//  @param      gz    z����ٶ�
//  @param      ax    x����ٶ�
//  @param      ay    y����ٶ�
//  @param      az    z����ٶ�
//  @param      dt    �������ݼ����ʱ��
//  @param      f_s   ��ֹƵ��
//  @return     void
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
void Gyro_Raw_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt, float f_s)
{
    // �����ǻ��ֵõ���ԭʼ����
    Pitch_integral +=  gx * (dt);
    Roll_integral +=  gy * (dt);
    Yaw_integral +=  gz * (dt);

    //���ٶȼƽ���õ���ԭʼ����
    Pitch_acc = -atan(ay/az);          //����ֵ
    norm_acc = sqrt(ax*ax+ay*ay+az*az);
    Roll_acc = asin(ax/-norm_acc);
    //Roll_acc = asin(ax/-G);  //Or... ? Mostly wrong

    //Roll_acc = windows_smooth(Roll_acc,5);

    tao = 1.0/(2.0*PI*f_s);
    //            1
    // tao = ----------   ��ֹƵ��
    //         2*pi*f_s

    //һ��RC�˲������ȥ���ٶȸ�Ƶ����
    Pitch_acc_RC += -(dt/(dt+tao))*Pitch_acc_RC + (dt/(dt+tao))*Pitch_acc;
    Roll_acc_RC += -(dt/(dt+tao))*Roll_acc_RC + (dt/(dt+tao))*Roll_acc;

    return;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���㻥���˲������ĽǶ�ֵ
//  @param      gx    x����ٶ�
//  @param      gy    y����ٶ�
//  @param      gz    z����ٶ�
//  @param      ax    x����ٶ�
//  @param      ay    y����ٶ�
//  @param      az    z����ٶ�
//  @param      dt    �������ݼ����ʱ��
//  @param      f_s   ��ֹƵ��
//  @return     void
//  @since      v1.2
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
void Gyro_Complementary_Update(float gx, float gy, float gz,float ax, float ay, float az, float dt, int f_s)
{
    tao = 1.0/(2.0*PI*f_s);
    //            1
    // tao = ----------   ��ֹƵ��
    //         2*pi*f_s

    //���ٶȼƽ���õ���ԭʼ����
    Pitch_acc = -atan(ay/az);          //����ֵ
    norm_acc = sqrt(ax*ax+ay*ay+az*az);
    Roll_acc = asin(ax/-norm_acc);

    Pitch_Complementary = (tao/(tao+dt))*(Pitch_Complementary+dt*gx)+(dt/(tao+dt))*Pitch_acc;
    Roll_Complementary = (tao/(tao+dt))*(Roll_Complementary+dt*gy)+(dt/(tao+dt))*Roll_acc;
    Yaw_Complementary = (tao/(tao+dt))*(Yaw_Complementary+dt*gz);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ�����ǽ�������ĽǶ�ԭʼֵ
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Integral(void)
{
    gyro_Integral[0] = Pitch_integral*180/PI;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Integral[1] = Roll_integral*180/PI; //Real Roll ���ݳ��ϵİ�װλ�øı�
    gyro_Integral[2] = Yaw_integral*180/PI;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�
    return gyro_Integral;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���ٶȽ�������ĽǶ�ԭʼֵ
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Acc(void)
{
    gyro_Acc[0] = Pitch_acc*180/PI;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Acc[1] = Roll_acc*180/PI; //Real Roll ���ݳ��ϵİ�װλ�øı�
    gyro_Acc[2] = 0;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�
    //Yaw �޷��ü��ٶȼƽ���
    return gyro_Acc;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���ٶȽ�������ĽǶ��ټ�һ��RC�˲���ĽǶ�
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Acc_RC(void)
{

    gyro_Acc_RC[0] = Pitch_acc_RC*180/PI;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Acc_RC[1] = Roll_acc_RC*180/PI; //Real Roll ���ݳ��ϵİ�װλ�øı�
    gyro_Acc_RC[2] = 0;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�
    //Yaw �޷��ü��ٶȼƽ���
    return gyro_Acc_RC;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ���������ĽǶ�
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Complementary(void)
{

    gyro_Complementary[0] = Pitch_Complementary*180/PI;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Complementary[1] = Roll_Complementary*180/PI; //Real Roll ���ݳ��ϵİ�װλ�øı�
    gyro_Complementary[2] = Yaw_Complementary*180/PI;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�
    //Yawֻ��Դ�������ǻ��֣�����׼ȷ����Ҫ����IMU
    return gyro_Complementary;
}


#endif



#if Kalman_filter

//�����ֺ��Ĵ���ʵ����Դ�� 2003/07/09 Trammel Hudson <hudson@rotomotion.com>
//

float Pitch_kalman = 0, Roll_kalman = 0, Yaw_kalman = 0;

float Q_angle = 0.16, Q_gyro = 0.04, R_angle = 2.5, dt = 0.005; //ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;
//float Q_angle = 0.02, Q_gyro = 0.005, R_angle = 2.5, dt = 0.006;
float K[2][2] = {{ 1, 0 }, { 0, 1 }};     //�������˲�
float Pdot[4] ={0,0,0,0};
const char C_0 = 1;
float q_bias,angle_err,PCt_0,PCt_1,E,K_0,K_1,t_0,t_1;
float g_fX_GYRO_ADD;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������˲���ʼ��,��������
//  @param      Q_a     ���ٶȹ��������������
//  @param      Q_g     �����ǹ��������������
//  @param      R_a     ���ٶȲ��������������
//  @return     void
//  @since      v1.0
//  Sample usage:
//  ������������:Q = [Q_a   0 ]  ������������R=[R_a] ���Ͳ���ΪQ_a=0.16,Q_g=0.02,R_a=2
//                   [ 0   Q_g]
//-------------------------------------------------------------------------------------------------------------------
void Kalman_Init(float Q_a, float Q_g, float R_a)
{
    Q_angle = Q_a;
    Q_gyro = Q_g;
    R_angle = R_a;

    return;
}

//SHIT CODE WARNING!
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Kalman�˲������ĽǶ�ֵ
//  @param      gx    x����ٶ�
//  @param      gy    y����ٶ�
//  @param      gz    z����ٶ�
//  @param      samperate    ��������,ʵ�ʵ���1/dt
//  @return     void
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
void Kalman_Filter_Update_Pitch(float gx, float gy, float gz, int samplerate)
{
    dt = 1.0/samplerate;

    Pitch_kalman+=(gx - q_bias) * dt;
    Pdot[0] = Q_angle - K[0][1] - K[1][0];// Pk-' ����������Э�����΢��
    Pdot[1] = -K[1][1];
    Pdot[2] = -K[1][1];
    Pdot[3] = Q_gyro;

    K[0][0] += Pdot[0] * dt;// Pk- ����������Э����΢�ֵĻ��� = ����������Э����
    K[0][1] += Pdot[1] * dt;
    K[1][0] += Pdot[2] * dt;
    K[1][1] += Pdot[3] * dt;

    angle_err = Pitch_acc - Pitch_kalman;// g_fCarAngle- angle;//zk-�������

    PCt_0 = C_0 * K[0][0];
    PCt_1 = C_0 * K[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;//Kk
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * K[0][1];
    /*****���¾���*****/
    K[0][0] -= K_0 * t_0;//����������Э����
    K[0][1] -= K_0 * t_1;
    K[1][0] -= K_1 * t_0;
    K[1][1] -= K_1 * t_1;
    /*****ͨ�������������������*****/
    Pitch_kalman += K_0 * angle_err;
    q_bias +=K_1 * angle_err;
    //g_fX_GYRO_ADD = gx-q_bias;//���ֵ��������ƣ���΢�� = ���ٶ�
}

//SHIT CODE WARNING!
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Kalman�˲������ĽǶ�ֵ
//  @param      gx    x����ٶ�
//  @param      gy    y����ٶ�
//  @param      gz    z����ٶ�
//  @param      samperate    ��������,ʵ�ʵ���1/dt
//  @return     void
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
void Kalman_Filter_Update_Roll(float gx, float gy, float gz, int samplerate)
{
    dt = 1.0/samplerate;

    Roll_kalman+=(gy - q_bias) * dt;
    Pdot[0] = Q_angle - K[0][1] - K[1][0];// Pk-' ����������Э�����΢��
    Pdot[1] = -K[1][1];
    Pdot[2] = -K[1][1];
    Pdot[3] = Q_gyro;

    K[0][0] += Pdot[0] * dt;// Pk- ����������Э����΢�ֵĻ��� = ����������Э����
    K[0][1] += Pdot[1] * dt;
    K[1][0] += Pdot[2] * dt;
    K[1][1] += Pdot[3] * dt;

    angle_err = Roll_acc - Roll_kalman;// g_fCarAngle- angle;//zk-�������

    PCt_0 = C_0 * K[0][0];
    PCt_1 = C_0 * K[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;//Kk
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * K[0][1];
    /*****���¾���*****/
    K[0][0] -= K_0 * t_0;//����������Э����
    K[0][1] -= K_0 * t_1;
    K[1][0] -= K_1 * t_0;
    K[1][1] -= K_1 * t_1;
    /*****ͨ�������������������*****/
    Roll_kalman += K_0 * angle_err;  //�������
    q_bias +=K_1 * angle_err;//�������
    //g_fX_GYRO_ADD = gy-q_bias;//���ֵ��������ƣ���΢�� = ���ٶ�
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡKalman�˲������ĽǶ�
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Kalman(void)
{

    gyro_Kalman[0] = Pitch_kalman*180/PI;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Kalman[1] = Roll_kalman*180/PI; //Real Roll ���ݳ��ϵİ�װλ�øı�
    //gyro_Kalman[2] = Yaw_kalman*180/PI;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�

    //Yawֻ��Դ�������ǻ��֣�����׼ȷ����Ҫ����IMU
    return gyro_Kalman;
}

#endif




#if Mahony_6_filter

float Pitch, Roll, Yaw;
#define G 9.7947f  // m/s^2 �Ϸ��������ٶ�
#define PI 3.141592653589793
#define Kp 1.50f
#define Ki 0.005f
float halfT = 1;
float GYRO_K = 1, ACCEL_K = 1;
float gyro[] = {0,0,0};//Pitch Roll Yaw

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Mahony��ʼ��
//  @param      loop_ms     ��������֮��ļ��ʱ��
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Mahony_Init(float loop_ms)
{
    halfT = loop_ms/1000./2;    //�������ڵ�һ�룬��λs
    //GYRO_K = 1./16.4/57.3;
    //ACCEL_K = G/4096;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ټ��� 1/Sqrt(x)
//  @param      x
//  @return     y       1/Sqrt(x)
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static float invSqrt(float x)       //���ټ��� 1/Sqrt(x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;        //������������ۼƻ���

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Mahony����������
//  @param      gx    x����ٶ�
//  @param      gy    y����ٶ�
//  @param      gz    z����ٶ�
//  @param      ax    x����ٶ�
//  @param      ay    y����ٶ�
//  @param      az    z����ٶ�
//  @return     void
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
void Mahony_Imu_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    //unsigned char i;
    float vx, vy, vz;                           //ʵ���������ٶ�
    float ex, ey, ez;                           //�����������
    float norm;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    //�����ٶ�ԭʼADֵת��Ϊm/s^2
    //ax = ax * ACCEL_K;
    //ay = ay * ACCEL_K;
    //az = az * ACCEL_K;
    //��������ADֵת��Ϊ ����/s
    //gx = gx * GYRO_K;
    //gy = gy * GYRO_K;
    //gz = gz * GYRO_K;
    if (ax * ay * az == 0)
        return;
    //���ٶȼƲ�������������(��������ϵ)
    norm = invSqrt(ax * ax + ay * ay + az * az);            //֮ǰ����д��invSqrt(ax*ax + ay+ay + az*az)�Ǵ���ģ������޸Ĺ�����
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //��Ԫ���Ƴ���ʵ����������(��������ϵ)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //������
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    //���������Ϊ���ٶ�
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
    //���ٶȲ���
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
    //������Ԫ��
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //��λ����Ԫ��
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    //��Ԫ������ŷ����
    Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.3f;
    Pitch = -asin(2.f * (q1q3 - q0q2)) * 57.3f;
    Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * 57.3f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMahony�˲������ĽǶ�
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Mahony(void)
{
    gyro_Mahony[0] = Pitch;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Mahony[1] = Roll; //Real Roll ���ݳ��ϵİ�װλ�øı�
    gyro_Mahony[2] = Yaw;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�
    return gyro_Mahony;
}

#endif



#if Madgwick_6_filter

#define beta    0.1f                                        // 2 * proportional gain (Kp)

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to auxiliary frame
float Pitch = 0.0f, Roll = 0.0f, Yaw = 0.0f;
float sampleFreq = 1;
float GYRO_K = 1;
float gyro[] = {0,0,0};//Pitch Roll Yaw

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Madgwick��ʼ��
//  @param      loop_ms     ��������֮��ļ��ʱ��
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Madgwick_Init(float loop_ms)
{
    sampleFreq = 1000. / loop_ms;   //sample frequency in Hz
    //GYRO_K = 1./16.4/57.3;

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ټ��� 1/Sqrt(x)
//  @param      x
//  @return     y       1/Sqrt(x)
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Madgwick����������
//  @param      gx    x����ٶ�
//  @param      gy    y����ٶ�
//  @param      gz    z����ٶ�
//  @param      ax    x����ٶ�
//  @param      ay    y����ٶ�
//  @param      az    z����ٶ�
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MadgwickAHRSupdate_6(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    //gx = gx * GYRO_K;
    //gy = gy * GYRO_K;
    //gz = gz * GYRO_K;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    Pitch = asin(-2.0f * (q1*q3 - q0*q2))* 57.3f;
    Roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.3f;
    Yaw = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* 57.3f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡMadgwick�˲������ĽǶ�
//  @param      void
//  @return     *float     ����,��ʽΪ[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Madgwick(void)
{
    gyro_Madgwick[0] = Pitch;//Real Pitch ���ݳ��ϵİ�װλ�øı�
    gyro_Madgwick[1] = Roll; //Real Roll ���ݳ��ϵİ�װλ�øı�
    gyro_Madgwick[2] = Yaw;  //Real Yaw  ���ݳ��ϵİ�װλ�øı�
    return gyro_Madgwick;
}


#endif


float FD_data_previous=0,First_Derivative_data=0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����һ��΢��
//  @param      data    ��΢�ֵ�����
//  @param      data    ��������֮���ʱ����
//  @return     float   һ��΢�ֺ������
//  @since      v1.0
//  Sample usage:����ѭ����
//-------------------------------------------------------------------------------------------------------------------
float First_Derivative (float data, float dt)
{
    First_Derivative_data = ((data - FD_data_previous)/dt);
    FD_data_previous = data;
    return First_Derivative_data;
}


float SD_data_previous[3]={0.0, 0.0, 0.0},Second_Derivative_data=0;
int SD_len=0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������΢��
//  @param      data    ��΢�ֵ�����
//  @param      data    ��������֮���ʱ����
//  @return     float   ����΢�ֺ������
//  @since      v1.0
//  Sample usage:����ѭ���У���������һ��΢��.   WARNNING:����ѭ��������������
//-------------------------------------------------------------------------------------------------------------------
float Second_Derivative (float data, float dt)
{
    if(SD_len <= 2)
    {
        SD_data_previous[SD_len] = data;
        SD_len++;
        return 0.0;
    }
    else
    {
        for (int i=0;i<=1;i++)
        {
            SD_data_previous[i]=SD_data_previous[i+1];
        }
        SD_data_previous[2] = data;

        Second_Derivative_data = (SD_data_previous[2]+SD_data_previous[0]-2*SD_data_previous[1])/(dt*dt);

        return Second_Derivative_data;
    }
}

