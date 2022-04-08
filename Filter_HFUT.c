/*
 * Filter_HFUT.cpp
 *
 *  Created on: 2022年3月17日
 *      Author: WeijuWU
 */

//---------------------------------------------------------------------------------------------------

//TO DO:
//加速度计的标定
//屎山代码优化,改成结构体形式,消除所有WARNNING
//二阶低通滤波
//测量噪声矩阵R的测量
//EKF,UKF

// Header files
#include <math.h>
#include "Filter_HFUT.h"
#include <stdio.h>

#if Complementary_filter || Kalman_filter // 卡尔曼滤波依赖于互补滤波里面的部分函数与变量

#define G 9.7947f  // m/s^2 合肥重力加速度
#define PI 3.141592653589793
float Pitch_integral = 0.0,Roll_integral = 0.0,Yaw_integral = 0.0;
float Pitch_acc = 0.0,Roll_acc = 0.0;//Yaw 无法用加速度计解算
float Pitch_acc_RC = 0.0,Roll_acc_RC = 0.0;
float Pitch_Complementary=0.0,Roll_Complementary=0.0,Yaw_Complementary=0.0;//Yaw只来源于陀螺仪积分
float Pitch_Complementary_last=0.0,Roll_Complementary_last=0.0,Yaw_Complementary_last=0.0;
float norm_acc;
float tao;


//SHIT CODE WARNING!!!
float gyro_Acc[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Integral[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Acc_RC[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Complementary[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Kalman[] = {0.0, 0.0, 0.0};//Pitch Roll Yaw 一阶微分 二阶微分
float gyro_Mahony[] = {0.0,0.0,0.0};//Pitch Roll Yaw
float gyro_Madgwick[] = {0.0,0.0,0.0};//Pitch Roll Yaw


#if Sliding_Windows_Filter

float wf[Sliding_Windows_Filter_Num];
int wf_len = 0;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      滑动窗口滤波算法
//  @param      N        窗口包含的数据个数
//  @param      data     需要滤波的数据
//  @return     data     初始化窗口时,返回原始值
//  @return     sum/N    初始化完成后,返回滤波后的值
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float Sliding_Windows_Update(float data,int N)
{
    if (wf_len <= N-1)//窗口初始化
    {
        //printf("wf_len%d:%f ",wf_len,wf[wf_len]);
        wf[wf_len] = data;
        wf_len++;
        return data;
    }
    else
    {
        //求窗口均值
        float sum = 0;
        for (int i=0;i<=N-1;i++)
        {
            sum += wf[i];
            //printf("i%d:%f ",i,wf[i]);
        }
        //滑动窗口
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
//  @brief      一阶RC低通滤波
//  @param      data    需要滤波的数据
//  @param      f_s     截止频率
//  @param      dt      两个数据之间的时间差
//  @return     float   滤波后的数据
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float RC_Filter_update(float data, float f_s, float dt)
{
    tao = 1.0/(2.0*PI*f_s);
    //            1
    // tao = ----------   截止频率
    //         2*pi*f_s
    RCF_data = RCF_data_last -(dt/(dt+tao))*RCF_data_last + (dt/(dt+tao))*data;
    RCF_data_last = RCF_data;


    return RCF_data;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算加速度计和陀螺仪计算出的原始值
//  @param      gx    x轴角速度
//  @param      gy    y轴角速度
//  @param      gz    z轴角速度
//  @param      ax    x轴加速度
//  @param      ay    y轴加速度
//  @param      az    z轴加速度
//  @param      dt    两个数据间隔的时间
//  @param      f_s   截止频率
//  @return     void
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
void Gyro_Raw_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt, float f_s)
{
    // 陀螺仪积分得到的原始数据
    Pitch_integral +=  gx * (dt);
    Roll_integral +=  gy * (dt);
    Yaw_integral +=  gz * (dt);

    //加速度计解算得到的原始数据
    Pitch_acc = -atan(ay/az);          //弧度值
    norm_acc = sqrt(ax*ax+ay*ay+az*az);
    Roll_acc = asin(ax/-norm_acc);
    //Roll_acc = asin(ax/-G);  //Or... ? Mostly wrong

    //Roll_acc = windows_smooth(Roll_acc,5);

    tao = 1.0/(2.0*PI*f_s);
    //            1
    // tao = ----------   截止频率
    //         2*pi*f_s

    //一阶RC滤波后的滤去加速度高频噪声
    Pitch_acc_RC += -(dt/(dt+tao))*Pitch_acc_RC + (dt/(dt+tao))*Pitch_acc;
    Roll_acc_RC += -(dt/(dt+tao))*Roll_acc_RC + (dt/(dt+tao))*Roll_acc;

    return;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算互补滤波出来的角度值
//  @param      gx    x轴角速度
//  @param      gy    y轴角速度
//  @param      gz    z轴角速度
//  @param      ax    x轴加速度
//  @param      ay    y轴加速度
//  @param      az    z轴加速度
//  @param      dt    两个数据间隔的时间
//  @param      f_s   截止频率
//  @return     void
//  @since      v1.2
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
void Gyro_Complementary_Update(float gx, float gy, float gz,float ax, float ay, float az, float dt, int f_s)
{
    tao = 1.0/(2.0*PI*f_s);
    //            1
    // tao = ----------   截止频率
    //         2*pi*f_s

    //加速度计解算得到的原始数据
    Pitch_acc = -atan(ay/az);          //弧度值
    norm_acc = sqrt(ax*ax+ay*ay+az*az);
    Roll_acc = asin(ax/-norm_acc);

    Pitch_Complementary = (tao/(tao+dt))*(Pitch_Complementary+dt*gx)+(dt/(tao+dt))*Pitch_acc;
    Roll_Complementary = (tao/(tao+dt))*(Roll_Complementary+dt*gy)+(dt/(tao+dt))*Roll_acc;
    Yaw_Complementary = (tao/(tao+dt))*(Yaw_Complementary+dt*gz);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取陀螺仪解算出来的角度原始值
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Integral(void)
{
    gyro_Integral[0] = Pitch_integral*180/PI;//Real Pitch 根据车上的安装位置改变
    gyro_Integral[1] = Roll_integral*180/PI; //Real Roll 根据车上的安装位置改变
    gyro_Integral[2] = Yaw_integral*180/PI;  //Real Yaw  根据车上的安装位置改变
    return gyro_Integral;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取加速度解算出来的角度原始值
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Acc(void)
{
    gyro_Acc[0] = Pitch_acc*180/PI;//Real Pitch 根据车上的安装位置改变
    gyro_Acc[1] = Roll_acc*180/PI; //Real Roll 根据车上的安装位置改变
    gyro_Acc[2] = 0;  //Real Yaw  根据车上的安装位置改变
    //Yaw 无法用加速度计解算
    return gyro_Acc;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取加速度解算出来的角度再加一阶RC滤波后的角度
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Acc_RC(void)
{

    gyro_Acc_RC[0] = Pitch_acc_RC*180/PI;//Real Pitch 根据车上的安装位置改变
    gyro_Acc_RC[1] = Roll_acc_RC*180/PI; //Real Roll 根据车上的安装位置改变
    gyro_Acc_RC[2] = 0;  //Real Yaw  根据车上的安装位置改变
    //Yaw 无法用加速度计解算
    return gyro_Acc_RC;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取互补出来的角度
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Complementary(void)
{

    gyro_Complementary[0] = Pitch_Complementary*180/PI;//Real Pitch 根据车上的安装位置改变
    gyro_Complementary[1] = Roll_Complementary*180/PI; //Real Roll 根据车上的安装位置改变
    gyro_Complementary[2] = Yaw_Complementary*180/PI;  //Real Yaw  根据车上的安装位置改变
    //Yaw只来源于陀螺仪积分，极不准确，需要九轴IMU
    return gyro_Complementary;
}


#endif



#if Kalman_filter

//本部分核心代码实际来源于 2003/07/09 Trammel Hudson <hudson@rotomotion.com>
//

float Pitch_kalman = 0, Roll_kalman = 0, Yaw_kalman = 0;

float Q_angle = 0.16, Q_gyro = 0.04, R_angle = 2.5, dt = 0.005; //注意：dt的取值为kalman滤波器采样时间;
//float Q_angle = 0.02, Q_gyro = 0.005, R_angle = 2.5, dt = 0.006;
float K[2][2] = {{ 1, 0 }, { 0, 1 }};     //卡尔曼滤波
float Pdot[4] ={0,0,0,0};
const char C_0 = 1;
float q_bias,angle_err,PCt_0,PCt_1,E,K_0,K_1,t_0,t_1;
float g_fX_GYRO_ADD;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      卡尔曼滤波初始化,参数设置
//  @param      Q_a     加速度过程噪声矩阵参数
//  @param      Q_g     陀螺仪过程噪声矩阵参数
//  @param      R_a     加速度测量噪声矩阵参数
//  @return     void
//  @since      v1.0
//  Sample usage:
//  过程噪声矩阵:Q = [Q_a   0 ]  测量噪声矩阵：R=[R_a] 典型参数为Q_a=0.16,Q_g=0.02,R_a=2
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
//  @brief      计算Kalman滤波出来的角度值
//  @param      gx    x轴角速度
//  @param      gy    y轴角速度
//  @param      gz    z轴角速度
//  @param      samperate    数据速率,实际等于1/dt
//  @return     void
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
void Kalman_Filter_Update_Pitch(float gx, float gy, float gz, int samplerate)
{
    dt = 1.0/samplerate;

    Pitch_kalman+=(gx - q_bias) * dt;
    Pdot[0] = Q_angle - K[0][1] - K[1][0];// Pk-' 先验估计误差协方差的微分
    Pdot[1] = -K[1][1];
    Pdot[2] = -K[1][1];
    Pdot[3] = Q_gyro;

    K[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
    K[0][1] += Pdot[1] * dt;
    K[1][0] += Pdot[2] * dt;
    K[1][1] += Pdot[3] * dt;

    angle_err = Pitch_acc - Pitch_kalman;// g_fCarAngle- angle;//zk-先验估计

    PCt_0 = C_0 * K[0][0];
    PCt_1 = C_0 * K[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;//Kk
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * K[0][1];
    /*****更新矩阵*****/
    K[0][0] -= K_0 * t_0;//后验估计误差协方差
    K[0][1] -= K_0 * t_1;
    K[1][0] -= K_1 * t_0;
    K[1][1] -= K_1 * t_1;
    /*****通过卡尔曼增益进行修正*****/
    Pitch_kalman += K_0 * angle_err;
    q_bias +=K_1 * angle_err;
    //g_fX_GYRO_ADD = gx-q_bias;//输出值（后验估计）的微分 = 角速度
}

//SHIT CODE WARNING!
//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算Kalman滤波出来的角度值
//  @param      gx    x轴角速度
//  @param      gy    y轴角速度
//  @param      gz    z轴角速度
//  @param      samperate    数据速率,实际等于1/dt
//  @return     void
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
void Kalman_Filter_Update_Roll(float gx, float gy, float gz, int samplerate)
{
    dt = 1.0/samplerate;

    Roll_kalman+=(gy - q_bias) * dt;
    Pdot[0] = Q_angle - K[0][1] - K[1][0];// Pk-' 先验估计误差协方差的微分
    Pdot[1] = -K[1][1];
    Pdot[2] = -K[1][1];
    Pdot[3] = Q_gyro;

    K[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
    K[0][1] += Pdot[1] * dt;
    K[1][0] += Pdot[2] * dt;
    K[1][1] += Pdot[3] * dt;

    angle_err = Roll_acc - Roll_kalman;// g_fCarAngle- angle;//zk-先验估计

    PCt_0 = C_0 * K[0][0];
    PCt_1 = C_0 * K[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;//Kk
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * K[0][1];
    /*****更新矩阵*****/
    K[0][0] -= K_0 * t_0;//后验估计误差协方差
    K[0][1] -= K_0 * t_1;
    K[1][0] -= K_1 * t_0;
    K[1][1] -= K_1 * t_1;
    /*****通过卡尔曼增益进行修正*****/
    Roll_kalman += K_0 * angle_err;  //后验估计
    q_bias +=K_1 * angle_err;//后验估计
    //g_fX_GYRO_ADD = gy-q_bias;//输出值（后验估计）的微分 = 角速度
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取Kalman滤波出来的角度
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Kalman(void)
{

    gyro_Kalman[0] = Pitch_kalman*180/PI;//Real Pitch 根据车上的安装位置改变
    gyro_Kalman[1] = Roll_kalman*180/PI; //Real Roll 根据车上的安装位置改变
    //gyro_Kalman[2] = Yaw_kalman*180/PI;  //Real Yaw  根据车上的安装位置改变

    //Yaw只来源于陀螺仪积分，极不准确，需要九轴IMU
    return gyro_Kalman;
}

#endif




#if Mahony_6_filter

float Pitch, Roll, Yaw;
#define G 9.7947f  // m/s^2 合肥重力加速度
#define PI 3.141592653589793
#define Kp 1.50f
#define Ki 0.005f
float halfT = 1;
float GYRO_K = 1, ACCEL_K = 1;
float gyro[] = {0,0,0};//Pitch Roll Yaw

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Mahony初始化
//  @param      loop_ms     两个数据之间的间隔时间
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Mahony_Init(float loop_ms)
{
    halfT = loop_ms/1000./2;    //计算周期的一半，单位s
    //GYRO_K = 1./16.4/57.3;
    //ACCEL_K = G/4096;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      快速计算 1/Sqrt(x)
//  @param      x
//  @return     y       1/Sqrt(x)
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
static float invSqrt(float x)       //快速计算 1/Sqrt(x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     //四元数
float exInt = 0, eyInt = 0, ezInt = 0;        //叉积计算误差的累计积分

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Mahony六轴计算更新
//  @param      gx    x轴角速度
//  @param      gy    y轴角速度
//  @param      gz    z轴角速度
//  @param      ax    x轴加速度
//  @param      ay    y轴加速度
//  @param      az    z轴加速度
//  @return     void
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
void Mahony_Imu_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    //unsigned char i;
    float vx, vy, vz;                           //实际重力加速度
    float ex, ey, ez;                           //叉积计算的误差
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

    //将加速度原始AD值转换为m/s^2
    //ax = ax * ACCEL_K;
    //ay = ay * ACCEL_K;
    //az = az * ACCEL_K;
    //将陀螺仪AD值转换为 弧度/s
    //gx = gx * GYRO_K;
    //gy = gy * GYRO_K;
    //gz = gz * GYRO_K;
    if (ax * ay * az == 0)
        return;
    //加速度计测量的重力方向(机体坐标系)
    norm = invSqrt(ax * ax + ay * ay + az * az);            //之前这里写成invSqrt(ax*ax + ay+ay + az*az)是错误的，现在修改过来了
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //四元数推出的实际重力方向(机体坐标系)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //叉积误差
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    //叉积误差积分为角速度
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
    //角速度补偿
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
    //更新四元数
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //单位化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    //四元数反解欧拉角
    Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3) * 57.3f;
    Pitch = -asin(2.f * (q1q3 - q0q2)) * 57.3f;
    Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3) * 57.3f;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取Mahony滤波出来的角度
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Mahony(void)
{
    gyro_Mahony[0] = Pitch;//Real Pitch 根据车上的安装位置改变
    gyro_Mahony[1] = Roll; //Real Roll 根据车上的安装位置改变
    gyro_Mahony[2] = Yaw;  //Real Yaw  根据车上的安装位置改变
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
//  @brief      Madgwick初始化
//  @param      loop_ms     两个数据之间的间隔时间
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
//  @brief      快速计算 1/Sqrt(x)
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
//  @brief      Madgwick六轴计算更新
//  @param      gx    x轴角速度
//  @param      gy    y轴角速度
//  @param      gz    z轴角速度
//  @param      ax    x轴加速度
//  @param      ay    y轴加速度
//  @param      az    z轴加速度
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
//  @brief      获取Madgwick滤波出来的角度
//  @param      void
//  @return     *float     数组,格式为[Pitch,Roll,Yaw]
//  @since      v1.0
//  Sample usage:放在循环中
//-------------------------------------------------------------------------------------------------------------------
float *GetGryo_Madgwick(void)
{
    gyro_Madgwick[0] = Pitch;//Real Pitch 根据车上的安装位置改变
    gyro_Madgwick[1] = Roll; //Real Roll 根据车上的安装位置改变
    gyro_Madgwick[2] = Yaw;  //Real Yaw  根据车上的安装位置改变
    return gyro_Madgwick;
}


#endif


float FD_data_previous=0,First_Derivative_data=0;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      计算一阶微分
//  @param      data    被微分的数据
//  @param      data    两个数据之间的时间间隔
//  @return     float   一阶微分后的数据
//  @since      v1.0
//  Sample usage:放在循环中
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
//  @brief      计算二阶微分
//  @param      data    被微分的数据
//  @param      data    两个数据之间的时间间隔
//  @return     float   二阶微分后的数据
//  @since      v1.0
//  Sample usage:放在循环中，不依赖于一阶微分.   WARNNING:单次循环不能连续调用
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

