/*
 * ICM42605.c
 *
 *  Created on: 2022年3月7日
 *      Author: WeijuWU
 */

#include "zf_stm_systick.h"
#include "zf_gpio.h"
#include "zf_spi.h"
#include "SEEKFREE_IIC.h"
#include "ICM42605.h"
#include "stdio.h"

int16 icm4_temp;
int16 icm4_gyro_x,icm4_gyro_y,icm4_gyro_z;
int16 icm4_acc_x,icm4_acc_y,icm4_acc_z;
float accx,accy,accz;
float gyrox,gyroy,gyroz;
float temp_data;

//-------------------------------------------------------------------------------------------------------------------
//  @brief      将原始ICM42605的值转化为有物理意义的加速度(单位m/s^2)和角速度(单位rad/s)
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               调用该函数前，先读取原始值;
//                              根据传感器寄存器配置调整本函数数值
//-------------------------------------------------------------------------------------------------------------------
void icm42605_AD_trun(void)
{
    //float g = 9.80665f;
    float g = 9.7947f; //合肥重力加速度
    float pi = 3.14159265359f;
    accx = icm4_acc_x * (8.0f * g / 32768.0f);//转化后,单位m/s^2
    accy = icm4_acc_y * (8.0f * g / 32768.0f);
    accz = icm4_acc_z * (8.0f * g / 32768.0f);
    gyrox = icm4_gyro_x * (2000.0f * pi / 180 / 32768.0f);//转化后,单位rad/s
    gyroy = icm4_gyro_y * (2000.0f * pi / 180 / 32768.0f);
    gyroz = icm4_gyro_z * (2000.0f * pi / 180 / 32768.0f);
    temp_data = (icm4_temp / 132.48) + 25;//转化后,单位摄氏度，To fix
}

//-------------------------------------------------------------------------------------------------------------------
//  使用软件IIC通信
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      自检ICM42605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm42605_self1_check(void)
{
    uint8 dat=0;
    while(0x42 != dat)   //读取ICM42605 ID, 默认ID是0x42
    {
        dat = simiic_read_reg(ICM42605_ADDRESS,ICM42605_WHO_AM_I,SIMIIC);
        systick_delay_ms(STM0, 10);
        //卡在这里原因有以下几点
        //1 ICM42605坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V

    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      复位ICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm42605_reset(void)
{
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_DEVICE_CONFIG,0x01);
    //至少等待1ms
    systick_delay_ms(STM0, 3);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               调用该函数前，请先调用模拟IIC的初始化
//-------------------------------------------------------------------------------------------------------------------
void icm42605_init(void)
{
    uint8 reg_val;

    simiic_init();
    systick_delay_ms(STM0, 10);  //上电延时

    //自检
    icm42605_self1_check();

    //复位
    icm42605_reset();

    //配置电源参数
    reg_val = simiic_read_reg(ICM42605_ADDRESS,ICM42605_PWR_MGMT0,SIMIIC);
    reg_val &= ~(1 << 5);//使能温度测量
    reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    //simiic_write_reg(ICM42605_ADDRESS,ICM42605_PWR_MGMT0,reg_val | 0x00);
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_PWR_MGMT0,0x1F);//LN模式(Low Nosie Mode)
    systick_delay_ms(STM0, 5);//延时,操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作


    //配置参数
    //simiic_write_reg(ICM42605_ADDRESS,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    //simiic_write_reg(ICM42605_ADDRESS,ICM20602_SMPLRT_DIV,0x07);               //采样速率 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_GYRO_CONFIG0,0x06);//±2000 dps, 1kHz输出
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_ACCEL_CONFIG0,0x26);//±8g , 1kHZ输出
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_GYRO_CONFIG1,0xA0);//温度传感器数字低通滤波频率设置为10Hz
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_GYRO_ACCEL_CONFIG0 ,0x11);//加速度计和陀螺仪低通截止频率设置, 默认
    /*
    reg_val = simiic_read_reg(ICM42605_ADDRESS,ICM42605_INT_CONFIG,SIMIIC);
    simiic_write_reg(ICM42605_ADDRESS, ICM42605_INT_CONFIG, reg_val | 0x18 | 0x03 );

    reg_val = simiic_read_reg(ICM42605_ADDRESS,ICM42605_INT_CONFIG1,SIMIIC);
    simiic_write_reg(ICM42605_ADDRESS, ICM42605_INT_CONFIG1, reg_val & ~(0x10) );

    reg_val = simiic_read_reg(ICM42605_ADDRESS,ICM42605_INT_SOURCE0,SIMIIC);
    simiic_write_reg(ICM42605_ADDRESS, ICM42605_INT_SOURCE0, reg_val | 0x08 );

    reg_val = simiic_read_reg(ICM42605_ADDRESS,ICM42605_INT_SOURCE3,SIMIIC);
    simiic_write_reg(ICM42605_ADDRESS, ICM42605_INT_SOURCE3, reg_val | 0x01 );
    */
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605温度数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_tempdata(void)
{
    uint8 dat[2];

    simiic_read_regs(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, dat, 2, SIMIIC);
    icm4_temp = (int16)(((uint16)dat[0]<<8 | dat[1]));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_accdata(void)
{
    uint8 dat[6];

    simiic_read_regs(ICM42605_ADDRESS, ICM42605_ACCEL_DATA_X1, dat, 6, SIMIIC);
    icm4_acc_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm4_acc_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm4_acc_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_gyro(void)
{
    uint8 dat[6];

    simiic_read_regs(ICM42605_ADDRESS, ICM42605_GYRO_DATA_X1, dat, 6, SIMIIC);
    icm4_gyro_x = (int16)(((uint16)dat[0]<<8 | dat[1]));
    icm4_gyro_y = (int16)(((uint16)dat[2]<<8 | dat[3]));
    icm4_gyro_z = (int16)(((uint16)dat[4]<<8 | dat[5]));
}


//-------------------------------------------------------------------------------------------------------------------
//  使用硬件SPI通信
//-------------------------------------------------------------------------------------------------------------------
#define SPI_NUM         SPI_0
#define SPI_SCK_PIN     SPI0_SCLK_P20_11    //接模块SPC
#define SPI_MOSI_PIN    SPI0_MOSI_P20_14    //接模块SDI
#define SPI_MISO_PIN    SPI0_MISO_P20_12    //接模块SDO
#define SPI_CS_PIN      SPI0_CS2_P20_13     //接模块CS


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM42605 SPI写寄存器
//  @param      cmd     寄存器地址
//  @param      val     需要写入的数据
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm4_spi_w_reg_byte(uint8 cmd, uint8 val)
{
    uint8 dat[2];

    dat[0] = cmd | ICM42605_SPI_W;
    dat[1] = val;

    spi_mosi(SPI_NUM,SPI_CS_PIN,dat,dat,2,1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM42605 SPI读寄存器
//  @param      cmd     寄存器地址
//  @param      *val    接收数据的地址
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm4_spi_r_reg_byte(uint8 cmd, uint8 *val)
{
    uint8 dat[2];

    dat[0] = cmd | ICM42605_SPI_R;
    dat[1] = *val;

    spi_mosi(SPI_NUM,SPI_CS_PIN,dat,dat,2,1);

    *val = dat[1];
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM42605 SPI多字节读寄存器
//  @param      *val    接收数据的地址
//  @param      num     读取数量
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm4_spi_r_reg_bytes(uint8 * val, uint8 num)
{
    spi_mosi(SPI_NUM,SPI_CS_PIN,val,val,num,1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM42605自检函数
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm42605_self3_check(void)
{
    uint8 dat=0;
    while(0x42 != dat)   //读取ICM42605 ID
    {
        icm4_spi_r_reg_byte(ICM42605_WHO_AM_I,&dat);
        systick_delay_ms(STM0, 200);
        printf("ERROR:IMU Connection Bad\r\n");
        //卡在这里原因有以下几点
        //1 ICM42605坏了，如果是新的这样的概率极低
        //2 接线错误或者没有接好
        //3 可能你需要外接上拉电阻，上拉到3.3V
    }

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      复位ICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm42605_reset_spi(void)
{
    icm4_spi_w_reg_byte(ICM42605_DEVICE_CONFIG,0x01);
    //至少等待1ms
    systick_delay_ms(STM0, 3);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      初始化ICM42605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm42605_init_spi(void)
{
    uint8 reg_val;

    systick_delay_ms(STM0, 10);  //上电延时

    (void)spi_init(SPI_NUM, SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CS_PIN, 3, 10*1000*1000);//硬件SPI初始化

    //自检
    icm42605_self3_check();

    //复位
    icm42605_reset_spi();

    //配置电源参数
    icm4_spi_r_reg_byte(ICM42605_PWR_MGMT0,&reg_val);
    reg_val &= ~(1 << 5);//使能温度测量
    reg_val |= ((3) << 2);//设置GYRO_MODE  0:关闭 1:待机 2:预留 3:低噪声
    reg_val |= (3);//设置ACCEL_MODE 0:关闭 1:关闭 2:低功耗 3:低噪声
    //icm4_spi_w_reg_byte(ICM42605_PWR_MGMT0,reg_val | 0x00);
    icm4_spi_w_reg_byte(ICM42605_PWR_MGMT0,0x1F);
    systick_delay_ms(STM0, 5);//延时,操作完PWR—MGMT0寄存器后 200us内不能有任何读写寄存器的操作

    icm4_spi_w_reg_byte(ICM42605_GYRO_CONFIG0,0x07);//±2000 dps, 200Hz输出
    icm4_spi_w_reg_byte(ICM42605_ACCEL_CONFIG0,0x27);//±8g , 200HZ输出
    icm4_spi_w_reg_byte(ICM42605_GYRO_CONFIG1,0xA0);//温度传感器数字低通滤波频率设置为10Hz
    icm4_spi_w_reg_byte(ICM42605_GYRO_ACCEL_CONFIG0 ,0x11);//加速度计和陀螺仪低通截止频率设置, 默认

    icm4_spi_r_reg_byte(ICM42605_INT_CONFIG,&reg_val);
    icm4_spi_w_reg_byte(ICM42605_INT_CONFIG,reg_val | 0x18 | 0x03);

    icm4_spi_r_reg_byte(ICM42605_INT_CONFIG1,&reg_val);
    icm4_spi_w_reg_byte(ICM42605_INT_CONFIG1,reg_val & ~(0x10));

    icm4_spi_r_reg_byte(ICM42605_INT_SOURCE0,&reg_val);
    icm4_spi_w_reg_byte(ICM42605_INT_SOURCE0,reg_val | 0x08);

    icm4_spi_r_reg_byte(ICM42605_INT_SOURCE3,&reg_val);
    icm4_spi_w_reg_byte(ICM42605_INT_SOURCE3,reg_val | 0x01);

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605温度数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_tempdata_spi(void)
{
    struct
    {
        uint8 reg;
        uint8 dat[2];
    }buf;

    buf.reg = ICM42605_TEMP_DATA1 | ICM42605_SPI_R;

    icm4_spi_r_reg_bytes(&buf.reg, 3);
    icm4_temp = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605加速度计数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_accdata_spi(void)
{
    struct
    {
        uint8 reg;
        uint8 dat[6];
    }buf;

    buf.reg = ICM42605_ACCEL_DATA_X1 | ICM42605_SPI_R;

    icm4_spi_r_reg_bytes(&buf.reg, 7);
    icm4_acc_x = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));
    icm4_acc_y = (int16)(((uint16)buf.dat[2]<<8 | buf.dat[3]));
    icm4_acc_z = (int16)(((uint16)buf.dat[4]<<8 | buf.dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取ICM42605陀螺仪数据
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               执行该函数后，直接查看对应的变量即可
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_gyro_spi(void)
{
    struct
    {
        uint8 reg;
        uint8 dat[6];
    }buf;

    buf.reg = ICM42605_GYRO_DATA_X1 | ICM42605_SPI_R;

    icm4_spi_r_reg_bytes(&buf.reg, 7);
    icm4_gyro_x = (int16)(((uint16)buf.dat[0]<<8 | buf.dat[1]));
    icm4_gyro_y = (int16)(((uint16)buf.dat[2]<<8 | buf.dat[3]));
    icm4_gyro_z = (int16)(((uint16)buf.dat[4]<<8 | buf.dat[5]));
}

