/*
 * ICM42605.c
 *
 *  Created on: 2022��3��7��
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
//  @brief      ��ԭʼICM42605��ֵת��Ϊ����������ļ��ٶ�(��λm/s^2)�ͽ��ٶ�(��λrad/s)
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ���øú���ǰ���ȶ�ȡԭʼֵ;
//                              ���ݴ������Ĵ������õ�����������ֵ
//-------------------------------------------------------------------------------------------------------------------
void icm42605_AD_trun(void)
{
    //float g = 9.80665f;
    float g = 9.7947f; //�Ϸ��������ٶ�
    float pi = 3.14159265359f;
    accx = icm4_acc_x * (8.0f * g / 32768.0f);//ת����,��λm/s^2
    accy = icm4_acc_y * (8.0f * g / 32768.0f);
    accz = icm4_acc_z * (8.0f * g / 32768.0f);
    gyrox = icm4_gyro_x * (2000.0f * pi / 180 / 32768.0f);//ת����,��λrad/s
    gyroy = icm4_gyro_y * (2000.0f * pi / 180 / 32768.0f);
    gyroz = icm4_gyro_z * (2000.0f * pi / 180 / 32768.0f);
    temp_data = (icm4_temp / 132.48) + 25;//ת����,��λ���϶ȣ�To fix
}

//-------------------------------------------------------------------------------------------------------------------
//  ʹ�����IICͨ��
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �Լ�ICM42605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm42605_self1_check(void)
{
    uint8 dat=0;
    while(0x42 != dat)   //��ȡICM42605 ID, Ĭ��ID��0x42
    {
        dat = simiic_read_reg(ICM42605_ADDRESS,ICM42605_WHO_AM_I,SIMIIC);
        systick_delay_ms(STM0, 10);
        //��������ԭ�������¼���
        //1 ICM42605���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V

    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��λICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm42605_reset(void)
{
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_DEVICE_CONFIG,0x01);
    //���ٵȴ�1ms
    systick_delay_ms(STM0, 3);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ���øú���ǰ�����ȵ���ģ��IIC�ĳ�ʼ��
//-------------------------------------------------------------------------------------------------------------------
void icm42605_init(void)
{
    uint8 reg_val;

    simiic_init();
    systick_delay_ms(STM0, 10);  //�ϵ���ʱ

    //�Լ�
    icm42605_self1_check();

    //��λ
    icm42605_reset();

    //���õ�Դ����
    reg_val = simiic_read_reg(ICM42605_ADDRESS,ICM42605_PWR_MGMT0,SIMIIC);
    reg_val &= ~(1 << 5);//ʹ���¶Ȳ���
    reg_val |= ((3) << 2);//����GYRO_MODE  0:�ر� 1:���� 2:Ԥ�� 3:������
    reg_val |= (3);//����ACCEL_MODE 0:�ر� 1:�ر� 2:�͹��� 3:������
    //simiic_write_reg(ICM42605_ADDRESS,ICM42605_PWR_MGMT0,reg_val | 0x00);
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_PWR_MGMT0,0x1F);//LNģʽ(Low Nosie Mode)
    systick_delay_ms(STM0, 5);//��ʱ,������PWR��MGMT0�Ĵ����� 200us�ڲ������κζ�д�Ĵ����Ĳ���


    //���ò���
    //simiic_write_reg(ICM42605_ADDRESS,ICM20602_CONFIG,0x01);                   //176HZ 1KHZ
    //simiic_write_reg(ICM42605_ADDRESS,ICM20602_SMPLRT_DIV,0x07);               //�������� SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_GYRO_CONFIG0,0x06);//��2000 dps, 1kHz���
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_ACCEL_CONFIG0,0x26);//��8g , 1kHZ���
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_GYRO_CONFIG1,0xA0);//�¶ȴ��������ֵ�ͨ�˲�Ƶ������Ϊ10Hz
    simiic_write_reg(ICM42605_ADDRESS,ICM42605_GYRO_ACCEL_CONFIG0 ,0x11);//���ٶȼƺ������ǵ�ͨ��ֹƵ������, Ĭ��
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
//  @brief      ��ȡICM42605�¶�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void get_icm42605_tempdata(void)
{
    uint8 dat[2];

    simiic_read_regs(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, dat, 2, SIMIIC);
    icm4_temp = (int16)(((uint16)dat[0]<<8 | dat[1]));
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡICM42605���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ��ȡICM42605����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  ʹ��Ӳ��SPIͨ��
//-------------------------------------------------------------------------------------------------------------------
#define SPI_NUM         SPI_0
#define SPI_SCK_PIN     SPI0_SCLK_P20_11    //��ģ��SPC
#define SPI_MOSI_PIN    SPI0_MOSI_P20_14    //��ģ��SDI
#define SPI_MISO_PIN    SPI0_MISO_P20_12    //��ģ��SDO
#define SPI_CS_PIN      SPI0_CS2_P20_13     //��ģ��CS


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM42605 SPIд�Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
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
//  @brief      ICM42605 SPI���Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
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
//  @brief      ICM42605 SPI���ֽڶ��Ĵ���
//  @param      *val    �������ݵĵ�ַ
//  @param      num     ��ȡ����
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm4_spi_r_reg_bytes(uint8 * val, uint8 num)
{
    spi_mosi(SPI_NUM,SPI_CS_PIN,val,val,num,1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ICM42605�Լ캯��
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm42605_self3_check(void)
{
    uint8 dat=0;
    while(0x42 != dat)   //��ȡICM42605 ID
    {
        icm4_spi_r_reg_byte(ICM42605_WHO_AM_I,&dat);
        systick_delay_ms(STM0, 200);
        printf("ERROR:IMU Connection Bad\r\n");
        //��������ԭ�������¼���
        //1 ICM42605���ˣ�������µ������ĸ��ʼ���
        //2 ���ߴ������û�нӺ�
        //3 ��������Ҫ����������裬������3.3V
    }

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��λICM40605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm42605_reset_spi(void)
{
    icm4_spi_w_reg_byte(ICM42605_DEVICE_CONFIG,0x01);
    //���ٵȴ�1ms
    systick_delay_ms(STM0, 3);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʼ��ICM42605
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void icm42605_init_spi(void)
{
    uint8 reg_val;

    systick_delay_ms(STM0, 10);  //�ϵ���ʱ

    (void)spi_init(SPI_NUM, SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CS_PIN, 3, 10*1000*1000);//Ӳ��SPI��ʼ��

    //�Լ�
    icm42605_self3_check();

    //��λ
    icm42605_reset_spi();

    //���õ�Դ����
    icm4_spi_r_reg_byte(ICM42605_PWR_MGMT0,&reg_val);
    reg_val &= ~(1 << 5);//ʹ���¶Ȳ���
    reg_val |= ((3) << 2);//����GYRO_MODE  0:�ر� 1:���� 2:Ԥ�� 3:������
    reg_val |= (3);//����ACCEL_MODE 0:�ر� 1:�ر� 2:�͹��� 3:������
    //icm4_spi_w_reg_byte(ICM42605_PWR_MGMT0,reg_val | 0x00);
    icm4_spi_w_reg_byte(ICM42605_PWR_MGMT0,0x1F);
    systick_delay_ms(STM0, 5);//��ʱ,������PWR��MGMT0�Ĵ����� 200us�ڲ������κζ�д�Ĵ����Ĳ���

    icm4_spi_w_reg_byte(ICM42605_GYRO_CONFIG0,0x07);//��2000 dps, 200Hz���
    icm4_spi_w_reg_byte(ICM42605_ACCEL_CONFIG0,0x27);//��8g , 200HZ���
    icm4_spi_w_reg_byte(ICM42605_GYRO_CONFIG1,0xA0);//�¶ȴ��������ֵ�ͨ�˲�Ƶ������Ϊ10Hz
    icm4_spi_w_reg_byte(ICM42605_GYRO_ACCEL_CONFIG0 ,0x11);//���ٶȼƺ������ǵ�ͨ��ֹƵ������, Ĭ��

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
//  @brief      ��ȡICM42605�¶�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ��ȡICM42605���ٶȼ�����
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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
//  @brief      ��ȡICM42605����������
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:               ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
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

