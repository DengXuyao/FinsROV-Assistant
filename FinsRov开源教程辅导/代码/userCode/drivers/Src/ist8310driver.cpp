/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310.c/h
  * @brief      IST8310����������������������ʼ���������������ݺ�����ͨ�Ŷ�ȡ����
  *             �������ǽ�MPU6500 IIC_SLV0����Ϊ�Զ���ȡIST8310���ݣ���ȡ
  *             MPU_EXT_SENS_DATA_00������IST8310��Status��ͨ���жϱ�־λ��������
  *             ���ݡ�
  * @note       IST8310ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

/**
 * @file    ist8310driver.cpp
 * @brief   IST8310 磁力计的初始化、寄存器配置与数据换算实现。
 * @details 所属模块：userCode/drivers，接口见 Inc/ist8310driver.h，底层
 *          I2C/延时/GPIO 由 MiddleWares/ist8310driver_middleware.cpp 提供。
 *          ist8310_init()：拉低/拉高 RSTN 完成硬件复位 -> 读 WHO_AM_I ->
 *          按 ist8310_write_reg_data_error 表逐项写寄存器并回读校验；
 *          ist8310_read_over()：从「状态 + 6 字节数据」缓冲区解析三轴磁场；
 *          ist8310_read_mag()：直接从 0x03 起连续读 6 字节得到三轴磁场。
 * @note    三轴数据均为小端 16 位有符号数，乘以 MAG_SEN(0.3) 换算为 uT；
 *          status_buf[0] 的 bit0 为数据就绪标志，未就绪时清除状态位但不更新数据。
 * @note    配置表每行 3 项：{寄存器地址, 期望值, 校验失败时的错误码}，
 *          回读不一致时直接返回对应错误码，便于定位初始化失败环节。
 */
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"

#define MAG_SEN 0.3f //ת���� uT

#define IST8310_WHO_AM_I 0x00       //ist8310 who am I �Ĵ���
#define IST8310_WHO_AM_I_VALUE 0x10 //�豸 ID

#define IST8310_WRITE_REG_NUM 4 //IST8310��Ҫ���õļĴ�����Ŀ

static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] =
    {
        {0x0B, 0x08, 0x01},
        {0x41, 0x09, 0x02},
        {0x42, 0xC0, 0x03},
        {0x0A, 0x0B, 0x04}};

/**
 * @brief 初始化 IST8310：复位、ID 校验、写配置寄存器并逐项回读。
 * @return IST8310_NO_ERROR(0) 成功；IST8310_NO_SENSOR(0x40) 表示 WHO_AM_I
 *         读回不等于 0x10；其它为配置表某项校验失败对应的错误码。
 * @note  RSTN 先拉低 50 ms 再拉高 50 ms 完成硬件复位；每项配置写入后
 *        延时 1 ms 再读回，不一致立即返回该行第 3 列错误码。
 */
uint8_t ist8310_init(void)
{
    /* 函数内部流程步骤：
     * 步骤 1：定义 wait_time=1ms、sleepTime=50ms，以及 res / writeNum 局部变量。
     * 步骤 2：调用 ist8310_GPIO_init() 与 ist8310_com_init() 初始化抽象层。
     * 步骤 3：RSTN 拉低 50 ms 再拉高 50 ms，完成磁力计硬件复位。
     * 步骤 4：读 WHO_AM_I 寄存器，若不等于 0x10 立即返回 IST8310_NO_SENSOR。
     * 步骤 5：按 write_reg_data_error 表循环 4 次：写寄存器 -> 延时 1 ms
     *         -> 回读 -> 延时 1 ms -> 与期望值比较，不一致返回该行错误码。
     * 步骤 6：全部通过则返回 IST8310_NO_ERROR。
     */
    static const uint8_t wait_time = 1;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

    ist8310_GPIO_init();
    ist8310_com_init();

    ist8310_RST_L();
    ist8310_delay_ms(sleepTime);
    ist8310_RST_H();
    ist8310_delay_ms(sleepTime);

    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE)
    {
        return IST8310_NO_SENSOR;
    }
    ist8310_delay_ms(wait_time);
    //set mpu6500 sonsor config and check
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++)
    {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
        ist8310_delay_ms(wait_time);
        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        ist8310_delay_ms(wait_time);
        if (res != ist8310_write_reg_data_error[writeNum][1])
        {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }

    return IST8310_NO_ERROR;
}

void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data)
{
    /* 函数内部流程步骤：
     * 步骤 1：检查 status_buf[0] 的 bit0 判断数据是否就绪。
     * 步骤 2：就绪则把 status 的 IST8310_DATA_READY_BIT(bit2) 置 1。
     * 步骤 3：依次取 status_buf[1..2]、[3..4]、[5..6] 拼成三个小端
     *         int16_t（高字节<<8 | 低字节），分别乘 MAG_SEN(0.3) 写入 mag[0..2]。
     * 步骤 4：未就绪时只清除 status 的 DATA_READY_BIT，保持 mag 原值不变。
     */

    if (status_buf[0] & 0x01)
    {
        int16_t temp_ist8310_data = 0;
        ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

        temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
        ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
        ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
        temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
        ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
    }
    else
    {
        ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
    }
}

/**
 * @brief 直接从磁力计数据寄存器读取并换算三轴磁场。
 * @param mag [输出] float[3]，单位 uT（原始 int16 * MAG_SEN）。
 * @details 从 0x03 起连续读 6 字节，按 X/Y/Z 各两个字节小端拼接为
 *          int16_t 后乘 0.3；不检查状态位，由调用方保证数据已更新。
 * @note  与 ist8310_read_over() 的区别：后者处理「状态字节 + 数据」的
 *        代理读取帧，本函数用于 MCU 直接发起的 I2C 采样。
 */
void ist8310_read_mag(float mag[3])
{
    /* 函数内部流程步骤：
     * 步骤 1：准备 6 字节缓冲区 buf 与临时 int16_t 变量。
     * 步骤 2：调用 ist8310_IIC_read_muli_reg(0x03, buf, 6) 连续读三轴数据。
     * 步骤 3：依次把 buf[1]<<8|buf[0]、buf[3]<<8|buf[2]、buf[5]<<8|buf[4]
     *         拼成小端有符号数，乘 MAG_SEN(0.3) 写入 mag[0..2]（uT）。
     */
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp_ist8310_data;
}
