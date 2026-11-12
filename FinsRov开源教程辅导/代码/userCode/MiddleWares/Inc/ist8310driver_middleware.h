/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310�������м�㣬���IST8310��IICͨ��,��Ϊ����MPU6500��SPIͨ��
  *             �������õ���ͨ��mpu6500��IIC_SLV0��ɶ�ȡ��IIC_SLV4���д�롣
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
 * @file    ist8310driver_middleware.h
 * @brief   IST8310 磁力计的硬件抽象层（I2C / GPIO / 延时）接口。
 * @details 所属模块：userCode/MiddleWares，实现位于
 *          Src/ist8310driver_middleware.cpp。把驱动层与具体 MCU 外设解耦：
 *          单字节/多字节寄存器读写、RSTN 复位引脚控制、毫秒与微秒延时，
 *          以及 GPIO/I2C 初始化钩子（当前由 CubeMX 完成，实现为空）。
 * @note    IST8310 只支持 I2C：地址 0x0E<<1（8 位形式），读命令最高位为 1；
 *          实际通信使用 HAL_I2C_Mem_Read/Write 与 hi2c3。
 * @note    原 DJI 版本通过 MPU6500 的 IIC_SLV0/SLV4 间接访问磁力计，
 *          本工程已改为 MCU 直接 I2C 访问，头文件中的历史说明保留。
 */
#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "main.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)  //IST8310��IIC��ַ
#define IST8310_IIC_READ_MSB (0x80) //IST8310��SPI��ȡ���͵�һ��bitΪ1

extern void ist8310_GPIO_init(void); //ist8310��io��ʼ��
extern void ist8310_com_init(void);  //ist8310��ͨѶ��ʼ��
/**
 * @name I2C 寄存器访问接口
 * @details single 版本读写一个字节；muli 版本（源码拼写保留）从 reg 起
 *          连续读/写 len 字节，利用 IST8310 的寄存器地址自动递增。
 *          devAddress 固定为 IST8310_IIC_ADDRESS，寄存器地址长度为 8 位，
 *          所有操作超时 100 ms。
 */
/**
 * @brief 读单个寄存器。
 * @param reg 寄存器地址。
 * @return 读回的字节。
 */
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
/** @brief 写单个寄存器：reg 写入 data。 */
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
/**
 * @brief 连续读多个寄存器。
 * @param reg 起始寄存器地址。
 * @param buf [输出] 接收缓冲区。
 * @param len 要读取的字节数。
 */
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
/**
 * @brief 连续写多个寄存器。
 * @param reg  起始寄存器地址。
 * @param data 待写入数据首地址。
 * @param len  写入字节数。
 */
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
/* ist8310driver_middleware.cpp 中函数内部的执行流程步骤：
 * GPIO_init/com_init()：函数体为空（CubeMX 已完成初始化）。
 * IIC_read_single_reg() / write_single_reg()：1) 以 hi2c3 + 器件地址 +
 *                      8 位寄存器地址调用 HAL_I2C_Mem_Read/Write；
 *                      2) 单字节读把结果返回，写则直接返回。
 * IIC_read_muli_reg() / write_muli_reg()：同上，但一次连续读/写 len 字节，
 *                      利用器件寄存器地址自动递增。
 * delay_ms()/delay_us()：前者转发 HAL_Delay；后者 for+while 空循环近似延时。
 * RST_H()/RST_L()    ：HAL_GPIO_WritePin 把 RSTN 置高/置低控制硬件复位。
 */
extern void ist8310_delay_ms(uint16_t ms);
extern void ist8310_delay_us(uint16_t us);
extern void ist8310_RST_H(void); //��λIO �ø�
extern void ist8310_RST_L(void); //��λIO �õ� �õػ�����ist8310����

#endif
