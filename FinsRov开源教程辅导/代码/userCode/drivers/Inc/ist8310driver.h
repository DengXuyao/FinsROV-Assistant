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
 * @file    ist8310driver.h
 * @brief   IST8310 三轴磁力计驱动接口（DJI 开源机器人框架移植）。
 * @details 所属模块：userCode/drivers，实现位于 Src/ist8310driver.cpp，
 *          硬件访问经 MiddleWares/ist8310driver_middleware.h 抽象。
 *          本驱动只负责：复位后读取 WHO_AM_I 确认器件、按配置表写入并回读
 *          校验寄存器、把 I2C 读回的原始字节流换算为三轴磁场强度（uT），
 *          供 devices/IMU 的 AHRS 算法提供航向参考。
 * @note    IST8310 只支持 I2C 接口；原 DJI 版本曾通过 MPU6500 的 IIC_SLV0
 *          代理读取，本工程已改为 hi2c3 直接访问（见中间件实现）。
 * @note    数据就绪位 IST8310_DATA_READY_BIT=2 对应状态字节 bit2，
 *          与 BMI088 的状态位定义一起构成 IMU 数据就绪标志字。
 */
#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "main.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  float mag[3];
} ist8310_real_data_t;

/**
 * @brief 初始化磁力计：GPIO/通信初始化、RSTN 硬件复位、WHO_AM_I 校验、写配置寄存器并回读。
 * @return IST8310_NO_ERROR(0) 成功；IST8310_NO_SENSOR(0x40) 表示器件无响应；
 *         其它值为配置表回读校验失败时对应的错误码（见 .cpp 中的表）。
 * @note  每次写寄存器后延时 1 ms 再回读；sleepTime 为 50 ms 复位保持时间。
 */
/* ist8310driver.cpp 中各函数内部的执行流程步骤：
 * ist8310_init()     ：1) 调用中间件初始化钩子；2) RSTN 拉低/拉高各 50ms
 *                      完成硬件复位；3) 读 WHO_AM_I，不等于 0x10 返回
 *                      NO_SENSOR；4) 逐项写配置寄存器、延时、回读校验，
 *                      不一致返回该行错误码；5) 全通过返回 NO_ERROR。
 * ist8310_read_over()：1) 检查 status_buf[0] bit0；2) 就绪则置 status 的
 *                      DATA_READY_BIT；3) 把 6 字节按小端拼成三个 int16
 *                      并乘 MAG_SEN 写入 mag[3]；4) 未就绪仅清状态位。
 * ist8310_read_mag() ：1) 从 0x03 连续读 6 字节；2) 逐轴小端拼接乘 MAG_SEN
 *                      写入 mag[3]。
 */
extern uint8_t ist8310_init(void);
/**
 * @brief 从「状态字节 + 6 字节三轴数据」缓冲区解析磁场值。
 * @param status_buf         接收缓冲区，bit0 为数据就绪标志。
 * @param ist8310_real_data  [输出] 更新 status 中 DATA_READY_BIT 与 mag[3]（uT）。
 * @note  未就绪时只清除状态位，不修改 mag；三轴均为小端 16 位有符号数。
 */
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
/**
 * @brief 直接从 0x03 起连续读 6 字节并换算三轴磁场。
 * @param mag [输出] float[3]，单位 uT。
 * @note  阻塞式 I2C 读取，不检查状态位，适合在控制周期中主动采样。
 */
extern void ist8310_read_mag(float mag[3]);
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *mpu6500_real_data);
extern void ist8310_read_mag(float mag[3]);
#endif
