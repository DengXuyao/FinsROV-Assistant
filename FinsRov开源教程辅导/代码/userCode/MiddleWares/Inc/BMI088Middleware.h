/**
 * @file    BMI088Middleware.h
 * @brief   BMI088 驱动与 STM32 HAL 之间的硬件抽象层（中间件）接口。
 * @details 所属模块：userCode/MiddleWares。BMI088driver.cpp 只处理传感器协议，
 *          所有与板级硬件相关的操作（GPIO 片选、SPI 单字节收发、延时）都通过
 *          本头文件声明的函数完成，便于把同一份驱动移植到不同 MCU/总线。
 *          实现位于 Src/BMI088Middleware.cpp。
 * @note    通过宏选择总线：当前定义 BMI088_USE_SPI（BMI088_USE_IIC 被注释）。
 *          只有 SPI 模式下才声明 4 个片选控制函数与 read_write_byte()，
 *          IIC 分支目前为空，切换到 IIC 需自行补齐对应的读写接口。
 * @note    BMI088 的加速度计与陀螺仪是两个独立器件、两个独立 CS 引脚，
 *          因此片选函数成对区分 ACCEL / GYRO；读写前拉低、完成后拉高。
 */

#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

/* 经 main.h 引入 HAL 类型与 CubeMX 生成的 CS1_ACCEL / CS1_GYRO 引脚宏 */
#include "main.h"

/* 总线选择：二者互斥，当前使用 SPI1；切换到 IIC 时需注释本行并实现 IIC 分支 */
#define BMI088_USE_SPI
//#define BMI088_USE_IIC

/** @brief 片选/复位等 GPIO 初始化钩子；CubeMX 已完成 GPIO 初始化，故实现为空。 */
/* BMI088Middleware.cpp 中函数内部的执行流程步骤：
 * GPIO_init/com_init()：函数体为空，实际初始化由 CubeMX 完成。
 * delay_ms()          ：把 ms 直接转发给 HAL_Delay()，依赖 SysTick。
 * delay_us()          ：1) for 循环 us 次；2) 每轮 a=10 空转 a-- 消耗周期。
 * ACCEL/GYRO_NS_L/H()：用 HAL_GPIO_WritePin 把对应 CS 引脚置 RESET/SET。
 * read_write_byte()   ：1) 准备 rx_data；2) HAL_SPI_TransmitReceive 阻塞
 *                      收发 1 字节；3) 返回 MISO 读回值。
 */
extern void BMI088_GPIO_init(void);
/** @brief 通信外设（SPI/IIC）初始化钩子；CubeMX 已完成初始化，故实现为空。 */
extern void BMI088_com_init(void);
/**
 * @brief 毫秒级阻塞延时，直接转发 HAL_Delay()。
 * @param ms 延时毫秒数。
 * @note  依赖 SysTick 中断，不能在 SysTick 中断上下文内调用。
 */
extern void BMI088_delay_ms(uint16_t ms);
/**
 * @brief 微秒级粗略延时，基于空循环软件延时。
 * @param us 期望延时微秒数；精度随主频与编译优化等级变化（按 168 MHz 标定）。
 */
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
/* 加速度计片选 CS1_ACCEL 拉低（选中，开始一帧 SPI 通信） */
extern void BMI088_ACCEL_NS_L(void);
/* 加速度计片选 CS1_ACCEL 拉高（释放，结束一帧 SPI 通信） */
extern void BMI088_ACCEL_NS_H(void);

/* 陀螺仪片选 CS1_GYRO 拉低（选中） */
extern void BMI088_GYRO_NS_L(void);
/* 陀螺仪片选 CS1_GYRO 拉高（释放） */
extern void BMI088_GYRO_NS_H(void);

/**
 * @brief SPI 全双工收发一个字节（阻塞式 HAL_SPI_TransmitReceive）。
 * @param reg 实际实现中参数名为 txdata，即待发送的 1 字节（寄存器地址或数据）。
 * @return 同一时钟周期内从 MISO 读回的 1 字节。
 * @note  SPI 读寄存器的首字节需带读标志（最高位置 1），由驱动层拼好后传入；
 *        超时固定 1000 ms；调用前后由上层控制对应 CS 引脚。
 */
extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)
/* IIC 分支预留：目前没有声明任何函数，需按 BMI088driver.cpp 的调用补齐 */
#endif

#endif
