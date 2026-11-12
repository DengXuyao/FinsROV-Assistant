/**
 * @file    BMI088Middleware.cpp
 * @brief   BMI088 硬件抽象层的 STM32 HAL 实现。
 * @details 所属模块：userCode/MiddleWares，接口见 Inc/BMI088Middleware.h。
 *          本文件把驱动层的硬件请求映射到具体的 GPIO、SPI1 与延时实现：
 *          CS1_ACCEL / CS1_GYRO 两个片选引脚、hspi1 阻塞式单字节收发、
 *          HAL_Delay 毫秒延时以及基于空循环的微秒延时。
 * @note    GPIO 与 SPI 外设均由 CubeMX 生成的代码初始化，因此 GPIO_init() /
 *          com_init() 为空实现，仅保留移植接口；BMI088_init() 仍会调用它们。
 * @note    本文件的读写是阻塞式单字节 SPI（非 bsp 层的 DMA 方式），
 *          主要用于传感器初始化、自检和寄存器配置等低频操作。
 * @note    微秒延时用 `int a = 10; while(a--);` 近似，常数按 168 MHz 主频
 *          经验标定，编译器优化等级或主频变化后需重新校准，不可用于精确定时。
 */
#include "BMI088Middleware.h"
#include "main.h"

/* CubeMX 在 spi.c 中定义的 SPI1 句柄，供阻塞式收发使用 */
extern SPI_HandleTypeDef hspi1;

/**
 * @brief GPIO 初始化钩子。
 * @note  CubeMX 的 MX_GPIO_Init() 已配置 CS1_ACCEL / CS1_GYRO 为输出，
 *        因此这里无需额外操作；移植到无 CubeMX 的平台时在此实现引脚初始化。
 */
void BMI088_GPIO_init(void)
{
    /* 函数内部流程步骤（两个初始化钩子均为空实现）：
     * 步骤 1：BMI088_GPIO_init() 不执行任何操作——CS1_ACCEL / CS1_GYRO
     *         的 GPIO 已由 CubeMX 的 MX_GPIO_Init() 配置完成。
     * 步骤 2：BMI088_com_init() 同样为空——SPI1 由 MX_SPI1_Init() 初始化。
     * 步骤 3：BMI088_init() 仍会调用二者作为移植钩子；换平台时在此补齐。
     */

}

/**
 * @brief 通信接口初始化钩子。
 * @note  CubeMX 的 MX_SPI1_Init() 已完成 SPI1 配置，此处留空；
 *        若驱动改用 IIC，则在此初始化对应 I2C 句柄。
 */
void BMI088_com_init(void)
{


}

/**
 * @brief 毫秒级延时。
 * @param ms 延时毫秒数。
 * @note  直接转发 HAL_Delay()，依赖 SysTick；BMI088 上电、软复位与模式切换
 *        的等待（如 BMI088_LONG_DELAY_TIME）都通过它实现。
 */
void BMI088_delay_ms(uint16_t ms)
{
    /* 函数内部流程步骤：
     * 步骤 1：把毫秒数原样传给 HAL_Delay()。
     * 步骤 2：HAL_Delay 依赖 SysTick 递减计数，阻塞到计数值归零后返回；
     *         主要用于传感器软复位后的长等待（BMI088_LONG_DELAY_TIME）。
     */
    HAL_Delay(ms);
}

/**
 * @brief 微秒级软件延时（阻塞空循环）。
 * @param us 期望的微秒数，每微秒执行约 10 次递减循环。
 * @note  循环变量 a 为局部变量且无 volatile，激进优化可能把循环整体优化掉；
 *        若发现时序不满足芯片要求（如 CS 建立/保持时间），应改用定时器或
 *        DWT 计数实现。
 */
void BMI088_delay_us(uint16_t us)
{
    /* 函数内部流程步骤：
     * 步骤 1：外层 for 循环执行 us 次，每次近似对应 1 微秒。
     * 步骤 2：每轮令 a=10（按 168 MHz 主频经验标定的循环次数）。
     * 步骤 3：while (a--) 空转消耗指令周期，循环结束即完成延时返回。
     * 注意：a 非 volatile，编译优化可能削弱延时效果，不可用于精确定时。
     */
    uint16_t i;
    for (i = 0; i < us; i++ )
    {
        int a = 10;  //delay based on mian clock, 168Mhz
        while (a-- );
    }
}




/**
 * @brief 选中 BMI088 加速度计：CS1_ACCEL 输出低电平。
 * @note  与 BMI088_ACCEL_NS_H() 成对使用；拉低后需按器件时序要求稍作延时
 *        再发起 SPI 传输。
 */
void BMI088_ACCEL_NS_L(void)
{
    /* 四个片选函数的内部流程步骤（结构相同，仅引脚/电平不同）：
     * 步骤 1：ACCEL_NS_L / GYRO_NS_L 用 HAL_GPIO_WritePin 输出
     *         GPIO_PIN_RESET，选中对应器件开始一帧 SPI 通信。
     * 步骤 2：ACCEL_NS_H / GYRO_NS_H 输出 GPIO_PIN_SET 释放片选，结束帧。
     * 步骤 3：引脚来自 CubeMX 的 CS1_ACCEL / CS1_GYRO 宏；
     *         中间件的 GPIO_init() 为空，因为 CubeMX 已完成配置。
     */
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
/** @brief 释放 BMI088 加速度计：CS1_ACCEL 输出高电平，结束本次 SPI 帧。 */
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

/**
 * @brief 选中 BMI088 陀螺仪：CS1_GYRO 输出低电平。
 * @note  加速度计与陀螺仪是独立从设备，两者片选不可互相替代。
 */
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
/** @brief 释放 BMI088 陀螺仪：CS1_GYRO 输出高电平，结束本次 SPI 帧。 */
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/**
 * @brief 通过 SPI1 同步发送并接收 1 字节。
 * @param txdata 待发送字节（寄存器地址、读命令或写入数据）。
 * @return 同一时刻 MISO 上采样到的字节；写操作时的返回值一般无意义。
 * @note  使用 HAL_SPI_TransmitReceive() 阻塞传输，超时 1000 ms；
 *        SPI 为全双工，主机发送的同时必然读回一个字节。
 */
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明局部 uint8_t rx_data 存放 MISO 读回值。
     * 步骤 2：调用 HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000)
     *         阻塞收发 1 字节；SPI 全双工，发送的同时必然采样到一字节。
     * 步骤 3：返回 rx_data（写寄存器时该值无意义，读寄存器时即为数据）。
     */
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

