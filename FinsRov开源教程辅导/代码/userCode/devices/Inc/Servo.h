//
// Created by admin on 2023/12/5.
//

/**
 * @file    Servo.h
 * @brief   舵机设备：直连 PWM 版 Servo 与经 I2C 扩展板版 Servo_I2C。
 * @details 所属模块：userCode/devices，两个类都继承 Device，实现位于 Src/Servo.cpp。
 *          Servo 直接使用片上定时器 PWM 通道输出，Servo_I2C 则通过
 *          TCA9548A 选通 + PCA9685 输出；两者对外接口一致，便于按硬件版本切换。
 * @note    舵机 PWM 脉宽 500~2500（对应 0~180°）线性映射；SERVO_NUM 在
 *          Usermain.h 中定义为 4，data[] 数组长度与之绑定。
 * @note    data_extract() 把串口字节流解析成各路 PWM 目标值，随后由
 *          Handle() 输出到硬件；Receive() 负责调用解析流程。
 */

#ifndef CONTROL_FRAME_MAIN_SERVO_H
#define CONTROL_FRAME_MAIN_SERVO_H

#include "Usermain.h"
#include "Extension.h"
#include "PID.h"
#include <cstdint>

/**
 * @class Servo
 * @brief 片上定时器 PWM 直驱的舵机控制器。
 */
/* Servo.cpp 中各函数内部的执行流程步骤（实现见 Src/Servo.cpp）：
 * Servo::Init()      ：1) 4 路 data[] 置中位 1500；2) 写 htim8_CH3 与
 *                      htim1_CH2/3/4 四个比较寄存器；3) 启动 USART6 空闲接收。
 * Servo_I2C::Init()  ：1) 按 Robot_Version 复制 ID 映射表；2) TCA 选通道 4、
 *                      PCA9685 设 50Hz；3) 各通道按脉宽换算计数值并置中位；
 *                      4) 启动 USART6 空闲接收。
 * Receive()          ：调用 data_extract(RxBuffer, data, SERVO_NUM) 解析指令。
 * data_extract()     ：1) 判断帧头 "MOT:"；2) 用 strtok 按逗号切分；
 *                      3) atoi 逐路写入 data[]（I2C 版对第 3 路取 3000-data）。
 * Servo::Handle()    ：把 data[] 直接写入四个定时器比较寄存器。
 * Servo_I2C::Handle()：1) 选通 TCA 通道 4；2) 检查全部值在 500~2500；
 *                      3) 合法则按 ID[] 换算计数值并写 PCA9685，否则整组丢弃。
 */
class Servo : public Device
{

public:
    // uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data[SERVO_NUM]; // PWM值，500-2500对应0-180度，线性关系
    /**
     * @brief 从串口接收缓冲区提取 num 路舵机 PWM 目标值。
     * @param rx   原始字节缓冲区（通常为 RxBuffer）。
     * @param data [输出] 解析后的 PWM 数组。
     * @param num  要解析的舵机路数。
     */
    void data_extract(uint8_t *rx, int32_t *data, int32_t num);

    /** @brief 初始化 PWM 定时器与各舵机中位。 */
    void Init();
    /** @brief 把 data[] 写入各 PWM 比较寄存器（周期调用）。 */
    void Handle();
    /** @brief 接收/解析串口数据并更新 data[]。 */
    void Receive();
    // static Servo servo;
};

/**
 * @class Servo_I2C
 * @brief 通过 I2C 扩展板（PCA9685）驱动的舵机控制器。
 * @note  ID[] 保存每路舵机在 PCA9685 上的通道号，data[] 保存目标 PWM。
 */
class Servo_I2C : public Device
{

public:
    // uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t ID[SERVO_NUM];   // 舵机编号
    int32_t data[SERVO_NUM]; // PWM值，500-2500对应0-180度，线性关系
    /**
     * @brief 从串口字节流提取各路舵机 PWM 值（同 Servo::data_extract）。
     * @param rx   原始字节缓冲区。
     * @param data [输出] 解析后的 PWM 数组。
     * @param num  舵机路数。
     */
    void data_extract(uint8_t *rx, int32_t *data, int32_t num);

    /** @brief 初始化 PCA9685 频率与各通道初始占空比。 */
    void Init();
    /** @brief 按 ID[]/data[] 通过 PCA9685 更新各路 PWM 输出。 */
    void Handle();
    /** @brief 接收/解析串口指令，更新 data[]。 */
    void Receive();
};

#endif // CONTROL_FRAME_MAIN_SERVO_H
