//
// Created by admin on 2023/10/29.
//

/**
 * @file    Extension.h
 * @brief   I2C 扩展板驱动接口：TCA9548A 多路复用器 + PCA9685 多路 PWM。
 * @details 所属模块：userCode/devices。用于扩展舵机/推进器等执行机构的
 *          PWM 通道：先通过 TCA9548A 选通某一路 I2C 总线，再访问该总线上的
 *          PCA9685（16 路 12 位 PWM 控制器）设置各通道的 ON/OFF 计数值。
 *          实现位于 Src/Extension.cpp，底层使用 HAL_I2C 与各中间件接口。
 * @note    TCA9548A 的通道参数是位掩码（可同时选通多路），PCA9685 的
 *          num 为 0~15 的通道号；地址宏均为 HAL 需要的 8 位形式（7 位地址<<1）。
 * @note    文件末尾被注释的 RxBuffer_servo / I2C_Extension* 是旧版本接口残留，
 *          当前有效接口为下面 5 个 TCA_/PCA_ 函数。
 */

#ifndef CONTROL_FRAME_MAIN_EXTENSION_H
#define CONTROL_FRAME_MAIN_EXTENSION_H

#include "Usermain.h"



/* TCA9548A 8 位 I2C 地址（0x70 << 1 中的 0x70 此处直接按 HAL 形式给出，依接线而定） */
#define TCA9548A_ADDR         0x70

/* TCA9548A 通道选通掩码：写入控制寄存器的一个字节，每位置 1 选通对应总线 */
#define TCA9548A_CHANNEL_0          0x01
#define TCA9548A_CHANNEL_1          0x02
#define TCA9548A_CHANNEL_2          0x04
#define TCA9548A_CHANNEL_3          0x08
#define TCA9548A_CHANNEL_4          0x10
#define TCA9548A_CHANNEL_5          0x20
#define TCA9548A_CHANNEL_6          0x40
#define TCA9548A_CHANNEL_7          0x80

/* PCA9685 的 8 位 I2C 地址（7 位 0x40 左移 1 位） */
#define PCA9685_ADDR 0x80

/* PCA9685 MODE1 控制寄存器：用于设置休眠/自动地址递增/响应方式 */
#define PCA9685_MODE1 0x0
/* 预分频寄存器：决定 16 路 PWM 的共同频率，写入前需先进入 SLEEP 模式 */
#define PCA9685_PRESCALE 0xFE

/* 第 0 路 PWM 的 ON/OFF 计数值寄存器（低/高字节），
 * 其余通道地址按 4 字节步进：LEDn_ON_L = LED0_ON_L + 4*n */
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

/*

extern uint8_t RxBuffer_servo[];


extern void I2C_Extension_init();
extern void I2C_Extension();*/

/**
 * @brief 选通 TCA9548A 的 I2C 通道。
 * @param channel 通道位掩码，可用 TCA9548A_CHANNEL_0~7 或它们的按位或；传 0 关闭全部通道。
 * @note  切换通道后需留出总线稳定时间；同一总线上器件地址冲突时靠该芯片隔离。
 */
/* Extension.cpp 中各函数内部的执行流程步骤（实现见 Src/Extension.cpp）：
 * TCA_SetChannel()：1) data = 0x01 << channel 得到单通道掩码；
 *                   2) HAL_I2C_Master_Transmit 把该字节写给 TCA9548A。
 * PCA_Read()      ：1) 先发 1 字节寄存器地址；2) 再收 1 字节并返回。
 * PCA_Write()     ：1) 组装 {寄存器地址, 数据} 两字节；2) 一次主发送写出。
 * PCA_Setfreq()   ：1) 频率乘修正系数；2) 算 prescale 并四舍五入；
 *                   3) 读 MODE1 并置 SLEEP；4) 写 PRESCALE；
 *                   5) 恢复 MODE1、延时 2ms、写 oldmode|0xA1 唤醒。
 * PCA_Setpwm()    ：1) 按 LED0_ON_L + 4*num 计算该通道 4 个寄存器地址；
 *                   2) 依次写 ON_L/ON_H/OFF_L/OFF_H（低字节在前）。
 */
extern void TCA_SetChannel(uint8_t channel);
/**
 * @brief 设置 PCA9685 某一路 PWM 的导通/关断计数。
 * @param num 通道号 0~15。
 * @param on  12 位导通计数值（0~4095）。
 * @param off 12 位关断计数值（0~4095），off-on 决定脉宽。
 * @note  通过一次写 LEDn_ON_L 起的 4 个连续寄存器完成，依赖 MODE1 的 AI 位。
 */
extern void PCA_Setpwm(uint8_t num, uint32_t on, uint32_t off);
/**
 * @brief 向 PCA9685 指定寄存器写入一字节。
 * @param adrr 寄存器地址（源码拼写保留）。
 * @param data 待写入的数据。
 */
extern void PCA_Write(uint8_t adrr,uint8_t data);
/**
 * @brief 设置 PCA9685 的 PWM 输出频率。
 * @param freq 目标频率（Hz），典型舵机为 50 Hz。
 * @note  内部按 25 MHz 内部振荡器计算 prescale = round(25e6/(4096*freq)) - 1，
 *        修改前须令 MODE1 进入 SLEEP，写完再恢复并等待振荡器稳定。
 */
extern void PCA_Setfreq(float freq);
/**
 * @brief 读取 PCA9685 指定寄存器的一个字节。
 * @param adrr 寄存器地址。
 * @return 读回的寄存器值，常用于校验配置是否写入成功。
 */
extern uint8_t PCA_Read(uint8_t adrr);




#endif //CONTROL_FRAME_MAIN_EXTENSION_H
