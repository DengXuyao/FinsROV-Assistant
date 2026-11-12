//
// Created by admin on 2023/11/28.
//
/**
 * @file    Legacy.h
 * @brief   遗留兼容层：旧版本工程的全局类型、Flash 参数存取与调试输出接口。
 * @details 所属模块：userCode/devices。汇集了不便归入具体设备类的历史接口，
 *          包括片内 Flash 参数区定义、磁力计数据结构、电源电压 ADC 监测、
 *          格式化串口输出以及 Flash 读写封装，实现位于 Src/Legacy.cpp。
 * @note    本文件把 HAL/标准库头文件放在 include guard 之前（历史写法），
 *          多次包含时这些头文件自身有保护，故不影响编译；新代码不要模仿。
 * @note    FLASH_SECTOR_9_ADDRESS 指向 STM32F407 的扇区 9 起始地址
 *          (0x080A0000)，写 Flash 前必须擦除整个扇区，且会丢失该扇区内其它数据。
 */
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* 可变参数格式化输出、字符串/数学函数（usart_printf、Flash 校验等使用） */
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifndef CONTROL_FRAME_MAIN_LEGACY_H
#define CONTROL_FRAME_MAIN_LEGACY_H

/* Flash 参数保存区：扇区 9 首地址，断电后仍需保留的标定数据放这里 */
#define FLASH_SECTOR_9_ADDRESS 0x080A0000

/**
 * @struct flash_data_t
 * @brief 需要持久化到 Flash 的标定参数集合。
 * @note  pressure_offset[4] 为 4 路水压传感器的零点偏移；结构体中其余字段
 *        （robot_ID / yaw_zero / pitch_zero / pat）为旧版本保留，已注释。
 *        改动结构体大小后需同步检查 bsp_flash_read/write 的读写长度。
 */
typedef struct {
    /*uint32_t robot_ID;
    uint32_t yaw_zero;
    uint32_t pitch_zero;
    uint32_t pat[125];*/
    uint32_t pressure_offset[4];
}flash_data_t;

/**
 * @struct ist8310_t
 * @brief 磁力计三轴数据（单位依调用方约定）及伴随温度。
 * @note  为旧版 IST8310 接口保留的轻量结构，新代码多直接使用 float mag[3]。
 */
typedef struct {
    float x,y,z;
    float temp;
}ist8310_t;

/* 全局 Flash 参数镜像：启动时读出，标定完成后写回 */
extern flash_data_t flashData;

/** @brief 触发/读取 ADC 电源电压监测，更新电池电压等状态（由主循环周期调用）。 */
/* Legacy.cpp 中各函数内部的执行流程步骤（实现见 Src/Legacy.cpp）：
 * bsp_ADC_vccMoni()：1) 前 200 次调用累加 ADC1 内部参考电压；2) 求平均得
 *                    vrefValue 并停止 ADC1；3) 之后每次读 ADC3 并按
 *                    raw*1.25/vrefValue 换算 vccMoni；4) 按分压比算 vccBat。
 * usart_printf()   ：1) 准备静态 tx_buf 与 va_list；2) va_start 取可变参数；
 *                    3) 格式化与发送语句当前被注释；4) va_end 后返回。
 * bsp_flash_write()：1) HAL_FLASH_Unlock；2) 配置扇区擦除参数并擦除扇区 9；
 *                    3) 按 uint32_t 逐字写入 flashData 内容；4) HAL_FLASH_Lock。
 * bsp_flash_read() ：1) 以 FLASH_SECTOR_9_ADDRESS 为源；2) memcpy 到 flashData。
 */
extern void bsp_ADC_vccMoni();
/**
 * @brief 类 printf 的格式化串口输出。
 * @param fmt 格式字符串。
 * @param ... 可变参数。
 * @note  内部用 va_list + vsnprintf 格式化后经串口发送，注意缓冲区长度与
 *        在中断上下文调用的安全性。
 */
extern void usart_printf(const char *fmt,...);
/**
 * @brief 把 flash_data_t 写入片内 Flash 扇区 9。
 * @param _flashData 待写入的数据指针。
 * @note  会先解锁、擦除扇区再按字写入并上锁；写入期间不要有高频中断访问 Flash。
 */
extern void bsp_flash_write(flash_data_t *_flashData);
/**
 * @brief 从片内 Flash 扇区 9 读出参数到 flash_data_t。
 * @param _flashData 接收数据的结构体指针。
 * @note  调用前 Flash 内容可能全为 0xFF（出厂未标定），调用方需校验有效性。
 */
extern void bsp_flash_read(flash_data_t *_flashData);


#endif //CONTROL_FRAME_MAIN_LEGACY_H
