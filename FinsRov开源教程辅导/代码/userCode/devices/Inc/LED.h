//
// Created by admin on 2023/12/9.
//

/**
 * @file    LED.h
 * @brief   板载 RGB（aRGB）状态指示灯设备类。
 * @details 所属模块：userCode/devices，继承 Device 抽象基类。
 *          实现位于 Src/LED.cpp：用 TIM5 的通道 1/2/3 三路 PWM 分别驱动
 *          蓝/绿/红三个颜色通道，颜色值采用 0xAARRGGBB 格式（高 8 位为亮度），
 *          Handle() 周期性调用跑马灯函数实现呼吸/渐变效果。
 * @note    LED_PERIOD 宏在 Usermain.h 中定义，决定一轮颜色循环的周期（ms）。
 */

#ifndef CONTROL_FRAME_MAIN_LED_H
#define CONTROL_FRAME_MAIN_LED_H

#include "Usermain.h"

/**
 * @class LED
 * @brief RGB 指示灯设备，Device 接口的具体实现。
 */
/* LED.cpp 中各函数内部的执行流程步骤（实现见 Src/LED.cpp）：
 * Init()            ：依次 HAL_TIM_PWM_Start 启动 TIM5 的 CH1/CH2/CH3。
 * Handle()          ：以 LED_PERIOD 为参数调用 aRGB_led_change()。
 * aRGB_led_show()   ：1) 拆出 alpha、R、G、B；2) 各分量乘 alpha；
 *                     3) 按 CH1=蓝/CH2=绿/CH3=红写比较寄存器。
 * aRGB_led_change() ：1) 计算与上次调用的时间差，首次调用只记时间戳返回；
 *                     2) 把时间差按 765/period 换算为相位增量；
 *                     3) 求 phase/group/sub_phase；4) 按三段渐变算 RGB；
 *                     5) 合成 aRGB 并输出；6) 更新时间戳与累计相位。
 * Receive()         ：无接收数据，函数体为空。
 */
class LED: public Device{

    /**
     * @brief 把 0xAARRGGBB 颜色值转换为三路 PWM 比较值并输出。
     * @param aRGB 颜色值：bit31..24 亮度 alpha，bit23..16 红，bit15..8 绿，bit7..0 蓝。
     * @note  RGB 各分量先乘以 alpha 再写入对应通道的 CCR，实现整体亮度调节；
     *        通道 1/2/3 依次对应蓝/绿/红（与硬件接线一致，不可随意调换）。
     */
    void aRGB_led_show(uint32_t aRGB);
    /**
     * @brief 跑马灯/渐变效果生成函数，必须周期性调用。
     * @param period 一轮红->绿->蓝渐变循环的周期（ms）。
     * @note  用 HAL_GetTick() 计算与上次调用的时间间隔，把相位映射到
     *        0~764（3×255）区间，分三段分别让红绿、绿蓝、蓝红交替渐变，
     *        再调用 aRGB_led_show() 输出；内部用 static 变量保存相位。
     */
    void aRGB_led_change(uint32_t period);

public:
    /** @brief 启动 TIM5 三路 PWM 输出（由主流程调用一次）。 */
    void Init();
    /** @brief 以 LED_PERIOD 为周期刷新跑马灯颜色，需按控制周期调用。 */
    void Handle();
    /** @brief 接收接口：LED 无需接收外部数据，实现为空。 */
    void Receive();
};




#endif //CONTROL_FRAME_MAIN_LED_H
