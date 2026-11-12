//
// Created by admin on 2023/12/5.
//
/**
 * @file    Watchdog.h
 * @brief   通信看门狗设备：监测上位机指令是否持续到达，超时进入安全状态。
 * @details 所属模块：userCode/devices，继承 Device，实现位于 Src/Watchdog.cpp。
 *          与主循环中的毫秒计数配合：每次收到有效串口指令重置计数，
 *          计数超过阈值则判定通信中断，令推进器停转（Flag_stop 置位），
 *          防止机器人在失联后继续保持推力。
 * @note    Flag_serial 表示本周期是否收到新指令（同时与 Usermain.h 中的
 *          全局 Flag_serial 配合使用）；Count_ms_dog 为距上次指令的毫秒数。
 * @note    本类是软件看门狗，与 STM32 独立看门狗 iwdg 是两个层次的保险。
 */
#include "Usermain.h"

#ifndef CONTROL_FRAME_MAIN_WATCHDOG_H
#define CONTROL_FRAME_MAIN_WATCHDOG_H

/* Watchdog.cpp 中各函数内部的执行流程步骤（实现见 Src/Watchdog.cpp）：
 * Init()    ：调用 MX_IWDG_Init() 启动独立看门狗硬件（模块当前未启用）。
 * Receive() ：收到 USART6 帧时把 Flag_serial 置 true，作为一次心跳。
 * Handle()  ：1) 有心跳则 Count_ms_dog 清零并复位标志；2) 无心跳则计数加一；
 *            3) 计数 < 2000 时 HAL_IWDG_Refresh() 喂狗；4) 超阈值停止喂狗，
 *            等待 IWDG 硬件溢出复位 MCU。
 */
class Watchdog: public Device{

public:

    bool Flag_serial;           // 串口数据到达标志（收到指令时置位）
    uint32_t Count_ms_dog;      // 失联计时（ms），递增到阈值即触发保护

    /** @brief 初始化标志与计时，必要时启动硬件/软件定时资源。 */
    void Init();
    /** @brief 周期检查计时并更新停机/在线状态，按控制周期调用。 */
    void Handle();
    /** @brief 接收串口事件，收到有效指令时喂狗（清零计数）。 */
    void Receive();

};



#endif //CONTROL_FRAME_MAIN_WATCHDOG_H
