//
// Created by admin on 2023/12/5.
//
// @file  Watchdog.cpp
// @brief 串口通信存活看门狗 / UART link-liveness watchdog
//        上位机持续发串口帧 -> 每拍喂 IWDG；长时间收不到数据 -> 停止喂狗，
//        由 IWDG 硬件复位 MCU（复位后 Propeller::Init() 会把 8 路 PWM 写回 1610us 停转）。
//        注意：这里保护的是「通信链路」，不是传统的「程序跑飞」检测，
//        因为喂狗条件绑定在「是否收到串口数据」上。
//
// @note  本模块当前【未启用】：Usermain.cpp 中 device[] 的 &watchdog 与
//        MX_IWDG_Init() 两处都被注释，IWDG 从未启动，不会发生超时复位。
// @note  Handle() 由主循环按 TIM1 更新率（实际 150Hz）调用；
//        Count_ms_dog 计的是「调用次数」而非毫秒，尽管变量名带 ms。
// @note  硬件超时来自 Src/iwdg.c：IWDG_PRESCALER_16 + Reload=0xfff + LSI≈32kHz
//        => 约 2.05s。软件阈值 2000 拍 @150Hz 约 13.3s，合计约 15.4s 才复位，
//        源码注释里的「2 秒」只是硬件溢出时间，不是失联到复位的总时长。
// @note  IWDG 超时是【纯硬件复位】：不产生中断、不返回错误码、不调用 Error_Handler()；
//        超时后 CPU 直接从 Reset_Handler 重新执行。

/**
 * @file    Watchdog.cpp
 * @brief   软件喂狗策略 + 独立看门狗 IWDG 的通信失联保护实现。
 * @details 三个接口配合：Init() 启动 IWDG 硬件；Receive() 在收到 USART6
 *          串口帧时置心跳标志；Handle() 每控制拍检查心跳，有则清零计数并
 *          喂狗，无则累加计数，超过阈值后停止喂狗，由硬件复位 MCU。
 *          复位后各设备 Init() 会把推进器 PWM 恢复为停转中位，实现安全降级。
 * @note    本模块当前未启用：Usermain.cpp 中 &watchdog 与 MX_IWDG_Init()
 *          两处均被注释；下方原有注释对超时时间的分析非常详细，务必一并阅读。
 */
#include "Watchdog.h"



// @brief 启动独立看门狗硬件（由设备框架在初始化阶段调用一次）。
//        MX_IWDG_Init() 内部会调 HAL_IWDG_Init()：写 KR=0xCCCC 启动计数、配置
//        PR/RLR 并等待寄存器同步，函数返回 HAL_OK/HAL_ERROR/HAL_TIMEOUT。
//        【注意】IWDG 一旦启动只能靠复位关闭；且本函数没有显式初始化
//        Flag_serial / Count_ms_dog / buzzerWorkingFlag 之外的成员，
//        Flag_serial 与 Count_ms_dog 依赖静态对象零初始化（值为 false / 0）。
// @note  HAL_IWDG_Init() 返回非 HAL_OK 时会调 Error_Handler()，
//        但本工程的 Error_Handler() 是空实现，会静默继续。
void Watchdog::Init(){
    /* 函数内部流程步骤：
     * 步骤 1：调用 MX_IWDG_Init()（内部 HAL_IWDG_Init 写 KR=0xCCCC 启动计数、
     *         配置预分频与重装载并等待寄存器同步）。
     * 步骤 2：函数返回，之后 IWDG 只能靠复位关闭；Flag_serial 与
     *         Count_ms_dog 依赖静态对象零初始化为 false / 0。
     */
    MX_IWDG_Init();
}

// @brief 记录「本拍收到了串口数据」（心跳标志）。
//        由 HAL_UARTEx_RxEventCallback() 广播调用：USART6 收到一帧空闲帧后，
//        device[] 里每个设备的 Receive() 都会被调用一次。
//        本函数不检查帧内容/来源，任何 USART6 帧都算一次心跳。
void Watchdog::Receive(){
    /* 函数内部流程步骤：
     * 步骤 1：由 HAL_UARTEx_RxEventCallback 在 USART6 收到空闲帧时调用。
     * 步骤 2：无条件把 Flag_serial 置 true，记为一次通信心跳；
     *         不检查帧内容与来源，任何 USART6 数据帧都会喂狗。
     */
    Flag_serial = true;// Uart data received. //串口收到数据
}

// @brief 每个控制拍决定「喂不喂狗」。
//        有心跳 -> 清零计数；无心跳 -> 计数++；计数 < 2000 则喂狗，否则停止喂狗，
//        等硬件计数器归零后由 IWDG 直接复位 MCU。
// @note  HAL_IWDG_Refresh() 是 HAL 函数，内部只执行 __HAL_IWDG_RELOAD_COUNTER
//        （写 KR=0xAAAA 重装计数），永远返回 HAL_OK，本身不检测超时；
//        这里连返回值都没有接。超时与否完全由硬件决定。
// @note  Count_ms_dog 超过 2000 后仍会一直 ++，没有钳位；
//        uint32_t 回绕（约 331 天 @150Hz）后理论上会重新开始喂狗。
void Watchdog::Handle(){
    /* 函数内部流程步骤：
     * 步骤 1：若 Flag_serial 为真（本拍收到串口帧），把 Count_ms_dog 清零，
     *         并把 Flag_serial 复位为 false，等待下一次心跳。
     * 步骤 2：否则 Count_ms_dog 加一，累计无通信的控制拍数。
     * 步骤 3：若计数仍小于 2000，调用 HAL_IWDG_Refresh(&hiwdg) 写 KR=0xAAAA
     *         重装硬件看门狗计数器（喂狗）。
     * 步骤 4：计数达到 2000 后停止喂狗，IWDG 硬件计数器归零时直接复位 MCU；
     *         复位后各设备 Init() 把执行机构恢复到安全中位。
     */
    if(Flag_serial){// Reset `Count_ms_dog` after data reception. //收到数据后重置Count_ms_dog
        Count_ms_dog = 0;
        Flag_serial = false;
    }
    else Count_ms_dog++;// Increment `Count_ms_dog` when no data are received. //未收到数据Count_ms_dog增加
    if(Count_ms_dog < 2000) HAL_IWDG_Refresh(&hiwdg); // If no data are received for 2 seconds, restart the program. //2秒未收到数据，重启程序
}

