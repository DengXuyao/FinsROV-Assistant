//
// Created by admin on 2023/12/9.
//

/**
 * @file    Buzzer.h
 * @brief   蜂鸣器设备类：用定时器 PWM 驱动无源蜂鸣器发出指定频率/音量。
 * @details 所属模块：userCode/devices。继承 Usermain.h 中的抽象基类 Device，
 *          实现 Init / Receive / Handle 三接口，由主循环统一调度。
 *          实现位于 Src/Buzzer.cpp：通过改变 htim4 通道 3 的预分频与比较
 *          寄存器产生方波，音量由占空比（CCR）近似控制。
 * @note    BuzzerFlag 作为外部写入的"声音模式/指令"位域，Handle() 每周期
 *          读取它决定鸣叫还是静音；buzzerWorkingFlag 记录当前是否正在发声，
 *          用于避免重复启停 PWM。
 */

#ifndef CONTROL_FRAME_MAIN_BUZZER_H
#define CONTROL_FRAME_MAIN_BUZZER_H

#include "Usermain.h"

/* 蜂鸣器所用定时器的时钟频率（Hz）：TIM4 挂在 APB1 倍频后的 84 MHz 上，
 * 用于由目标频率反算预分频/重装载值：ARR = 时钟 / 频率 */
#define BUZZER_CLOCK_FREQUENCY 84000000
/* CubeMX 生成的定时器句柄；修改引脚/定时器时同步改此宏 */
#define BUZZER_CLOCK htim4
/* PWM 输出通道，对应蜂鸣器驱动引脚 */
#define BUZZER_CLOCK_CHANNEL TIM_CHANNEL_3
/* 满音量（100%）时的比较寄存器参考值，音量百分比按此值换算占空比 */
#define REFERENCE_MAX_VOL_CCR 5000

/**
 * @class Buzzer
 * @brief 蜂鸣器设备，Device 接口的具体实现。
 */
/* Buzzer.cpp 中各函数内部的执行流程步骤（本文件仅声明，实现见 Src/Buzzer.cpp）：
 * Init()   ：1) BuzzerFlag 清零；2) HAL_TIM_Base_Start 启动 TIM4 计数；
 *            3) HAL_TIM_PWM_Start 使能 CH3 输出。
 * Handle() ：1) 判断 BuzzerFlag <= 3；2) 成立则 bsp_BuzzerOn(50,1) 发声并把
 *            计数加一；3) 否则 bsp_BuzzerOff() 静音。
 * bsp_BuzzerOn()：1) 由时钟/(PSC+1)/频率算 ARR；2) 由音量算 CCR 并限幅
 *            到 ARR/2；3) 置工作标志；4) 写 ARR/CCR；5) 返回实际占空比。
 * bsp_BuzzerOff()：1) 检查工作标志；2) 非零则清零并 HAL_TIM_PWM_Stop。
 * Receive()：无数据可接收，函数体为空。
 */
class Buzzer: public Device{

    /**
     * @brief BSP 层：按频率与音量启动 PWM 输出。
     * @param _freq        目标发声频率（Hz），决定音调。
     * @param _targetVolPct 目标音量百分比（0~100），决定占空比。
     * @return 实际写入的比较值/换算结果（由实现定义）。
     */
    float bsp_BuzzerOn(float _freq,float _targetVolPct);
    /** @brief BSP 层：停止 PWM 输出，令蜂鸣器静音。 */
    void bsp_BuzzerOff();

public:

    uint32_t BuzzerFlag;        // 外部（串口指令等）写入的蜂鸣器控制标志/鸣叫模式
    uint8_t buzzerWorkingFlag;  // 当前发声状态：非 0 表示 PWM 已启动

    /** @brief 初始化蜂鸣器相关定时器与默认静音状态（由主流程调用一次）。 */
    void Init();
    /** @brief 接收/解析蜂鸣器控制指令，更新 BuzzerFlag。 */
    void Receive();
    /** @brief 周期性处理：根据 BuzzerFlag 决定启停与音调，需按控制周期调用。 */
    void Handle();

};


#endif //CONTROL_FRAME_MAIN_BUZZER_H
