//
// Created by admin on 2023/12/9.
//
/**
 * @file    Buzzer.cpp
 * @brief   蜂鸣器设备实现：定时器 PWM 频率/占空比控制。
 * @details 所属模块：userCode/devices，配套声明见 Inc/Buzzer.h。
 *          Init() 启动 TIM4 时基与 CH3 PWM；Handle() 按 BuzzerFlag 决定鸣叫；
 *          bsp_BuzzerOn() 由目标频率反算 ARR、由音量百分比反算 CCR 并写寄存器；
 *          bsp_BuzzerOff() 停止 PWM 输出。
 * @note    ARR 依赖当前定时器预分频 PSC，因此频率计算使用
 *          BUZZER_CLOCK_FREQUENCY / (PSC+1) / freq；若 CubeMX 修改了 PSC，
 *          实际音调会自动跟随（无需改代码）。
 * @note    占空比被限制在 50%（CCR <= ARR/2），避免无源蜂鸣器驱动波形对称
 *          性过差导致音量/发热问题；返回值即实际占空比的两倍（0~1）。
 */
#include "Buzzer.h"

/**
 * @brief 启动蜂鸣器所用的定时器时基与 PWM 通道，并清空控制标志。
 * @note  HAL_TIM_Base_Start() 让计数器开始运行，HAL_TIM_PWM_Start() 使能
 *        CH3 输出；此后只有调用 bsp_BuzzerOff() 才会停止输出通道。
 */
void Buzzer::Init(){
    /* 函数内部流程步骤：
     * 步骤 1：把 BuzzerFlag 清零（初始不鸣叫）。
     * 步骤 2：HAL_TIM_Base_Start(&BUZZER_CLOCK) 启动 TIM4 计数器。
     * 步骤 3：HAL_TIM_PWM_Start(&BUZZER_CLOCK, BUZZER_CLOCK_CHANNEL)
     *         使能 CH3 PWM 输出，等待 Handle()/bsp 层改写频率与占空比。
     */
    BuzzerFlag = 0;
    HAL_TIM_Base_Start(&BUZZER_CLOCK);
    HAL_TIM_PWM_Start(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL);

}

/** @brief 接收接口：蜂鸣器无独立接收数据，指令由外部直接写 BuzzerFlag，故为空。 */
void Buzzer::Receive(){

}

/**
 * @brief 周期性鸣叫逻辑：BuzzerFlag 作为"剩余鸣叫周期数"使用。
 * @note  BuzzerFlag <= 3 时以 50 Hz、音量参数 1 发声并把计数加一；
 *        计数超过 3 后关闭蜂鸣器。也就是说置 BuzzerFlag 为 0 可触发
 *        约 4 个控制周期的短提示音；其它代码也可直接调用 bsp 层函数。
 */
void Buzzer::Handle(){
    /* 函数内部流程步骤：
     * 步骤 1：判断 BuzzerFlag 是否 <= 3（用作剩余鸣叫拍数）。
     * 步骤 2：成立则调用 bsp_BuzzerOn(50, 1) 以 50Hz 发声，并让标志加一。
     * 步骤 3：超过 3 后调用 bsp_BuzzerOff() 关闭 PWM，形成约 4 拍短提示音。
     */
    if(BuzzerFlag <= 3) {
        bsp_BuzzerOn(50,1);
        BuzzerFlag++;
    }
    else bsp_BuzzerOff();
}

/**
 * @brief 按频率与音量刷新 PWM 的 ARR / CCR，开启或更新蜂鸣器发声。
 * @param _freq         目标频率（Hz），决定 ARR。
 * @param _targetVolPct 音量参数（0~1 或百分比由调用方约定），与
 *                      REFERENCE_MAX_VOL_CCR 相乘得到 CCR 基准。
 * @return 实际占空比 = 2 * CCR / ARR（范围 0~1）。
 * @note  arr 由定时器实际计数时钟除以频率得到；cpr 先按音量计算再限幅到
 *        arr/2（50% 占空比上限）；最后写 ARR 与比较寄存器即时改变波形。
 * @note  buzzerWorkingFlag 只在首次调用时置位，用于记录状态；这里并不
 *        重新调用 HAL_TIM_PWM_Start()，因为 Init() 已启动通道。
 */
float Buzzer::bsp_BuzzerOn(float _freq,float _targetVolPct){
    /* 函数内部流程步骤：
     * 步骤 1：声明 uint16_t 型 arr（重装载值）与 cpr（比较值）。
     * 步骤 2：按 arr = BUZZER_CLOCK_FREQUENCY/(PSC+1)/_freq 计算 PWM 周期，
     *         频率越高 arr 越小。
     * 步骤 3：按 cpr = _targetVolPct * REFERENCE_MAX_VOL_CCR * 0.5 估算占空比。
     * 步骤 4：若 cpr > arr/2 则钳位到 arr/2，限制最大占空比 50%。
     * 步骤 5：首次调用时把 buzzerWorkingFlag 置 1 记录发声状态。
     * 步骤 6：写 ARR 与 CCR 立即刷新波形，返回 2*cpr/arr（实际占空比）。
     */



    uint16_t arr,cpr;
    arr = (uint16_t)((BUZZER_CLOCK_FREQUENCY/ (float)(BUZZER_CLOCK.Instance->PSC+1))/_freq);
    //cpr = (uint16_t)(duty*(float)arr);
    cpr = (uint16_t)(_targetVolPct*REFERENCE_MAX_VOL_CCR*0.5);
    if(cpr > arr/2){
        cpr = arr/2;
    }


    if(!buzzerWorkingFlag){
        buzzerWorkingFlag = 1;

    }
    BUZZER_CLOCK.Instance->ARR = arr;
    __HAL_TIM_SetCompare(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL,cpr);

    return (float)cpr/(float)arr*2;
}

/**
 * @brief 关闭蜂鸣器 PWM 输出。
 * @note  仅在工作标志为真时调用 HAL_TIM_PWM_Stop()，避免重复停止；
 *        时基仍在运行，下次 bsp_BuzzerOn() 写 CCR 后需重新启动通道
 *        才能再次输出（此处保留了原实现的这一行为）。
 */
void Buzzer::bsp_BuzzerOff(){
    /* 函数内部流程步骤：
     * 步骤 1：检查 buzzerWorkingFlag，为 0 说明已静音，直接返回。
     * 步骤 2：非 0 则把标志清零。
     * 步骤 3：HAL_TIM_PWM_Stop() 停止 CH3 输出（时基仍在运行）。
     */
    if(buzzerWorkingFlag){
        buzzerWorkingFlag = 0;
        HAL_TIM_PWM_Stop(&BUZZER_CLOCK,BUZZER_CLOCK_CHANNEL);
    }
}
