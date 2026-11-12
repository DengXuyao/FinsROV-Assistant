//
// Created by admin on 2023/12/9.
//
/**
 * @file    LED.cpp
 * @brief   RGB 状态灯实现：TIM5 三路 PWM 输出与跑马灯渐变效果。
 * @details 所属模块：userCode/devices，配套声明见 Inc/LED.h。
 *          aRGB_led_show() 把 0xAARRGGBB 颜色值拆成 R/G/B 三个通道并按
 *          alpha 缩放，写入 TIM5 的 CH3(红)/CH2(绿)/CH1(蓝) 比较寄存器；
 *          aRGB_led_change() 依据 HAL_GetTick() 的时间差推进 0~764 的相位，
 *          分三段在红->绿->蓝之间循环渐变，实现跑马/呼吸灯效果。
 * @note    aRGB_led_change() 内部使用 static 变量保存上次调用时间与相位，
 *          必须被周期性调用；首次调用只记录时间戳并直接返回，不输出颜色。
 * @note    相位按 interval = Δt * 765 / period 缩放，使 period（ms）对应
 *          一整轮颜色循环；period 为 0 会导致除零，调用方需保证其大于 0。
 */
#include "LED.h"

/**
 * @brief 启动 TIM5 三个 PWM 通道（红/绿/蓝）。
 * @note  由主流程调用一次；此后只需改写比较寄存器即可改变颜色，
 *        通道映射：CH1=蓝、CH2=绿、CH3=红（与硬件接线保持一致）。
 */
void LED::Init(){
    /* 函数内部流程步骤：
     * 步骤 1：HAL_TIM_PWM_Start 启动 TIM5 通道 1（蓝）。
     * 步骤 2：启动通道 2（绿）。
     * 步骤 3：启动通道 3（红）；此后只需改写 CCR 即可改变颜色。
     */
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
}

/** @brief 接收接口：状态灯无需接收外部帧，实现为空。 */
void LED::Receive(){

}

/**
 * @brief 周期刷新状态灯颜色。
 * @note  以 Usermain.h 中的 LED_PERIOD 为循环周期调用跑马灯函数；
 *        调用频率越高颜色过渡越平滑。
 */
void LED::Handle(){
    /* 函数内部流程步骤：
     * 步骤 1：主循环每拍进入本函数。
     * 步骤 2：以 LED_PERIOD 为周期参数调用 aRGB_led_change() 推进跑马灯相位；
     *         调用越频繁，颜色过渡越平滑。
     */
    aRGB_led_change(LED_PERIOD);
}


/**
 * @brief 输出一个 0xAARRGGBB 颜色值到三路 PWM。
 * @param aRGB bit31..24 亮度 alpha，bit23..16 红，bit15..8 绿，bit7..0 蓝。
 * @note  各颜色分量先与 alpha 相乘实现整体亮度调节，再写入对应 CCR；
 *        alpha、各分量用 static 局部量保存（保持原实现，跨调用无副作用）。
 * @note  CCR 值越大该颜色通道越亮，按共阴/共阳接法不同可能是反向 PWM，
 *        若颜色反相需在硬件或此处取反。
 */
void LED::aRGB_led_show(uint32_t aRGB){
    /* 函数内部流程步骤：
     * 步骤 1：从 aRGB 的 bit31..24 取出 alpha（整体亮度）。
     * 步骤 2：分别取 bit23..16 红、bit15..8 绿、bit7..0 蓝，
     *         并各自乘 alpha 完成亮度缩放。
     * 步骤 3：按硬件接线写 TIM5 的 CH1=蓝、CH2=绿、CH3=红 三个 CCR，
     *         PWM 占空比随即改变，颜色立即生效。
     */
    static uint8_t alpha;
    static uint16_t red,green,blue;
    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);

}


/**
 * @brief Running-light (LED chaser) function; must be invoked periodically.
 * @param period /Cycle period of the running-light pattern.
 */
/**
 * @brief 跑马灯函数，需周期性调用
 * @param period 跑马灯循环周期
 */
void LED::aRGB_led_change(uint32_t period){
    /* 函数内部流程步骤：
     * 步骤 1：从 static 的 aRGB_value 中拆出上次颜色的红/绿/蓝分量。
     * 步骤 2：计算与上次调用的间隔 interval = HAL_GetTick() - lastTick；
     *         若 lastTick == 0 说明是首次进入，只记录时间戳并直接返回。
     * 步骤 3：把时间增量按 765/period 缩放为相位增量，使 period 毫秒
     *         对应一整轮 0~764 的颜色循环。
     * 步骤 4：phase = (interval + lastPhase) % 765，求 group = phase/255、
     *         sub_phase = phase%255，确定当前渐变段与段内进度。
     * 步骤 5：switch(group)：0 段红->绿、1 段绿->蓝、2 段蓝->红，
     *         用 255-sub_phase 与 sub_phase 让两色互补变化，第三段颜色置 0。
     * 步骤 6：把三个分量与 alpha=0xff 合成 0xAARRGGBB，调用 aRGB_led_show() 输出。
     * 步骤 7：更新 lastTick = HAL_GetTick()、lastPhase += interval，
     *         保留不足一个整数的小数相位供下一拍使用。
     */
    static uint32_t aRGB_value = 0xffff0000;   // 当前颜色，初值为不透明红色
    static uint32_t lastTick = 0;              // 上次调用的 HAL_GetTick() 时间戳
    static float lastPhase = 0;                // 上次累计的相位（浮点保留不足一个单位的增量）
    uint32_t red,green,blue;
    red = ((aRGB_value & 0x00FF0000) >> 16) ;
    green = ((aRGB_value & 0x0000FF00) >> 8) ;
    blue = ((aRGB_value & 0x000000FF) >> 0) ;

    /* 计算距上次调用的时间，并把时间增量按 765 个相位单位/period 缩放 */
    float interval = HAL_GetTick()- lastTick;
    if(lastTick == 0){// Handle the first entry into this state. //处理首次进入
        lastTick = HAL_GetTick();
        return;
    }
    interval = interval * 765/(float)period;
    //if(interval == 0)interval = 1;
    /* 相位落在 0~764：group 决定当前渐变段，sub_phase 决定段内进度 */
    uint32_t phase =  (uint32_t)(interval+ lastPhase)%765;
    uint32_t group = phase/255;
    uint32_t sub_phase = phase%255;

    switch(group){
        case 0:
            /* 第一段：红逐渐减弱，绿逐渐增强（蓝灭） */
            red = 255 - sub_phase;
            green = sub_phase;
            blue = 0;
            break;
        case 1:
            /* 第二段：绿逐渐减弱，蓝逐渐增强 */
            red = 0;
            green = 255 - sub_phase;
            blue = sub_phase;
            break;
        case 2:
            /* 第三段：蓝逐渐减弱，红逐渐增强，回到起点 */
            red = sub_phase;
            green = 0;
            blue = 255 - sub_phase;
            break;
        default:
            /* 理论不可达（group 只能是 0/1/2），兜底为纯红 */
            red = 255;
            green = 0;
            blue = 0;
            break;
    }
    /* 合成 0xAARRGGBB（alpha 固定 0xff）后输出，再更新时间与相位 */
    aRGB_value = 0xff000000;
    aRGB_value |= red << 16;
    aRGB_value |= green << 8;
    aRGB_value |= blue;

    aRGB_led_show(aRGB_value);

    lastTick = HAL_GetTick();
    lastPhase = lastPhase + interval;
}
