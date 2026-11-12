//
// Created by admin on 2023/12/5.
//
/**
 * @file    Servo.cpp
 * @brief   舵机设备实现：定时器直驱版 Servo 与 I2C 扩展板版 Servo_I2C。
 * @details 所属模块：userCode/devices，配套声明见 Inc/Servo.h。
 *          - Servo 用 htim8_CH3 与 htim1_CH2/3/4 四个 PWM 通道直接输出；
 *          - Servo_I2C 按 Robot_Version 选择通道映射表 ID_V30~V40，
 *            经 TCA9548A 通道 4 访问 PCA9685（50 Hz），把脉宽 500~2500 us
 *            换算为 12 位计数值 data*4096/20000 后输出。
 *          两个版本的 Receive() 都调用 data_extract() 解析 "MOT:" 开头的
 *          逗号分隔指令，Handle() 只在数值合法（500~2500）时刷新输出。
 * @note    Init() 中通过 HAL_UARTEx_ReceiveToIdle_IT() 启动 USART6 空闲中断接收；
 *          data_extract() 使用 strtok 会原地修改 RxBuffer，不可重入。
 * @note    Servo_I2C::data_extract() 对第 3 路（i==2）做 3000-data 的反向处理，
 *          用于匹配该舵机的机械安装方向，换硬件时需复核。
 */
#include "Servo.h"

/* 各硬件版本舵机在 PCA9685 上的通道编号（左上/左下/右上/右下顺序） */
int32_t ID_V30[4] = {8, 9, 10, 11}; // Channel indices on the PWM expansion board for the four servos: upper-left, lower-left, upper-right, lower-right. // 4个舵机接在PWM扩展板的编号，左上-左下-右上-右下
int32_t ID_V31[4] = {8, 9, 10, 11};
int32_t ID_V32[4] = {8, 9, 11, 10};
int32_t ID_V33[4] = {8, 9, 10, 11};
int32_t ID_V40[4] = {8, 9, 10, 11}; // Test needed. //还需测试

void Servo::Init()
{
    /* 函数内部流程步骤：
     * 步骤 1：循环把 SERVO_NUM 路 data[i] 置为中位 1500。
     * 步骤 2：按硬件接线把四路分别写入 htim8_CH3(data[3])、
     *         htim1_CH2(data[0])、htim1_CH3(data[2])、htim1_CH4(data[1])。
     * 步骤 3：HAL_UARTEx_ReceiveToIdle_IT() 启动 USART6 空闲中断接收，
     *         后续指令由 Receive() 解析。
     */
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        data[i] = 1500;
    }
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[1]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

/**
 * @name Init / Handle / Receive 三组接口
 * @details Init()：直驱版把 data[] 置中位 1500 并写入 4 个定时器 PWM 通道；
 *          扩展板版按 Robot_Version 复制 ID 映射、选通 TCA 通道 4、把
 *          PCA9685 设为 50 Hz 并把各通道置中位；两者都启动 USART6 空闲接收。
 *          Receive()：调用 data_extract() 解析 RxBuffer；
 *          Handle()：直驱版直接刷新比较寄存器，扩展板版先检查所有值都在
 *          500~2500 范围内，再按 ID[] 写入 PCA9685（非法帧整组丢弃）。
 * @note  PCA9685 计数值 = floor(data*4096/20000 + 0.5)，即 50 Hz 下把
 *        微秒脉宽映射为 12 位占空比。
 */
void Servo_I2C::Init()
{
    switch (Robot_Version)
    {
    case V30:
        std::memcpy(ID, ID_V30, sizeof(ID));
        break;
    case V31:
        std::memcpy(ID, ID_V31, sizeof(ID));
        break;
    case V32:
        std::memcpy(ID, ID_V32, sizeof(ID));
        break;
    case V33:
        std::memcpy(ID, ID_V33, sizeof(ID));
        break;
    case V40:
        std::memcpy(ID, ID_V40, sizeof(ID));
        break;
    }
    TCA_SetChannel(4);
    PCA_Write(PCA9685_MODE1, 0x0);
    PCA_Setfreq(50); // Hz
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        data[i] = 1500;
				// ------ TODO: `i + 8` denotes the servo channel index on the expansion board. Update this mapping according to the wiring; currently set to channels 8–11.
        //------TODO：i+8为舵机在扩展板上的接口编号，根据接线修改，目前为8-11号
        PCA_Setpwm(i + 8, 0, floor(data[i] * 4096 / 20000 + 0.5f));
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo::Receive()
{
    data_extract(RxBuffer, data, SERVO_NUM);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo_I2C::Receive()
{
    /* 函数内部流程步骤：
     * 步骤 1：调用 data_extract(RxBuffer, data, SERVO_NUM) 解析串口帧。
     * 步骤 2：指令中的第 3 路值会在解析时被取反(3000 - value)，
     *         以匹配该舵机机械安装方向。
     * 步骤 3：这里不重启接收中断（被注释），由 Usermain.cpp 的
     *         HAL_UARTEx_RxEventCallback 统一清空缓冲并重新开启。
     */
    data_extract(RxBuffer, data, SERVO_NUM);
    // HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Servo::Handle()
{
    /* 函数内部流程步骤：
     * 步骤 1：把 data[3] 写入 htim8_CH3 比较寄存器。
     * 步骤 2：把 data[0]/data[2]/data[1] 分别写入 htim1 的 CH2/CH3/CH4。
     * 步骤 3：四路 PWM 立即刷新；本函数不做范围检查（与 I2C 版不同）。
     */
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, data[3]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[1]);
}

void Servo_I2C::Handle()
{
    /* 函数内部流程步骤：
     * 步骤 1：先 TCA_SetChannel(4) 选通舵机所在 PCA9685 的 I2C 通道。
     * 步骤 2：把 flag_range 置 true，遍历 SERVO_NUM 路目标 PWM，
     *         任一值不在 500~2500 范围内则置 false。
     * 步骤 3：全部合法时才进入输出循环。
     * 步骤 4：每路按 floor(data[i]*4096/20000 + 0.5) 把微秒脉宽换算为
     *         12 位计数值，并调用 PCA_Setpwm(ID[i], 0, count) 输出。
     * 步骤 5：若存在非法值则整组丢弃本拍输出，保持上一拍状态（失效保护）。
     */
    // if(tim_id == 10){
    TCA_SetChannel(4);

    bool flag_range = true;
    for (int i = 0; i < SERVO_NUM; ++i)
    {
        if (data[i] < 500 || data[i] > 2500)
            flag_range = false;
    }
    if (flag_range)
    {
        for (int i = 0; i < SERVO_NUM; ++i)
        {
            PCA_Setpwm(ID[i], 0, floor(data[i] * 4096 / 20000 + 0.5f));
        }
    }

    //}
}

/**
 * @brief 解析 "MOT:" 开头的逗号分隔指令，更新直驱版舵机目标 PWM。
 * @param rx   串口接收缓冲区（会被 strtok 原地修改）。
 * @param data [输出] 各路 PWM 目标值。
 * @param num  最多解析的路数（SERVO_NUM）。
 * @note  帧头不匹配时保持原值；数值合法性检查在 Handle() 中进行。
 */
void Servo::data_extract(uint8_t *rx, int32_t *data, int32_t num)
{
    /* 函数内部流程步骤：
     * 步骤 1：用 strncmp 判断帧头是否为 "MOT:"，不匹配直接返回（保持原值）。
     * 步骤 2：把指针移到 rx+4，用 strtok(data_str, ",") 取第一个字段。
     * 步骤 3：while 循环在 token 非空且 i < num 时，atoi(token) 写入 data[i]，
     *         再 strtok(NULL, ",") 取下一个字段并 i++。
     * 步骤 4：字段取完或达到 num 路后结束；strtok 会原地修改 RxBuffer。
     */
    if (strncmp((char *)rx, "MOT:", 4) == 0)
    {

        char *data_str = (char *)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num)
        {
            data[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}

void Servo_I2C::data_extract(uint8_t *rx, int32_t *data, int32_t num)
{
    if (strncmp((char *)rx, "MOT:", 4) == 0)
    {
        char *data_str = (char *)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num)
        {
            data[i] = atoi(token);
					  if (i == 2) {data[i] = 3000 - data[i];} 
            token = strtok(NULL, ",");
            i++;
        }
    }
}