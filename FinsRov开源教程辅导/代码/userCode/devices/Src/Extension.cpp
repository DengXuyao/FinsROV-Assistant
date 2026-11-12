//
// Created by admin on 2023/10/29.
//
/**
 * @file    Extension.cpp
 * @brief   I2C 扩展板（TCA9548A）与 PWM 扩展板（PCA9685）的底层访问实现。
 * @details 所属模块：userCode/devices，接口见 Inc/Extension.h。所有函数都通过
 *          hi2c2 以阻塞式 HAL_I2C_Master_Transmit/Receive 访问：访问 PCA9685
 *          前必须先用 TCA_SetChannel() 选通对应 I2C 通道，否则总线上无应答。
 *          函数清单：TCA_SetChannel() 选通道；PCA_Read()/PCA_Write() 单寄存器
 *          读写；PCA_Setfreq() 设置 16 路 PWM 共用的频率；PCA_Setpwm() 设置
 *          某一路的导通/关断计数值（即脉宽）。
 * @note    TCA9548A_ADDR 在头文件里是 7 位形式，此处发送时再左移 1 位并或上
 *          读位 0x01；PCA9685_ADDR 已是 8 位形式，直接使用。
 * @note    PCA9685 内部振荡器 25 MHz、12 位计数（4096 级），频率公式
 *          prescale = round(25e6/(4096*freq)) - 1；写 PRESCALE 前必须先让
 *          MODE1 进入 SLEEP，写完恢复并延时等待振荡器稳定。
 * @warning HAL 超时设得很大（10000 / 0xffff ms），总线挂死时会长时间阻塞，
 *          不宜在高频控制中断中调用；文件末尾大段注释是旧版本扩展板驱动。
 */
#include "Extension.h"
// This file is used to handle the I2C expansion board (TCA) and the PWM expansion board (PCA).
// The I2C expansion board is connected to the C-board I2C header (second row from the top, four wires: left—SDA, SCL, VCC, GND—right).
// The PWM expansion board is connected to port 0 of the I2C expansion board.
//此文件用来处理i2c扩展板（TCA）及pwm扩展板（PCA）
//i2c扩展板接在C板i2c口（上方第二排4根线，左-SDA-SCL-VCC-GND-右）
//pwm扩展板接在i2c扩展板的0号口

// Enable the I2C bus on `channel`.
// 开通第 channel 路的I2C 
void TCA_SetChannel(uint8_t channel)
{
    /* 函数内部流程步骤：
     * 步骤 1：把通道号转换为单路选通掩码 data = 0x01 << channel。
     * 步骤 2：以 (TCA9548A_ADDR << 1) | 0x01 为 8 位器件地址，
     *         用 HAL_I2C_Master_Transmit 向 TCA9548A 写这 1 字节。
     * 步骤 3：写完后该路 I2C 总线被接通，后续读写都落到该通道上的器件
     *         （0 号口接 PCA9685，其余口接水压计/舵机板等）。
     */
    uint8_t data;
    data = 0x01 << channel;
    HAL_I2C_Master_Transmit(&hi2c2, (TCA9548A_ADDR << 1) | 0x01, &data, 1, 0xffff);
    //HAL_I2C_Master_Transmit(&hi2c2, TCA9548A_ADDR, &data, 1, 0xffff);
}

uint8_t PCA_Read(uint8_t startAddress)
{
    /* 函数内部流程步骤：
     * 步骤 1：把待读寄存器地址放入 tx[0]。
     * 步骤 2：HAL_I2C_Master_Transmit 向 PCA9685_ADDR 发送这 1 字节地址。
     * 步骤 3：HAL_I2C_Master_Receive 再读回 1 字节到 buffer[0]。
     * 步骤 4：返回 buffer[0]（两次传输超时均为 10000 ms）。
     */
		// Set the register address from which data reading starts.
    //设置开始读取数据的寄存器地址
    uint8_t tx[1];
    uint8_t buffer[1];
    tx[0] = startAddress;
    HAL_I2C_Master_Transmit(&hi2c2,PCA9685_ADDR,tx,1,10000);
    HAL_I2C_Master_Receive(&hi2c2,PCA9685_ADDR,buffer,1,10000);
    return buffer[0];
}

void PCA_Write(uint8_t startAddress, uint8_t buffer)
{
    /* 函数内部流程步骤：
     * 步骤 1：准备 2 字节数组 tx。
     * 步骤 2：tx[0] 放目标寄存器地址，tx[1] 放待写数据。
     * 步骤 3：HAL_I2C_Master_Transmit 一次发送这两个字节（超时 10000ms），
     *         PCA9685 把数据写入该寄存器。
     */
		// Set the register address from which data reading starts.
    //设置开始读取数据的寄存器地址
    uint8_t tx[2];
    tx[0] = startAddress;
    tx[1] = buffer;
    HAL_I2C_Master_Transmit(&hi2c2,PCA9685_ADDR, tx,2,10000); // Use the HAL I2C communication function to write the value to the register address. //利用HAL库的I2C通讯函数对寄存器地址写值
}

// Set the PWM frequency to `freq`.
// 设置PWM频率为 freq
/**
 * @brief 设置 PCA9685 全部 16 路共用的 PWM 频率。
 * @param freq 目标频率（Hz），舵机/推进器通常为 50 Hz。
 * @details 先用 1.016 经验系数修正晶振偏差，再按
 *          prescale = round(25 MHz / (4096 * freq)) - 1 计算预分频值；
 *          读回 MODE1 -> 置 SLEEP 位(0x10) 写入 -> 写 PRESCALE ->
 *          恢复原 MODE1 -> 延时 2 ms 等待振荡器稳定 -> 写 oldmode|0xA1
 *          重新使能并开启寄存器地址自动递增(AI)。
 * @note  修改频率会短暂停止所有 PWM 输出；必须在 TCA 通道选通后调用。
 */
void PCA_Setfreq(float freq)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明 prescale、oldmode、newmode 与 prescaleval 中间量。
     * 步骤 2：把目标频率乘 1.016 经验系数修正内部振荡器偏差。
     * 步骤 3：按 25e6/(4096*freq) - 1 计算预分频值，
     *         再用 floor(val + 0.5) 四舍五入为整数 prescale。
     * 步骤 4：读回 MODE1 到 oldmode，构造 newmode = (oldmode & 0x7F) | 0x10
     *         置 SLEEP 位后写入（改频率必须先休眠）。
     * 步骤 5：向 PRESCALE 寄存器写 prescale。
     * 步骤 6：恢复 oldmode，HAL_Delay(2) 等待振荡器稳定，
     *         最后写 oldmode | 0xA1 唤醒并开启寄存器地址自动递增。
     */
    uint8_t prescale,oldmode,newmode;
    double prescaleval;
    freq *= 1.016; 			// In practical operation, a bias is observed; multiply by the correction factor 0.83.//实际使用过程中存在偏差需要×矫正系数=0.83
    // prescaleval = 25000000;
    prescaleval = 25000000.0/(4096.0*freq);
    //prescaleval /= freq;
    prescaleval -= 1;
    prescale = floor(prescaleval + 0.5f);		// `floor` is a rounding-down function.	//floor向下取整函数
    oldmode = PCA_Read(PCA9685_MODE1);
    newmode = (oldmode&0x7F) | 0x10; // sleep睡眠
    PCA_Write(PCA9685_MODE1, newmode); // go to sleep; the device must be placed into sleep mode before the frequency can be configured. //需要进入随眠状态才能设置频率
    PCA_Write(PCA9685_PRESCALE, prescale); // Configure the prescaler (pre-divider) value. // 设置预分频系数
    PCA_Write(PCA9685_MODE1, oldmode);
    HAL_Delay(2);
    PCA_Write(PCA9685_MODE1, oldmode | 0xA1);
}

// @brief /Set the PWM value for the specified output channel.
// @param num /PWM output channel index (0–15). Channels 0–7 correspond to thrusters; channels 8–11 correspond to servos.
// @param on  /PWM rising-edge count (0–4096); typically 'on = 0'.
// @param off /PWM falling-edge count (0–4096); the duty cycle is 'off'/4096.

//  @brief 设置对应输出引脚的PWM值
//  @param num PWM输出引脚0 ~ 15, 0 ~ 7 对应推进器， 8 ~ 11 对应舵机
//  @param on PWM上升计数值0 ~ 4096, 一般取 on = 0
//  @param off PWM下降计数值0 ~ 4096, 占空比为 off/4096
void PCA_Setpwm(uint8_t num, uint32_t on, uint32_t off) 
{
    /* 函数内部流程步骤：
     * 步骤 1：以 LED0_ON_L + 4*num 为基准计算当前通道 4 个寄存器地址。
     * 步骤 2：写 ON_L = on 低 8 位、ON_H = on>>8（通常 on=0）。
     * 步骤 3：写 OFF_L = off 低 8 位、OFF_H = off>>8，
     *         脉宽由 off/4096 决定，四次单寄存器写完成一路输出。
     */
    PCA_Write(LED0_ON_L+4*num, on);
    PCA_Write(LED0_ON_H+4*num, on>>8);
    PCA_Write(LED0_OFF_L+4*num, off);
    PCA_Write(LED0_OFF_H+4*num, off>>8);
}



/*void PWM_Extension_init(){
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(50);//Hz
    for(int i=0;i<6;++i){
        PCA_Setpwm(i,0,307);//1500*4096/20000
    }
}

void PWM_Extension(){
    for(int i=0;i<6;++i){
			PCA_Setpwm(i,0,Servo::servo.data[i]*4096/20000);

    }
}

void I2C_Extension_init(){
    TCA_SetChannel(0);
    PWM_Extension_init();
    for(int i=0;i<5;++i){
        TCA_SetChannel(i+1);
        //Init_data_pressure(i);
        PressureSensor::pres_sensor.Init(i);
    }

}

void I2C_Extension(){
    TCA_SetChannel(0);
    PWM_Extension();
    for(int i=0;i<5;++i){
        TCA_SetChannel(i+1);
        //Get_data_pressure(i);
        PressureSensor::pres_sensor.Handle(i);
    }

}
*/



/*函数作用：初始化舵机驱动板参数：1.PWM频率2.初始化舵机角度*/
/*void PCA_MG90_Init(float hz,uint8_t angle)
{
    uint32_t off=0;
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(hz);//设置PWM频率
    off=(uint32_t)(145+angle*2.4);
    PCA_Setpwm(0,0,off);PCA_Setpwm(1,0,off);PCA_Setpwm(2,0,off);PCA_Setpwm(3,0,off);
    PCA_Setpwm(4,0,off);PCA_Setpwm(5,0,off);PCA_Setpwm(6,0,off);PCA_Setpwm(7,0,off);
    PCA_Setpwm(8,0,off);PCA_Setpwm(9,0,off);PCA_Setpwm(10,0,off);PCA_Setpwm(11,0,off);
    PCA_Setpwm(12,0,off);PCA_Setpwm(13,0,off);PCA_Setpwm(14,0,off);PCA_Setpwm(15,0,off);
    HAL_Delay(500);
}*/
/*函数作用：控制舵机转动；参数：1.输出端口，可选0~15； 2.结束角度，可选0~180；*/
/*void PCA_MG90(uint8_t num,uint8_t end_angle)
{
    uint32_t off=0;
    off=(uint32_t)(158+end_angle*2.2);
    PCA_Setpwm(num,0,off);
}*/


