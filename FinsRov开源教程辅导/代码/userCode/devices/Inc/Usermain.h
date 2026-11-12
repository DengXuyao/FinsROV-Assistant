//
// Created by LEGION on 2021/10/4.
//

/**
 * @file    Usermain.h
 * @brief   用户层总头文件：全局配置宏、设备类型/版本枚举、Device 抽象基类。
 * @details 所属模块：userCode/devices。几乎所有 devices/*.cpp 都包含本文件，
 *          以统一获得 STM32 HAL 句柄、标准库、工程级常量与设备接口约定。
 *          主流程在 Src/Usermain.cpp 中创建各设备对象并按固定控制周期轮询
 *          Init() -> Receive() -> Handle()，实现"配置-接收-处理"的统一框架。
 * @note    修改 SERIAL_LENGTH_MAX、SENSOR_NUM、SERVO_NUM、PROPELLER_NUM 等
 *          数量宏时，须同步检查各设备数组大小与串口协议解析长度（见 TODO）。
 * @note    RxBuffer 为串口接收缓冲区，由中断/DMA 写入、主循环解析；
 *          Count_ms_dog / Flag_stop / Flag_serial 为看门狗与串口状态全局量。
 */

#ifndef RM_FRAME_C_DEVICE_H
#define RM_FRAME_C_DEVICE_H
/* main.h 引入 HAL、CubeMX 外设句柄与 uint8_t 等基础类型 */
#include "main.h"


/* 本工程使用的外设句柄：CAN、DMA、I2C、独立看门狗、SPI、定时器、GPIO、串口 */
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"


/* 标准库：可变参数、内存操作、数学运算、格式化输出、动态分配 */
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

//------TODO:修改串口长度上限、各器件数量

#define CONTROL_FREQUENCY 60        // 更新频率
/* 主控制循环频率（Hz），决定各设备 Handle() 的调用周期与积分步长 */
#define SERIAL_LENGTH_MAX 100       // 串口指令最大长度
/* 串口接收缓冲区 RxBuffer 的最大字节数，也是单帧指令长度上限 */
#define SENSOR_NUM 4                // 水压计数量
#define SERVO_NUM 4                 // 舵机数量
#define PROPELLER_NUM 8             // 推进器数量
#define LED_PERIOD 500              // 亮灯周期
/* RGB 状态灯一轮渐变的周期（ms），由 LED::Handle() 使用 */
#define PI 3.14159265358979323846

/**
 * @brief 把 NUM 原地限幅到 [MIN, MAX] 区间。
 * @note  这是语句宏（用大括号包裹的 if/else），会直接修改实参且没有 do-while
 *        包裹，因此在 if/else 后单独使用可能有悬挂 else 风险；
 *        宏参数会被多次求值，不要传入带副作用的表达式（如 i++）；
 *        MIN/MAX 建议使用常量或简单变量。PID.cpp 与各设备限幅广泛使用它。
 */
/* INRANGE 宏展开后的执行流程步骤（宏体内含续行符，故不在其中插入注释）：
 * 步骤 1：把 NUM 与 MIN 比较，若 NUM < MIN 则原地赋值为 MIN。
 * 步骤 2：否则再把 NUM 与 MAX 比较，若 NUM > MAX 则原地赋值为 MAX。
 * 步骤 3：两者都不满足时保持 NUM 不变；整个宏是语句块，会直接修改实参。
 */
#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
/*枚举类型定义------------------------------------------------------------*/

/* 串口接收缓冲区，中断/DMA 写入；长度受 SERIAL_LENGTH_MAX 限制 */
extern uint8_t RxBuffer[SERIAL_LENGTH_MAX];

//extern void ChassisStart();
//extern void ChassisHandle();
//extern void CtrlHandle();
//extern void ARMHandle();

/**
 * @enum DEVICE_TYPE_E
 * @brief 设备类型编号，用于协议帧中区分不同设备/数据通道。
 * @note  IM=惯导、PS=压力传感器、PR=推进器、SE=舵机、LE=指示灯。
 */
/*
 * 设备类型枚举
 */
typedef enum {
    IM, // imu
    PS, // pressure sensor
    PR, // propeller
    SE, // servo
    LE  // led
} DEVICE_TYPE_E;

/**
 * @enum VERSION_E
 * @brief 机器人硬件版本枚举，供上层按版本选择引脚映射/控制参数。
 */
typedef enum{
    V30,
    V31,
    V32,
    V33,
    V40
} VERSION_E;

/* 全局硬件版本，通常在初始化时根据拨码/配置赋值 */
extern VERSION_E Robot_Version;

//extern const int length;

/*extern void I2C_Extension_init();
extern void I2C_Extension();
*/
//extern void Update_level();


/* 毫秒计数（看门狗喂狗/超时判断用），由定时器中断递增 */
extern int Count_ms_dog;
/* 停机标志：非 0 时主循环让执行机构进入安全状态（推进器停转等） */
extern int Flag_stop;
/* 串口指令接收完成/在线状态标志，供主循环判断是否需要解析新指令 */
extern int Flag_serial;

/*结构体定义--------------------------------------------------------------*/



/*类型定义----------------------------------------------------------------*/



/**
 * @class Device
 * @brief 所有用户设备的抽象基类，定义统一的三段式生命周期接口。
 * @note  纯虚函数要求派生类（IMU、Sensor、Propeller、Servo、LED、Buzzer、
 *        Watchdog 等）全部实现；主循环通过 Device* 多态调用，无需了解具体类型。
 */
class Device {
public:
    DEVICE_TYPE_E deviceType;   // 设备类型标识，用于协议分发与调试识别
    //uint32_t deviceID;

    /** @brief 初始化硬件与内部状态，主流程启动时调用一次。 */
    virtual void Init() = 0;

    /** @brief 周期性执行设备主逻辑（刷新输出、状态机等），按 CONTROL_FREQUENCY 调用。 */
    virtual void Handle() = 0;

    /** @brief 接收并解析该设备相关的输入数据（串口指令/总线数据）。 */
    virtual void Receive() = 0;

    //virtual void ErrorHandle() = 0;

};

/**
 * @brief 以十进制文本形式发送一个浮点数（用于上位机调试/状态回报）。
 * @param value        待发送数值。
 * @param decimalPlaces 保留的小数位数。
 * @param endSign      是否追加帧结束符（如换行），true 时追加。
 */
void send_float(float value, uint8_t decimalPlaces, bool endSign);

/**
 * @brief 以十进制文本形式发送一个整数。
 * @param value   待发送数值。
 * @param endSign 是否追加帧结束符。
 */
void send_int(int value, bool endSign);

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/



#endif //RM_FRAME_C_DEVICE_H
