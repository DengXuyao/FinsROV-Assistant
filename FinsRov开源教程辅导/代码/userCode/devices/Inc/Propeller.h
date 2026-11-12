//
// Created by admin on 2023/10/30.
//

/**
 * @file    Propeller.h
 * @brief   推进器（水下机器人 8 路推进器）控制设备与运动控制参数。
 * @details 所属模块：userCode/devices，继承 Device，实现位于 Src/Propeller.cpp。
 *          负责把上位机指令（运动状态字/目标深度/目标姿态/目标速度）转换为
 *          各推进器的 PWM 值，再经 I2C 扩展板（TCA9548A + PCA9685）输出。
 *          控制层次：指令 -> 运动状态机（STOP/FLOAT/...）或闭环控制
 *          （定深/定姿态/定速，含多组串级 PID）-> 垂直/水平推力分配 -> PWM 输出。
 * @note    PWM 以 1500 为中位（停转），1000~1500 反转、1500~2000 正转，
 *          离 1500 越远推力越大；每路还有独立的旋转方向 Sign 与死区补偿。
 * @note    Propeller_Parameter_t 集中保存所有可调参数（含结构下标映射、
 *          各运动预置 PWM 与 11 组 PID 参数），便于按硬件版本切换/在线调参。
 */

#ifndef CONTROL_FRAME_MAIN_PWM_H
#define CONTROL_FRAME_MAIN_PWM_H

#include "Usermain.h"
/* I2C 扩展板：TCA9548A 选通道 + PCA9685 输出 PWM */
#include "Extension.h"
/* PID_Regulator_t / PID：定深、定姿、定速闭环 */
#include "PID.h"
/* STL 容器：array 存参数表、unordered_map 做指令字符到运动状态的映射 */
#include <array>
#include <unordered_map>
#include <functional>

using namespace std;

/* 前向声明：Propeller_Command_t 中的成员函数指针需要该类型 */
class Propeller_I2C;

/**
 * @enum Motion_State
 * @brief 预定义的整机运动状态，用于查表得到 4 个水平推进器的基础 PWM 组合。
 */
enum Motion_State{
    STOP,           // 停止
    FLOAT,          // 定深悬浮（仅垂直推进器工作）
    FRONT,          // 前进
    BACK,           // 后退
    LEFT,           // 左移
    RIGHT,          // 右移
    CLOCKWISE,      // 顺时针自转
    ANTICLOCKWISE,  // 逆时针自转
    MOTION_STATE_NUM// 状态总数，仅用作数组大小，不是有效状态
};

/**
 * @struct Propeller_Command_t
 * @brief 一条文本指令到成员处理函数的映射项。
 */
typedef struct Propeller_Command{
    uint8_t len;                        // 指令字符串长度（用于快速比较）
    const char* str;                    // 指令文本
    void (Propeller_I2C::*handler)();   // 匹配后调用的成员函数
}Propeller_Command_t;


/**
 * @struct Propeller_Parameter_t
 * @brief 推进器布置与控制的全部可调参数集合。
 * @note  所有数组长度与硬件路数绑定：Sign/Compensation 为 8 路推进器，
 *        InID/OutID 及各类预置 PWM 为 4 路（一组垂直或水平推进器编号）。
 */
struct Propeller_Parameter_t{
    std::array<int8_t, 8> Sign;         // 每路推进器正反转方向符号（+1/-1）
    std::array<uint8_t, 4> InID;        // 内置（内侧）推进器编号映射
    std::array<uint8_t, 4> OutID;       // 外置（外侧）推进器编号映射
    int32_t InitPWM;                    // 上电默认 PWM（应为中位 1500）
    std::array<int32_t, 8> Compensation; // 死区补偿值
    /* 各运动状态下 4 路推进器的预置 PWM 增量，实现开环基础动作 */
    std::array<int32_t, 4> FloatPWM;
    std::array<int32_t, 4> FrontPWM;
    std::array<int32_t, 4> BackPWM;
    std::array<int32_t, 4> LeftPWM;
    std::array<int32_t, 4> RightPWM;
    std::array<int32_t, 4> ClockwisePWM;
    std::array<int32_t, 4> AnticlockwisePWM;
    std::array<int32_t, 4> StopPWM;
    /* 闭环控制器参数：定深、姿态（横滚/俯仰/航向）、速度及其串级内/外环 */
    PID_Regulator_t DepthPID_P;
    PID_Regulator_t PitchPID_P;
    PID_Regulator_t RollPID_P;
    PID_Regulator_t YawPID_P;
    PID_Regulator_t YawInPID_P;
    PID_Regulator_t YawOutPID_P;
    PID_Regulator_t RollInPID_P;
    PID_Regulator_t RollOutPID_P;
    PID_Regulator_t PitchInPID_P;
    PID_Regulator_t PitchOutPID_P;

    /** @brief 默认构造：各字段由 Init() 中的参数表赋值，此处不做初始化。 */
    Propeller_Parameter_t() = default;
};

/**
 * @struct Propeller_PWM_Component_t
 * @brief 各控制分量对最终 PWM 的贡献，便于调试与限幅分析。
 */
typedef struct Propeller_PWM_Component{
    float Depth;                        // 定深（垂直）分量
    float Roll,Pitch,Yaw;               // 三轴姿态/角速度控制分量
    float Vx,Vy,Vz;                     // 三轴速度控制分量
    float Roll_angle, Pitch_angle, Yaw_angle;  // 三轴角度环分量
} Propeller_PWM_Component_t;


/**
 * @class Propeller_I2C
 * @brief 经 I2C 扩展板驱动 8 路推进器的运动控制器。
 */
/* Propeller.cpp 中主要函数内部的执行流程步骤（实现见 Src/Propeller.cpp）：
 * Init()          ：1) 装载 Parameter_V33 到各 PID；2) 清零目标值与各标志、
 *                   状态设 STOP；3) 建立 W/S/A/D/Z/E/Q 到运动状态的映射及
 *                   state_PWM_map；4) TCA 选通道 4、PCA9685 设 50Hz；
 *                   5) 8 路 PWM 置中位并启动 USART6 空闲接收。
 * Receive()       ：1) 匹配 ACL/ANG/TES 命令表并调用对应成员函数；
 *                   2) 按 flag_float 分支解析 DN/UP、运动字符、H: 深度、
 *                   RPY 开关、OFF/ON 等指令；3) 更新目标值与 current_PWM。
 * Handle()        ：1) 选通 TCA 通道 4；2) 垂直推力分配；3) 水平推力分配；
 *                   4) 逐路限幅 1000~2000 并换算 12 位计数值；5) PCA 输出。
 * float_ctrl()    ：1) 定深 PID 得 Depth 分量；2) 在 CALCULATE 节拍用
 *                   姿态外环算目标角速度；3) 内环 PID 得 Roll/Pitch 分量。
 * angle_ctrl()    ：1) 归一化航向误差；2) 外环得目标角速度；3) 内环得 Yaw 分量。
 * vertical/horizontal_PWM_allocation()：按 factors / 状态查表把各分量分配到
 *                   4 路推进器，偏离中位时加减死区补偿写 current_PWM。
 */
class Propeller_I2C: public Device
{
private:
    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t current_PWM[PROPELLER_NUM]; // PWM值，1500为不转，1500-2000正转，1000-1500反转，离1500越远转速越快
    //int32_t data_receive[5];
    /* 闭环目标值：深度、三轴角度与角速度、三轴线速度 */
    float Target_depth;
    float Target_roll = 0.0;
    float Target_roll_v = 0.0;
    float Target_pitch = 0.0;
    float Target_pitch_v = 0.0;
    float Target_yaw = 0.0;
    float Target_yaw_v = 0.0;
    /* 角度/角速度误差，由目标与 IMU 姿态反馈相减得到 */
    float yaw_diff;
    float yaw_v_diff;
    float pitch_diff;
    float pitch_v_diff;
    float roll_diff;
    float roll_v_diff;
    float Target_speed[3];              // 目标速度 Vx/Vy/Vz
    /* 工作模式标志：悬浮/角度闭环/限幅/PWM 是否允许输出等 */
    bool flag_float;
    bool flag_angle;
    bool flag_range;
    bool flag_roll;
    bool flag_PWM_output;
    Motion_State motion_state;          // 当前运动状态（查表用）
    int roll_state;                     // 横滚状态机当前状态
    int roll_state_total;               // 横滚状态机状态总数
    /* 闭环 PID 实例：深度/姿态/速度单环，以及姿态串级内、外环 */
    PID DepthPID, RollPID, PitchPID;
    PID VxPID, VyPID, YawPID;
    PID RollAnglePID, PitchAnglePID, YawAnglePID, YawInPID, YawOutPID;
    PID RollInPID, RollOutPID;
    PID PitchInPID, PitchOutPID;

    /** @brief 定深（悬浮）控制：由深度误差计算垂直推力。 */
    void float_ctrl();
    /** @brief 速度闭环控制：由目标速度与反馈速度计算推力。 */
    void speed_ctrl();
    /** @brief 姿态闭环控制：角度/角速度串级 PID 计算三轴修正量。 */
    void angle_ctrl();
    /** @brief 角度单位换算：度 -> 弧度。 */
    float deg2rad(float);
    /** @brief 角度单位换算：弧度 -> 度。 */
    float rad2deg(float);
    /** @brief 把角度归一化到 [-180,180)，避免航向跨 ±180° 时误差跳变。 */
    float normalize_angle(float euler_angle);

    /** @brief 垂直方向推力分配：把定深/俯仰/横滚分量分配到 4 个垂直推进器。 */
    void vertical_PWM_allocation();
    /** @brief 水平方向推力分配：把前后/左右/航向分量分配到 4 个水平推进器。 */
    void horizontal_PWM_allocation();

    unordered_map<char, Motion_State> msg_state_map;   // 单字符指令 -> 运动状态
    std::array<std::array<int32_t, 4>, MOTION_STATE_NUM> state_PWM_map; // 状态 -> 4 路 PWM 增量表
    Propeller_PWM_Component_t PWM_component;          // 本次运算各控制分量记录
    Propeller_Parameter_t Parameter;                  // 全部可调参数
    static Propeller_Command_t propeller_command[];    // 文本指令表（定义在 .cpp）

public:
    /** @brief 初始化参数表、PID 参数、I2C 扩展板与 PWM 中位输出。 */
    void Init();
    /** @brief 周期控制：状态机/PID 计算 -> 推力分配 -> 限幅 -> 输出 PWM。 */
    void Handle();
    /** @brief 解析上位机指令，更新目标值与 motion_state。 */
    void Receive();
    
    /** @brief 指令处理示例：按预设 PWM 依次测试各推进器。 */
    void command_test();
    /** @brief 指令处理：开启/关闭姿态闭环。 */
    void command_set_angle_closeloop();
    /** @brief 指令处理：设置目标姿态角。 */
    void command_set_angle_value();
};


/*extern void Anglectrl_servo();

extern void Speedctrl_propeller_init();

extern void Speedctrl_propeller();

extern void Anglectrl_servo_init();
*/

#endif //CONTROL_FRAME_MAIN_PWM_H
