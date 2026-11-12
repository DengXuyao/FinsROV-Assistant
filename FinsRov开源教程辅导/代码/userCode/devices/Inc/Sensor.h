//
// Created by admin on 2023/11/27.
//
#include "usermain.h"
#include "Matrix.h"
#include "Extension.h"
#include "MahonyAHRS.h"
#include "Legacy.h"

/**
 * @file    Sensor.h
 * @brief   环境感知设备：MS5837-30BA 水压计阵列与声呐（Sonar）。
 * @details 所属模块：userCode/devices，继承 Device，实现位于 Src/Sensor.cpp。
 *          PressureSensor 通过 TCA9548A 多路复用器逐个访问最多 6 路（宏
 *          SENSOR_NUM 为 4）MS5837 深度传感器：读取 PROM 校准系数 ->
 *          按状态机轮流采集 D1(压力)/D2(温度) ADC 值 -> 温度补偿换算压强 ->
 *          扣除水面零偏得到深度 -> 由多个测点的深度与安装坐标拟合水面平面
 *          方程 Ax+By+Cz+D=0，反推机器人横滚/俯仰与姿态参考。
 *          Sonar 负责读取声呐测距值并扣除安装/零点偏移。
 * @note    ps_state 是采集状态机（读温度 -> 读压力 -> 计算），每次 Handle()
 *          只推进一个传感器的一步，避免长时间阻塞 I2C 总线。
 * @note    data_plane[5] 保存平面系数 A、B、C、D 及拉格朗日乘数（最小二乘
 *          拟合水面时使用）；flag_calibrate/flag_read_pres 由外部指令置位。
 * @note    各硬件版本的水面大气压基准（PRESSURE_x_Vxx）按 Robot_Version 选择，
 *          更换作业水域或气压变化明显时应重新校准零偏。
 */
#ifndef CONTROL_FRAME_MAIN_SENSOR_H
#define CONTROL_FRAME_MAIN_SENSOR_H



/**
 * @name MS5837-30BA 器件命令与标定常量
 * @details B02_IIC_ADDRESS 为 MS5837 在 I2C 总线上的 8 位写地址；
 *          ResetCommand 触发软复位，PROM_RD 起始地址 0xA0 后按 2 字节步进
 *          读取 7 个校准字，ADC_RD=0x00 读取 24 位转换结果；
 *          D1_*/D2_* 分别是压力/温度转换命令，OSR256~4096 决定过采样率
 *          （精度越高、转换时间越长），当前代码使用 OSR1024。
 *          PRESSURE_x_Vxx 为各硬件版本 4 路传感器在水面测得的大气压零偏，
 *          由 Robot_Version 在 Init_single() 中选择（详见 Sensor.cpp）。
 */
#define B02_IIC_ADDRESS 0xEC

#define MS5837_30BA_ResetCommand     0x1E                //��λ
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM��ȡ,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC��ȡ

#define MS5837_30BA_D1_OSR256					 0x40
#define MS5837_30BA_D1_OSR512					 0x42
#define MS5837_30BA_D1_OSR1024					 0x44
#define MS5837_30BA_D1_OSR2048					 0x46
#define MS5837_30BA_D1_OSR4096					 0x48
#define MS5837_30BA_D2_OSR256					 0x50
#define MS5837_30BA_D2_OSR512					 0x52
#define MS5837_30BA_D2_OSR1024					 0x54
#define MS5837_30BA_D2_OSR2048					 0x56
#define MS5837_30BA_D2_OSR4096					 0x58

#define PRESSURE_0_V30 1005.646
#define PRESSURE_1_V30 1004.711
#define PRESSURE_2_V30 1004.350
#define PRESSURE_3_V30 1005.058

#define PRESSURE_0_V31 1014.227
#define PRESSURE_1_V31 1003.288
#define PRESSURE_2_V31 1003.028
#define PRESSURE_3_V31 1003.799

#define PRESSURE_0_V32 1009.775
#define PRESSURE_1_V32 1010.957
#define PRESSURE_2_V32 1010.074
#define PRESSURE_3_V32 1009.850

#define PRESSURE_0_V33 1004.227
#define PRESSURE_1_V33 1003.288
#define PRESSURE_2_V33 1003.028
#define PRESSURE_3_V33 1003.799

#define PRESSURE_0_V40 1003.599
#define PRESSURE_1_V40 1004.981
#define PRESSURE_2_V40 1004.411
#define PRESSURE_3_V40 1003.102

typedef struct Sensor_Site{
    float x[SENSOR_NUM];
    float y[SENSOR_NUM];
    float z[SENSOR_NUM];
} Sensor_Site_t;

enum class PS_HANDLE_STATE {
        GET_TEMPERATURE = 0X00,
        GET_PRESSURE = 0X01,
        CALCULATE = 0X02
    };

/**
 * @class PressureSensor
 * @brief 多路 MS5837-30BA 水压计采集与水面姿态解算设备。
 * @details 私有部分按职责分组：MS5837_* 为器件级命令/CRC/换算函数，
 *          Init_single/Handle_single/Calibrate_single/OutputData_single 为
 *          单路操作，*_all / Calibrate / OutputData / Solve_plane* 为多路
 *          汇总与最小二乘平面拟合；public 的 data_* 数组供运动控制读取。
 * @note  静态成员 pressure_sensor 便于在中断/全局函数中访问唯一实例；
 *        flag_calibrate 触发水面零偏标定，结果可写入 Flash（见 Legacy.h）。
 */
/* Sensor.cpp 中主要函数内部的执行流程步骤（实现见 Src/Sensor.cpp）：
 * Init()          ：1) 填 4 个测点三维坐标 site；2) 读 Flash 标定参数；
 *                   3) 逐路 TCA 选通道并 Init_single()；4) 初始化水面方程
 *                   系数与两个标志。
 * Handle()        ：1) flag_calibrate 为真则 Calibrate() 并清标志；
 *                   2) 否则 Handle_all()；3) 用 4 路深度平均值算 data_depth，
 *                   用两两差值组合算 data_roll / data_pitch。
 * Handle_all()    ：三拍状态机，依次「发 D2 转换命令 -> 读 D2 并发 D1 命令
 *                   -> 读 D1 并做温度补偿/零偏/卡尔曼滤波」，循环往复。
 * MS5837_30BA_GetData()：读 D2/D1 -> 算 dT/SENS/OFF_ 与 TEMP -> 二阶补偿
 *                   -> 套压强公式返回（CRC 未过返回 -1）。
 * Calibrate_single()：1) 采样 100 次求平均零偏；2) 拆成 7 位十进制数字；
 *                   3) 拼成文本帧经 USART6 输出。
 * Solve_plane*()  ：1) 由测点坐标与深度构造矩阵/梯度；2) Matrix::inv 求逆；
 *                   3) 迭代更新平面系数 data_plane。
 * Sonar           ：Init() 采样 100 次求零偏；Handle() 发空格请求测距；
 *                   Receive() 解析 0xFF 帧头 3 字节并扣除零偏。
 */
class PressureSensor: public  Device{

    unsigned char MS5837_30BA_Crc4(int id);
    unsigned long MS5837_30BA_GetConversion(uint8_t command);
    uint8_t MS5837_30BA_PROM(int id);
    void MS5837_30BA_ReSet(void);
    float MS5837_30BA_GetData(int id);
    //signed int MS5837_30BA_GetTemp(int id);

    void Init_single(int id);
    void Handle_single(int id);
    void Handle_all();
    void Calibrate_single(int id);
    void Calibrate();
    void OutputData_single(int id);
    void OutputData();
    void Solve_plane_3(float* data,float* h,float* x,float* y,float* z);
    void Solve_plane(float* data,float* h,float* x,float* y,float* z,int num);
    void Update_plane();//计算水面方程

    void Delay_us(uint32_t us);

    //void get_angle(float q[4], float *yaw, float *pitch, float *roll);

    signed int dT,TEMP;
    uint32_t Cal_C[6][7];
    int32_t OFFi,SENSi,Ti;
    int64_t OFF2;
    int64_t SENS2;
    uint32_t TEMP2;
    int64_t OFF_,SENS;
    uint32_t D1_Pres,D2_Temp;
    unsigned char flag_ok[6];
    
    int32_t CurrentID;

    Sensor_Site_t site;

public:

    void Init();
    void Handle();
    void Receive();

    float Temperature;
    signed int data_temp[6];
    float data_pressure_offset[6]; // 压强零偏值，在水上测量
    float data_pressure_raw[6]; // 水下压强原始值
    float data_pressure[6]; // 水下压强去掉零偏，即为深度值
    float data_plane[5]; // 水面方程Ax+By+Cz+D=0,A*A+B*B+C*C=0,5个数据分别为A、B、C、D、拉格朗日乘数
    float data_depth; // 深度数据，单位cm
    float data_roll,data_pitch;
    float data_level[3];
    //float quat[4];
    float pitch,roll; // 俯仰角、横滚角
    bool flag_calibrate; // 校准水压计零偏的标志
    bool flag_read_pres; // 读取水压计读数的标志

    //uint8_t flag_Busy;
    static PressureSensor pressure_sensor;
    PS_HANDLE_STATE ps_state = PS_HANDLE_STATE::GET_TEMPERATURE;

};

/**
 * @class Sonar
 * @brief 声呐测距设备：USART6 请求-应答式测距与零点校准。
 * @details Init() 上电连续采样 100 次求平均得到 data_offset；Handle()
 *          发送一个空格字符触发一次测距；Receive() 解析以 0xFF 开头的
 *          3 字节帧得到距离并扣除零点偏移，帧头不符时 data 置 -1。
 * @note  与所有设备共用全局 RxBuffer，且共用 USART6 接收回调；
 *        static Sonar sonar 提供全局访问入口。
 */
class Sonar: public Device{

    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data;
    int32_t data_offset;

public:

    void Init();
    void Handle();
    void Receive();
    static Sonar sonar;
};

#endif //CONTROL_FRAME_MAIN_SENSOR_H
