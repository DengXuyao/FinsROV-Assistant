//
// Created by LEGION on 2021/10/17.
//

/**
 * @file    IMU.h
 * @brief   惯性测量单元（IMU）设备：BMI088 + IST8310 数据采集与姿态解算。
 * @details 所属模块：userCode/devices，继承 Device 抽象基类，实现位于 Src/IMU.cpp。
 *          数据链路：SPI1 + DMA2_Stream0 依次突发读取 BMI088 陀螺/加速度/温度
 *          寄存器 -> 换算为物理量并做零偏/椭球校准 -> 滑动加权平均滤波 ->
 *          Mahony/Madgwick AHRS 融合磁力计得到四元数 -> 转换成欧拉角、
 *          角速度与积分位移；同时用 PID 控制加热电阻维持 IMU 恒温。
 * @note    单例访问：类内静态成员 IMU::imu 供中断回调等无法传对象的场合使用；
 *          DMA2_Stream0_IRQHandler 以 extern "C" 声明，需与启动文件的中断向量一致。
 * @note    DMA 状态机 IMU_DMA_State_e 在陀螺/加速度/温度三种读帧间轮转，
 *          每次 DMA 完成中断调用 DMA_IT_Handle() 解析并触发下一帧。
 * @note    量程灵敏度 BMI088_ACCEL_SEN / BMI088_GYRO_SEN 在 .cpp 中按
 *          BMI088driver.h 的量程宏赋值，改量程时须保持一致。
 */
#ifndef RM_FRAME_C_IMU_H
#define RM_FRAME_C_IMU_H

#include "BMI088driver.h"
#include "ist8310driver.h"
#include "Usermain.h"
#include "Kalman_Filter.h"
#include "MahonyAHRS.h"
#include "bsp_spi.h"
#include "PID.h"
#include <stdint.h>

/**
 * @name DMA 帧长度、状态位与缓冲区偏移
 * @details SPI 读 BMI088 时首字节为「寄存器地址 | 0x80」，随后每字节返回
 *          一个数据，因此各帧长度 = 1(命令) + 数据字节数 + 加速度计额外空读字节；
 *          对应的 *_RX_BUF_DATA_OFFSET 指出有效数据从接收缓冲区第几字节开始。
 */
// 输出数据个数
#define OUTPUT_NUM                3
 
// 是否融合磁力计数据
#define IMU_USE_MAG               1 //new

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS        0u
#define IMU_SPI_SHFITS       1u
#define IMU_UPDATE_SHFITS    2u
#define IMU_NOTIFY_SHFITS    3u

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1u
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2u

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16

//平均滤波采样值
#define SUM_WIN_SIZE 8
#define IMU_TARGET_TEMP 45.0f
#define MPU6500_TEMP_PWM_MAX 2000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1

extern float BMI088_ACCEL_SEN;
extern float BMI088_GYRO_SEN;

//轴偏
/*constexpr float C1 = 1.010006001930344f;
constexpr float C2 = -0.007492008326987f;
constexpr float C3 = -0.009254187692463f;
constexpr float C4 = 0.016494226767657f;
constexpr float C5 = 1.004509527569225f;
constexpr float C6 = 0.022698224839392f;
constexpr float C7 = -0.006601428647036f;
constexpr float C8 = -0.000182664761106f;
constexpr float C9 = 1.016633267989222f;
constexpr float Cx = 0.099323298159297f;
constexpr float Cy = 0.017485774998530f;
constexpr float Cz = -0.166100036392575f;*/

//椭球误差
#define MAG_SCALE_X 0.889875
#define MAG_SCALE_Y 0.880329
#define MAG_SCALE_Z 1.229795
#define MAG_OFFSET_X 8.418958
#define MAG_OFFSET_Y -21.033503
#define MAG_OFFSET_Z -4.048424

/*枚举类型定义------------------------------------------------------------*/
/*结构体定义--------------------------------------------------------------*/


/**
 * @name IMU 数据结构
 * @details Raw：直接来自传感器（含零偏与滤波前数据）；
 *          Pro：处理后数据（含四元数 ins_quat 与欧拉角 ins_angle）；
 *          Attitude：姿态角及其角速度、取反角速度（便于控制环符号处理）；
 *          State：各路数据更新标志与 DMA 启动标志（volatile，供中断修改）；
 *          DMA_State：DMA 轮转状态机；Position：由加速度积分得到的速度/位移；
 *          Filter：滑动加权平均滤波器（环形数组 + 权重 factor，K 为权重和）；
 *          Buffer：三组 SPI DMA 收发缓冲区，TX 首字节为带读标志的寄存器地址
 *          (0x82/0x92/0xA2)，其余为 0xFF 空字节用于产生时钟。
 */
typedef struct IMU_Raw_Data{
    float accel[3], gyro[3], temp, mag[3], accel_offset[3], gyro_offset[3], time;
} IMU_Raw_Data_t;

typedef struct IMU_Pro_Data{
    float accel[3], gyro[3], temp, mag[3], ins_quat[4], ins_angle[3];
} IMU_Pro_Data_t;

typedef struct IMU_Attitude{
    float yaw, pitch, roll;
    float yaw_v, pitch_v, roll_v;
    float neg_yaw_v, neg_pitch_v, neg_roll_v;
} IMU_Attitude_t;

typedef struct IMU_State{
    volatile uint8_t gyro_update_flag = 0;
    volatile uint8_t accel_update_flag = 0;
    volatile uint8_t accel_temp_update_flag = 0;
    volatile uint8_t mag_update_flag = 0;
    volatile uint8_t imu_start_dma_flag = 0;
}IMU_State_t;

typedef enum IMU_DMA_State{
    IMU_DMA_IDLE = 0,
    IMU_DMA_READ_GYRO,
    IMU_DMA_READ_ACCEL,
    IMU_DMA_READ_TEMP
}IMU_DMA_State_e;

typedef struct IMU_Position{
    float _accel[3] = {0};
    float velocity[3] = {0};
    float _velocity[3] = {0};
    float displace[3] = {0};
    float vy;
    float xy;
}IMU_Position_t;

typedef struct IMU_Filter{
    float current;
    float history[SUM_WIN_SIZE];//历史值，其中history[SUM_WIN_SIZE-1]为最近的记录

    int buff_init = 0; //前SUM_WIN_SIZE-1次填充后才能开始输出
    int index = 0; //环形数组可放数据的位置

    int factor[SUM_WIN_SIZE] = {1,2,3,4,5,6,7,8}; //加权系数
    int K=36; //1+2+3+4+5+6+7+8
}IMU_Filter_t;

typedef struct IMU_Buffer{
	
    uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
    uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
    uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
    uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
}IMU_Buffer_t;

/*类型定义----------------------------------------------------------------*/

/**
 * @class IMU
 * @brief 惯导设备：以静态实例 IMU::imu 提供全局访问的姿态与运动状态。
 * @details 私有方法按职责分组：trigger_*_dma/DMA_IT_Handle 管理 SPI DMA
 *          状态机；offset_correction/calibrate_offset 处理零偏；AHRS_update/
 *          AHRS_out/get_angle 完成姿态解算（当前实际路径在 DMA_IT_Handle
 *          中直接调用 MahonyAHRSupdate）；tempPid/imu_temp_control/
 *          IMU_temp_PWM 实现恒温控制；attitude_update 刷新姿态角与角速度。
 * @note  Init/Handle/Receive 实现 Device 接口；DMA_IT_Handle 由
 *        DMA2_Stream0_IRQHandler 调用，须与中断向量表中的 DMA 流一致。
 * @note  buf / raw_data / pro_data / attitude / position 等成员均为 public，
 *        供 Propeller 等控制器直接读取姿态反馈。
 */
/* IMU.cpp 中主要函数内部的执行流程步骤（实现见 Src/IMU.cpp）：
 * Init()          ：1) BMI088_init()，失败 Error_Handler；2) 若启用磁力计再
 *                   ist8310_init()；3) 手动 SPI 写 0x7D=0x04 唤醒加速度计；
 *                   4) 配置恒温 PID、四元数初值与零偏；5) SPI1_DMA_init()
 *                   建立 DMA 缓冲区并把 dma_state 置 IDLE。
 * Handle()        ：仅在 dma_state==IDLE 时置 READ_GYRO 并 trigger_gyro_dma()。
 * trigger_*_dma() ：1) 确认 TX/RX DMA 流均空闲；2) 拉低对应 CS；
 *                   3) SPI1_DMA_enable() 装载对应 TX/RX 缓冲与帧长。
 * DMA_IT_Handle() ：1) 判 TC 标志并清除；2) 按状态依次解析陀螺/加速度/温度；
 *                   3) 最后一帧做恒温、磁力计、零偏、AHRS、欧拉角、姿态刷新；
 *                   4) 可选调试输出；5) 状态回到 IDLE。
 * offset_correction()：1) 陀螺/加计逐轴减零偏；2) 磁力计减硬铁偏移后乘尺度。
 * imu_temp_control() ：1) 未达温时全功率加热并累计稳定拍数；2) 达温后切
 *                   到 PID，输出限幅非负；3) 写 htim10_CH1 的 CCR。
 */
class IMU : public Device
{
    void ErrorHandle();

    //读取数据
    void imu_cmd_spi_dma(void);
    void trigger_gyro_dma(void);
    void trigger_accel_dma(void);
    void trigger_temp_dma(void);

    //数据处理
    void offset_correction();
    void calibrate_offset();    //获取加速度零偏值

    //姿态解算
    void AHRS_update(float quat[4], float time, float accel[3], float mag[3]);
    void AHRS_out(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
    void get_angle(float q[4], float *yaw, float *pitch, float *roll);
    bool flag_Test;
    //温度控制
    PID tempPid;
    uint8_t first_temperate = 0;
    uint32_t count_imu;
    void imu_temp_control(float temp);
    void IMU_temp_PWM(uint16_t pwm);
    //位移获取
    void attitude_update();

public:

    void Init();
    void Handle();
    void Receive();
    void DMA_IT_Handle(void);

	static IMU imu;
    
    IMU_Buffer_t buf;
    IMU_Raw_Data_t raw_data;
    IMU_Pro_Data_t pro_data;
    IMU_Attitude_t attitude;
    IMU_Position_t position;
    IMU_Filter_t axFilter;
    IMU_Filter_t ayFilter;
    IMU_DMA_State_e dma_state;
};



//extern float variance;

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
/*外部函数声明-------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
extern void DMA2_Stream0_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_IMU_H
