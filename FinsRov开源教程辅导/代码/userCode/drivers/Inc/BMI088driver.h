/**
 * @file    BMI088driver.h
 * @brief   Bosch BMI088 六轴 IMU（3 轴加速度计 + 3 轴陀螺仪）驱动接口。
 * @details 所属模块：userCode/drivers，寄存器定义见 BMI088reg.h，硬件访问
 *          经 MiddleWares/BMI088Middleware.h 抽象，实现位于 Src/BMI088driver.cpp。
 *          驱动负责：上电初始化与自检、量程/带宽/中断配置、通过 SPI/IIC
 *          读取原始 16 位数据、按量程灵敏度换算为物理单位，并做加速度计
 *          椭球校准补偿；devices/IMU 层再基于这些数据做姿态解算。
 * @note    加速度计与陀螺仪在 BMI088 内部是两个独立器件，各有自己的
 *          CHIP_ID、片选和初始化流程（见 bmi088_accel_init / bmi088_gyro_init）。
 * @note    量程通过下方宏选择（当前 ±3g / ±2000 dps），改量程时必须同步
 *          修改对应的 *_SEN 灵敏度换算系数，否则数据比例错误。
 * @note    陀螺仪输出经灵敏度换算后单位为 rad/s，可直接送 Mahony/Madgwick
 *          姿态更新函数；加速度单位为 g（重力加速度倍数）。
 */

#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

/* 经 main.h 引入 HAL 的 uint8_t/int16_t 等类型与 __packed 定义 */
#include "main.h"

/* 片内温度换算：temp(℃) = 原始值 * TEMP_FACTOR + TEMP_OFFSET */
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

/* 一次配置写入的寄存器数量，用于连续写/校验时序 */
#define BMI088_WRITE_ACCEL_REG_NUM  6
#define BMI088_WRITE_GYRO_REG_NUM   6

/* 数据就绪状态字节中的位定义（供中断/轮询判断各传感器数据是否更新） */
#define BMI088_GYRO_DATA_READY_BIT          0
#define BMI088_ACCEL_DATA_READY_BIT         1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT    2

/* 上电/软复位后的长延时（ms），等待器件内部稳定 */
#define BMI088_LONG_DELAY_TIME      80
/* 通信重试等待时间（ms），超过则判定器件无响应 */
#define BMI088_COM_WAIT_SENSOR_TIME 150


/* 7 位 IIC 地址左移 1 位，留出最低位作为读写位（HAL_I2C_Mem_* 使用此形式） */
#define BMI088_ACCEL_IIC_ADDRESSE   (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE    (0x68 << 1)

/* 加速度计量程选择：同一时刻只能启用一个，当前为 ±3g */
#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

/* 陀螺仪量程选择：同一时刻只能启用一个，当前为 ±2000 dps */
#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


/* 加速度计各量程的灵敏度：原始 LSB 换算为 g 的系数 */
#define BMI088_ACCEL_3G_SEN     0.0008974358974f
#define BMI088_ACCEL_6G_SEN     0.00179443359375f
#define BMI088_ACCEL_12G_SEN    0.0035888671875f
#define BMI088_ACCEL_24G_SEN    0.007177734375f


/* 陀螺仪各量程灵敏度：原始 LSB 换算为 rad/s 的系数 */
#define BMI088_GYRO_2000_SEN    0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN    0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN     0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN     0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN     0.000066579027251980956150958662738366f

/**
 * @name 加速度计椭球拟合校准系数
 * @details 用于补偿零偏与三轴不正交/尺度误差，形式为
 *          a_cal = C * (a_raw - b)：C01~C09 为 3×3 尺度/耦合矩阵元素，
 *          Cx/Cy/Cz 为三轴零偏。数值由离线标定得到，换传感器或安装方向
 *          改变后需重新标定。
 */
//椭球误差
#define C01 1.008498226940123
#define C02 -0.005816852352529
#define C03 0.009441457186672
#define C04 0.023028251248462
#define C05 1.005148480220749
#define C06 0.028993786334924
#define C07 -0.016200542266908
#define C08 -0.000371166789066
#define C09 1.004063669935703
#define Cx 0.049056146688075
#define Cy 0.039100860921584
#define Cz -0.190826310443001


/**
 * @struct bmi088_raw_data_t
 * @brief 一次 DMA/总线突发读取得到的原始寄存器数据布局（紧凑排列，无填充）。
 * @note  __packed 保证与传感器寄存器字节顺序一一对应，可直接按字节流解读；
 *        accel/temp/gyro 均为小端 16 位有符号整数。
 */
typedef  struct BMI088_RAW_DATA
{
    uint8_t status;     // 数据就绪/状态字节
    int16_t accel[3];   // 加速度计 X/Y/Z 原始值
    int16_t temp;       // 片内温度传感器原始值
    int16_t gyro[3];    // 陀螺仪 X/Y/Z 原始角速度
} __packed bmi088_raw_data_t;

/**
 * @struct bmi088_real_data_t
 * @brief 换算为物理单位后的传感器数据，供上层算法直接使用。
 */
typedef struct BMI088_REAL_DATA
{
    uint8_t status;     // 数据状态（同原始结构）
    float accel[3];     // 三轴加速度，单位 g
    float temp;         // 片内温度，单位 ℃
    float gyro[3];      // 三轴角速度，单位 rad/s
    float time;         // 传感器时间戳，单位 s（由 24 位 sensor time 换算）
} bmi088_real_data_t;


/**
 * @brief BMI088 初始化错误码枚举（匿名 enum，作为 uint8_t 返回）。
 * @note  0x00 表示成功；0x01~0x0D 依次对应某个初始化步骤的寄存器回读校验失败，
 *        便于定位是电源配置、量程、带宽还是中断映射出错；
 *        0x80/0x40 为加速度计/陀螺仪自检失败位，0xFF 表示器件无响应。
 */
enum
{
    BMI088_NO_ERROR                     = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR           = 0x01,
    BMI088_ACC_PWR_CONF_ERROR           = 0x02,
    BMI088_ACC_CONF_ERROR               = 0x03,
    BMI088_ACC_SELF_TEST_ERROR          = 0x04,
    BMI088_ACC_RANGE_ERROR              = 0x05,
    BMI088_INT1_IO_CTRL_ERROR           = 0x06,
    BMI088_INT_MAP_DATA_ERROR           = 0x07,
    BMI088_GYRO_RANGE_ERROR             = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR         = 0x09,
    BMI088_GYRO_LPM1_ERROR              = 0x0A,
    BMI088_GYRO_CTRL_ERROR              = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR        = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR         = 0x40,
    BMI088_NO_SENSOR                    = 0xFF,
};




/**
 * @brief 完整初始化 BMI088：通信接口、GPIO、加速度计与陀螺仪自检/配置。
 * @return BMI088_NO_ERROR(0) 成功；否则返回上方枚举中的错误码。
 * @note  内部会调用中间件的 GPIO/com 初始化、软复位并等待上电稳定，
 *        失败时可用返回码定位具体寄存器配置环节。
 */
/* BMI088driver.cpp 中主要函数内部的执行流程步骤：
 * BMI088_init()      ：1) GPIO/com 初始化；2) 加速度计自检，失败置 0x80 位，
 *                      成功则配置加速度计；3) 陀螺仪自检，失败置 0x40 位，
 *                      成功则配置陀螺仪；4) 返回按位或的错误码。
 * *_init()           ：1) 连读两次 CHIP_ID 验通信；2) 软复位并长延时；
 *                      3) 再读 CHIP_ID 校验；4) 遍历配置表逐项写入并回读。
 * *_self_test()      ：加速度计施加正/负激励比较输出差值；陀螺仪触发 BIST
 *                      并轮询 BIST_RDY、检查 BIST_FAIL。
 * *_read_over()      ：1) 按小端拼 int16；2) 乘灵敏度；3) 加速度再做椭球
 *                      校准、温度做 11 位符号扩展、时间按 39.0625us 换算。
 * SPI 底层读写       ：写「地址+数据」；读「地址|0x80」后发哑字节取回数据，
 *                      多字节读依靠寄存器地址自动递增。
 */
extern uint8_t BMI088_init(void);
/** @brief 加速度计自检：触发正负激励并比较输出差值是否落在器件规格范围内。
 *  @return 0 表示通过，否则返回 BMI088_SELF_TEST_ACCEL_ERROR 对应错误。 */
extern uint8_t bmi088_accel_self_test(void);
/** @brief 陀螺仪自检：触发 BIST 并检查 RATE_OK / BIST_FAIL 标志。
 *  @return 0 表示通过，否则返回自检错误码。 */
extern uint8_t bmi088_gyro_self_test(void);
/** @brief 仅初始化加速度计（电源模式、量程、ODR/带宽、DRDY 中断映射）。
 *  @return 0 成功，其他为对应步骤的错误码。 */
extern uint8_t bmi088_accel_init(void);
/** @brief 仅初始化陀螺仪（量程、带宽、电源模式、DRDY 中断配置）。
 *  @return 0 成功，其他为对应步骤的错误码。 */
extern uint8_t bmi088_gyro_init(void);

/**
 * @brief 把加速度计 DMA 读回的原始字节流解析为物理量。
 * @param rx_buf 原始字节缓冲区（含状态、三轴数据、温度与传感器时间）。
 * @param accel  [输出] 三轴加速度（g），已完成灵敏度换算与椭球校准。
 * @param time   [输出] 由 24 位传感器时间换算得到的时间戳（s）。
 */
extern void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time);
/**
 * @brief 把陀螺仪 DMA 读回的原始字节流解析为角速度。
 * @param rx_buf 原始字节缓冲区。
 * @param gyro   [输出] 三轴角速度（rad/s），按当前量程灵敏度换算。
 */
extern void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3]);
/**
 * @brief 从温度寄存器字节流解析片内温度。
 * @param rx_buf     原始温度字节缓冲区。
 * @param temperate  [输出] 温度值（℃）。
 */
extern void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate);
/**
 * @brief 一次性读取并换算陀螺、加速度与温度（阻塞式寄存器读取版本）。
 * @param gyro      [输出] 三轴角速度（rad/s）。
 * @param accel     [输出] 三轴加速度（g）。
 * @param temperate [输出] 片内温度（℃）。
 */
extern void BMI088_read(float gyro[3], float accel[3], float *temperate);
/** @brief 获取最近一次读取到的 24 位传感器时间计数值。
 *  @return 传感器时间计数（约 39 us/LSB，具体见驱动实现）。 */
extern uint32_t get_BMI088_sensor_time(void);
/** @brief 获取最近一次换算得到的片内温度。
 *  @return 温度值（℃）。 */
extern float get_BMI088_temperate(void);
/** @brief 获取最近一次陀螺仪的原始 16 位三轴数据。
 *  @param gyro [输出] int16_t[3] 原始值。 */
extern void get_BMI088_gyro(int16_t gyro[3]);
/** @brief 获取最近一次换算后的三轴加速度。
 *  @param accel [输出] float[3]，单位 g。 */
extern void get_BMI088_accel(float accel[3]);


/** @brief 读取并打印/校验陀螺仪 WHO_AM_I（应为 0x0F），用于排查通信故障。 */
extern void BMI088_read_gyro_who_am_i(void);
/** @brief 读取并打印/校验加速度计 WHO_AM_I（应为 0x1E），用于排查通信故障。 */
extern void BMI088_read_accel_who_am_i(void);

#endif
