/**
 * @file    BMI088driver.cpp
 * @brief   BMI088 六轴 IMU 驱动实现：初始化自检、SPI 寄存器读写与数据换算。
 * @details 所属模块：userCode/drivers，接口见 Inc/BMI088driver.h，寄存器见
 *          BMI088reg.h，硬件抽象见 MiddleWares/BMI088Middleware.h。
 *          1) SPI 协议封装：写寄存器直接发「地址 + 数据」；读寄存器首字节为
 *             (reg | 0x80)，加速度计还需额外一次空读 0x55 才返回有效数据
 *             （见 BMI088_accel_read_single_reg 宏），陀螺仪无需空读；
 *             每个宏在读写前后自动拉低/拉高对应器件的 CS。
 *          2) 配置表：write_BMI088_accel/gyro_reg_data_error[][] 每行是
 *             {寄存器, 期望值, 失败错误码}，初始化时逐项写入并回读校验，
 *             任何一项不符即返回对应错误码，便于定位故障环节。
 *          3) 初始化：BMI088_init() 先自检加速度计与陀螺仪，再分别执行
 *             自检通信 -> 软复位 -> WHO_AM_I 校验 -> 写配置表；自检通过
 *             施加正/负激励并比较输出差值（加速度计）或触发 BIST 检查
 *             RATE_OK/BIST_FAIL 位（陀螺仪）。
 *          4) 数据换算：read_over 系列把 DMA 收到的字节流拼成小端 int16，
 *             乘灵敏度得到 g / rad/s；加速度额外用 C01~C09 椭球校准矩阵
 *             补偿；温度是 11 位有符号数，换算为 ℃；sensor time 按
 *             39.0625 us/计数换算。
 * @note    BMI088_ACCEL_SEN 当前赋值 BMI088_ACCEL_6G_SEN，而
 *          BMI088driver.h 中选择的量程宏为 BMI088_ACCEL_RANGE_3G，
 *          两者不一致会导致加速度比例偏差，改量程时务必同步确认。
 * @note    BMI088_read() 与 get_* 系列是阻塞式寄存器读取（用于调试/自检），
 *          高频采样请使用 IMU.cpp 中的 SPI + DMA 突发读取路径。
 */
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"


float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;



#if defined(BMI088_USE_SPI)

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//#define BMI088_accel_write_muli_reg( reg,  data, len) { BMI088_ACCEL_NS_L(); BMI088_write_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//#define BMI088_gyro_write_muli_reg( reg,  data, len) { BMI088_GYRO_NS_L(); BMI088_write_muli_reg( ( reg ), ( data ), ( len ) ); BMI088_GYRO_NS_H(); }
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

/**
 * @brief 初始化 BMI088 的两个器件并把自检/配置错误汇总返回。
 * @return BMI088_NO_ERROR 表示全部通过；否则为按位或的错误码：
 *         加速度计/陀螺仪自检失败(0x80/0x40) 或各寄存器配置错误码。
 * @note  先调用中间件的 GPIO/com 初始化钩子，再分别执行自检；只有自检
 *        通过才继续对应器件的寄存器配置，避免对损坏器件做无效写入。
 * @warning 错误码用按位或合并，配置类错误码(0x01~0x0D) 与自检位可同时出现。
 */
uint8_t BMI088_init(void)
{
    /* 函数内部流程步骤：
     * 步骤 1：把 error 初始化为 BMI088_NO_ERROR。
     * 步骤 2：调用 BMI088_GPIO_init() 与 BMI088_com_init() 完成抽象层初始化
     *         （当前为空实现，实际初始化由 CubeMX 完成）。
     * 步骤 3：执行加速度计自检：失败则把 BMI088_SELF_TEST_ACCEL_ERROR(0x80)
     *         按位或进 error；成功才调用 bmi088_accel_init() 配置寄存器。
     * 步骤 4：执行陀螺仪自检：失败则把 BMI088_SELF_TEST_GYRO_ERROR(0x40)
     *         按位或进 error；成功才调用 bmi088_gyro_init()。
     * 步骤 5：返回合并后的 error；调用方据此判断是自检失败还是某寄存器配置失败。
     */
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    // self test pass and init
    if (bmi088_accel_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    }
    else
    {
        error |= bmi088_accel_init();
    }

    if (bmi088_gyro_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    }
    else
    {
        error |= bmi088_gyro_init();
    }

    return error;
}

uint8_t bmi088_accel_init(void)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明回读值 res 与写寄存器计数 write_reg_num。
     * 步骤 2：连续两次读加速度计 CHIP_ID（中间延时 COM_WAIT_SENSOR_TIME），
     *         验证 SPI 通信是否已建立。
     * 步骤 3：向 ACC_SOFTRESET 写 0xB6 触发软复位，并延时 LONG_DELAY_TIME。
     * 步骤 4：复位后再读两次 CHIP_ID；若不等于 0x1E 返回 BMI088_NO_SENSOR。
     * 步骤 5：遍历 6 项配置表：写寄存器 -> 延时 -> 回读 -> 延时 -> 比对，
     *         不一致立即返回该行的错误码（电源/量程/ODR/中断映射等）。
     * 步骤 6：全部校验通过返回 BMI088_NO_ERROR。
     */
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明 write_reg_num 与回读值 res。
     * 步骤 2：连续两次读陀螺仪 CHIP_ID（各延时 COM_WAIT_SENSOR_TIME）验通信。
     * 步骤 3：写 GYRO_SOFTRESET=0xB6 软复位并延时 LONG_DELAY_TIME。
     * 步骤 4：复位后再读两次 CHIP_ID，若不等于 0x0F 返回 BMI088_NO_SENSOR。
     * 步骤 5：遍历 6 项陀螺配置表（量程/带宽/电源/DRDY/INT3-4 配置与映射），
     *         每项写入后回读比对，不一致返回对应错误码。
     * 步骤 6：全部通过返回 BMI088_NO_ERROR。
     */
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t bmi088_accel_self_test(void)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明正/负激励两次的三轴读数、6 字节缓冲与 6 行自检配置表
     *         （ODR 1600Hz、±24G、电源/自检寄存器及对应错误码）。
     * 步骤 2：读两次 CHIP_ID 验通信，软复位并等待，再读两次 CHIP_ID，
     *         不等于 0x1E 返回 NO_SENSOR。
     * 步骤 3：逐项写入前 4 行自检配置并回读校验，每项额外等待 50ms 以上。
     * 步骤 4：分别写入正激励(0x0D)与负激励(0x09)，各校验、延时后从
     *         ACCEL_XOUT_L 连续读 6 字节，拼成三轴 int16 存入 self_test_accel。
     * 步骤 5：写 SELF_TEST_OFF 并回读确认，随后再次软复位。
     * 步骤 6：比较两次读数差：X/Y 需 >=1365、Z 需 >=680，否则返回
     *         BMI088_SELF_TEST_ACCEL_ERROR；通过则返回 NO_ERROR。
     */

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    volatile uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    //reset the accel sensor
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}
uint8_t bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    do
    {

        BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

void BMI088_read_gyro_who_am_i(void)
{
    uint8_t buf;
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, buf);
}


void BMI088_read_accel_who_am_i(void)
{
    volatile uint8_t buf;
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, buf);
    buf = 0;

}





void BMI088_temperature_read_over(uint8_t *rx_buf, float *temperate)
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

}

/**
 * @brief 解析加速度计 DMA 帧：三轴原始值 -> 灵敏度换算 -> 椭球校准 -> 时间戳。
 * @param rx_buf 指向有效数据首字节（已跳过命令/空读字节），至少含 7 字节。
 * @param accel  [输出] 校准后的三轴加速度（g）。
 * @param time   [输出] 由传感器时间字节换算的时间（39.0625 us/计数）。
 * @note  每轴为小端 int16；校准矩阵 C01~C09 补偿尺度与轴间耦合；
 *        源码注释指出官方 24 位 sensor time 拼接写法会越界，此处只用
 *        rx_buf[6] 低 8 位，时间戳精度有限。
 */
void BMI088_accel_read_over(uint8_t *rx_buf, float accel[3], float *time)
{
    int16_t bmi088_raw_temp;
    uint32_t sensor_time;
    float a[3] = {0};

    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    a[0] = bmi088_raw_temp * BMI088_ACCEL_SEN ;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    a[1] = bmi088_raw_temp * BMI088_ACCEL_SEN ;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    a[2] = bmi088_raw_temp * BMI088_ACCEL_SEN ;
    accel[0] = a[0] * C01 + a[1] * C02 + a[2] * C03;
    accel[1] = a[0] * C04 + a[1] * C05 + a[2] * C06;
    accel[2] = a[0] * C07 + a[1] * C08 + a[2] * C09;
    //sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]); 官方代码疑似超界
    sensor_time = (uint32_t)( rx_buf[6]);
    *time = sensor_time * 39.0625f;

}

void BMI088_gyro_read_over(uint8_t *rx_buf, float gyro[3])
{
    /* 三个 *_read_over() 函数内部的执行流程步骤：
     * 步骤 1：按小端格式把相邻两字节拼成 int16_t 原始值。
     * 步骤 2：陀螺帧对 rx_buf[0..5] 拼三次，每次乘 BMI088_GYRO_SEN
     *         写入 gyro[0..2]（rad/s）。
     * 步骤 3：加速度帧同样拼三轴并乘 BMI088_ACCEL_SEN，再用 C01~C09
     *         矩阵做椭球校准写入 accel[]，最后用 rx_buf[6] 换算时间戳。
     * 步骤 4：温度帧把 (rx_buf[0]<<3 | rx_buf[1]>>5) 拼成 11 位有符号数，
     *         >1023 时减 2048，再按 TEMP_FACTOR/TEMP_OFFSET 换算为摄氏度。
     */
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}

void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    /* 函数内部流程步骤：
     * 步骤 1：准备 8 字节 buf 与临时 int16_t。
     * 步骤 2：从 ACCEL_XOUT_L 连续读 6 字节，按小端拼出三轴加速度原始值，
     *         乘 ACCEL_SEN 后再用 C01~C09 椭球矩阵校准写入 accel[]。
     * 步骤 3：从 GYRO_CHIP_ID 起连续读 8 字节；仅当 buf[0] 等于 0x0F 时，
     *         取 buf[2..7] 拼出三轴角速度并乘 GYRO_SEN 写入 gyro[]。
     * 步骤 4：从 TEMP_M 读 2 字节拼成 11 位有符号温度（>1023 时减 2048），
     *         按 factor*raw + offset 换算摄氏温度写入 *temperate。
     */
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    float a[3] = {0};

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    a[0] = bmi088_raw_temp * BMI088_ACCEL_SEN ;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    a[1] = bmi088_raw_temp * BMI088_ACCEL_SEN ;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    a[2] = bmi088_raw_temp * BMI088_ACCEL_SEN ;
    accel[0] = a[0] * C01 + a[1] * C02 + a[2] * C03;
    accel[1] = a[0] * C04 + a[1] * C05 + a[2] * C06;
    accel[2] = a[0] * C07 + a[1] * C08 + a[2] * C09;
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

uint32_t get_BMI088_sensor_time(void)
{
    uint32_t sensor_time = 0;
    uint8_t buf[3];
    BMI088_accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);

    sensor_time = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return sensor_time;
}

float get_BMI088_temperate(void)
{
    uint8_t buf[2];
    float temperate;
    int16_t temperate_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    temperate_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temperate_raw_temp > 1023)
    {
        temperate_raw_temp -= 2048;
    }

    temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return temperate;
}

void get_BMI088_gyro(int16_t gyro[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t gyro_raw_temp;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);

    gyro_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    gyro[0] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[1] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[2] = gyro_raw_temp ;
}

void get_BMI088_accel(float accel[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t accel_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    accel_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = accel_raw_temp * BMI088_ACCEL_SEN;
}

#if defined(BMI088_USE_SPI)

/**
 * @name SPI 底层单字节/多字节寄存器读写
 * @details 写：先发寄存器地址（最高位 0），再发数据；
 *          读单字节：发 (reg | 0x80) 后再发一个哑字节 0x55 产生时钟并
 *          取回 MISO 数据；读多字节：发 (reg | 0x80) 后连续发 len 个
 *          0x55，依次保存读回字节，利用器件地址自动递增。
 *          加速度计的额外空读由上层宏 BMI088_accel_read_single_reg 完成。
 * @note  四个函数为文件内 static，仅在 BMI088_USE_SPI 分支编译；
 *        IIC 分支为空，切换总线需自行实现同名函数。
 */
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len )
//{
//    BMI088_read_write_byte( reg );
//    while( len != 0 )
//    {

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
