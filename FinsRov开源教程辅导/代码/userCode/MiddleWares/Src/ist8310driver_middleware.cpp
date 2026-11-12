/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310�������м�㣬���IST8310��IICͨ��,��Ϊ����MPU6500��SPIͨ��
  *             �������õ���ͨ��mpu6500��IIC_SLV0��ɶ�ȡ��IIC_SLV4���д�롣
  * @note       IST8310ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

/**
 * @file    ist8310driver_middleware.cpp
 * @brief   IST8310 硬件抽象层的 STM32 HAL 实现（I2C3 + RSTN + 延时）。
 * @details 所属模块：userCode/MiddleWares，接口见 Inc/ist8310driver_middleware.h。
 *          四个寄存器访问函数统一用 HAL_I2C_Mem_Read / HAL_I2C_Mem_Write，
 *          设备地址 IST8310_IIC_ADDRESS、8 位寄存器地址、超时 100 ms；
 *          RST_H/RST_L 控制 RSTN 引脚完成复位；延时分别用 HAL_Delay 与
 *          按 168 MHz 主频标定的空循环实现。
 * @note    GPIO_init / com_init 为空实现，因为 CubeMX 的 MX_GPIO_Init() 与
 *          MX_I2C3_Init() 已完成初始化；移植到其它平台时在此补齐。
 * @note    微秒延时为近似值且可能被编译优化，要求精确时序时应换用定时器。
 */
#include "ist8310driver_middleware.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c3;


void ist8310_GPIO_init(void)
{
}

void ist8310_com_init(void)
{
}


/**
 * @brief 读取 IST8310 单个寄存器（阻塞式 I2C Mem 读）。
 * @param reg 8 位寄存器地址。
 * @return 读回的一字节；通信失败时返回 HAL 未写入的初值，调用方需留意。
 * @note  使用 hi2c3、设备地址 IST8310_IIC_ADDRESS、8 位地址长度、100 ms 超时。
 */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明局部变量 res 存放读回字节。
     * 步骤 2：调用 HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg,
     *         I2C_MEMADD_SIZE_8BIT, &res, 1, 100) 发起一次寄存器读。
     * 步骤 3：返回 res；通信失败时该值保持未初始化，调用方需靠 ID 校验兜底。
     */
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}
/**
 * @name 其余底层接口说明
 * @details ist8310_IIC_write_single_reg / write_muli_reg：HAL_I2C_Mem_Write
 *          单字节与多字节写；ist8310_IIC_read_muli_reg：从 reg 起连续读 len
 *          字节（磁力计数据帧用）；ist8310_delay_ms 转发 HAL_Delay，
 *          delay_us 为按 168 MHz 主频标定的空循环；
 *          ist8310_GPIO_init / com_init 为空钩子（CubeMX 已完成初始化）；
 *          RST_H / RST_L 置高/拉低 RSTN 复位引脚，配合毫秒延时完成硬件复位。
 * @note  所有 I2C 操作使用 hi2c3、8 位寄存器地址、100 ms 超时，
 *        未检查 HAL 返回值，总线异常时需上层通过 WHO_AM_I 校验发现。
 */
/**
 * @brief 写单个寄存器。
 * @param reg  寄存器地址。
 * @param data 待写入字节。
 */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    /* 函数内部流程步骤：
     * 步骤 1：以 hi2c3 为总线、IST8310_IIC_ADDRESS 为器件地址。
     * 步骤 2：调用 HAL_I2C_Mem_Write(reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100)
     *         把 1 字节写入指定寄存器。
     * 步骤 3：函数无返回值，配置是否生效由驱动层回读校验。
     */
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

}
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 函数内部流程步骤：
     * 步骤 1：以 IST8310_IIC_ADDRESS 为设备地址、reg 为 8 位起始寄存器地址。
     * 步骤 2：调用 HAL_I2C_Mem_Read(&hi2c3, ..., buf, len, 100) 发起
     *         一次「写寄存器地址 + 重复起始 + 连续读 len 字节」的组合传输。
     * 步骤 3：利用器件寄存器地址自动递增，把结果顺序写入 buf；
     *         函数不检查 HAL 返回值，超时 100 ms。
     */
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
void ist8310_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}
void ist8310_delay_us(uint16_t us)
{
    /* 函数内部流程步骤：
     * 步骤 1：外层 for 循环执行 us 次（每次约 1 微秒）。
     * 步骤 2：每轮把局部计数 a 置 10（按 168 MHz 主频标定）。
     * 步骤 3：while (a--) 空转消耗 CPU 周期，全部循环结束后返回。
     */
        uint16_t i;
    for (i = 0; i < us; i++ )
    {
        int a = 10;  //delay based on mian clock, 168Mhz
        while (a-- );
    }
}

void ist8310_RST_H(void)
{
    /* 复位相关函数的内部流程步骤：
     * 步骤 1：ist8310_RST_L() 用 HAL_GPIO_WritePin 把 IST8310_RSTN 拉低，
     *         使器件保持复位状态（驱动层保持 50ms）。
     * 步骤 2：ist8310_RST_H() 把同一引脚置高释放复位（再保持 50ms），
     *         随后驱动才读取 WHO_AM_I 确认器件就绪。
     * 步骤 3：GPIO_init()/com_init() 为空钩子，引脚与 I2C3 均由 CubeMX 初始化。
     */
    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_SET);
}
extern void ist8310_RST_L(void)
{
    HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_RESET);
}
