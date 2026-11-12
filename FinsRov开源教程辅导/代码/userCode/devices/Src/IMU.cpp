//
// Created by LEGION on 2021/10/17.
//

/**
 * @file    IMU.cpp
 * @brief   IMU 设备实现：BMI088/IST8310 采集、姿态解算与恒温控制。
 * @details 所属模块：userCode/devices，配套声明见 Inc/IMU.h。
 *          初始化（Init）：BMI088_init() -> ist8310_init() -> 手动 SPI 写
 *          0x7D=0x04 让加速度计退出 suspend -> 配置恒温 PID、四元数初值
 *          {1,0,0,0} 与零偏 -> SPI1_DMA_init() 建立 DMA 收发的首组缓冲区。
 *          运行（Handle + 中断）：Handle() 仅在 dma_state == IDLE 时发起
 *          trigger_gyro_dma()；随后 DMA2_Stream0 完成中断调用 DMA_IT_Handle()，
 *          按 GYRO -> ACCEL -> TEMP 顺序依次拉高/拉低片选、切换缓冲区并
 *          触发下一帧。最后一帧完成时完成温度控制、磁力计读取、零偏补偿、
 *          Mahony 九轴姿态更新、欧拉角转换与姿态状态刷新，再回到 IDLE。
 * @note    Receive() 解析 "BEG"/"END"/"IVA:BEG"/"IVA:END" 等调试指令，
 *          控制测试标志与角度数据输出；调试数据通过 send_float() 发送。
 * @note    imu_temp_control() 先把加热 PWM 开到最大以快速升温，达到目标
 *          IMU_TARGET_TEMP 并稳定 200 拍后切换为 PID 恒温，并把积分限幅
 *          设为最大功率的一半以加快收敛；PWM 由 htim10_CH1 输出。
 * @note    offset_correction() 对陀螺/加计扣除零偏，对磁力计做硬铁偏移
 *          (MAG_OFFSET_*) 与软铁尺度 (MAG_SCALE_*) 校准。
 * @warning DMA 完成中断中做了较多浮点运算（AHRS、欧拉角），需确保中断
 *          优先级与执行时间不影响控制循环；片选 GPIO 与 bsp_spi 的 DMA
 *          配置强耦合，修改 CubeMX 的 SPI/DMA 流号时需同步检查。
 */
#include "IMU.h"
#include "ist8310driver.h"
#include "UART_Base.h"
#include "Sensor.h"
#include "Usermain.h"

IMU IMU::imu;

bool is_output_angle = 0;
// bool is_output_accl = 0;

void IMU::Init()
{
    /* 函数内部流程步骤：
     * 步骤 1：BMI088_init() 初始化加速度计/陀螺仪，失败则 Error_Handler()。
     * 步骤 2：定义 IMU_USE_MAG 时再 ist8310_init() 初始化磁力计。
     * 步骤 3：手动拉低 PA4(加速度计 CS)，SPI 依次发送 0x7D(写地址) 与
     *         0x04(开启测量)，每次等待 BUSY_TX 结束，延时 1ms 后拉高 PA4，
     *         让复位后默认挂起的加速度计进入正常模式。
     * 步骤 4：构造恒温 PID 参数(1500, 0.08, 0, 限幅 MPU6500_TEMP_PWM_MAX)
     *         并写入 tempPid.PIDInfo。
     * 步骤 5：四元数置 {1,0,0,0}、欧拉角清零，设置陀螺零偏 0 与
     *         加速度零偏经验值 (0.07, -0.17, 0)。
     * 步骤 6：HAL_SPI_Init 重新确认 SPI1，SPI1_DMA_init() 用陀螺缓冲区
     *         建立首组 DMA 收发，最后 dma_state 置 IMU_DMA_IDLE。
     */
    if (BMI088_init() != BMI088_NO_ERROR)
        Error_Handler();
#ifdef IMU_USE_MAG
    if (ist8310_init() != IST8310_NO_ERROR)
        Error_Handler();
#endif
    // BMI088_read(raw_data.gyro, raw_data.accel, &raw_data.temp);
		
		// After a reset, the accelerometer defaults to suspend mode. These nine lines write the value 0x04 to register address 0x7D, switching the accelerometer to normal mode.
    //加速度计复位后默认是暂停模式，这9行代码，向地址0x7D处写入0x04值，使加速度计进入正常模式
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);    // Set PA4 to low to assert the accelerometer chip select.  //PA4置0，片选加速度计
    uint8_t data = (0x7D & 0x7F);    // For Bit #0 and Bits #1–7: Bit #0 = 0 indicates a write operation.   //Bit #0和Bit #1-7，Bit #0是0，表示写
    HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    // Wait until SPI transmission completes. // 等待SPI发送完成
    data = 0x04;    //Bit #8-15
    HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);    // Wait until SPI transmission completes. // 等待SPI发送完成
    HAL_Delay(1);    // Delay for 1 ms. //延时1ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);    // Set PA4 to high to deassert the accelerometer chip select. // PA4置1，取消片选加速度计


    PID_Regulator_t _tempPID(1500, 0.08, 0, MPU6500_TEMP_PWM_MAX, MPU6500_TEMP_PWM_MAX, 0, MPU6500_TEMP_PWM_MAX);

    tempPid.PIDInfo = _tempPID;

    pro_data.ins_quat[0] = 1.0f;
    pro_data.ins_quat[1] = 0.0f;
    pro_data.ins_quat[2] = 0.0f;
    pro_data.ins_quat[3] = 0.0f;
    pro_data.ins_angle[0] = 0.0f;
    pro_data.ins_angle[1] = 0.0f;
    pro_data.ins_angle[2] = 0.0f;
    raw_data.gyro_offset[0] = 0.0f;
    raw_data.gyro_offset[1] = 0.0f;
    raw_data.gyro_offset[2] = 0.0f;
    raw_data.accel_offset[0] = 0.07f;
    raw_data.accel_offset[1] = -0.17f;
    raw_data.accel_offset[2] = 0.0f;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
    SPI1_DMA_init((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    dma_state = IMU_DMA_IDLE;

}

void IMU::Handle()
{
		// Normal operation.
    // 普通使用
    // BMI088_read(raw_data.gyro, raw_data.accel, &raw_data.temp);
    // ist8310_read_mag(raw_data.mag);    // Read magnetometer data and store it in `mag`. //读取磁力计数据，存储在mag中
    // offset_correction();
    // MahonyAHRSupdate(pro_data.ins_quat, pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2], pro_data.accel[0], pro_data.accel[1], pro_data.accel[2], pro_data.mag[0], pro_data.mag[1], pro_data.mag[2]);    // Compute the quaternion from the acquired accelerometer, gyroscope, and magnetometer measurements, and update it in-place in the `quat` array. //对得到的加速度计、陀螺仪和磁力计数据进行计算得到四元数，直接在数组quat中更新
	// // MahonyAHRSupdateIMU(pro_data.ins_quat, pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2], pro_data.accel[0], pro_data.accel[1], pro_data.accel[2]);    // Compute the quaternion from the acquired accelerometer and gyroscope measurements, and update it in-place in the `quat` array. //对得到的加速度计、陀螺仪数据进行计算得到四元数，直接在数组quat中更新
	// get_angle(pro_data.ins_quat, pro_data.ins_angle, pro_data.ins_angle+1, pro_data.ins_angle+2);    // Compute Euler angles (in radians) from the quaternion, and update them in-place in the `INS_angle` array. //对四元数计算得到弧度制欧拉角，直接在INS_angle数组中更新

		// Enable DMA transfer for the IMU.
    // 开启IMU DMA传输
    if (dma_state == IMU_DMA_IDLE)
    {
        dma_state = IMU_DMA_READ_GYRO;
        trigger_gyro_dma();
    }
    // if(PressureSensor::pressure_sensor.ps_state == PS_HANDLE_STATE::CALCULATE){
    //     for(int i=0; i<3;i++){
    //         send_float(pro_data.gyro[i], 2, 0);
    //     }
    // }
}

void IMU::Receive()
{
    if (strncmp((char *)RxBuffer, "BEG", 3) == 0)
    {
        flag_Test = true;
    }
    if (strncmp((char *)RxBuffer, "END", 3) == 0)
    {
        flag_Test = false;
    }
    if (strncmp((char *)RxBuffer, "IVA:BEG", 7) == 0)
    {
        is_output_angle = 1;
    }
    if (strncmp((char *)RxBuffer, "IVA:END", 7) == 0)
    {
        is_output_angle = 0;
    }

    // if (strncmp((char *)RxBuffer, "AVA:BEG", 7) == 0)
    // {
    //     is_output_accl = 1;
    // }
    // if (strncmp((char *)RxBuffer, "AVA:BEG", 7) == 0)
    // {
    //     is_output_accl = 0;
    // }
    
}

void IMU::trigger_gyro_dma(void)
{
    /* 三个 trigger_*_dma() 函数内部的执行流程步骤（结构相同）：
     * 步骤 1：检查 hspi1.hdmatx 与 hdmarx 的 SxCR.EN 位，
     *         两条 DMA 流都空闲才继续，避免打断上一帧。
     * 步骤 2：拉低对应器件 CS：陀螺用 CS1_GYRO，加速度/温度用 CS1_ACCEL。
     * 步骤 3：以本帧的 TX/RX 缓冲区地址与帧长（GYRO 8 / ACCEL 9 / TEMP 4）
     *         调用 SPI1_DMA_enable() 启动 DMA 全双工传输。
     * 步骤 4：返回；传输完成后由 DMA2_Stream0 中断进入 DMA_IT_Handle()。
     */
    if(!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    }
}

void IMU::trigger_accel_dma(void)
{
    if(!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_dma_tx_buf, (uint32_t)buf.accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    }
}

void IMU::trigger_temp_dma(void)
{
    if(!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_temp_dma_tx_buf, (uint32_t)buf.accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    }
}

/**
 * @brief SPI RX DMA 传输完成处理：切换片选、解析数据并推进 DMA 状态机。
 * @details 仅在 hspi1.hdmarx 的 TC 标志置位时执行，先清标志再按 dma_state：
 *          - READ_GYRO  ：释放 GYRO CS，解析陀螺 -> 触发加速度帧；
 *          - READ_ACCEL ：释放 ACCEL CS，解析加速度/时间 -> 触发温度帧；
 *          - READ_TEMP  ：释放 ACCEL CS，解析温度 -> 恒温控制、磁力计读取、
 *            零偏补偿、Mahony 九轴更新、欧拉角转换、姿态刷新与调试输出，
 *            最后把状态机置回 IDLE，等待 Handle() 发起下一轮。
 * @note  该函数由 DMA2_Stream0_IRQHandler 在中断上下文调用，drv 层解析函数
 *        只做位拼接与乘加；调试输出仅在水压计状态为 CALCULATE 时进行。
 */
void IMU::DMA_IT_Handle()
{
    /* 函数内部流程步骤：
     * 步骤 1：先检查 RX DMA 的 TC 标志，未置位则直接返回（非本中断源）。
     * 步骤 2：用 __HAL_DMA_CLEAR_FLAG 清除 TC 标志，避免重复进入。
     * 步骤 3：按 dma_state 分支：
     *         READ_GYRO  -> 拉高 GYRO CS，BMI088_gyro_read_over() 解析角速度，
     *                       切到 READ_ACCEL 并 trigger_accel_dma()；
     *         READ_ACCEL -> 拉高 ACCEL CS，BMI088_accel_read_over() 解析
     *                       加速度与时间戳，切到 READ_TEMP 并 trigger_temp_dma()；
     *         READ_TEMP  -> 拉高 ACCEL CS，解析温度 -> imu_temp_control()
     *                       恒温 PID -> ist8310_read_mag() 读磁场 ->
     *                       offset_correction() 扣零偏/磁力计校准 ->
     *                       MahonyAHRSupdate() 更新四元数 -> get_angle()
     *                       转欧拉角 -> attitude_update() 刷新姿态结构。
     * 步骤 4：最后一帧中若水压计处于 CALCULATE 状态，用 send_float()
     *         输出 4 路压力调试数据（3 个逗号分隔 + 1 个换行）。
     * 步骤 5：把 dma_state 置回 IMU_DMA_IDLE，等待 Handle() 发起下一轮采集。
     */
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        switch (dma_state)
        {
        case IMU_DMA_IDLE:
            break;
        case IMU_DMA_READ_GYRO:
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, raw_data.gyro);
            dma_state = IMU_DMA_READ_ACCEL;
            trigger_accel_dma();
            break;
        case IMU_DMA_READ_ACCEL:
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
            BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, raw_data.accel, &raw_data.time);
            dma_state = IMU_DMA_READ_TEMP;
            trigger_temp_dma();
            break;
        case IMU_DMA_READ_TEMP:
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
            BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &raw_data.temp);
            imu_temp_control(raw_data.temp);
            ist8310_read_mag(raw_data.mag);
            offset_correction();
            MahonyAHRSupdate(pro_data.ins_quat, pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2], pro_data.accel[0], pro_data.accel[1], pro_data.accel[2], pro_data.mag[0], pro_data.mag[1], pro_data.mag[2]);    // Compute the quaternion from the acquired accelerometer, gyroscope, and magnetometer measurements, and update it in-place in the `quat` array. //对得到的加速度计、陀螺仪和磁力计数据进行计算得到四元数，直接在数组quat中更新
            // MahonyAHRSupdateIMU(pro_data.ins_quat, pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2], pro_data.accel[0], pro_data.accel[1], pro_data.accel[2]);   //Compute the quaternion from the acquired accelerometer and gyroscope measurements, and update it in-place in the `quat` array. //对得到的加速度计、陀螺仪数据进行计算得到四元数，直接在数组quat中更新
            get_angle(pro_data.ins_quat, pro_data.ins_angle, pro_data.ins_angle+1, pro_data.ins_angle+2);    // Compute Euler angles (in radians) from the quaternion, and update them in-place in the `INS_angle` array. //对四元数计算得到弧度制欧拉角，直接在INS_angle数组中更新
            attitude_update();
            
						// Output test
						// 测试输出
            if (PressureSensor::pressure_sensor.ps_state == PS_HANDLE_STATE::CALCULATE)
            {
            // send_float(pro_data.ins_angle[0]*180/PI, 2, 0);
            // send_float(pro_data.ins_angle[1]*180/PI, 2, 0);
            // send_float(pro_data.ins_angle[2]*180/PI, 2, 1);
            // send_float(pro_data.accel[0], 2, 0);
            // send_float(pro_data.accel[1], 2, 0);
            // send_float(pro_data.accel[2], 2, 1);
							for(int i=0; i<3;i++)
							 {
									 send_float(PressureSensor::pressure_sensor.data_pressure[i], 2, 0);
							 }
							 send_float(PressureSensor::pressure_sensor.data_pressure[3], 2, 1);
            }
            dma_state = IMU_DMA_IDLE;
            break;
        }
    }
}

// DMA transfer-complete interrupt.
// DMA传输完成中断
void DMA2_Stream0_IRQHandler(void)
{
    IMU::imu.DMA_IT_Handle();
}

void IMU::ErrorHandle()
{
}

/**
 * @brief 由四元数 q={w,x,y,z} 计算 ZYX 顺序欧拉角（弧度）。
 * @param q     输入四元数（应已归一化）。
 * @param yaw   [输出] 航向角，atan2 形式，范围 (-π, π]。
 * @param pitch [输出] 俯仰角，asin 形式，范围 [-π/2, π/2]。
 * @param roll  [输出] 横滚角，atan2 形式。
 * @note  在俯仰接近 ±90°（万向锁）时 yaw/roll 会退化；姿态解算结果经
 *        attitude_update() 写入 attitude 结构供控制器读取。
 */
void IMU::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

void IMU::imu_temp_control(float temp)
{
    /* 函数内部流程步骤：
     * 步骤 1：声明 tempPWM 与 static 达温稳定计数 temp_constant_time。
     * 步骤 2：若 first_temperate 已置位（进入 PID 阶段）：调用
     *         tempPid.PIDCalc(IMU_TARGET_TEMP, temp)，把负输出钳为 0，
     *         转成 uint16_t 后由 IMU_temp_PWM() 写 CCR。
     * 步骤 3：否则仍处于快速升温阶段：temp > 目标时累加稳定计数，
     *         超过 200 拍后置 first_temperate=1、把积分分量限幅设为
     *         最大功率的一半以加快收敛。
     * 步骤 4：本阶段无论温度如何都输出 MPU6500_TEMP_PWM_MAX - 1（全功率加热）。
     */
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        tempPid.PIDCalc(IMU_TARGET_TEMP, temp);
        if (tempPid.PIDInfo.output < 0.0f)
        {
            tempPid.PIDInfo.output = 0.0f;
        }
        tempPWM = (uint16_t)tempPid.PIDInfo.output;

        IMU_temp_PWM(tempPWM);

    }
    else
    {
				// in beginning, max power
        // 在没有达到设置的温度，一直最大功率加热
        if (temp > IMU_TARGET_TEMP)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
								// Once the target temperature is reached, set the integral term to half of the maximum power to accelerate convergence.
                // 达到设置温度，将积分项设置为一半最大功率，加速收敛
                first_temperate = 1;
                tempPid.PIDInfo.componentKiMax = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}

void IMU::IMU_temp_PWM(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief Measure the gyroscope and accelerometer biases and store them in the `raw_data.accel_offset` and `raw_data.gyro_offset` arrays. 测得陀螺仪和加速度计零偏值，存入raw_data.accel_offset和raw_data.gyro_offset数组
 * @param accel
 * @param _accel
 */
void IMU::calibrate_offset()
{
    float accel[3], gyro[3], temp;
    for (int i = 0; i < 1000; i++)
    {
        BMI088_read(gyro, accel, &temp);
        raw_data.accel_offset[0] += accel[0];
        raw_data.accel_offset[1] += accel[1];
        raw_data.accel_offset[2] += accel[2];
        raw_data.gyro_offset[0] += gyro[0];
        raw_data.gyro_offset[1] += gyro[1];
        raw_data.gyro_offset[2] += gyro[2];
        HAL_Delay(1);
    }
    raw_data.accel_offset[0] /= 1000;
    raw_data.accel_offset[1] /= 1000;
    raw_data.accel_offset[2] /= 1000;
    raw_data.gyro_offset[0] /= 1000;
    raw_data.gyro_offset[1] /= 1000;
    raw_data.gyro_offset[2] /= 1000;
}

void IMU::offset_correction()
{
    for (int i=0; i<3; i++)
    {
        pro_data.gyro[i] = raw_data.gyro[i] - raw_data.gyro_offset[i];
        pro_data.accel[i] = raw_data.accel[i] - raw_data.accel_offset[i];
    }
    pro_data.mag[0] = (raw_data.mag[0] - MAG_OFFSET_X) * MAG_SCALE_X;
    pro_data.mag[1] = (raw_data.mag[1] - MAG_OFFSET_Y) * MAG_SCALE_Y;
    pro_data.mag[2] = (raw_data.mag[2] - MAG_OFFSET_Z) * MAG_SCALE_Z;
}

void IMU::attitude_update()
{
		// `ins_angle[0]`, `ins_angle[1]`, and `ins_angle[2]` correspond to yaw, pitch, and roll, respectively.
    // ins_angle 012对应yaw，pitch，roll
    attitude.yaw = pro_data.ins_angle[0];
    attitude.pitch = pro_data.ins_angle[1];
    attitude.roll = pro_data.ins_angle[2];
    attitude.yaw_v = pro_data.gyro[2];
    attitude.pitch_v = pro_data.gyro[1];
    attitude.roll_v = pro_data.gyro[0];
    attitude.neg_yaw_v = - attitude.yaw_v;
    attitude.neg_pitch_v = - attitude.pitch_v;
    attitude.neg_roll_v = - attitude.roll_v;
}