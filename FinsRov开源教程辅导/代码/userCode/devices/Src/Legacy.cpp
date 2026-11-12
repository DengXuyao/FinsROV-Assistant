//
// Created by admin on 2023/11/28.
//

/**
 * @file    Legacy.cpp
 * @brief   遗留兼容层实现：电源电压 ADC 监测、调试打印、片内 Flash 参数存取。
 * @details 所属模块：userCode/devices，配套声明见 Inc/Legacy.h。
 *          bsp_ADC_vccMoni() 上电时先对内部参考电压通道（ADC1）采样 200 次
 *          求平均作为标定基准，之后每次调用由 ADC3 的读数与 1.25 V 参考电压
 *          换算出监测电压，再按电阻分压比 22/222 推算电池电压；
 *          bsp_flash_write/read() 以扇区 9 为参数区，按 32 位字读写
 *          flash_data_t（当前为 4 个水压计零偏）。
 * @note    usart_printf() 目前格式化与发送语句都被注释掉（只保留 va_start/
 *          va_end 框架），相当于空实现，需要调试输出时自行恢复并注意缓冲区长度。
 * @note    Flash 写操作会擦除整个扇区 9，写入期间不可执行 Flash 中的代码，
 *          且结构体大小应为 4 字节的整数倍（循环按 uint32_t* 步进）。
 */
#include "Legacy.h"


volatile float vccMoni = 0;   // 由 ADC3 换算得到的监测点电压（V）
volatile float vccBat = 0;    // 按分压比推算的电池电压（V）

/**
 * @brief 周期执行电源电压监测；首次调用前 200 次用于标定参考电压。
 * @details calibrationCplt == 0 时，每次调用从 ADC1 读一次内部参考电压并
 *          累加，累计 200 次后取平均得到 vrefValue，随后停止 ADC1 并转到
 *          ADC3 测量；之后每次调用直接以保存的 vrefValue 换算：
 *              vccMoni = ADC3_raw * 1.25 / vrefValue
 *              vccBat  = vccMoni / 22 * 222
 *          其中 1.25 V 为内部参考电压标称值，22/222 为分压电阻比例。
 * @note  使用 HAL_ADC_PollForConversion() 阻塞轮询（超时 2 ms），
 *        应在主循环而非高优先级中断中调用；calibration 相关变量为 static，
 *        标定只在上电后执行一次。
 */
void bsp_ADC_vccMoni(){
    /* 函数内部流程步骤：
     * 步骤 1：声明 static 的标定完成标志、计数与参考电压累加和/均值。
     * 步骤 2：calibrationCplt == 0 时进入标定：前 200 次调用每次启动 ADC1、
     *         轮询转换并读取内部参考电压值累加到 vrefSum。
     * 步骤 3：第 200 次后取 vrefValue = vrefSum/200，停止 ADC1，
     *         并立即启动 ADC3 读取一次监测电压。
     * 步骤 4：换算 vccMoni = ADC3原始值 * 1.25 / vrefValue（1.25V 为参考标称值）。
     * 步骤 5：标定完成后每次调用只重复步骤 4 的 ADC3 采样与换算。
     * 步骤 6：最后按分压比 vccBat = vccMoni / 22 * 222 得到电池电压。
     */
    static uint8_t calibrationCplt = 0;
    static uint32_t calibrationCnt = 0;
    static uint32_t vrefSum = 0,vrefValue = 0;
    if(calibrationCplt == 0){
        if (calibrationCnt == 0){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else if(calibrationCnt < 200){
            calibrationCnt ++;
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            vrefValue = HAL_ADC_GetValue(&hadc1);
            vrefSum += vrefValue;
        }else{
            calibrationCplt = 1;
            vrefValue = vrefSum/200.0f;
            HAL_ADC_Stop(&hadc1);

            HAL_ADC_Start(&hadc3);
            HAL_ADC_PollForConversion(&hadc3, 2);
            vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

        }

    }else{
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 2);
        vccMoni = HAL_ADC_GetValue(&hadc3)*1.25f/(float)vrefValue;

    }
    vccBat = vccMoni/22.0f*222.0f;
}

/**
 * @brief // Using the UART redirection function, either a USB virtual COM port or a hardware UART can be selected; the corresponding driver must be installed for operation. //利用串口重定向函数，可选usb虚拟串口或硬件串口，使用需安装相应驱动
 * @param fmt
 * @param ...
 */
void usart_printf(const char *fmt,...){
    /* 函数内部流程步骤（当前为空实现）：
     * 步骤 1：准备静态 256 字节 tx_buf、va_list 与长度变量 len。
     * 步骤 2：va_start(ap, fmt) 取可变参数列表。
     * 步骤 3：原本应调用 vsnprintf 格式化（该行被注释），
     *         再经 HAL_UART_Transmit_DMA 或 CDC_Transmit_FS 发送（同样被注释）。
     * 步骤 4：va_end(ap) 收尾并返回；恢复发送前不会产生任何输出。
     */

        static uint8_t tx_buf[256] = {0};//TODO 爆栈？
        static va_list ap;
        static uint16_t len;

        va_start(ap,fmt);

        /*len = vsprintf((char*)tx_buf,fmt,ap);*/

        va_end(ap);

        //HAL_UART_Transmit_DMA(&huart1,tx_buf,len);
//    CDC_Transmit_FS(tx_buf,len);

    }

    /* 全局 Flash 参数镜像：上电读取、标定后写回 */
    flash_data_t flashData;
    /**
     * @brief 把 flash_data_t 写入 STM32F407 的 Flash 扇区 9。
     * @param _flashData 待写入结构体指针。
     * @note  流程：解锁 Flash -> 配置按扇区擦除（Sector=FLASH_SECTOR_9、
     *        1 个扇区、电压范围 3）-> 擦除 -> 从 FLASH_SECTOR_9_ADDRESS 起
     *        按 uint32_t 字循环写入 -> 上锁。擦除会清掉该扇区全部原有内容。
     * @warning 未检查 HAL 返回值，也未处理写入失败；结构体大小需为 4 的倍数。
     */
    void bsp_flash_write(flash_data_t *_flashData){
    /* 函数内部流程步骤：
     * 步骤 1：HAL_FLASH_Unlock() 解锁 Flash 写保护。
     * 步骤 2：填 FLASH_EraseInitTypeDef：按扇区擦除、扇区 9、1 个扇区、
     *         电压范围 3，调用 HAL_FLASHEx_Erase() 擦除（忽略 SectorError）。
     * 步骤 3：把 flash_ptr 指向 FLASH_SECTOR_9_ADDRESS。
     * 步骤 4：用 uint32_t* 指针遍历 _flashData，逐字 HAL_FLASH_Program()
     *         写入并把 flash_ptr 加 4，直到覆盖 sizeof(flash_data_t)/4 个字。
     * 步骤 5：HAL_FLASH_Lock() 重新上锁，防止误写。
     */
        HAL_FLASH_Unlock();
        FLASH_EraseInitTypeDef eraseInit;
        eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseInit.Sector = FLASH_SECTOR_9;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASHEx_Erase(&eraseInit,nullptr);
    uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;

    for (uint32_t *ptr = (uint32_t*)_flashData;ptr < (uint32_t*)(sizeof(flash_data_t)/4 + _flashData);ptr++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,flash_ptr,*ptr);
        flash_ptr += 4;
    }
    HAL_FLASH_Lock();
}
/**
 * @brief 从 Flash 扇区 9 读取参数到结构体。
 * @param _flashData 接收数据的结构体指针。
 * @note  直接以 memcpy 从映射地址拷贝 sizeof(flash_data_t)/4 个单位
 *        （注意第三个参数为字数而非字节数，此处保留了原实现的写法）；
 *        flash_ptr 局部变量声明后未使用。
 */
void bsp_flash_read(flash_data_t *_flashData){
    /* 函数内部流程步骤：
     * 步骤 1：把 flash_ptr 指向 FLASH_SECTOR_9_ADDRESS（该局部变量后续未使用）。
     * 步骤 2：memcpy 从该映射地址拷贝 sizeof(flash_data_t)/4 个单位到
     *         _flashData（注意第三个参数是字数而非字节数，保留原实现写法）。
     * 步骤 3：返回；若 Flash 从未写入，读到的内容通常是全 0xFF，
     *         由调用方判断参数有效性。
     */
    uint32_t flash_ptr = FLASH_SECTOR_9_ADDRESS;
    memcpy(_flashData,(uint32_t*)FLASH_SECTOR_9_ADDRESS,sizeof(flash_data_t)/4);
}

