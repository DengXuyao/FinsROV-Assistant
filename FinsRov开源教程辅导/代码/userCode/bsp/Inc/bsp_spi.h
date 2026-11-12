/**
 * @file    bsp_spi.h
 * @brief   BSP 层 SPI1 双 DMA 全双工收发的对外接口声明。
 * @details 所属模块：userCode/bsp（板级支持包）。本头文件只声明由 CubeMX
 *          生成的 3 个 extern 句柄以及 SPI1_DMA_init()、SPI1_DMA_enable()
 *          两个入口，实现位于 Src/bsp_spi.cpp。上层驱动（BMI088 等）通过
 *          这两个函数完成 SPI1 与外设之间一帧定长数据的同步收发。
 *          实现绕过 HAL_SPI_TransmitReceive_DMA()，直接配置 SPI1->CR2 与
 *          DMA 流的 PAR/M0AR/NDTR/CR 寄存器，属于寄存器级轻量封装。
 * @note    hspi1、hdma_spi1_rx、hdma_spi1_tx 由 CubeMX 在 spi.c / dma.c 中
 *          定义，调用本模块接口前必须已完成 MX_SPI1_Init() 与 MX_DMA_Init()，
 *          且 hspi1.hdmarx / hspi1.hdmatx 已关联到对应的 DMA 流。
 * @note    tx_buf / rx_buf 以 uint32_t 数值形式传入内存地址（调用方需强转），
 *          缓冲区必须位于 DMA 可访问的 SRAM 且在传输全程有效；TX 与 RX 长度
 *          必须相等，否则全双工时钟与数据无法对齐。
 * @note    SPI1 的 NSS/CS 片选不由本模块操作，由 CubeMX 硬件 NSS 或上层
 *          GPIO 控制；一帧传输前后是否拉低 CS 由上层负责。
 * @warning 本头文件没有 extern "C" 包裹，而实现为 .cpp，因此只适合被 C++
 *          编译单元包含；C 文件需调用时应在调用侧自行声明 C 链接符号。
 */
#ifndef BSP_SPI_H
#define BSP_SPI_H
/* 经 main.h 引入 HAL 的 SPI/DMA 句柄类型、SPI1 寄存器映射与 SET_BIT 等位操作宏 */
#include "main.h"


/* CubeMX 生成的 SPI1 句柄：提供 Instance 指向 SPI1 寄存器组及 hdmarx/hdmatx */
extern SPI_HandleTypeDef hspi1;
/* CubeMX 生成的 SPI1 接收 DMA 流句柄（实现中直接访问其 Instance 寄存器） */
extern DMA_HandleTypeDef hdma_spi1_rx;
/* CubeMX 生成的 SPI1 发送 DMA 流句柄 */
extern DMA_HandleTypeDef hdma_spi1_tx;


/**
 * @brief 初始化 SPI1 的 DMA 收发链路（只配置一次）。
 * @param tx_buf 发送缓冲区首地址（以 uint32_t 传入）。
 * @param rx_buf 接收缓冲区首地址（以 uint32_t 传入）。
 * @param num    一帧收发的数据项数，同时写入 RX/TX 流的 NDTR。
 * @note  依次使能 SPI_CR2 的 TXDMAEN/RXDMAEN、使能 SPI 外设，然后关闭两条
 *        DMA 流并等待 EN 位清零，清除传输完成标志，设置外设地址为 &SPI1->DR、
 *        存储器地址与计数器；最后为 RX 流使能传输完成中断（DMA_IT_TC）。
 * @note  本函数不会启动传输，启动需再调用 SPI1_DMA_enable()。
 */
/* bsp_spi.cpp 中两个函数内部的执行流程步骤：
 * SPI1_DMA_init()  ：1) 置 CR2 的 TXDMAEN/RXDMAEN；2) 使能 SPI；
 *                    3) 关闭 RX 流并等 EN 清零、清 TCIF2、设 PAR/M0AR/NDTR、
 *                       使能 TC 中断；4) 对 TX 流重复第 3 步（不使能中断）。
 * SPI1_DMA_enable()：1) 关闭两条流并等 EN 清零；2) 清 TC/HT/TE/DME/FE 标志；
 *                    3) 重设 M0AR 与 NDTR；4) 先使能 RX、再使能 TX 启动传输。
 * 中断侧：RX 传输完成中断触发上层解析，本模块不处理数据。
 */
extern void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);

/**
 * @brief 重新启动一帧 SPI1 DMA 全双工传输。
 * @param tx_buf 本次发送缓冲区首地址，可覆盖 init 时的地址。
 * @param rx_buf 本次接收缓冲区首地址，可覆盖 init 时的地址。
 * @param ndtr   本次收发数据项数，同时写入两条 DMA 流的计数器。
 * @note  先关闭并确认两条流停止，清除 hspi1.hdmarx / hdmatx 的 TC、HT、TE、
 *        DME、FE 五类标志，再重设 M0AR 与 NDTR，最后同时使能 RX 与 TX 流；
 *        RX 流先使能可避免首字节丢失。
 * @note  传输完成由 init() 中使能的 RX DMA 完成中断通知上层；调用方须保证
 *        上一帧已结束，否则重配寄存器会打断正在进行的传输。
 */
extern void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif
