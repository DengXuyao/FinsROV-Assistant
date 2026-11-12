/**
 * @file    bsp_uart.h
 * @brief   BSP 层串口 DMA 双缓冲 + 空闲中断接收框架的接口声明。
 * @details 所属模块：userCode/bsp。针对不定长串口帧（如遥控器/上位机指令）
 *          设计：DMA 在两块接收缓冲区之间循环切换（DBM），串口总线空闲
 *          （IDLE）时触发中断，在中断里计算本帧长度、切换目标缓冲区并调用
 *          用户解码回调。实现位于 Src/bsp_uart.c（C 语言），因此用
 *          extern "C" 包裹以便被上层 C++ 代码（devices 层）直接调用。
 * @note    使用流程：填充一个 bsp_uart_recevieX_handle 结构体 ->
 *          调用 uart_recieveX_init() 启动 DMA -> 在 USARTx_IRQHandler 中
 *          调用 uart_recieveX_irq() -> 在 dataDecode 回调里读取 curBuf/curNum。
 * @warning 实现中外设地址被写死为 USART3->DR，故当前只适用于 USART3；
 *          换用其他串口需修改 Src/bsp_uart.c 中的 PAR 赋值与 IRQ 入口。
 */

#ifndef __BSP_UART_H
#define __BSP_UART_H
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>


/**
 * @struct bsp_uart_recevieX_handle
 * @brief  一套串口接收通道的全部配置与运行状态（名字中的 X 表示可实例化多路）。
 * @note   结构体由调用方分配并初始化，init() 只读取配置，irq() 会更新
 *         curBuf / curNum 并触发回调；因此对象须为全局或静态存储期。
 */
struct bsp_uart_recevieX_handle{
	UART_HandleTypeDef *huart;          // 指向 CubeMX 生成的串口句柄（当前实现按 USART3 使用）
	DMA_HandleTypeDef *hdma_uart_rx;    // 指向该串口接收 DMA 流句柄
	uint8_t *buf1;                      // 双缓冲中的 Memory 0 缓冲区首地址
	uint8_t *buf2;                      // 双缓冲中的 Memory 1 缓冲区首地址
	uint32_t maxNum;                    // 单块缓冲区容量，同时作为 DMA 的 NDTR 初值
	uint8_t *curBuf;                    // [输出] 本次空闲中断锁定的已完成数据缓冲区
	uint32_t curNum;                    // [输出] 本次收到的有效字节数 = maxNum - NDTR
	void  (*dataDecode)(void);          // 一帧接收完成时调用的用户解码回调函数（无参数，从本结构体取数据）
};
/* C++ 调用 C 实现时禁止名字改编，保证链接符号与 .c 文件一致 */
#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief 初始化串口接收：使能 DMA 请求与空闲中断，配置双缓冲循环 DMA。
 * @param huartReceive 已填好句柄、DMA、两块缓冲区与回调的接收通道结构体。
 * @note  依次设置 USART_CR3 的 DMAR、使能 RXNE/IDLE 中断、关闭并等待 DMA
 *        停止、设定 PAR 为 USART3->DR、M0AR/M1AR 为两块缓冲区、NDTR 为
 *        maxNum，再置位 DBM（双缓冲）与 CIRC（循环）后使能 DMA。
 * @note  RXNE 中断仅用于清除标志/维持框架运行，真正的帧结束判定依赖 IDLE；
 *        必须在 CubeMX 完成串口与 DMA 初始化之后调用。
 */
/* bsp_uart.c 中两个函数内部的执行流程步骤：
 * uart_recieveX_init()：1) 取句柄；2) 置 CR3.DMAR；3) 使能 RXNE/IDLE 中断；
 *                      4) 关 DMA 并等 EN 清零；5) 设 PAR/M0AR/M1AR/NDTR；
 *                      6) 置 DBM+CIRC；7) 使能 DMA。
 * uart_recieveX_irq() ：1) 读 SR/CR1/CR3；2) 仅 RXNE 时清标志返回；
 *                      3) IDLE 时按 CT 位判断完成缓冲区，暂停 DMA、
 *                         用 maxNum-NDTR 算帧长、复位 NDTR、翻转 CT 位、
 *                         重启 DMA；4) 设 curBuf 并调用 dataDecode()；
 *                      5) 其它情况交给 HAL_UART_IRQHandler()。
 */
extern void uart_recieveX_init(struct bsp_uart_recevieX_handle *huartReceive);

/**
 * @brief 串口中断服务例程中调用的分帧处理函数。
 * @param huartReceive 与 init() 相同的接收通道结构体。
 * @note  读取 SR/CR1/CR3 后分三种情况：仅 RXNE 时清除奇偶/帧错误标志；
 *        IDLE（一帧结束）时根据 DMA_SxCR_CT 判断当前目标缓冲区，暂停 DMA、
 *        用 maxNum - NDTR 计算帧长 curNum、复位 NDTR、手工切换 CT 位、
 *        重新使能 DMA，然后设置 curBuf 并调用 dataDecode()；其余情况交给
 *        HAL_UART_IRQHandler() 处理。
 * @warning 回调在中断上下文执行，应尽快完成；且必须在下一帧到来前处理完
 *          curBuf 中的数据，否则会被后续 DMA 写入覆盖。
 */
extern void uart_recieveX_irq(struct bsp_uart_recevieX_handle *huartReceive);

#ifdef __cplusplus
};
#endif
#endif /*__BSP_UART_H*/
