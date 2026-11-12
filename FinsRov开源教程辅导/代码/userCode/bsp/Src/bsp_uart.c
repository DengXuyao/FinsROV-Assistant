/**
 * @file    bsp_uart.c
 * @brief   串口 DMA 双缓冲 + IDLE 空闲中断接收框架实现（C 语言）。
 * @details 所属模块：userCode/bsp，接口见 Inc/bsp_uart.h。DMA 工作在双缓冲
 *          循环模式：一个缓冲区接收数据的同时，CPU 可以处理另一个已完成
 *          缓冲区，从而避免高速不定长帧的拷贝与丢帧。串口总线空闲即视为
 *          一帧结束，中断里根据 NDTR 剩余计数还原帧长并调用解码回调。
 * @note    当前实现把 DMA 外设地址写死为 &USART3->DR，仅适用于 USART3；
 *          中断入口需在 stm32f4xx_it.c 的 USART3_IRQHandler 中调用
 *          uart_recieveX_irq()。
 * @warning 初始化后 DMA 持续运行，回调在中断上下文被调用，解码逻辑必须短小；
 *          单帧长度超过 maxNum 时计数会翻转，需由上层保证帧长不超缓冲区。
 */
#include "bsp_uart.h"


/**
 * @brief 启动串口双缓冲 DMA 接收，配置寄存器并使能相关中断。
 * @param huartReceive 接收通道配置结构体（串口句柄、DMA 句柄、两块缓冲区、
 *                     容量 maxNum、解码回调）。
 * @note  关键点：
 *        - USART_CR3.DMAR 允许接收事件直接产生 DMA 请求，无需 CPU 逐字节搬运；
 *        - 同时使能 RXNE 与 IDLE 中断，IDLE 用于识别帧结束，RXNE 分支只清标志；
 *        - 修改 DMA 前关闭流并轮询 SxCR.EN 清零；
 *        - M0AR/M1AR 分别指向 buf1/buf2，NDTR=maxNum；
 *        - DMA_SxCR_DBM 开启双缓冲，DMA_SxCR_CIRC 开启循环（源码保留了
 *          对这一点与手册描述不一致的 TODO 注释）。
 * @note  函数不校验指针与 maxNum，调用方须保证缓冲区有效且足够长。
 */
void uart_recieveX_init(struct bsp_uart_recevieX_handle* huartReceive) {
    /* 函数内部流程步骤：
     * 步骤 1：从配置结构体取出 huart 与 hdma_uart_rx 局部指针，简化后续访问。
     * 步骤 2：SET_BIT(huart->CR3, USART_CR3_DMAR) 允许串口接收事件触发 DMA。
     * 步骤 3：使能 RXNE 与 IDLE 两个中断：RXNE 用于清标志，IDLE 判定帧结束。
     * 步骤 4：关闭 DMA 流并 while 轮询 SxCR.EN 直到真正停止。
     * 步骤 5：设 PAR=&USART3->DR（外设地址写死为 USART3）、M0AR=buf1、
     *         M1AR=buf2、NDTR=maxNum（单缓冲区容量）。
     * 步骤 6：置位 DMA_SxCR_DBM 开启双缓冲，再置位 CIRC 开启循环模式。
     * 步骤 7：__HAL_DMA_ENABLE() 启动 DMA，此后接收无需 CPU 干预，
     *         每帧结束由 uart_recieveX_irq() 处理。
     */
	UART_HandleTypeDef *huart = huartReceive->huart;
	DMA_HandleTypeDef *hdma_uart_rx = huartReceive->hdma_uart_rx;
    //enable the DMA transfer for the receiver request
    //使能 DMA 串口接收
	
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    //disable DMA
    //失效 DMA
    __HAL_DMA_DISABLE(hdma_uart_rx);
    while(hdma_uart_rx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma_uart_rx);
    }
    hdma_uart_rx->Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区 1
    hdma_uart_rx->Instance->M0AR = (uint32_t)(huartReceive->buf1);
    //memory buffer 2
    //内存缓冲区 2
    hdma_uart_rx->Instance->M1AR = (uint32_t)(huartReceive->buf2);
    //data length
    //数据长度
    hdma_uart_rx->Instance->NDTR = huartReceive->maxNum;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_uart_rx->Instance->CR, DMA_SxCR_DBM);
    SET_BIT(hdma_uart_rx->Instance->CR, DMA_SxCR_CIRC);//TODO 和手册里写的不一样，完全不懂
    //enable DMA
    //使能 DMA
    __HAL_DMA_ENABLE(hdma_uart_rx);
}



/**
 * @brief 串口中断入口：识别帧结束、切换双缓冲并触发解码回调。
 * @param huartReceive 由 init() 初始化的同一接收通道结构体。
 * @details 读取 SR/CR1/CR3 后分三种分支：
 *          1) 仅 RXNE 且无 IDLE：说明还在收帧中，只清除 PE 标志；
 *          2) IDLE 置位：说明总线空闲、一帧结束。此时检查 DMA_SxCR_CT
 *             判断刚才是哪块缓冲区在接收——CT=RESET 表示当前目标为 Memory 0，
 *             已完成的帧在 buf1；否则完成的帧在 buf2。随后暂停 DMA、
 *             以 maxNum - NDTR 计算 curNum、把 NDTR 复位为 maxNum、
 *             手工翻转 CT 位让 DMA 切换到另一块缓冲区、重新使能 DMA，
 *             最后设置 curBuf 并调用 dataDecode()；
 *          3) 其它情况交给 HAL_UART_IRQHandler() 处理错误/其它中断源。
 * @note  清 IDLE 标志靠 __HAL_UART_CLEAR_PEFLAG()（读 SR 再读 DR 的序列）；
 *        手工改 CT 位前必须关闭 DMA 且等 EN 清零，避免硬件自动切换冲突。
 * @warning 回调运行在中断上下文，且 curBuf 指向的缓冲区马上会被后续帧复用，
 *          解码逻辑必须短且不能做阻塞操作。
 */
void uart_recieveX_irq(struct bsp_uart_recevieX_handle* huartReceive) {
    /* 函数内部流程步骤：
     * 步骤 1：取 huart / hdma_uart_rx 局部指针，读取 SR、CR1、CR3 快照。
     * 步骤 2：仅有 RXNE 且无 IDLE：说明正在收帧，只清 PE 等标志后返回。
     * 步骤 3：IDLE 置位表示一帧结束，先 __HAL_UART_CLEAR_PEFLAG 清标志，
     *         再用 DMA_SxCR_CT 判断当前目标缓冲区：CT=RESET 表示 Memory 0
     *         （完成的数据在 buf1），否则为 Memory 1（数据在 buf2）。
     * 步骤 4：关闭 DMA，用 curNum = maxNum - NDTR 还原本帧字节数，
     *         并把 NDTR 重置回 maxNum。
     * 步骤 5：手工写 CT 位（|= 或 &= ~）把 DMA 切换到另一块缓冲区，
     *         再重新使能 DMA，保证下一帧写入空闲缓冲区。
     * 步骤 6：设置 curBuf 指向刚完成的缓冲区并调用 dataDecode() 解码。
     * 步骤 7：其它情况（错误/非预期中断）交给 HAL_UART_IRQHandler() 处理。
     */
	UART_HandleTypeDef *huart = huartReceive->huart;
	DMA_HandleTypeDef *hdma_uart_rx = huartReceive->hdma_uart_rx;

    uint32_t isrflags   = READ_REG(huart->Instance->SR);
    uint32_t cr1its     = READ_REG(huart->Instance->CR1);
    uint32_t cr3its     = READ_REG(huart->Instance->CR3);

    /* 状态分支判定：RXNE 表示仍在收字节，IDLE 表示总线空闲即一帧结束。
     * cr1its/cr3its 读取是为了与 HAL 中断处理流程保持一致（当前未进一步使用）。 */
    if ((isrflags & UART_FLAG_RXNE) && !(isrflags & UART_FLAG_IDLE))//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(huart);
    } else if (isrflags & UART_FLAG_IDLE) {

        __HAL_UART_CLEAR_PEFLAG(huart);

        if ((hdma_uart_rx->Instance->CR & DMA_SxCR_CT) == RESET) {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(hdma_uart_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            huartReceive->curNum = huartReceive->maxNum - hdma_uart_rx->Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_uart_rx->Instance->NDTR = huartReceive->maxNum;

            //set memory buffer 1
            //设定缓冲区1
            hdma_uart_rx->Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(hdma_uart_rx);
			
			huartReceive->curBuf = huartReceive->buf1;
			huartReceive->dataDecode();
			
        } else {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(hdma_uart_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            huartReceive->curNum = huartReceive->maxNum - hdma_uart_rx->Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_uart_rx->Instance->NDTR = huartReceive->maxNum;

            //set memory buffer 0
            //设定缓冲区0
            hdma_uart_rx->Instance->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(hdma_uart_rx);

			huartReceive->curBuf = huartReceive->buf2;
			huartReceive->dataDecode();
        }
    }else{
        HAL_UART_IRQHandler(huart);
    }
}