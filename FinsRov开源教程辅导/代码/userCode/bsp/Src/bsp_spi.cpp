/**
 * @file    bsp_spi.cpp
 * @brief   SPI1 全双工 DMA 收发的寄存器级实现。
 * @details 所属模块：userCode/bsp，接口见 Inc/bsp_spi.h。不调用
 *          HAL_SPI_TransmitReceive_DMA()，而是直接操作 SPI1->CR2 以及 DMA2
 *          各流的 PAR / M0AR / NDTR / CR 寄存器，以便用最小开销重复启动
 *          固定长度的 IMU 数据帧传输（BMI088 驱动即通过本模块读传感器）。
 * @note    依赖 CubeMX 初始化好的全局句柄 hspi1、hdma_spi1_rx、hdma_spi1_tx；
 *          代码中出现的 DMA_LISR_TCIF2 / TCIF3 分别与 DMA2_Stream2（RX）、
 *          DMA2_Stream3（TX）对应，若 CubeMX 中改了流号，标志位也需同步修改。
 * @note    修改 DMA 寄存器前必须先清除 SxCR.EN 并轮询确认其已清零，
 *          否则 PAR/M0AR/NDTR 等配置寄存器写入无效（硬件保护）。
 */
#include "bsp_spi.h"
#include "main.h"



/**
 * @brief 一次性初始化 SPI1 与 RX/TX 两条 DMA 流。
 * @param tx_buf 发送缓冲区首地址（uint32_t 形式的地址值）。
 * @param rx_buf 接收缓冲区首地址（uint32_t 形式的地址值）。
 * @param num    一帧收发的数据项数，写入两条流的 NDTR。
 * @note  执行顺序：
 *        1) 置位 SPI_CR2 的 TXDMAEN/RXDMAEN，使 SPI 事件可触发 DMA 请求；
 *        2) __HAL_SPI_ENABLE 使能 SPI 外设；
 *        3) 关闭 RX 流并等待 EN 清零，清 TCIF2 标志，设置 PAR=&SPI1->DR、
 *           M0AR=rx_buf、NDTR=num，最后使能 RX 传输完成中断；
 *        4) 对 TX 流重复同样的关闭/清标志/PAR/M0AR/NDTR 配置，但不使能
 *           其中断（帧结束以 RX 完成中断为准）。
 * @note  本函数只做配置，不使能 DMA 流；真正的启动在 SPI1_DMA_enable()。
 */
void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
    /* 函数内部流程步骤：
     * 步骤 1：置位 SPI1->CR2 的 TXDMAEN 与 RXDMAEN，允许 SPI 事件触发 DMA 请求。
     * 步骤 2：__HAL_SPI_ENABLE() 使能 SPI1 外设。
     * 步骤 3：配置 RX 流：先 __HAL_DMA_DISABLE，再 while 轮询 SxCR.EN 直到清零
     *         （硬件保护：EN=1 时配置寄存器不可写）。
     * 步骤 4：清 RX 传输完成标志 DMA_LISR_TCIF2。
     * 步骤 5：设置 PAR=&SPI1->DR、M0AR=rx_buf、NDTR=num。
     * 步骤 6：__HAL_DMA_ENABLE_IT(RX, DMA_IT_TC) 使能接收完成中断。
     * 步骤 7：配置 TX 流：同样先关闭并等待 EN 清零，清 DMA_LISR_TCIF3，
     *         再设 PAR、M0AR=tx_buf、NDTR=num（不使能其中断）。
     * 步骤 8：返回。此时两条 DMA 流均未使能，需 SPI1_DMA_enable() 启动传输。
     */
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

    __HAL_SPI_ENABLE(&hspi1);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);

    hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);

    __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }


    __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);

    hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);


}

/**
 * @brief 重新装载地址与长度，并启动一帧 SPI1 DMA 全双工传输。
 * @param tx_buf 本次发送缓冲区地址（可覆盖 init 时的地址，实现连续多帧切换）。
 * @param rx_buf 本次接收缓冲区地址。
 * @param ndtr   本次收发数据项数，同时写入两条流的 NDTR。
 * @note  执行顺序：关闭 RX/TX 流并轮询 EN 清零 -> 通过 hspi1.hdmarx /
 *        hdmatx 清除 TC/HT/TE/DME/FE 五类 DMA 标志 -> 重设 M0AR 与 NDTR
 *        -> 依次使能 RX 流、TX 流。RX 先使能可确保不会漏掉 TX 产生的第一个字节。
 * @note  帧结束由 init() 中使能的 RX 传输完成中断（DMA_IT_TC）通知上层；
 *        调用前须确认上一帧已完成，且缓冲区在传输期间不被修改。
 * @warning 这里只设置 M0AR 与 NDTR，不重写 PAR / CR2，因此必须在
 *          SPI1_DMA_init() 之后调用；若期间 DMA 被其他地方重配，需重新 init。
 */
void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
    /* 函数内部流程步骤：
     * 步骤 1：同时关闭 RX/TX 两条 DMA 流。
     * 步骤 2：分别 while 轮询各自的 SxCR.EN，确认 EN 清零后才继续。
     * 步骤 3：通过 hspi1.hdmarx / hdmatx 清除 TC、HT、TE、DME、FE 五类标志，
     *         避免上一帧的残留标志立刻触发中断。
     * 步骤 4：重设两条流的 M0AR 为本次 tx_buf / rx_buf。
     * 步骤 5：用 __HAL_DMA_SET_COUNTER 把两条流 NDTR 都设为 ndtr。
     * 步骤 6：先使能 RX 流、再使能 TX 流（RX 先就绪防止丢失首字节），
     *         SPI 时钟随即产生，一帧全双工传输开始。
     */
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
    while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_rx);
    }
    while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_spi1_tx);
    }
    //clear flag
    //清除标志位
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
    __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));
    //set memory address
    //设置数据地址
    hdma_spi1_rx.Instance->M0AR = rx_buf;
    hdma_spi1_tx.Instance->M0AR = tx_buf;
    //set data length
    //设置数据长度
    __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
    __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);
    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_spi1_rx);
    __HAL_DMA_ENABLE(&hdma_spi1_tx);
}




