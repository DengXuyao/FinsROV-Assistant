/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

/**
 * @file    UART_Base.h
 * @brief   基于中断 + 静态环形队列的轻量串口发送驱动（模板单例）。
 * @details 所属模块：userCode/devices。用模板参数 ID 选择串口句柄，避免为
 *          每个串口复制代码；发送采用"先入队、再由中断驱动逐帧搬运"的模式：
 *          Transmit() 把数据放进 FixedUartQueue 并尝试启动一次 IT 发送，
 *          每次发送完成回调 OnTxDone() 置位 isTxFinished 并继续装载下一帧，
 *          Handle() 则作为兜底在主循环中驱动 TxLoader()。
 *          队列见 StaticQueue.h，容量 TX_QUEUE_SIZE=20，每帧最长 32 字节。
 * @note    uartHandleList 按 USART 编号索引：下标 1 -> huart1，下标 6 -> huart6，
 *          其余为 nullptr；ID 必须是有效下标，否则会把空句柄传给 HAL。
 * @note    所有发送相关的成员变量（isTxFinished、txQueue）在中断与主循环间共享，
 *          若两者可能并发访问，调用方需要保证临界区；本类自身未加锁。
 * @warning 单帧数据长度超过 UART_MAX_DATA_SIZE 时入队会失败并静默丢弃；
 *          队列满时会先丢弃最旧一帧再入队（见 Transmit()）。
 */

#ifndef FINEMOTE_UARTBASELITE_H
#define FINEMOTE_UARTBASELITE_H

#include "Usermain.h"
#include "StaticQueue.h"

#define TX_QUEUE_SIZE 20                          // 单个串口最多排队的发送帧数
constexpr size_t UART_MAX_DATA_SIZE = 32;         // 每帧最大字节数

/* 串口编号 -> CubeMX 句柄表；下标即模板参数 ID，未使用的串口留 nullptr */
constexpr UART_HandleTypeDef *uartHandleList[] = {nullptr, &huart1, nullptr, nullptr, nullptr, nullptr, &huart6};

/**
 * @class UARTBaseLite
 * @brief 单例式串口发送器。
 * @tparam ID uartHandleList 的下标（串口编号），如 1 表示 USART1、6 表示 USART6。
 */
template <uint8_t ID>
class UARTBaseLite
{
public:
    /**
     * @brief 取得该串口对应的唯一实例（函数内局部静态对象，C++11 起线程安全初始化）。
     * @return 单例引用。
     * @note  构造函数私有，禁止外部创建/拷贝；不同 ID 实例化出互不共享队列的对象。
     */
    static UARTBaseLite &GetInstance()
    {
        /* 函数内部流程步骤：
         * 步骤 1：声明函数内局部静态对象 instance（首次调用时构造，
         *         构造函数为私有且默认实现，txQueue 全零、isTxFinished=true）。
         * 步骤 2：返回该对象的引用；同一模板参数 ID 的所有调用共享同一实例，
         *         不同 ID 各自拥有独立的发送队列与状态。
         */
        static UARTBaseLite instance;
        return instance;
    }

    /**
     * @brief 主循环周期调用：在空闲时尝试装载下一帧发送（中断未触发时的兜底）。
     */
    void Handle()
    {
        /* 函数内部流程步骤：
         * 步骤 1：主循环每拍调用本函数，它只做一件事——转调 TxLoader()。
         * 步骤 2：TxLoader() 在队列非空且上一帧发完时启动下一帧，
         *         作为发送完成中断未及时触发时的兜底驱动。
         */
        TxLoader();
    }

    /**
     * @brief 提交一帧数据到发送队列，并尽快启动 IT 发送。
     * @param data 待发送数据首地址。
     * @param size 数据字节数（<= UART_MAX_DATA_SIZE）。
     * @note  队列满时先 dequeue() 丢弃最旧一帧再入队，保证新指令优先；
     *        入队失败（如 size 超限）直接返回，不发任何数据。
     * @note  数据在入队时已 memcpy 到队列内部，调用方可立即复用原缓冲区。
     */
    void Transmit(uint8_t *data, uint16_t size)
    {
        /* 函数内部流程步骤：
         * 步骤 1：若队列已满，先 dequeue() 丢弃最旧一帧，保证新指令优先。
         * 步骤 2：调用 enqueue(data, size) 把数据复制进静态队列；
         *         入队失败（如超过 UART_MAX_DATA_SIZE）则直接返回。
         * 步骤 3：调用 TxLoader()，若当前空闲就立即启动一次 IT 发送；
         *         若正在发送，则等 OnTxDone() 回调再装载下一帧。
         */
        if (txQueue.isFull())
        {
            txQueue.dequeue();
        }
        if (!txQueue.enqueue(data, size))
        {
            return;
        }
        TxLoader();
    }

    /**
     * @brief 发送完成回调：由 HAL_UART_TxCpltCallback 调用。
     * @note  置位 isTxFinished 后立刻 TxLoader()，形成"完成一帧 -> 启动下一帧"
     *        的链式发送，避免主循环调度延迟造成的总线空闲。
     */
    void OnTxDone()
    {
        /* 函数内部流程步骤：
         * 步骤 1：把 isTxFinished 置 true，表示上一帧 IT 发送已完成。
         * 步骤 2：立刻调用 TxLoader() 装载队列中的下一帧，
         *         形成「发送完成中断 -> 启动下一帧」的链式驱动，
         *         队列空时 TxLoader() 直接返回，等待下次 Transmit()。
         */
        isTxFinished = true;
        TxLoader();
    }

private:
    UARTBaseLite() = default;

    /**
     * @brief 若队列非空且上一帧已发完，取出队首并启动 HAL_UART_Transmit_IT()。
     * @note  取出引用后立即把 isTxFinished 置 false 并 dequeue()，
     *        HAL 会直接引用队列槽位的 data 缓冲区直到发送完成，因此该槽位
     *        在发送完成前不能被后续入队覆盖（当前实现依赖队列容量足够大）。
     */
    void TxLoader()
    {
        /* 函数内部流程步骤：
         * 步骤 1：检查队列非空且 isTxFinished 为真（上一帧已发完），
         *         否则直接返回，等待下一次 Handle()/OnTxDone() 再试。
         * 步骤 2：取队首元素引用 elem，读取其 data 缓冲区与 size。
         * 步骤 3：调用 HAL_UART_Transmit_IT(uartHandleList[ID], ...) 启动
         *         非阻塞发送，HAL 会在中断中继续搬运该缓冲区。
         * 步骤 4：立刻把 isTxFinished 置 false 防止重复启动。
         * 步骤 5：dequeue() 让出队首槽位；发送完成回调 OnTxDone() 会
         *         再次调用本函数，形成逐帧链式发送。
         */
        if (!txQueue.isEmpty() && isTxFinished)
        {
            auto &elem = txQueue.front();
            HAL_UART_Transmit_IT(uartHandleList[ID], const_cast<uint8_t*>(elem.data.data()), elem.size);
            isTxFinished = false;
            txQueue.dequeue();
        }
    }

    FixedUartQueue<TX_QUEUE_SIZE, UART_MAX_DATA_SIZE> txQueue;  // 待发送帧的静态环形队列
    volatile bool isTxFinished = true;                           // 当前是否有 IT 发送在进行（volatile：中断中修改）
};


#endif // FINEMOTE_UARTBASELITE_H
