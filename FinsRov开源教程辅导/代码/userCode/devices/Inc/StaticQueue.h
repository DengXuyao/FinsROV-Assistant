/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

/**
 * @file    StaticQueue.h
 * @brief   编译期定容、零动态分配的环形队列（UART / I2C 发送缓冲）。
 * @details 所属模块：userCode/devices。为 MCU 上中断驱动的通信设计：容量 N 与
 *          单帧最大长度 MAX_DATA_SIZE 都是模板参数，元素存放在 std::array
 *          中，不使用 new/malloc，避免堆碎片与分配失败。
 *          提供两套队列：
 *            - FixedUartQueue：纯字节帧（串口发送），元素含 data[] 与 size；
 *            - FixedI2CQueue ：带设备地址的帧，元素含 header(devAddress,size)
 *              与 data[]，供 I2C 多路发送调度使用。
 *          两者都用 frontIdx / rearIdx / itemCount 实现循环数组，入队/出队
 *          复杂度 O(1)，满队列入队返回 false 而不是覆盖数据。
 * @note    非线程/中断安全：若中断与主循环同时访问同一队列，需在调用处
 *          关中断或加临界区；front() 前必须先检查 isEmpty()，否则读到旧槽位。
 * @note    dequeue() 不拷贝数据、只清零 size 并移动头指针，调用方应在
 *          front()/dequeue() 之间把数据交给 DMA/中断发送（或先复制出去）。
 */

#ifndef STATICQUEUE_HPP
#define STATICQUEUE_HPP

/* size_t、定长整型、std::array、memcpy 与类型萃取支持 */
#include <cstdint>
#include <cstddef>
#include <array>
#include <cstring>
#include <type_traits>

/**
 * @struct UartQueueElement
 * @brief 串口发送队列的一个元素：定长字节数组 + 本帧有效长度。
 * @tparam MAX_DATA_SIZE 单帧最大字节数；data 在构造时值初始化为全 0。
 */
template <size_t MAX_DATA_SIZE>
struct UartQueueElement
{
    std::array<uint8_t, MAX_DATA_SIZE> data{};
    uint16_t size = 0;
};

/**
 * @struct I2CQueueElementHeader
 * @brief I2C 发送帧头：目标设备地址与数据长度。
 * @note  devAddress 通常已是 8 位形式（7 位地址 << 1），与 HAL_I2C 接口一致。
 */
struct I2CQueueElementHeader
{
    uint16_t devAddress = 0;
    uint16_t size       = 0;
};

/**
 * @class FixedUartQueue
 * @brief 定容环形字节帧队列。
 * @tparam N 队列最多容纳的帧数。
 * @tparam MAX_DATA_SIZE 每帧最大字节数。
 */
template <size_t N, size_t MAX_DATA_SIZE>
class FixedUartQueue
{
private:
    std::array<UartQueueElement<MAX_DATA_SIZE>, N> buffer{};  // 静态存储的所有槽位
    size_t frontIdx   = 0;   // 队首（下一个要出队/访问）的槽位下标
    size_t rearIdx    = 0;   // 队尾（下一个入队位置）的槽位下标
    size_t itemCount  = 0;   // 当前帧数，用于区分空/满（避免留空一个槽位）

public:
    FixedUartQueue() = default;

    /**
     * @brief 拷贝一帧数据入队。
     * @param data 源数据首地址。
     * @param size 本帧字节数，不能超过 MAX_DATA_SIZE。
     * @return true 入队成功；队列已满或数据超长时返回 false（不修改队列）。
     * @note  使用 memcpy 复制到 rearIdx 槽位，再以模 N 方式推进 rearIdx；
     *        未覆盖的尾部字节保留旧值，但由 size 限定有效范围。
     */
    bool enqueue(const uint8_t *data, uint16_t size)
    {
        /* 函数内部流程步骤：
         * 步骤 1：先判满 isFull() 与长度 size > MAX_DATA_SIZE，
         *         任一成立即返回 false，队列保持不变。
         * 步骤 2：取 rearIdx 指向的槽位引用 slot。
         * 步骤 3：memcpy 把 data 的 size 字节复制到 slot.data，并写 slot.size。
         * 步骤 4：rearIdx = (rearIdx + 1) % N 循环后移，itemCount 加一。
         * 步骤 5：返回 true；未覆盖的尾部字节保留旧值，由 size 界定有效范围。
         */
        if (isFull() || size > MAX_DATA_SIZE)
        {
            return false;
        }
        auto &slot = buffer[rearIdx];
        std::memcpy(slot.data.data(), data, size);
        slot.size = size;

        rearIdx   = (rearIdx + 1) % N;
        ++itemCount;
        return true;
    }

    /**
     * @brief 丢弃队首一帧。
     * @return true 成功出队；队列为空时返回 false。
     * @note  只把该槽位的 size 清零并推进 frontIdx，不清空 data，
     *        因此复用槽位时旧字节会保留到下次 memcpy 覆盖为止。
     */
    bool dequeue()
    {
        /* 函数内部流程步骤：
         * 步骤 1：先判空 isEmpty()，空队列返回 false。
         * 步骤 2：把队首槽位 size 置 0（不清 data，节省拷贝）。
         * 步骤 3：frontIdx = (frontIdx + 1) % N 循环后移。
         * 步骤 4：itemCount 减一并返回 true；原槽位后续可被入队复用。
         */
        if (isEmpty())
        {
            return false;
        }
        // 可选：清零已出队元素内容
        buffer[frontIdx].size = 0;
        frontIdx  = (frontIdx + 1) % N;
        --itemCount;
        return true;
    }

    /** @brief 返回队首元素的可写引用；调用前必须确认 !isEmpty()。 */
    UartQueueElement<MAX_DATA_SIZE> &front()
    {
        /* 访问类函数的内部流程步骤（两组队列结构相同）：
         * 步骤 1：front() / frontHeader() / frontData() 都直接返回
         *         buffer[frontIdx] 或其成员的引用/指针，不做边界检查，
         *         调用方必须先确认 !isEmpty()。
         * 步骤 2：isEmpty() 判断 itemCount == 0，isFull() 判断 itemCount == N，
         *         size() 返回 itemCount；三者都是单行常量操作。
         * 步骤 3：const 版本提供只读引用，便于在 const 上下文访问队首。
         */
        // 调用者应先检查 isEmpty()
        return buffer[frontIdx];
    }

    /** @brief 返回队首元素的只读引用（const 版本）。 */
    const UartQueueElement<MAX_DATA_SIZE> &front() const
    {
        return buffer[frontIdx];
    }

    /** @brief 队列是否为空。 */
    bool isEmpty() const { return itemCount == 0; }
    /** @brief 队列是否已满（itemCount 达到容量 N）。 */
    bool isFull()  const { return itemCount == N; }
    /** @brief 当前排队帧数。 */
    size_t size()  const { return itemCount; }
};

/**
 * @class FixedI2CQueue
 * @brief 带目标设备地址的定容 I2C 发送队列。
 * @tparam N 最大帧数。
 * @tparam MAX_DATA_SIZE 每帧最大字节数。
 * @note  与 FixedUartQueue 结构相同，但每帧额外保存 devAddress，
 *        出队前可通过 frontHeader()/frontData() 分别取头与数据指针。
 */
template <size_t N, size_t MAX_DATA_SIZE>
class FixedI2CQueue
{
private:
    /** @brief 队列内部节点：帧头 + 定长数据区。 */
    struct I2CNode {
        I2CQueueElementHeader header{};
        std::array<uint8_t, MAX_DATA_SIZE> data{};
    };

    std::array<I2CNode, N> buffer{};  // 静态存储的所有节点
    size_t frontIdx   = 0;            // 队首下标
    size_t rearIdx    = 0;            // 下一个入队位置
    size_t itemCount  = 0;            // 当前帧数

public:
    FixedI2CQueue() = default;

    /**
     * @brief 把带设备地址的一帧数据入队。
     * @param devAddress 目标 I2C 设备地址（HAL 使用的 8 位形式）。
     * @param data       源数据首地址。
     * @param size       数据字节数，不能超过 MAX_DATA_SIZE。
     * @return true 成功；满队列或超长返回 false。
     */
    bool enqueue(uint16_t devAddress, const uint8_t *data, uint16_t size)
    {
        /* 函数内部流程步骤：
         * 步骤 1：队列已满或 size > MAX_DATA_SIZE 时返回 false。
         * 步骤 2：取 rearIdx 槽位 slot，写 header.devAddress 与 header.size。
         * 步骤 3：memcpy 把数据复制到 slot.data（只复制 size 字节）。
         * 步骤 4：rearIdx 模 N 后移、itemCount 加一，返回 true。
         * 步骤 5：出队前上层通过 frontHeader()/frontData() 取地址与目标设备。
         */
        if (isFull() || size > MAX_DATA_SIZE)
        {
            return false;
        }

        auto &slot = buffer[rearIdx];
        slot.header.devAddress = devAddress;
        slot.header.size       = size;
        std::memcpy(slot.data.data(), data, size);

        rearIdx   = (rearIdx + 1) % N;
        ++itemCount;
        return true;
    }

    /** @brief 丢弃队首一帧（清零其 header.size 并推进队首）。
     *  @return true 出队成功；空队列返回 false。 */
    bool dequeue()
    {
        /* 函数内部流程步骤：
         * 步骤 1：判空 isEmpty()，空队列返回 false。
         * 步骤 2：把队首节点 header.size 清零标记无效（保留 devAddress/data）。
         * 步骤 3：frontIdx 模 N 后移，itemCount 减一，返回 true。
         */
        if (isEmpty())
        {
            return false;
        }
        buffer[frontIdx].header.size = 0;
        frontIdx  = (frontIdx + 1) % N;
        --itemCount;
        return true;
    }

    /** @brief 取队首帧头的可写引用（可读/改 devAddress、size）。 */
    I2CQueueElementHeader &frontHeader()
    {
        return buffer[frontIdx].header;
    }

    /** @brief 取队首帧头的只读引用（const 版本）。 */
    const I2CQueueElementHeader &frontHeader() const
    {
        return buffer[frontIdx].header;
    }

    /** @brief 取队首数据区首地址（可写），供 HAL 直接发送。 */
    uint8_t *frontData()
    {
        return buffer[frontIdx].data.data();
    }

    /** @brief 取队首数据区首地址（只读，const 版本）。 */
    const uint8_t *frontData() const
    {
        return buffer[frontIdx].data.data();
    }

    /** @brief 队列是否为空。 */
    bool isEmpty() const { return itemCount == 0; }
    /** @brief 队列是否已满。 */
    bool isFull()  const { return itemCount == N; }
    /** @brief 当前排队帧数。 */
    size_t size()  const { return itemCount; }
};

#endif // STATICQUEUE_HPP
