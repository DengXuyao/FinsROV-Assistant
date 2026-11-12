//
// Created by admin on 2023/9/19.
//
/**
 * @file    Kalman_Filter.cpp
 * @brief   标量卡尔曼滤波器 update() 的递推实现。
 * @details 所属模块：userCode/algorithms，配套声明见 Inc/Kalman_Filter.h。
 *          状态模型假设被测量在相邻采样间近似不变（x(k) = x(k-1) + 过程噪声），
 *          观测模型为 z(k) = x(k) + 测量噪声。因此没有状态转移矩阵和观测矩阵，
 *          只跟踪标量状态 est 及其误差协方差 P，计算量极小，适合在控制循环内
 *          对每个传感器通道各实例化一个滤波器。
 * @note    无 HAL 依赖、无动态内存分配；所有状态保存在对象成员中，
 *          调用顺序即时间顺序，务必按固定周期调用。
 * @note    若 P_pred + R == 0（例如 Q、R、P 全为 0），增益计算将除零，
 *          构造参数需避免这种情况。
 */
#include <Kalman_Filter.h>
#include <vector>

/**
 * @brief 一次完整的卡尔曼递推：预测 -> 求增益 -> 用测量修正。
 * @details 逐步对应公式：
 *          1) 预测：状态不变 pred = est，协方差因过程噪声膨胀 P_pred = P + Q；
 *          2) 增益：K = P_pred / (P_pred + R)，K 越接近 1 越采信新测量；
 *          3) 更新：est = pred + K*(measurement - pred)，括号即新息（残差）；
 *             P = (1-K)*P_pred，融合后不确定性必然下降；
 *          4) 返回新的 est，并保留 P 供下一拍使用。
 * @param measurement 本次传感器测量值（新息的来源）。
 * @return 滤波后的状态估计值，同时已写回成员 est。
 */
// Update the filter: input the measurement and output the filtered estimate.
// 更新滤波器，输入测量值，输出滤波后的估计值
double KalmanFilter::update(double measurement)
{
    /* 函数内部流程步骤：
     * 步骤 1：预测状态。常值模型下 x_pred 直接取上一拍估计 est，不变。
     * 步骤 2：预测协方差。P_pred = P + Q，把过程噪声累加进不确定性。
     * 步骤 3：计算卡尔曼增益 K = P_pred / (P_pred + R)。
     *         Q 大 / R 小时 K 趋近 1，更信任本次测量；反之趋近 0，更平滑。
     * 步骤 4：用新息 (measurement - pred) 修正估计：est = pred + K * 新息。
     * 步骤 5：收缩协方差 P = (1 - K) * P_pred，供下一拍迭代使用。
     * 步骤 6：返回新的 est（已写回成员），调用方按固定周期重复以上步骤。
     */
		// Prediction
    // 预测步骤
    /* 常值模型下状态预测不变，只有不确定性因过程噪声 Q 增大 */
    double pred = est;
    double P_pred = P + Q;

		// Compute the Kalman gain.
    // 计算卡尔曼增益
    double K = P_pred / (P_pred + R);

		// Update
    // 更新步骤
    est = pred + K * (measurement - pred);
    P = (1 - K) * P_pred;

    return est;
}   