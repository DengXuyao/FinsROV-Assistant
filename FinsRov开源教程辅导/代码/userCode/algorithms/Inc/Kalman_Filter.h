//
// Created by admin on 2023/9/19.
//

/**
 * @file    Kalman_Filter.h
 * @brief   一维（标量）卡尔曼滤波器与限幅模板 clamp。
 * @details 所属模块：userCode/algorithms。用于对单个标量信号（如压力传感器、
 *          深度、速度等读数）做递推平滑：以"上一时刻估计值"作为状态预测，
 *          再用新的测量值按卡尔曼增益加权修正。
 *          采用最简单的常值状态模型：
 *              预测：x_pred = est,      P_pred = P + Q
 *              增益：K      = P_pred / (P_pred + R)
 *              更新：est    = x_pred + K * (z - x_pred), P = (1 - K) * P_pred
 *          实现位于 Src/Kalman_Filter.cpp，不依赖 HAL，可在 MCU 与 PC 上通用。
 * @note    Q 越大越信任测量值（跟随快、噪声大）；R 越大越信任模型预测
 *          （更平滑、响应慢）。构造时应保证 Q、R、P 非负，且 P_pred + R != 0。
 * @note    工程曾计划引入 Eigen 做矩阵运算（见下方被注释的 include），
 *          当前版本为纯标量实现，不依赖任何第三方矩阵库。
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/* string / vector 属于历史遗留依赖：当前实现并未使用 STL 容器，
 * 保留是为了兼容工程原有编译环境。 */
#include <string>
#include <vector>
//#include <Dense>//包含Eigen矩阵运算库，用于矩阵计算
/* cmath 为浮点数学运算提供声明 */
#include <cmath>
//using Eigen::MatrixXd;

/**
 * @brief 把 v 限幅到闭区间 [lo, hi]，constexpr 版本可用于编译期求值。
 * @tparam T 支持 operator< 比较的类型（int / float / double 等）。
 * @param  v  待限幅的值。
 * @param  lo 下限（含）。
 * @param  hi 上限（含）。
 * @return v < lo 时返回 lo；v > hi 时返回 hi；否则返回 v。
 * @note   返回 const 引用，不要长期持有返回值的引用；与 Usermain.h 中的
 *          INRANGE 宏不同，本函数不修改实参，而是返回结果值。
 */
template <typename T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
    /* 函数内部流程步骤：
     * 步骤 1：先用 v < lo 判断是否低于下限，是则返回 lo。
     * 步骤 2：否则用 hi < v 判断是否高于上限，是则返回 hi。
     * 步骤 3：两者都不满足说明在区间内，返回 v 本身（三步均为 const 引用返回）。
     */
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

/**
 * @class KalmanFilter
 * @brief 标量卡尔曼滤波器，按固定周期反复调用 update() 递推状态估计。
 * @note  内部状态在成员中持续保存，多个实例相互独立；若被中断与主循环
 *        同时访问同一实例，需要外部临界区保护。
 */
class KalmanFilter {
public:
    /**
     * @brief 构造滤波器并初始化噪声参数与初始状态。
     * @param process_noise     Q，过程噪声方差，描述模型自身不确定度。
     * @param measurement_noise R，测量噪声方差，描述传感器读数不确定度。
     * @param estimation_error  P，初始估计误差协方差，影响起始阶段收敛速度。
     * @param initial_estimate  状态初值 est，通常取第一次有效测量值。
     */
    /* 构造函数内部流程（初始化列表，按成员声明顺序执行）：
     * 步骤 1：Q  = process_noise（过程噪声方差）。
     * 步骤 2：R  = measurement_noise（测量噪声方差）。
     * 步骤 3：P  = estimation_error（初始估计误差协方差）。
     * 步骤 4：est = initial_estimate（状态初值）；随后函数体为空，构造结束。
     */
    KalmanFilter(double process_noise, double measurement_noise, double estimation_error, double initial_estimate)
        : Q(process_noise), R(measurement_noise), P(estimation_error), est(initial_estimate) {}

    /**
     * @brief 输入一次新测量，完成"预测—计算增益—更新"，输出滤波后的估计值。
     * @param measurement 本次传感器测量值 z。
     * @return 更新后的状态估计值 est。
     * @note   应按固定采样周期调用；采样周期改变时 Q 需相应调整。
     */
    double update(double measurement);

private:
    double Q;       // 过程噪声方差
    double R;       // 测量噪声方差
    double P;       // 估计误差协方差
    double est;   // 当前状态估计
};

#endif
