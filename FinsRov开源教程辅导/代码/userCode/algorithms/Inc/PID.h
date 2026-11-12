//
// Created by LEGION on 2021/10/19.
//

/**
 * @file    PID.h
 * @brief   位置式（增量误差历史）PID 控制器的数据结构与类声明。
 * @details 所属模块：userCode/algorithms。PID_Regulator_t 同时保存参数、误差
 *          历史、各分量及限幅值，PID 类在 Src/PID.cpp 中实现 Reset() 与
 *          PIDCalc()。用于推进器转速、深度/姿态等闭环控制。
 *          控制律：output = Kp*e(k) + Ki*Σe(k) + Kd*(e(k)-e(k-1))，
 *          比例、积分、微分三项及总输出分别做限幅。
 * @note    依赖 Usermain.h 中的 INRANGE 宏做限幅，且该宏会直接修改传入变量；
 *          构造 PID_Regulator_t 时须保证 ki != 0，否则积分限幅会出现除零。
 * @note    本类无采样周期项，kp/ki/kd 的实际效果与调用周期绑定，
 *          PIDCalc() 应按固定控制周期调用（见 CONTROL_FREQUENCY）。
 */

#ifndef RM_FRAME_C_PID_H
#define RM_FRAME_C_PID_H

#include "Usermain.h"

/**
 * @struct PID_Regulator_t
 * @brief 单个 PID 环的全部运行参数与状态（便于整包保存/恢复/在线调参）。
 */
struct PID_Regulator_t {
    float ref;              // 目标值（setpoint）
    float fdb;              // 反馈值（测量值）
    float err[4];           // 误差历史环形槽位；实际只用 err[2] 上一拍、err[3] 当前拍
    float errSum;           // 误差累积和，用于积分项
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float componentKp;      // 本次计算得到的比例分量（限幅后）
    float componentKi;      // 本次计算得到的积分分量（限幅后）
    float componentKd;      // 本次计算得到的微分分量（限幅后）
    float componentKpMax;   // 比例分量限幅上限（下限取相反数）
    float componentKiMax;   // 积分分量限幅上限（下限取相反数）
    float componentKdMax;   // 微分分量限幅上限（下限取相反数）
    float output;           // 三项之和经限幅后的最终控制量
    float outputMax;        // 总输出限幅上限（下限取相反数）
    /** @brief 默认构造：不初始化任何成员，需由 Reset() 或外部显式赋值。 */
    PID_Regulator_t(){};
    /**
     * @brief 带参数构造：一次性给定增益与各限幅值。
     * @param kp/ki/kd 比例、积分、微分系数。
     * @param pM/iM/dM 比例、积分、微分分量限幅上限。
     * @param oM 总输出限幅上限。
     * @note  仅初始化上述 7 个参数，误差历史与输出等状态未置零，
     *        使用前应先调用 PID::Reset() 清零。
     */
    /* 构造函数内部流程（初始化列表按成员声明顺序执行）：
     * 步骤 1：把形参 kp/ki/kd 写入同名成员（形参屏蔽成员名，故 kp(kp) 形式）。
     * 步骤 2：把 pM/iM/dM 分别写入 componentKpMax/componentKiMax/componentKdMax。
     * 步骤 3：把 oM 写入 outputMax；函数体为空，误差历史与输出等状态未初始化，
     *         使用前需调用 PID::Reset() 清零。
     */
    PID_Regulator_t(float kp, float ki, float kd, float pM, float iM, float dM, float oM):
        kp(kp), ki(ki), kd(kd), componentKpMax(pM), componentKiMax(iM), componentKdMax(dM), outputMax(oM){};
    /** @brief 拷贝构造：按字节整包复制，包括误差历史与全部状态。 */
    PID_Regulator_t(const PID_Regulator_t& OTHER){
        /* 函数内部流程步骤：
         * 步骤 1：以 sizeof(PID_Regulator_t) 为长度，把 OTHER 的整块内存
         *         用 std::memcpy 复制到 this，一次完成全部参数与状态拷贝。
         * 步骤 2：返回构造好的对象（包含误差历史、积分与输出状态）。
         */
        std::memcpy(this, &OTHER, sizeof(PID_Regulator_t));
    }
};


/**
 * @class PID
 * @brief 封装一个 PID 环的状态（PIDInfo）与计算接口。
 * @note  PIDInfo 为 public，允许上层直接读写参数实现在线调参；
 *        多个控制环应各自使用一个独立实例，避免状态串扰。
 */
class PID{
public:
    PID_Regulator_t PIDInfo{};  // 本环的参数与运行状态，值初始化为零
    /**
     * @brief 用外部参数结构体整包覆盖本环参数与状态。
     * @param pidRegulator 源参数指针；为 nullptr 时不做任何操作。
     */
    void Reset(PID_Regulator_t * pidRegulator);
    /** @brief 清零误差历史、积分累积、三个分量与输出，保留增益及限幅参数。 */
    void Reset();
    /**
     * @brief 计算一次 PID 控制量（使用内部保存的 outputMax 限幅）。
     * @param target   目标值。
     * @param feedback 反馈值。
     * @return 限幅后的控制输出，同时写入 PIDInfo.output。
     */
    float PIDCalc(float target,float feedback);
    /**
     * @brief 先更新总输出限幅，再计算 PID 控制量。
     * @param target   目标值。
     * @param feedback 反馈值。
     * @param max      本次及后续使用的总输出限幅上限。
     * @return 限幅后的控制输出。
     */
    float PIDCalc(float target,float feedback,float max);
};

#endif //RM_FRAME_C_PID_H
