/**
 * @file    PID.cpp
 * @brief   位置式 PID 控制器的状态复位与控制量计算实现。
 * @details 所属模块：userCode/algorithms，配套声明见 Inc/PID.h。
 *          控制律（位置式）：
 *              e(k)     = ref - fdb
 *              P 项     = kp * e(k)
 *              I 项     = ki * Σe(k)（先对 Σe 限幅，抑制积分饱和）
 *              D 项     = kd * (e(k) - e(k-1))
 *              output   = clamp(P + I + D, ±outputMax)
 *          三个分量分别限幅后再求和并整体限幅，便于观察各项贡献并防止某一项
 *          单独过大。限幅由 Usermain.h 的 INRANGE 宏原地完成。
 * @note    err[3] 存当前误差、err[2] 存上一拍误差，err[0]/err[1] 为预留槽位；
 *          每次计算结束把 err[3] 搬到 err[2]，故调用周期必须固定，否则微分项
 *          与积分项的物理含义会随周期变化。
 * @note    PIDInfo 中的 kp/ki/kd 与各 *Max 需由上层在调用前设置（可用带参构造
 *          或 Reset(PID_Regulator_t*) 整包导入），本文件不负责初始化参数。
 */
#include "PID.h"

/**
 * @brief 无参复位：清空误差历史、积分累积、三个分量与输出。
 * @note  只清"状态"，保留 kp/ki/kd 与所有限幅参数，适合停机后重新启动控制环、
 *        或目标/反馈突变时避免旧积分继续起作用。
 * @note  err[0..3] 全部清零，因此复位后的第一次 PIDCalc 微分项为 kp 无关的
 *        kd * e(k)，可能出现一次微分冲击，必要时可先空跑一拍。
 */
void PID::Reset() {
    /* 函数内部流程步骤：
     * 步骤 1：清零本拍目标 ref 与反馈 fdb。
     * 步骤 2：把误差历史 err[0]~err[3] 全部清零（含上一拍 err[2]）。
     * 步骤 3：清零积分累积 errSum，避免重启后旧积分继续作用。
     * 步骤 4：清零三个分量 componentKp/Ki/Kd。
     * 步骤 5：清零输出 output；不触碰 kp/ki/kd 与各 *Max 参数，
     *         使复位后仍沿用已整定好的控制参数。
     */
     PIDInfo.ref = 0;
     PIDInfo.fdb = 0;
     PIDInfo.err[0] = 0;
     PIDInfo.err[1] = 0;
     PIDInfo.err[2] = 0;
     PIDInfo.err[3] = 0;
     PIDInfo.errSum = 0;

     PIDInfo.componentKp = 0;
     PIDInfo.componentKi = 0;
     PIDInfo.componentKd = 0;

     PIDInfo.output = 0;

}
/**
 * @brief 用外部参数结构体整包替换本环的 PIDInfo。
 * @param pidRegulator 指向源参数/状态结构体；nullptr 时静默跳过。
 * @note  使用默认拷贝赋值（内部为 memcpy 语义），连误差历史、积分累积、
 *        输出状态一起覆盖，常用于从配置表批量加载多环参数。
 * @note  若只想清除运行状态而保留当前参数，应调用无参 Reset()。
 */
void PID::Reset(PID_Regulator_t *pidRegulator) {
    /* 函数内部流程步骤：
     * 步骤 1：先判断 pidRegulator 是否为 nullptr，为空则不做任何操作。
     * 步骤 2：非空时用默认赋值把整个结构体覆盖到 PIDInfo，
     *         包括增益、限幅、误差历史、积分累积与输出状态。
     */
    if(pidRegulator != nullptr)PIDInfo = *pidRegulator;
}

/**
 * Compute the PID control output.
 * @param target   /Reference (setpoint) value.
 * @param feedback /Feedback (measured) value.
 * @return /Control output.
 */
/**
 * 计算pid算法的控制量
 * @param target 目标量
 * @param feedback 反馈量
 * @return 控制量
 */
float PID::PIDCalc(float target,float feedback) {
    /* 函数内部流程步骤：
     * 步骤 1：把 target / feedback 记入 PIDInfo.ref / fdb，
     *         计算当前误差 err[3] = ref - fdb（err[2] 仍为上一拍误差）。
     * 步骤 2：比例项 componentKp = err[3] * kp。
     * 步骤 3：累加误差 errSum += err[3]，并按 ±componentKiMax/ki 限幅
     *         （积分抗饱和），再得积分项 componentKi = errSum * ki。
     * 步骤 4：微分项 componentKd = (err[3] - err[2]) * kd，只用相邻两拍。
     * 步骤 5：用 INRANGE 宏分别对 P/I/D 三个分量做 ±*Max 限幅。
     * 步骤 6：三项求和得 output，再按 ±outputMax 做总输出限幅。
     * 步骤 7：把 err[3] 搬到 err[2] 作为下一拍的历史误差，返回 output。
     */
    /* 记录本拍目标/反馈，并计算当前误差 e(k)；err[2] 仍是上一拍误差 e(k-1) */
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[3] * PIDInfo.kp;
    PIDInfo.errSum += PIDInfo.err[3];
    /* 积分抗饱和：把误差累积和限制在「积分分量限幅 / ki」对应的范围内，
     * 避免长时间偏差导致积分项过大、退出饱和滞后（要求 ki 非 0）。 */
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);

    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;

    /* 对 P/I/D 三个分量分别限幅，防止单项过大；上下限对称（±Max）。 */
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);

    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);

    PIDInfo.err[2] = PIDInfo.err[3];
    return PIDInfo.output;
}

/**
 * @brief 带输出限幅设置的 PID 计算重载。
 * @param target   目标值。
 * @param feedback 反馈值。
 * @param max      本次及后续计算使用的总输出限幅上限（下限自动取 -max）。
 * @return 限幅后的控制量。
 * @note  先覆盖 PIDInfo.outputMax 再调用两参数版本，因此该限幅值会持久保存，
 *        适合在不同工作模式（如限功率/全功率）切换时动态调整输出能力；
 *        若只需临时限幅，调用后应自行恢复原值。
 */
float PID::PIDCalc(float target, float feedback, float max) {
    /* 函数内部流程步骤：
     * 步骤 1：把 max 写入 PIDInfo.outputMax，覆盖原总输出限幅（持久生效）。
     * 步骤 2：直接调用两参数版 PID::PIDCalc(target, feedback) 完成全部计算。
     * 步骤 3：把其返回值原样返回给调用方。
     */
    PIDInfo.outputMax = max;
    return PID::PIDCalc(target,feedback);
}

