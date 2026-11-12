//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
/**
 * @file    MahonyAHRS.h
 * @brief   Mahony / Madgwick 姿态解算（AHRS）算法的全局参数与函数声明。
 * @details 所属模块：userCode/algorithms。原始实现来自 SOH Madgwick 公开的
 *          Mahony AHRS 与 Madgwick IMU/MARG 算法，工程将其移植为 C++ 编译单元，
 *          实现位于 Src/MahonyAHRS.cpp，供 devices/IMU 层融合陀螺仪、加速度计
 *          与磁力计数据，输出机体相对地理坐标系的四元数。
 *          提供 4 个更新函数：
 *            - MahonyAHRSupdate()    ：九轴（含磁力计，PI 互补滤波）
 *            - MahonyAHRSupdateIMU()  ：六轴（仅陀螺+加计，航向会漂移）
 *            - MadgwickAHRSupdate()   ：九轴（梯度下降法）
 *            - MadgwickAHRSupdateIMU()：六轴（梯度下降法）
 * @note    四元数用 float q[4] = {w, x, y, z} 传入，调用方负责保存初值
 *          （一般为 {1, 0, 0, 0}）并在每次调用后读回；陀螺仪数据单位必须是
 *          rad/s，加速度/磁力计可为任意单位（函数内部会归一化）。
 * @note    采样频率在 .cpp 中以宏 sampleFreq 固定为 150 Hz，若实际调用频率
 *          不同需同步修改，否则积分与增益的时间尺度会失准。
 * @note    磁力计三轴全为 0 时自动退化为对应的 IMU（六轴）算法，避免归一化
 *          除零产生 NaN；加速度计三轴全为 0 时跳过反馈，仅靠陀螺仪积分。
 * @note    全局变量用 volatile 修饰，因为它们可能在中断/主循环间被在线调参。
 */
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

/**
 * @name    可调增益与姿态全局变量
 * @details 这些变量在 .cpp 中定义并以默认值初始化，声明为 volatile 以保证
 *          在线调参（例如通过调试串口/中断修改）时读写不被编译器优化掉。
 * @note    twoKp / twoKi 是 2 倍增益（算法推导中系数自带 1/2，故这里存 2*K）；
 *          twoKp 越大，姿态越快向加速度/磁力计给出的参考方向收敛，但抗振动
 *          能力变差；twoKi 用于消除陀螺常值零偏，过大易引起超调。
 * @note    beta 是 Madgwick 梯度下降法的步长（收敛速率），仅在
 *          MadgwickAHRSupdate* 系列函数中使用。
 * @note    q0~q3 为兼容原始实现保留的外部四元数声明（.cpp 中该定义已注释），
 *          实际姿态四元数由调用方以 float q[4] 参数传入并保存。
 */
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
extern volatile float beta;
//---------------------------------------------------------------------------------------------------
/* 四个更新函数在 MahonyAHRS.cpp 中的通用内部流程步骤：
 * 步骤 1：检查磁力计/加速度计是否有效，无效时退化到六轴版本或跳过反馈。
 * 步骤 2：用 invSqrt 把加速度（及磁场）向量归一化。
 * 步骤 3：预计算四元数乘积缓存，推出估计的重力/磁场参考方向。
 * 步骤 4：Mahony 系用叉积求误差并做 PI 反馈；Madgwick 系求目标函数梯度
 *         并归一化修正步长（受 twoKp/twoKi 或 beta 控制）。
 * 步骤 5：由修正后的角速度计算四元数导数，按 1/sampleFreq 积分。
 * 步骤 6：归一化四元数并写回调用方传入的 q[4]。
 * invSqrt()：位模式转整数 -> 0x5f3759df 魔数估计 -> 转回 float -> 一次牛顿迭代。
 */
// Function declarations

/**
 * @brief Mahony 九轴姿态更新（PI 互补滤波，含地磁航向修正）。
 * @param q 长度为 4 的四元数数组 {w,x,y,z}，既作输入也作输出，原地更新。
 * @param gx/gy/gz 机体坐标系三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度（比力），任意量纲，函数内会归一化。
 * @param mx/my/mz 三轴磁场强度，任意量纲，函数内会归一化。
 * @note  磁力计全 0 时自动转调 MahonyAHRSupdateIMU()；加速度全 0 时跳过
 *        反馈，仅用陀螺积分。修正强度由全局量 twoKp、twoKi 决定。
 */
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
/**
 * @brief Mahony 六轴姿态更新（仅陀螺仪 + 加速度计，无磁力计）。
 * @param q 四元数数组 {w,x,y,z}，原地更新。
 * @param gx/gy/gz 三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度，作为重力参考方向。
 * @note  只修正横滚/俯仰，航向角无绝对参考，会随陀螺零偏缓慢漂移；
 *        本函数也是九轴函数在无磁数据时的退化入口。
 */
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
/**
 * @brief Madgwick 九轴姿态更新（梯度下降法，含地磁航向修正）。
 * @param q 四元数数组 {w,x,y,z}，原地更新。
 * @param gx/gy/gz 三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度，任意量纲。
 * @param mx/my/mz 三轴磁场强度，任意量纲。
 * @note  修正步长由全局量 beta 控制；磁力计全 0 时转调
 *        MadgwickAHRSupdateIMU() 以规避归一化除零。
 */
void MadgwickAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
/**
 * @brief Madgwick 六轴姿态更新（梯度下降法，仅陀螺仪 + 加速度计）。
 * @param q 四元数数组 {w,x,y,z}，原地更新。
 * @param gx/gy/gz 三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度，用于构造重力参考方向的误差函数。
 * @note  与 Mahony 六轴版本一样不修正航向；加速度全 0 时仅做陀螺积分。
 */
void MadgwickAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
