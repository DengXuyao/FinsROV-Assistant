//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

/**
 * @file    MahonyAHRS.cpp
 * @brief   Mahony 与 Madgwick 两套姿态融合算法的完整实现。
 * @details 所属模块：userCode/algorithms，声明见 Inc/MahonyAHRS.h。
 *          四个更新函数都以「陀螺仪积分预测姿态 + 加速度/磁力计提供绝对参考
 *          方向修正」为核心：
 *            - Mahony 系：用估计方向与测量方向的叉积作为误差，经 PI
 *              （twoKp / twoKi）反馈叠加到角速度上，再积分四元数微分方程；
 *            - Madgwick 系：构造目标函数并用梯度下降求修正步长 s0~s3，
 *              以 beta 为步长从四元数导数中扣除，再积分。
 *          九轴版本同时用重力与地磁参考方向修正航向，六轴版本只用重力，
 *          因此航向角会随陀螺零偏缓慢漂移。
 * @note    输入单位：gx/gy/gz 必须为 rad/s；ax/ay/az、mx/my/mz 只需保持同一
 *          坐标系（机体坐标系，通常 x 前、y 左/右、z 上，需与传感器安装一致），
 *          函数内部会归一化，量纲不影响结果；四元数顺序为 q[0]=w, q[1..3]=x,y,z。
 * @note    坐标系为 NED/ENU 取决于传入的传感器符号约定，算法本身假设
 *          归一化重力指向 z 轴、地磁场位于 x-z 平面。
 * @note    sampleFreq 硬编码为 150 Hz，积分步长 1/sampleFreq 由此宏决定；
 *          实际调用频率变化时必须同步修改该宏。
 * @note    invSqrt() 使用 0x5f3759df 快速平方根倒数（含一次牛顿迭代），
 *          依赖 IEEE754 float 与 32 位 long 的内存布局，非通用平台需替换。
 */
//---------------------------------------------------------------------------------------------------
// Definitions

/* Madgwick 梯度下降步长：越大收敛越快，但高频噪声/振动下姿态抖动更明显 */
#define betaDef		0.0001f	
/* 姿态解算调用频率（Hz），决定积分时间步长 1/sampleFreq；须与 IMU 采样定时器一致 */
#define sampleFreq	150.0f			// sample frequency in Hz
/* Mahony PI 反馈的 2*Kp：加速度/磁力计误差的即时修正强度，默认 10 */
#define twoKpDef	(2.0f * 5.0f)	// 2 * proportional gain
/* Mahony PI 反馈的 2*Ki：陀螺零偏积分补偿强度，默认 0 表示关闭积分反馈 */
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
/**
 * @name    全局运行变量
 * @details twoKp/twoKi 可由外部在线调节以整定 Mahony 滤波器的收敛速度；
 *          integralFBx/y/z 保存按 Ki 缩放后的误差积分，用于补偿陀螺常值零偏，
 *          仅在本文件内使用（未加 static）；beta 供 Madgwick 梯度下降使用。
 * @note    被注释掉的 q0~q3 是原始 Madgwick 实现的全局四元数版本，
 *          工程改为由调用方传入 float q[4]，故此处不再定义。
 */
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

volatile float beta = betaDef;								// 2 * proportional gain (Kp)

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

/**
 * @brief Mahony 九轴姿态更新：用重力与地磁参考方向对陀螺积分做 PI 修正。
 * @details 主要步骤：
 *          1) 磁力计三轴全 0 时退化为六轴 MahonyAHRSupdateIMU() 并返回；
 *          2) 归一化加速度与磁场数据；
 *          3) 由四元数推出机体估计的重力方向 (halfvx..vz) 与磁场方向
 *             (halfwx..wz)，其中磁场参考方向先把测量值旋到水平面得到 bx/bz；
 *          4) 估计方向与测量方向叉积得到误差 (halfex/ey/ez)；
 *          5) 误差按 twoKi 积分（补偿陀螺零偏）、按 twoKp 比例叠加到角速度；
 *          6) 用修正后的角速度积分四元数微分方程并归一化。
 * @param q 四元数 {w,x,y,z}，原地读写。
 * @param gx/gy/gz 三轴角速度，单位 rad/s，函数内会被反馈量修改（局部副本）。
 * @param ax/ay/az 三轴加速度，任意量纲，内部归一化。
 * @param mx/my/mz 三轴磁场，任意量纲，内部归一化。
 * @note  加速度全为 0 时跳过 2~5 步，仅执行陀螺积分；
 *        twoKi <= 0 时清零积分项以防积分饱和；积分步长固定为 1/sampleFreq。
 */
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	/* 函数内部流程步骤：
	 * 步骤 1：声明四元数乘积缓存与参考/估计方向、误差、积分辅助变量。
	 * 步骤 2：若磁力计三轴全为 0，改调 MahonyAHRSupdateIMU() 后立即返回，
	 *         避免对零向量归一化得到 NaN。
	 * 步骤 3：加速度计有效时，先用 invSqrt() 归一化加速度，再归一化磁场。
	 * 步骤 4：预计算 q0q0..q3q3 等 10 个四元数乘积，减少后面重复运算。
	 * 步骤 5：把磁场测量旋到地理水平面得到参考 (bx, bz)，再由四元数推得
	 *         估计的重力方向 halfv* 与磁场方向 halfw*。
	 * 步骤 6：估计方向与测量方向做叉积并求和，得到三轴误差 halfex/ey/ez。
	 * 步骤 7：twoKi > 0 时按 1/sampleFreq 累积积分反馈并叠加到角速度；
	 *         否则清零积分项防饱和；随后叠加 twoKp 比例反馈。
	 * 步骤 8：角速度乘 0.5*(1/sampleFreq) 得到四元数增量，用 qa/qb/qc
	 *         保存旧分量后按四元数微分方程更新 q[0..3]。
	 * 步骤 9：对四元数整体归一化，消除数值积分带来的模长漂移。
	 */
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q[0] * q[0];
        q0q1 = q[0] * q[1];
        q0q2 = q[0] * q[2];
        q0q3 = q[0] * q[3];
        q1q1 = q[1] * q[1];
        q1q2 = q[1] * q[2];
        q1q3 = q[1] * q[3];
        q2q2 = q[2] * q[2];
        q2q3 = q[2] * q[3];
        q3q3 = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

/**
 * @brief Mahony 六轴姿态更新：只用重力方向修正陀螺积分（无航向绝对参考）。
 * @details 与九轴版本的差别仅在误差来源：
 *          1) 归一化加速度后，由四元数推得估计重力方向 (halfvx..vz)；
 *          2) halfex/ey/ez 只取 (a_measured × v_estimated)，不含磁场叉积项；
 *          3) 其余积分反馈、比例反馈、四元数积分与归一化完全相同。
 * @param q 四元数 {w,x,y,z}，原地读写。
 * @param gx/gy/gz 三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度（比力），函数内归一化后作为重力参考。
 * @note  加速度全 0 时跳过反馈，仅靠陀螺积分；因此横滚/俯仰可靠，
 *        航向角会随陀螺零慢慢漂移，需要磁力计或外部航向参考时请改用九轴版本。
 * @note  本函数同时被 MahonyAHRSupdate() 在磁力计数据无效时调用。
 */
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
	/* 函数内部流程步骤：
	 * 步骤 1：声明归一化倒数、估计重力方向 halfv*、误差 halfe* 与 qa/qb/qc。
	 * 步骤 2：仅当加速度三轴不全为 0 时才进入反馈分支（否则跳过 3~7，
	 *         避免对零向量归一化；退化情况仅执行陀螺积分）。
	 * 步骤 3：用 invSqrt(ax²+ay²+az²) 归一化加速度向量。
	 * 步骤 4：由当前四元数直接计算估计重力方向 halfvx/halfvy/halfvz。
	 * 步骤 5：用测量加速度与估计重力方向的叉积得到三轴误差 halfex/ey/ez
	 *         （无磁场项，故航向无绝对参考）。
	 * 步骤 6：twoKi > 0 时按 1/sampleFreq 累积积分误差并叠加到 gx/gy/gz；
	 *         否则清零 integralFB* 防止积分饱和。
	 * 步骤 7：把 twoKp * halfe* 的比例反馈叠加到角速度。
	 * 步骤 8：gx/gy/gz 乘 0.5*(1/sampleFreq) 得四元数增量，保存旧 q 到
	 *         qa/qb/qc 后积分更新 q[0..3]（用旧值避免串行覆盖）。
	 * 步骤 9：invSqrt 归一化四元数后返回。
	 */
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

/**
 * @brief Madgwick 九轴姿态更新：梯度下降法融合陀螺、加速度计与磁力计。
 * @details 主要步骤：
 *          1) 磁力计全 0 时退化为 MadgwickAHRSupdateIMU() 并返回；
 *          2) 由陀螺角速度计算四元数导数 qDot1~qDot4；
 *          3) 归一化加速度与磁场数据，把磁场测量旋转到地理水平面，
 *             得到参考方向 _2bx/_2bz（只保留水平分量与垂直分量）；
 *          4) 构造"估计方向 - 测量方向"的目标函数，对其求梯度得到
 *             修正方向 s0~s3，归一化后按步长 beta 从 qDot 中扣除；
 *          5) 以 1/sampleFreq 积分四元数导数、归一化并写回 q[4]。
 * @param q 四元数 {w,x,y,z}，原地读写。
 * @param gx/gy/gz 三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度，任意量纲，内部归一化。
 * @param mx/my/mz 三轴磁场，任意量纲，内部归一化。
 * @note  beta 越大收敛越快但越易受加速度/磁场噪声干扰；加速度全 0 时
 *        跳过 3~4 步，仅用陀螺积分；与 Mahony 版本不使用 twoKp/twoKi。
 */
void MadgwickAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    /* 函数内部流程步骤：
     * 步骤 1：把 q[0..3] 读入局部 q0~q3，声明导数、梯度与大量乘积缓存。
     * 步骤 2：磁力计三轴全 0 时改调 MadgwickAHRSupdateIMU() 并返回。
     * 步骤 3：由陀螺角速度计算四元数导数 qDot1~qDot4 = 0.5 * q ⊗ ω。
     * 步骤 4：加速度有效时分别归一化加速度与磁场向量。
     * 步骤 5：预计算 _2q*、q0q0..q3q3 等辅助量，减少后续重复乘法。
     * 步骤 6：把磁场旋到地理水平面得到参考 _2bx/_2bz，并展开成 _4bx/_4bz。
     * 步骤 7：按目标函数梯度解析式计算修正方向 s0~s3（含重力与磁场误差），
     *         再用 invSqrt 归一化该梯度步长。
     * 步骤 8：从 qDot 中扣除 beta * s*，得到融合后的四元数导数。
     * 步骤 9：按 1/sampleFreq 积分 q0~q3，归一化后写回 q[0..3]。
     */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float q0=q[0];
    float q1=q[1];
    float q2=q[2];
    float q3=q[3];

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(q, gx, gy, gz, ax, ay, az);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    q[0]=q0;
    q[1]=q1;
    q[2]=q2;
    q[3]=q3;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

/**
 * @brief Madgwick 六轴姿态更新：梯度下降法融合陀螺与加速度计。
 * @details 与九轴版本的区别：目标函数只包含重力方向误差，不含磁场项，
 *          因此 s0~s3 的梯度表达式更短（只用到 _2q*、_4q*、_8q* 等辅助量）。
 *          流程仍为：陀螺计算 qDot -> 归一化加速度 -> 计算并归一化梯度
 *          -> 按 beta 从 qDot 扣除 -> 积分四元数 -> 归一化并写回 q[4]。
 * @param q 四元数 {w,x,y,z}，原地读写。
 * @param gx/gy/gz 三轴角速度，单位 rad/s。
 * @param ax/ay/az 三轴加速度，任意量纲，内部归一化。
 * @note  加速度全 0 时跳过梯度修正，仅做陀螺积分；航向无绝对参考会漂移。
 * @note  也被 MadgwickAHRSupdate() 在磁力计数据无效时作为退化入口调用。
 */
void MadgwickAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az) {
    /* 函数内部流程步骤：
     * 步骤 1：把 q[0..3] 读入 q0~q3，声明导数、梯度与 _2q*/_4q*/_8q* 缓存。
     * 步骤 2：由陀螺角速度计算四元数导数 qDot1~qDot4。
     * 步骤 3：加速度不全为 0 时，先 invSqrt 归一化加速度（否则跳过 4~6）。
     * 步骤 4：预计算 2/4/8 倍四元数项与各平方项，避免重复乘法。
     * 步骤 5：由「估计重力方向与测量加速度之差」的目标函数梯度求 s0~s3，
     *         再用 invSqrt 归一化步长。
     * 步骤 6：从 qDot 中减去 beta * s*，得到修正后的四元数导数。
     * 步骤 7：按 1/sampleFreq 积分 q0~q3，归一化后写回 q[0..3]。
     */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    float q0=q[0];
    float q1=q[1];
    float q2=q[2];
    float q3=q[3];

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q[0]=q0;
    q[1]=q1;
    q[2]=q2;
    q[3]=q3;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

/**
 * @brief 快速计算 1/sqrt(x)（经典 Quake III 快速平方根倒数算法）。
 * @param x 必须为非负浮点数；本文件的调用点均为平方和，恒为正。
 * @return x 的平方根倒数的近似值。
 * @details 把 float 的位模式当作 long 整数，用魔数 0x5f3759df 减去右移一位
 *          的值得到初值，再做一次牛顿迭代 y = y*(1.5 - 0.5*x*y*y) 提高精度。
 *          相比 sqrtf() + 除法显著更快，适合在 MCU 上高频归一化向量/四元数。
 * @note  依赖 IEEE754 单精度浮点布局且 long 为 32 位；在 long 为 64 位的
 *        平台上结果错误，应改用 1.0f / sqrtf(x)。
 * @note  仅做一次牛顿迭代，相对误差约千分之几，对姿态归一化足够。
 */
float invSqrt(float x) {
	/* 函数内部流程步骤：
	 * 步骤 1：halfx = 0.5 * x，为牛顿迭代准备常数；y 先取 x 本身。
	 * 步骤 2：通过 *(long*)&y 把 float 的 IEEE754 位模式当作整数读出到 i。
	 * 步骤 3：i = 0x5f3759df - (i >> 1) 得到 1/sqrt(x) 的魔数初值估计。
	 * 步骤 4：把 i 的位模式再解释回 float 存入 y。
	 * 步骤 5：做一次牛顿迭代 y = y * (1.5 - halfx*y*y) 提高精度并返回 y。
	 */
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
